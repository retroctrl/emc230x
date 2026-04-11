// Copyright (c) 2024 Jake Swensen
// SPDX-License-Identifier: MPL-2.0
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

//! The EMC230x device family is a fan controller with up to five independently
//! controlled PWM fan drivers.

#![no_std]

#[cfg(any(test, feature = "std"))]
extern crate std;

#[cfg(any(test, feature = "alloc"))]
extern crate alloc;

use core::fmt::{self, Debug, Formatter};
use embedded_hal_async as hal;
pub use fans::{FanControl, FanDutyCycle, FanRpm, FanSelect};
use hal::i2c::I2c;

pub use error::Error;
use registers::*;

mod error;
mod probe_result;
mod registers;

pub use probe_result::ProbeResult;

/// Default I2C address for the EMC2301 device
pub const EMC2301_I2C_ADDR: u8 = 0b0010_1111;

// I2C addresses for the EMC230x family, selected by the ADDR resistor configuration.
pub const EMC230X_I2C_ADDR_0: u8 = 0x2C;
pub const EMC230X_I2C_ADDR_1: u8 = 0x2D;
pub const EMC230X_I2C_ADDR_2: u8 = 0x2E;
pub const EMC230X_I2C_ADDR_3: u8 = 0x2F;
pub const EMC230X_I2C_ADDR_4: u8 = 0x4C;
pub const EMC230X_I2C_ADDR_5: u8 = 0x4D;

const EMC230X_ADDRESSES: [u8; 6] = [
    EMC230X_I2C_ADDR_0,
    EMC230X_I2C_ADDR_1,
    EMC230X_I2C_ADDR_2,
    EMC230X_I2C_ADDR_3,
    EMC230X_I2C_ADDR_4,
    EMC230X_I2C_ADDR_5,
];

/// Simplified RPM factor for calculating RPM from raw values
///
/// See Equation 4-3, page 17 of the datasheet. ((SIMPLIFIED_RPM_FACTOR * m) / COUNT)
const _SIMPLIFIED_RPM_FACTOR: f64 = 3_932_160.0;

/// Fetch a read-only register from the device
macro_rules! register_ro {
    ($get:ident, $return_type:ty) => {
        pub async fn $get(&mut self) -> Result<$return_type, Error> {
            self.read_register::<$return_type>(<$return_type>::ADDRESS)
                .await
        }
    };
}

/// Fetch and set a register from the device which applies to all fans
macro_rules! register {
    ($get:ident, $set:ident, $return_type:ty) => {
        pub async fn $get(&mut self) -> Result<$return_type, Error> {
            self.read_register::<$return_type>(<$return_type>::ADDRESS)
                .await
        }

        pub async fn $set(&mut self, value: $return_type) -> Result<(), Error> {
            self.write_register(<$return_type>::ADDRESS, value.into())
                .await?;
            Ok(())
        }
    };
}

/// Fetch and set a register from the device which applies to a specific fan
macro_rules! fan_register {
    ($get:ident, $set:ident, $reg_type:ty) => {
        pub async fn $get(&mut self, sel: FanSelect) -> Result<$reg_type, Error> {
            let sel = self.resolve_fan(sel)?;
            let reg = fan_register_address(sel, <$reg_type>::OFFSET)?;
            let value = self.read_register(reg).await?;
            Ok(value)
        }

        pub async fn $set(&mut self, sel: FanSelect, value: $reg_type) -> Result<(), Error> {
            let sel = self.resolve_fan(sel)?;
            let reg = fan_register_address(sel, <$reg_type>::OFFSET)?;
            self.write_register(reg, value.into()).await?;
            Ok(())
        }
    };
}

/// Round an `f64` to the nearest integer of type `$ty`.
///
/// Workaround for `core` not providing `round()` in `no_std`.
macro_rules! round_to {
    ($value:expr, $ty:ty) => {{
        let v: f64 = $value;
        let raw = v as $ty;
        if v - raw as f64 >= 0.5 {
            raw + 1
        } else {
            raw
        }
    }};
}
pub(crate) use round_to;

pub struct Emc230x<I2C> {
    /// I2C bus
    i2c: I2C,

    /// I2C address of the device
    address: u8,

    /// Device Product Identifier
    ///
    /// The product identification will determine the number of fans the device supports.
    pid: ProductId,

    /// Configurable number of poles in a fan. Typically 2.
    poles: [u8; 5],

    /// Enable or disable reverse fan indexing.
    ///
    /// When enabled, logical fan indices are mirrored around the device's fan count. For example,
    /// on an EMC2305 (5 fans), `FanSelect(1)` will map to hardware fan 5, `FanSelect(2)` to
    /// hardware fan 4, and so on. This is useful when the physical fan wiring order is the reverse
    /// of the hardware channel order.
    reverse_fan_index: bool,
}

impl<I2C> Debug for Emc230x<I2C> {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        f.debug_struct("Emc230x")
            .field("address", &self.address)
            .field("pid", &self.pid)
            .field("poles", &self.poles)
            .field("reverse_fan_index", &self.reverse_fan_index)
            .finish()
    }
}

impl<I2C: I2c> Emc230x<I2C> {
    /// Manufacturer ID
    const MANUFACTURER_ID: u8 = 0x5D;

    /// Tachometer measurement frequency (kHz)
    const TACH_FREQUENCY_HZ: f64 = 32_768.0;

    /// Determine if the device at the specified address is an EMC230x device
    async fn is_emc230x(i2c: &mut I2C, address: u8) -> Result<ProductId, Error> {
        let mfg_id: ManufacturerId = Self::raw_read(i2c, address, ManufacturerId::ADDRESS).await?;
        if mfg_id.mfg_id() == Self::MANUFACTURER_ID {
            let pid: ProductId = Self::raw_read(i2c, address, ProductId::ADDRESS).await?;
            Ok(pid)
        } else {
            Err(Error::InvalidManufacturerId)
        }
    }

    /// Probe the I2C bus for any EMC230x devices.
    ///
    /// Checks all six possible EMC230x I2C addresses (0x2C–0x2F, 0x4C–0x4D) and returns a
    /// [`ProbeResult`] containing each address where an EMC230x was identified.
    /// Issues up to 12 I2C transactions (2 per address).
    ///
    /// The I2C bus is borrowed so the caller can pass it to [`Emc230x::new`]
    /// using one of the returned addresses.
    ///
    /// # Example
    /// ```ignore
    /// let found = Emc230x::probe(&mut i2c).await;
    /// if let Some(addr) = found.iter().next() {
    ///     let dev = Emc230x::new(i2c, addr).await?;
    /// }
    /// ```
    pub async fn probe(i2c: &mut I2C) -> ProbeResult {
        let mut result = ProbeResult::default();
        for (slot, &address) in EMC230X_ADDRESSES.iter().enumerate() {
            if Self::is_emc230x(i2c, address).await.is_ok() {
                result.0[slot] = true;
            }
        }
        result
    }

    /// Initialize a new EMC230x device at the specified address
    pub async fn new(i2c: I2C, address: u8) -> Result<Self, Error> {
        let mut i2c = i2c;
        let pid = Self::is_emc230x(&mut i2c, address).await?;

        // Assume 2 poles for all fans by default. This is common for most fans and is a safe default.
        let poles = [2; 5];

        let reverse_fan_index = false;

        // Form the device so that some defaults can be set
        let mut dev = Self {
            i2c,
            address,
            pid,
            poles,
            reverse_fan_index,
        };

        // Set all fan outputs to push-pull to avoid waveform distortion
        let mut output_cfg = pwm_output_config::PwmOutputConfig::default();
        let count = dev.count();
        for fan in 1..=count {
            output_cfg.push_pull(fan);
        }
        dev.set_pwm_output_config(output_cfg).await?;

        // Set RPM range to 500 RPM for all drives to capture slower fans
        for fan in 1..=count {
            let mut cfg = dev.fan_configuration1(FanSelect(fan)).await?;
            cfg.set_rngx(fan_configuration1::Range::Rpm500);
            dev.set_fan_configuration1(FanSelect(fan), cfg).await?;
        }

        // Device is configured
        Ok(dev)
    }

    /// Get the I2C address of the device
    fn address(&self) -> u8 {
        self.address
    }

    /// Get the number of fans the device supports
    pub fn count(&self) -> u16 {
        self.pid.num_fans()
    }

    /// Returns `true` if reverse fan indexing is enabled.
    pub fn reverse_fan_index(&self) -> bool {
        self.reverse_fan_index
    }

    /// Enable or disable reverse fan indexing.
    ///
    /// When enabled, logical fan indices are mirrored around the device's fan count. For example,
    /// on an EMC2305 (5 fans), `FanSelect(1)` will map to hardware fan 5, `FanSelect(2)` to
    /// hardware fan 4, and so on. This is useful when the physical fan wiring order is the reverse
    /// of the hardware channel order.
    pub fn set_reverse_fan_index(&mut self, enabled: bool) {
        self.reverse_fan_index = enabled;
    }

    /// Get the number of poles for the selected fan (used in RPM calculations)
    pub fn fan_poles(&self, sel: FanSelect) -> Result<u8, Error> {
        let sel = self.resolve_fan(sel)?;
        Ok(self.poles[sel.0 as usize - 1])
    }

    /// Set the number of poles for the selected fan (used in RPM calculations)
    ///
    /// It is unlikely that this value will need to change unless a non-standard fan is used.
    /// If it does need to change, there are likely other configuration changes that need to
    /// happen as well.
    pub fn set_fan_poles(&mut self, sel: FanSelect, poles: u8) -> Result<(), Error> {
        let sel = self.resolve_fan(sel)?;
        self.poles[sel.0 as usize - 1] = poles;
        Ok(())
    }

    /// Get the tachometer frequency of the device
    fn tach_freq(&self) -> f64 {
        Self::TACH_FREQUENCY_HZ
    }

    /// Get the mode of the fan
    pub async fn mode(&mut self, sel: FanSelect) -> Result<FanControl, Error> {
        self.valid_fan(sel)?;
        let config = self.fan_configuration1(sel).await?;

        if config.enagx() {
            // RPM mode: read the tach target registers and convert to RPM
            let raw_low = self.tach_target_low_byte(sel).await?;
            let raw_high = self.tach_target_high_byte(sel).await?;

            let raw = u16::from_le_bytes([raw_low.into(), raw_high.into()]) >> 3;
            let rpm = self.calc_raw_rpm(sel, raw).await?;

            Ok(FanControl::Rpm(rpm))
        } else {
            // Duty cycle mode: read the fan drive setting
            let drive = self.fan_setting(sel).await?;
            Ok(FanControl::DutyCycle(drive.duty_cycle()))
        }
    }

    /// Set the mode of the fan
    pub async fn set_mode(&mut self, sel: FanSelect, mode: FanControl) -> Result<(), Error> {
        self.valid_fan(sel)?;
        let mut config = self.fan_configuration1(sel).await?;

        match mode {
            FanControl::DutyCycle(duty) => {
                // Disable RPM mode first if it is enabled.
                //
                // The device appears to set the fan drive with the duty cycle corresponding to the
                // last target RPM set. If the mode bit is set after the duty cycle, the desired
                // duty cycle gets overwritten.
                config.set_enagx(false);
                self.set_fan_configuration1(sel, config).await?;

                self.set_duty_cycle(sel, duty).await?;
            }
            FanControl::Rpm(rpm) => {
                self.set_rpm(sel, rpm).await?;

                config.set_enagx(true);
                self.set_fan_configuration1(sel, config).await?;
            }
        }

        Ok(())
    }

    /// Fetch the current duty cycle of the fan
    pub async fn duty_cycle(&mut self, sel: FanSelect) -> Result<FanDutyCycle, Error> {
        self.valid_fan(sel)?;
        let drive = self.fan_setting(sel).await?;
        let duty = drive.duty_cycle();
        Ok(duty)
    }

    /// Set the duty cycle of the fan
    pub async fn set_duty_cycle(
        &mut self,
        sel: FanSelect,
        duty: FanDutyCycle,
    ) -> Result<(), Error> {
        self.valid_fan(sel)?;
        let drive = FanDriveSetting::from_duty_cycle(duty);
        self.set_fan_setting(sel, drive).await?;
        Ok(())
    }

    /// Fetch the current RPM of the fan
    pub async fn rpm(&mut self, sel: FanSelect) -> Result<FanRpm, Error> {
        self.valid_fan(sel)?;
        let raw_low = self.tach_reading_low_byte(sel).await?;
        let raw_high = self.tach_reading_high_byte(sel).await?;

        let raw = u16::from_le_bytes([raw_low.into(), raw_high.into()]) >> 3;
        let rpm = self.calc_raw_rpm(sel, raw).await?;

        Ok(rpm)
    }

    /// Set the target RPM of the fan
    pub async fn set_rpm(&mut self, sel: FanSelect, rpm: FanRpm) -> Result<(), Error> {
        self.valid_fan(sel)?;

        let raw = self.calc_raw_rpm(sel, rpm).await?;
        let count = (raw << 3).to_le_bytes();

        self.set_tach_target_low_byte(sel, count[0].into()).await?;
        self.set_tach_target_high_byte(sel, count[1].into()).await?;
        Ok(())
    }

    /// Fetch the current duty cycle and RPM of the fan
    pub async fn report(&mut self, sel: FanSelect) -> Result<(FanDutyCycle, FanRpm), Error> {
        self.valid_fan(sel)?;
        let duty = self.duty_cycle(sel).await?;
        let rpm = self.rpm(sel).await?;
        Ok((duty, rpm))
    }

    /// Minimum configured duty cycle the fan will run at.
    pub async fn min_duty(&mut self, sel: FanSelect) -> Result<FanDutyCycle, Error> {
        self.valid_fan(sel)?;
        let drive = self.minimum_drive(sel).await?;
        Ok(drive.duty_cycle())
    }

    /// Set the minimum duty cycle the fan will run at.
    pub async fn set_min_duty(&mut self, sel: FanSelect, duty: FanDutyCycle) -> Result<(), Error> {
        self.valid_fan(sel)?;
        let drive = FanMinimumDrive::from_duty_cycle(duty);
        self.set_minimum_drive(sel, drive).await?;
        Ok(())
    }

    /// Returns `true` if a spinning fan is detected on the selected channel.
    ///
    /// Reads the [`FanStallStatus`] register and checks the per-fan stall bit. A clear bit
    /// means the tachometer is receiving pulses - indicating a connected, spinning fan. A set
    /// bit means the tach count exceeded the [`ValidTachCount`] threshold, which occurs when
    /// no fan is connected **or** when a connected fan is not spinning.
    ///
    /// This is the closest the hardware can indicate fan presence; it cannot distinguish
    /// between an absent fan and a stalled one.
    pub async fn fan_detected(&mut self, sel: FanSelect) -> Result<bool, Error> {
        let sel = self.resolve_fan(sel)?;
        let status = self.stall_status().await?;
        let stalled = (u8::from(status) >> (sel.0 as u8 - 1)) & 1 != 0;
        Ok(!stalled)
    }

    /// Calculate either the RPM or raw value of the RPM based on the input value.
    async fn calc_raw_rpm(&mut self, sel: FanSelect, value: u16) -> Result<u16, Error> {
        let cfg = self.fan_configuration1(sel).await?;

        let poles = self.fan_poles(sel)? as f64;
        let n = cfg.edgx().num_edges() as f64;
        let m = cfg.rngx().tach_count_multiplier() as f64;
        let f_tach = self.tach_freq();

        let value = ((1.0 / poles) * (n - 1.0)) / (value as f64 * (1.0 / m)) * f_tach * 60.0;
        Ok(round_to!(value, u16))
    }

    /// Write a value to a register on the device
    async fn write_register(&mut self, reg: u8, data: u8) -> Result<(), Error> {
        let addr = self.address();
        let data = [reg, data];
        self.i2c.write(addr, &data).await.map_err(|_| Error::I2c)
    }

    /// Read a value from a register on the device attached to the I2C bus
    async fn raw_read<T: TryFrom<u8>>(i2c: &mut I2C, address: u8, reg: u8) -> Result<T, Error> {
        let mut data = [0];
        i2c.write_read(address, &[reg], data.as_mut_slice())
            .await
            .map_err(|_| Error::I2c)?;

        let data: T = data[0]
            .try_into()
            .map_err(|_| Error::RegisterTypeConversion)?;

        Ok(data)
    }

    /// Read a value from a register on the device
    async fn read_register<T: TryFrom<u8>>(&mut self, reg: u8) -> Result<T, Error> {
        let addr = self.address();
        let data = Self::raw_read(&mut self.i2c, addr, reg).await?;
        Ok(data)
    }

    /// Determine if the fan number is valid by comparing it to the number of fans the device supports.
    fn valid_fan(&self, select: FanSelect) -> Result<(), Error> {
        if select.0 <= self.count() && select.0 != 0 {
            Ok(())
        } else {
            Err(Error::InvalidFan)
        }
    }

    /// Resolve and validate a user-facing [`FanSelect`] to a hardware fan index.
    ///
    /// When [`reverse_fan_index`](Self::reverse_fan_index) is enabled, mirror the fan selection.
    /// When reverse indexing is disabled, the fan select is returned unchanged.
    fn resolve_fan(&self, sel: FanSelect) -> Result<FanSelect, Error> {
        self.valid_fan(sel)?;
        if self.reverse_fan_index {
            Ok(FanSelect(self.count() - sel.0 + 1))
        } else {
            Ok(sel)
        }
    }

    /// Release the I2C bus from the device
    pub fn release(self) -> I2C {
        self.i2c
    }

    // General register access
    register!(config, set_config, Configuration);
    register_ro!(status, FanStatus);
    register_ro!(stall_status, FanStallStatus);
    register_ro!(spin_status, FanSpinStatus);
    register_ro!(drive_fail_status, FanDriveFailStatus);
    register!(interrupt_enable, set_interrupt_enable, FanInterruptEnable);
    register!(pwm_polarity_config, set_pwm_polarity_config, PwmPolarityConfig);
    register!(pwm_output_config, set_pwm_output_config, PwmOutputConfig);
    register!(pwm_base_f45, set_pwm_base_f45, PwmBase45);
    register!(pwm_base_f123, set_pwm_base_f123, PwmBase123);

    // Fan specific register access
    fan_register!(fan_setting, set_fan_setting, FanDriveSetting);
    fan_register!(pwm_divide, set_pwm_divide, PwmDivide);
    fan_register!(fan_configuration1, set_fan_configuration1, FanConfiguration1);
    fan_register!(fan_configuration2, set_fan_configuration2, FanConfiguration2);
    fan_register!(gain, set_gain, PidGain);
    fan_register!(spin_up_configuration, set_spin_up_configuration, FanSpinUpConfig);
    fan_register!(max_step, set_max_step, MaxStepSize);
    fan_register!(minimum_drive, set_minimum_drive, FanMinimumDrive);
    fan_register!(valid_tach_count, set_valid_tach_count, ValidTachCount);
    fan_register!(drive_fail_band_low_byte, set_drive_fail_band_low_byte, DriveFailBandLow);
    fan_register!(drive_fail_band_high_byte, set_drive_fail_band_high_byte, DriveFailBandHigh);
    fan_register!(tach_target_low_byte, set_tach_target_low_byte, TachTargetLow);
    fan_register!(tach_target_high_byte, set_tach_target_high_byte, TachTargetHigh);
    fan_register!(tach_reading_high_byte, set_tach_reading_high_byte, TachReadingHigh);
    fan_register!(tach_reading_low_byte, set_tach_reading_low_byte, TachReadingLow);

    // Chip registers
    register_ro!(software_lock, SoftwareLock);
    register_ro!(product_features, ProductFeatures);
    register_ro!(product_id, ProductId);

    /// Dump all the info and registers from the EMC230x Device
    pub async fn dump_info(&mut self) -> Result<(), Error> {
        macro_rules! defmt_info_register {
            ($dev:expr, $reg:tt) => {
                let value = $dev.$reg().await?;
                defmt::info!("{}: {:#04x}", stringify!($reg), u8::from(value));
            };
        }

        macro_rules! defmt_info_fan_register {
            ($dev:expr, $reg:tt, $fan:expr) => {
                let value = $dev.$reg(FanSelect($fan)).await?;
                defmt::info!("{}: {:#04x}", stringify!($reg), u8::from(value));
            };
        }

        let count = self.count();

        defmt::info!("Address: {:#04x}", self.address());
        defmt::info!("Fan Count: {}", count);

        defmt_info_register!(self, software_lock);
        defmt_info_register!(self, product_features);
        defmt_info_register!(self, product_id);

        defmt_info_register!(self, config);
        defmt_info_register!(self, status);
        defmt_info_register!(self, stall_status);
        defmt_info_register!(self, spin_status);
        defmt_info_register!(self, drive_fail_status);
        defmt_info_register!(self, interrupt_enable);
        defmt_info_register!(self, pwm_polarity_config);
        defmt_info_register!(self, pwm_output_config);
        defmt_info_register!(self, pwm_base_f45);
        defmt_info_register!(self, pwm_base_f123);

        for fan in 1..=count {
            defmt::info!("Fan: {} ----------------------", fan);
            defmt_info_fan_register!(self, fan_setting, fan);
            defmt_info_fan_register!(self, pwm_divide, fan);
            defmt_info_fan_register!(self, fan_configuration1, fan);
            defmt_info_fan_register!(self, fan_configuration2, fan);
            defmt_info_fan_register!(self, gain, fan);
            defmt_info_fan_register!(self, spin_up_configuration, fan);
            defmt_info_fan_register!(self, max_step, fan);
            defmt_info_fan_register!(self, minimum_drive, fan);
            defmt_info_fan_register!(self, valid_tach_count, fan);
            defmt_info_fan_register!(self, drive_fail_band_low_byte, fan);
            defmt_info_fan_register!(self, drive_fail_band_high_byte, fan);
            defmt_info_fan_register!(self, tach_target_low_byte, fan);
            defmt_info_fan_register!(self, tach_target_high_byte, fan);
            defmt_info_fan_register!(self, tach_reading_high_byte, fan);
            defmt_info_fan_register!(self, tach_reading_low_byte, fan);
        }

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cTransaction};
    use std::{vec, vec::Vec};

    use crate::registers::tach_reading::TachReading;

    /// Transaction expectation builder for a [`Emc230x`] device.
    #[derive(Clone, Debug)]
    struct Emc230xExpectationBuilder {
        address: u8,
        _product_id: ProductId,
        transactions: Vec<I2cTransaction>,
    }

    impl Emc230xExpectationBuilder {
        /// Create expectations for a mock [`Emc230x`] device that has not been initialized.
        fn new(address: u8, pid: ProductId) -> Self {
            let mut transactions = vec![
                I2cTransaction::write_read(address, vec![ManufacturerId::ADDRESS], vec![0x5D]),
                I2cTransaction::write_read(address, vec![ProductId::ADDRESS], vec![pid.into()]),
            ];

            // Set the output configuration to push-pull for all fans
            let mut output_cfg = 0x00_u8;
            for fan in 1..=pid.num_fans() {
                output_cfg |= 1 << (fan - 1);
            }
            transactions
                .push(I2cTransaction::write(address, vec![PwmOutputConfig::ADDRESS, output_cfg]));

            for fan in 1..=pid.num_fans() {
                transactions.push(I2cTransaction::write_read(
                    address,
                    vec![FanConfiguration1::fan_address(FanSelect(fan))
                        .expect("Could not set fan address")],
                    vec![FanConfiguration1::default().into()],
                ));

                transactions.push(I2cTransaction::write(
                    address,
                    vec![
                        FanConfiguration1::fan_address(FanSelect(fan))
                            .expect("Could not set fan address"),
                        0x0B,
                    ],
                ));
            }

            Self {
                address,
                _product_id: pid,
                transactions,
            }
        }

        /// Set expectations to retrieve the duty cycle of a fan.
        fn duty_cycle(&mut self, select: FanSelect, duty_cycle: u8) {
            let raw = FanDriveSetting::from_duty_cycle(duty_cycle);

            self.transactions.push(I2cTransaction::write_read(
                self.address,
                vec![FanDriveSetting::fan_address(select).expect("Could not set fan address")],
                vec![raw.into()],
            ));
        }

        /// Set expectations to retrieve the RPM of a fan.
        fn rpm(&mut self, select: FanSelect, rpm: u16) {
            let mut default_cfg = FanConfiguration1::default();
            default_cfg.set_rngx(fan_configuration1::Range::Rpm500);

            let raw = (_SIMPLIFIED_RPM_FACTOR * default_cfg.rngx().tach_count_multiplier() as f64
                / rpm as f64) as u16;
            let count: TachReading = TachReading::from(raw);

            self.transactions.push(I2cTransaction::write_read(
                self.address,
                vec![TachReadingLow::fan_address(select).expect("Could not set fan address")],
                vec![count.raw_low()],
            ));

            self.transactions.push(I2cTransaction::write_read(
                self.address,
                vec![TachReadingHigh::fan_address(select).expect("Could not set fan address")],
                vec![count.raw_high()],
            ));

            self.transactions.push(I2cTransaction::write_read(
                self.address,
                vec![FanConfiguration1::fan_address(select).expect("Could not set fan address")],
                vec![default_cfg.into()],
            ));
        }

        fn build(self) -> Vec<I2cTransaction> {
            self.transactions
        }
    }

    #[tokio::test]
    async fn probe_no_devices() {
        use embedded_hal::i2c::{ErrorKind, NoAcknowledgeSource};

        let expectations: Vec<I2cTransaction> = EMC230X_ADDRESSES
            .iter()
            .map(|&addr| {
                I2cTransaction::write_read(addr, vec![ManufacturerId::ADDRESS], vec![0])
                    .with_error(ErrorKind::NoAcknowledge(NoAcknowledgeSource::Address))
            })
            .collect();

        let mut i2c = I2cMock::new(&expectations);
        let result = Emc230x::probe(&mut i2c).await;
        assert!(result.is_empty());
        i2c.done();
    }

    #[tokio::test]
    async fn probe_one_device() {
        use embedded_hal::i2c::{ErrorKind, NoAcknowledgeSource};

        let mut expectations: Vec<I2cTransaction> = Vec::new();
        for &addr in EMC230X_ADDRESSES.iter() {
            if addr == EMC230X_I2C_ADDR_3 {
                expectations.push(I2cTransaction::write_read(
                    addr,
                    vec![ManufacturerId::ADDRESS],
                    vec![0x5D],
                ));
                expectations.push(I2cTransaction::write_read(
                    addr,
                    vec![ProductId::ADDRESS],
                    vec![ProductId::Emc2301.into()],
                ));
            } else {
                expectations.push(
                    I2cTransaction::write_read(addr, vec![ManufacturerId::ADDRESS], vec![0])
                        .with_error(ErrorKind::NoAcknowledge(NoAcknowledgeSource::Address)),
                );
            }
        }

        let mut i2c = I2cMock::new(&expectations);
        let result = Emc230x::probe(&mut i2c).await;
        assert_eq!(result.len(), 1);
        let addrs: Vec<u8> = result.iter().collect();
        assert_eq!(addrs, vec![EMC230X_I2C_ADDR_3]);
        i2c.done();
    }

    #[tokio::test]
    async fn probe_wrong_manufacturer_id() {
        use embedded_hal::i2c::{ErrorKind, NoAcknowledgeSource};

        let mut expectations: Vec<I2cTransaction> = Vec::new();
        for &addr in EMC230X_ADDRESSES.iter() {
            if addr == EMC230X_I2C_ADDR_0 {
                // Responds but returns wrong manufacturer ID
                expectations.push(I2cTransaction::write_read(
                    addr,
                    vec![ManufacturerId::ADDRESS],
                    vec![0x00],
                ));
            } else {
                expectations.push(
                    I2cTransaction::write_read(addr, vec![ManufacturerId::ADDRESS], vec![0])
                        .with_error(ErrorKind::NoAcknowledge(NoAcknowledgeSource::Address)),
                );
            }
        }

        let mut i2c = I2cMock::new(&expectations);
        let result = Emc230x::probe(&mut i2c).await;
        assert!(result.is_empty());
        i2c.done();
    }

    #[tokio::test]
    async fn probe_multiple_devices() {
        use embedded_hal::i2c::{ErrorKind, NoAcknowledgeSource};

        let mut expectations: Vec<I2cTransaction> = Vec::new();
        for &addr in EMC230X_ADDRESSES.iter() {
            if addr == EMC230X_I2C_ADDR_0 {
                expectations.push(I2cTransaction::write_read(
                    addr,
                    vec![ManufacturerId::ADDRESS],
                    vec![0x5D],
                ));
                expectations.push(I2cTransaction::write_read(
                    addr,
                    vec![ProductId::ADDRESS],
                    vec![ProductId::Emc2305.into()],
                ));
            } else if addr == EMC230X_I2C_ADDR_5 {
                expectations.push(I2cTransaction::write_read(
                    addr,
                    vec![ManufacturerId::ADDRESS],
                    vec![0x5D],
                ));
                expectations.push(I2cTransaction::write_read(
                    addr,
                    vec![ProductId::ADDRESS],
                    vec![ProductId::Emc2303.into()],
                ));
            } else {
                expectations.push(
                    I2cTransaction::write_read(addr, vec![ManufacturerId::ADDRESS], vec![0])
                        .with_error(ErrorKind::NoAcknowledge(NoAcknowledgeSource::Address)),
                );
            }
        }

        let mut i2c = I2cMock::new(&expectations);
        let result = Emc230x::probe(&mut i2c).await;
        assert_eq!(result.len(), 2);
        let addrs: Vec<u8> = result.iter().collect();
        assert_eq!(addrs, vec![EMC230X_I2C_ADDR_0, EMC230X_I2C_ADDR_5]);
        i2c.done();
    }

    #[tokio::test]
    async fn new() {
        let expectations =
            Emc230xExpectationBuilder::new(EMC2301_I2C_ADDR, ProductId::Emc2301).build();

        let i2c = I2cMock::new(&expectations);
        let dev = crate::Emc230x::new(i2c, EMC2301_I2C_ADDR)
            .await
            .expect("Could not create device");

        let mut i2c = dev.release();
        i2c.done();
    }

    #[tokio::test]
    async fn get_duty_cycle() {
        let expected_duty_cycle = [100, 75, 50, 25, 0];

        let mut expectations = Emc230xExpectationBuilder::new(EMC2301_I2C_ADDR, ProductId::Emc2301);

        for value in expected_duty_cycle {
            expectations.duty_cycle(FanSelect(1), value);
        }

        let expectations = expectations.build();

        let i2c = I2cMock::new(&expectations);
        let mut dev = crate::Emc230x::new(i2c, EMC2301_I2C_ADDR)
            .await
            .expect("Could not create device");

        for expected in expected_duty_cycle {
            let result = dev
                .duty_cycle(FanSelect(1))
                .await
                .expect("Could not get duty cycle");
            assert_eq!(expected, result);
        }

        let mut i2c = dev.release();
        i2c.done();
    }

    #[tokio::test]
    async fn get_rpm() {
        let expected_rpm: [u16; 15_500] = core::array::from_fn(|i| (i + 500) as u16);
        let mut expectations = Emc230xExpectationBuilder::new(EMC2301_I2C_ADDR, ProductId::Emc2301);

        for value in expected_rpm {
            expectations.rpm(FanSelect(1), value);
        }

        let expectations = expectations.build();

        let i2c = I2cMock::new(&expectations);
        let mut dev = crate::Emc230x::new(i2c, EMC2301_I2C_ADDR)
            .await
            .expect("Could not create device");

        for expected in expected_rpm {
            let result = dev.rpm(FanSelect(1)).await.expect("Could not get RPM");

            // Allow a ±1% margin of error due to count step size varying over different settings
            let range = std::ops::Range {
                start: expected as f64 * 0.99,
                end: expected as f64 * 1.01,
            };
            assert!(
                range.contains(&(result as f64)),
                "RPM was out of expected range; Expected: {} in Range: {:?} Got: {}",
                expected,
                range,
                result
            );
        }

        let mut i2c = dev.release();
        i2c.done();
    }

    #[tokio::test]
    async fn emc2305_duty_cycle_fan5() {
        let mut expectations =
            Emc230xExpectationBuilder::new(EMC230X_I2C_ADDR_0, ProductId::Emc2305);
        expectations.duty_cycle(FanSelect(5), 75);
        let expectations = expectations.build();

        let i2c = I2cMock::new(&expectations);
        let mut dev = Emc230x::new(i2c, EMC230X_I2C_ADDR_0)
            .await
            .expect("Could not create device");

        let result = dev
            .duty_cycle(FanSelect(5))
            .await
            .expect("Could not get duty cycle for fan 5");
        assert_eq!(75, result);

        let mut i2c = dev.release();
        i2c.done();
    }

    #[tokio::test]
    async fn emc2305_rpm_fan5() {
        let expected_rpm: u16 = 1000;
        let mut expectations =
            Emc230xExpectationBuilder::new(EMC230X_I2C_ADDR_0, ProductId::Emc2305);
        expectations.rpm(FanSelect(5), expected_rpm);
        let expectations = expectations.build();

        let i2c = I2cMock::new(&expectations);
        let mut dev = Emc230x::new(i2c, EMC230X_I2C_ADDR_0)
            .await
            .expect("Could not create device");

        let result = dev
            .rpm(FanSelect(5))
            .await
            .expect("Could not get RPM for fan 5");

        let range = std::ops::Range {
            start: expected_rpm as f64 * 0.99,
            end: expected_rpm as f64 * 1.01,
        };
        assert!(
            range.contains(&(result as f64)),
            "RPM was out of expected range; Expected: {} in Range: {:?} Got: {}",
            expected_rpm,
            range,
            result
        );

        let mut i2c = dev.release();
        i2c.done();
    }

    #[tokio::test]
    async fn fan_detected_spinning() {
        let mut expectations = Emc230xExpectationBuilder::new(EMC2301_I2C_ADDR, ProductId::Emc2301);
        // stall status = 0x00 -> fan 1 not stalled
        expectations.transactions.push(I2cTransaction::write_read(
            EMC2301_I2C_ADDR,
            vec![FanStallStatus::ADDRESS],
            vec![0x00],
        ));
        let expectations = expectations.build();

        let i2c = I2cMock::new(&expectations);
        let mut dev = Emc230x::new(i2c, EMC2301_I2C_ADDR)
            .await
            .expect("Could not create device");

        let result = dev
            .fan_detected(FanSelect(1))
            .await
            .expect("fan_detected failed");
        assert!(result);

        let mut i2c = dev.release();
        i2c.done();
    }

    #[tokio::test]
    async fn fan_detected_stalled() {
        let mut expectations = Emc230xExpectationBuilder::new(EMC2301_I2C_ADDR, ProductId::Emc2301);
        // stall status = 0x01 -> bit 0 set -> fan 1 stalled
        expectations.transactions.push(I2cTransaction::write_read(
            EMC2301_I2C_ADDR,
            vec![FanStallStatus::ADDRESS],
            vec![0x01],
        ));
        let expectations = expectations.build();

        let i2c = I2cMock::new(&expectations);
        let mut dev = Emc230x::new(i2c, EMC2301_I2C_ADDR)
            .await
            .expect("Could not create device");

        let result = dev
            .fan_detected(FanSelect(1))
            .await
            .expect("fan_detected failed");
        assert!(!result);

        let mut i2c = dev.release();
        i2c.done();
    }

    #[tokio::test]
    async fn fan_detected_emc2305_fan5_stalled() {
        let mut expectations =
            Emc230xExpectationBuilder::new(EMC230X_I2C_ADDR_0, ProductId::Emc2305);
        // stall status = 0x10 -> bit 4 set -> fan 5 stalled
        expectations.transactions.push(I2cTransaction::write_read(
            EMC230X_I2C_ADDR_0,
            vec![FanStallStatus::ADDRESS],
            vec![0x10],
        ));
        let expectations = expectations.build();

        let i2c = I2cMock::new(&expectations);
        let mut dev = Emc230x::new(i2c, EMC230X_I2C_ADDR_0)
            .await
            .expect("Could not create device");

        let result = dev
            .fan_detected(FanSelect(5))
            .await
            .expect("fan_detected failed");
        assert!(!result);

        let mut i2c = dev.release();
        i2c.done();
    }

    #[tokio::test]
    async fn valid_fan() {
        let expectations = Emc230xExpectationBuilder::new(EMC2301_I2C_ADDR, ProductId::Emc2301);
        let expectations = expectations.build();
        let i2c = I2cMock::new(&expectations);
        let dev = Emc230x::new(i2c, EMC2301_I2C_ADDR)
            .await
            .expect("Could not create device");

        let result = dev.valid_fan(FanSelect(1));
        assert!(result.is_ok());

        let result = dev.valid_fan(FanSelect(0));
        assert!(result.is_err());

        let result = dev.valid_fan(FanSelect(6));
        assert!(result.is_err());

        let mut i2c = dev.release();
        i2c.done();
    }

    #[tokio::test]
    async fn mode_duty_cycle() {
        let duty = 75_u8;
        let mut expectations = Emc230xExpectationBuilder::new(EMC2301_I2C_ADDR, ProductId::Emc2301);

        let mut cfg = FanConfiguration1::default();
        cfg.set_rngx(fan_configuration1::Range::Rpm500);
        expectations.transactions.push(I2cTransaction::write_read(
            EMC2301_I2C_ADDR,
            vec![FanConfiguration1::fan_address(FanSelect(1)).expect("fan address")],
            vec![cfg.into()],
        ));

        let raw = FanDriveSetting::from_duty_cycle(duty);
        expectations.transactions.push(I2cTransaction::write_read(
            EMC2301_I2C_ADDR,
            vec![FanDriveSetting::fan_address(FanSelect(1)).expect("fan address")],
            vec![raw.into()],
        ));

        let expectations = expectations.build();
        let i2c = I2cMock::new(&expectations);
        let mut dev = Emc230x::new(i2c, EMC2301_I2C_ADDR)
            .await
            .expect("Could not create device");

        let result = dev.mode(FanSelect(1)).await.expect("Could not get mode");
        assert_eq!(result, FanControl::DutyCycle(duty));

        let mut i2c = dev.release();
        i2c.done();
    }

    #[tokio::test]
    async fn mode_rpm() {
        let target_rpm: u16 = 1000;
        let mut expectations = Emc230xExpectationBuilder::new(EMC2301_I2C_ADDR, ProductId::Emc2301);

        let mut cfg = FanConfiguration1::default();
        cfg.set_rngx(fan_configuration1::Range::Rpm500);
        cfg.set_enagx(true);
        expectations.transactions.push(I2cTransaction::write_read(
            EMC2301_I2C_ADDR,
            vec![FanConfiguration1::fan_address(FanSelect(1)).expect("fan address")],
            vec![cfg.into()],
        ));

        let raw_count = (_SIMPLIFIED_RPM_FACTOR * cfg.rngx().tach_count_multiplier() as f64
            / target_rpm as f64) as u16;
        let count = TachReading::from(raw_count);

        expectations.transactions.push(I2cTransaction::write_read(
            EMC2301_I2C_ADDR,
            vec![TachTargetLow::fan_address(FanSelect(1)).expect("fan address")],
            vec![count.raw_low()],
        ));
        expectations.transactions.push(I2cTransaction::write_read(
            EMC2301_I2C_ADDR,
            vec![TachTargetHigh::fan_address(FanSelect(1)).expect("fan address")],
            vec![count.raw_high()],
        ));

        expectations.transactions.push(I2cTransaction::write_read(
            EMC2301_I2C_ADDR,
            vec![FanConfiguration1::fan_address(FanSelect(1)).expect("fan address")],
            vec![cfg.into()],
        ));

        let expectations = expectations.build();
        let i2c = I2cMock::new(&expectations);
        let mut dev = Emc230x::new(i2c, EMC2301_I2C_ADDR)
            .await
            .expect("Could not create device");

        let result = dev.mode(FanSelect(1)).await.expect("Could not get mode");

        match result {
            FanControl::Rpm(rpm) => {
                let range = std::ops::Range {
                    start: target_rpm as f64 * 0.99,
                    end: target_rpm as f64 * 1.01,
                };
                assert!(
                    range.contains(&(rpm as f64)),
                    "RPM was out of expected range; Expected: {} in Range: {:?} Got: {}",
                    target_rpm,
                    range,
                    rpm
                );
            }
            other => panic!("Expected FanControl::Rpm, got {:?}", other),
        }

        let mut i2c = dev.release();
        i2c.done();
    }

    #[tokio::test]
    async fn resolve_fan_no_reverse() {
        let expectations =
            Emc230xExpectationBuilder::new(EMC230X_I2C_ADDR_5, ProductId::Emc2305).build();
        let i2c = I2cMock::new(&expectations);
        let dev = Emc230x::new(i2c, EMC230X_I2C_ADDR_5)
            .await
            .expect("Could not create device");

        // Without reverse, fan indices should pass through unchanged
        for fan in 1..=5_u16 {
            let resolved = dev.resolve_fan(FanSelect(fan)).expect("resolve_fan failed");
            assert_eq!(resolved.0, fan);
        }

        let mut i2c = dev.release();
        i2c.done();
    }

    #[tokio::test]
    async fn resolve_fan_reversed_emc2305() {
        let expectations =
            Emc230xExpectationBuilder::new(EMC230X_I2C_ADDR_5, ProductId::Emc2305).build();
        let i2c = I2cMock::new(&expectations);
        let mut dev = Emc230x::new(i2c, EMC230X_I2C_ADDR_5)
            .await
            .expect("Could not create device");

        dev.set_reverse_fan_index(true);
        assert!(dev.reverse_fan_index());

        // On EMC2305 (5 fans): 1->5, 2->4, 3->3, 4->2, 5->1
        assert_eq!(dev.resolve_fan(FanSelect(1)).unwrap().0, 5);
        assert_eq!(dev.resolve_fan(FanSelect(2)).unwrap().0, 4);
        assert_eq!(dev.resolve_fan(FanSelect(3)).unwrap().0, 3);
        assert_eq!(dev.resolve_fan(FanSelect(4)).unwrap().0, 2);
        assert_eq!(dev.resolve_fan(FanSelect(5)).unwrap().0, 1);

        let mut i2c = dev.release();
        i2c.done();
    }

    #[tokio::test]
    async fn resolve_fan_reversed_emc2302() {
        let expectations =
            Emc230xExpectationBuilder::new(EMC230X_I2C_ADDR_1, ProductId::Emc2302).build();
        let i2c = I2cMock::new(&expectations);
        let mut dev = Emc230x::new(i2c, EMC230X_I2C_ADDR_1)
            .await
            .expect("Could not create device");

        dev.set_reverse_fan_index(true);

        // On EMC2302 (2 fans): 1->2, 2->1
        assert_eq!(dev.resolve_fan(FanSelect(1)).unwrap().0, 2);
        assert_eq!(dev.resolve_fan(FanSelect(2)).unwrap().0, 1);

        // Fan 3 should be invalid on a 2-fan device
        assert!(dev.resolve_fan(FanSelect(3)).is_err());

        let mut i2c = dev.release();
        i2c.done();
    }

    #[tokio::test]
    async fn reversed_duty_cycle_emc2305() {
        // With reverse enabled, reading duty_cycle(FanSelect(1)) should hit hardware fan 5
        // (register at FAN5_BASE + offset = 0x70)
        let mut expectations =
            Emc230xExpectationBuilder::new(EMC230X_I2C_ADDR_4, ProductId::Emc2305);

        // The mock expects I2C reads at the *hardware* fan 5 address
        expectations.duty_cycle(FanSelect(5), 80);

        let expectations = expectations.build();
        let i2c = I2cMock::new(&expectations);
        let mut dev = Emc230x::new(i2c, EMC230X_I2C_ADDR_4)
            .await
            .expect("Could not create device");

        dev.set_reverse_fan_index(true);

        // User asks for fan 1, but it resolves to hardware fan 5
        let result = dev
            .duty_cycle(FanSelect(1))
            .await
            .expect("Could not get duty cycle");
        assert_eq!(80, result);

        let mut i2c = dev.release();
        i2c.done();
    }

    #[tokio::test]
    async fn reversed_fan_detected_emc2305() {
        // With reverse enabled, fan_detected(FanSelect(1)) on EMC2305 should check
        // hardware fan 5 (bit 4 of stall status register)
        let mut expectations =
            Emc230xExpectationBuilder::new(EMC230X_I2C_ADDR_4, ProductId::Emc2305);

        // Stall status = 0x10 -> bit 4 set -> hardware fan 5 stalled
        expectations.transactions.push(I2cTransaction::write_read(
            EMC230X_I2C_ADDR_4,
            vec![FanStallStatus::ADDRESS],
            vec![0x10],
        ));

        let expectations = expectations.build();
        let i2c = I2cMock::new(&expectations);
        let mut dev = Emc230x::new(i2c, EMC230X_I2C_ADDR_4)
            .await
            .expect("Could not create device");

        dev.set_reverse_fan_index(true);

        // User asks about fan 1 -> resolves to hardware fan 5 -> bit 4 is set -> stalled
        let result = dev
            .fan_detected(FanSelect(1))
            .await
            .expect("fan_detected failed");
        assert!(!result);

        let mut i2c = dev.release();
        i2c.done();
    }

    #[tokio::test]
    async fn reversed_fan_poles_emc2305() {
        let expectations =
            Emc230xExpectationBuilder::new(EMC230X_I2C_ADDR_4, ProductId::Emc2305).build();
        let i2c = I2cMock::new(&expectations);
        let mut dev = Emc230x::new(i2c, EMC230X_I2C_ADDR_4)
            .await
            .expect("Could not create device");

        dev.set_reverse_fan_index(true);

        // Set poles for logical fan 1 (resolves to hardware fan 5)
        dev.set_fan_poles(FanSelect(1), 4)
            .expect("set_fan_poles failed");

        // Reading logical fan 1 should return what we set
        assert_eq!(dev.fan_poles(FanSelect(1)).unwrap(), 4);

        // Logical fan 5 (resolves to hardware fan 1) should still have the default
        assert_eq!(dev.fan_poles(FanSelect(5)).unwrap(), 2);

        // Disable reverse and verify the value is at the correct hardware position
        dev.set_reverse_fan_index(false);

        // Hardware fan 5 should have poles=4 (what we set via logical fan 1)
        assert_eq!(dev.fan_poles(FanSelect(5)).unwrap(), 4);
        // Hardware fan 1 should have the default poles=2
        assert_eq!(dev.fan_poles(FanSelect(1)).unwrap(), 2);

        let mut i2c = dev.release();
        i2c.done();
    }
}
