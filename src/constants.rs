// Copyright (c) 2026 Jake Swensen
// SPDX-License-Identifier: MPL-2.0
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

/// Default I2C address for the EMC2301 device
pub const EMC2301_I2C_ADDR: u8 = 0b0010_1111;

// I2C addresses for the EMC230x family, selected by the ADDR resistor configuration.
pub const EMC230X_I2C_ADDR_0: u8 = 0x2C;
pub const EMC230X_I2C_ADDR_1: u8 = 0x2D;
pub const EMC230X_I2C_ADDR_2: u8 = 0x2E;
pub const EMC230X_I2C_ADDR_3: u8 = 0x2F;
pub const EMC230X_I2C_ADDR_4: u8 = 0x4C;
pub const EMC230X_I2C_ADDR_5: u8 = 0x4D;

pub(crate) const EMC230X_ADDRESSES: [u8; 6] = [
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
pub(crate) const _SIMPLIFIED_RPM_FACTOR: f64 = 3_932_160.0;
