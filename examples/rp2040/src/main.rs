#![no_std]
#![no_main]

use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_executor::Spawner;
use embassy_rp::{
    bind_interrupts,
    gpio::{Level, Output},
    i2c::{self, Async, I2c as EmbassyI2c, InterruptHandler as I2cInterruptHandler},
    peripherals::{PIN_25, USB},
    usb::{Driver, InterruptHandler as UsbInterruptHandler},
};
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, mutex::Mutex};
use embassy_time::Timer;
use emc230x::{Emc230x, FanControl, FanSelect};
use {defmt_rtt as _, panic_probe as _};

#[cfg(not(feature = "i2c1"))]
use embassy_rp::peripherals::I2C0;
#[cfg(feature = "i2c1")]
use embassy_rp::peripherals::I2C1;

#[cfg(not(feature = "i2c1"))]
bind_interrupts!(struct Irqs {
    I2C0_IRQ => I2cInterruptHandler<I2C0>;
    USBCTRL_IRQ => UsbInterruptHandler<USB>;
});

#[cfg(feature = "i2c1")]
bind_interrupts!(struct Irqs {
    I2C1_IRQ => I2cInterruptHandler<I2C1>;
    USBCTRL_IRQ => UsbInterruptHandler<USB>;
});

#[cfg(not(feature = "i2c1"))]
type I2cBus = Mutex<NoopRawMutex, EmbassyI2c<'static, I2C0, Async>>;
#[cfg(feature = "i2c1")]
type I2cBus = Mutex<NoopRawMutex, EmbassyI2c<'static, I2C1, Async>>;

#[cfg(not(feature = "i2c1"))]
type SharedDevice<'a> = Emc230x<I2cDevice<'a, NoopRawMutex, EmbassyI2c<'static, I2C0, Async>>>;
#[cfg(feature = "i2c1")]
type SharedDevice<'a> = Emc230x<I2cDevice<'a, NoopRawMutex, EmbassyI2c<'static, I2C1, Async>>>;

macro_rules! info {
    ($($arg:tt)*) => {{
        defmt::info!($($arg)*);
        log::info!($($arg)*);
    }};
}

macro_rules! error {
    ($($arg:tt)*) => {{
        defmt::error!($($arg)*);
        log::error!($($arg)*);
    }};
}

async fn unwrap_or_halt<T, E>(result: Result<T, E>, msg: &str) -> T {
    match result {
        Ok(val) => val,
        Err(_) => {
            error!("{}", msg);
            loop {
                Timer::after_secs(10).await;
            }
        }
    }
}

#[embassy_executor::task]
async fn heartbeat_task(led_pin: embassy_rp::Peri<'static, PIN_25>) {
    let mut led = Output::new(led_pin, Level::Low);
    loop {
        led.set_high();
        Timer::after_millis(50).await;
        led.set_low();
        Timer::after_millis(4950).await;
    }
}

#[embassy_executor::task]
async fn usb_logger_task(driver: Driver<'static, USB>) {
    embassy_usb_logger::run!(4096, log::LevelFilter::Info, driver);
}

async fn print_all_fans(devices: &mut [Option<SharedDevice<'_>>]) {
    for (i, dev) in devices.iter_mut().flatten().enumerate() {
        for fan in 1..=dev.count() {
            let detected = unwrap_or_halt(
                dev.fan_detected(FanSelect(fan)).await,
                "Failed to read fan detected",
            )
            .await;
            if !detected {
                info!("Device {}: Fan {}: not detected", i, fan);
                continue;
            }
            let duty =
                unwrap_or_halt(dev.duty_cycle(FanSelect(fan)).await, "Failed to read duty cycle")
                    .await;
            let rpm = unwrap_or_halt(dev.rpm(FanSelect(fan)).await, "Failed to read RPM").await;
            info!("Device {}: Fan {}: Duty Cycle: {}%; RPM: {}", i, fan, duty, rpm);
        }
    }
}

async fn set_all_fans(devices: &mut [Option<SharedDevice<'_>>], mode: FanControl) {
    for dev in devices.iter_mut().flatten() {
        for fan in 1..=dev.count() {
            unwrap_or_halt(dev.set_mode(FanSelect(fan), mode).await, "Failed to set fan mode")
                .await;
        }
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    spawner.spawn(heartbeat_task(p.PIN_25)).unwrap();

    let usb_driver = Driver::new(p.USB, Irqs);
    spawner.spawn(usb_logger_task(usb_driver)).unwrap();

    // After a UF2 boot the USB CDC device needs time to enumerate on the host
    // before log messages can be delivered. Without this delay all early log
    // messages are dropped on the USB transport (RTT is unaffected).
    Timer::after_millis(1500).await;

    info!("Starting EMC230x Fan Controller Example");

    #[cfg(not(feature = "i2c1"))]
    let i2c_raw = i2c::I2c::new_async(p.I2C0, p.PIN_1, p.PIN_0, Irqs, i2c::Config::default());
    #[cfg(feature = "i2c1")]
    let i2c_raw = i2c::I2c::new_async(p.I2C1, p.PIN_3, p.PIN_2, Irqs, i2c::Config::default());

    let i2c_bus: I2cBus = Mutex::new(i2c_raw);

    // Probe the bus for all attached EMC230x devices
    let found = {
        let mut probe_dev = I2cDevice::new(&i2c_bus);
        Emc230x::probe(&mut probe_dev).await
    };

    if found.is_empty() {
        error!("No EMC230x devices found on the I2C bus");
        loop {
            Timer::after_secs(10).await;
        }
    }

    info!("Found {} EMC230x device(s)", found.len());

    // Initialize all discovered devices
    let mut devices: [Option<SharedDevice>; 6] = [None, None, None, None, None, None];
    for (slot, addr) in found.iter().enumerate() {
        match Emc230x::new(I2cDevice::new(&i2c_bus), addr).await {
            Ok(dev) => {
                info!("Initialized EMC230x at address 0x{:02x}", addr);
                devices[slot] = Some(dev);
            }
            Err(_) => error!("Failed to initialize EMC230x at address {}", addr),
        }
    }

    info!("EMC230x Fan Controller Example");

    // The devices should start with fans set to 100% duty cycle
    print_all_fans(&mut devices).await;

    // Control fans via duty cycle
    info!("Setting all fans to 85% duty cycle");
    set_all_fans(&mut devices, FanControl::DutyCycle(85)).await;
    Timer::after_secs(5).await;
    print_all_fans(&mut devices).await;

    // Or control fans via RPM targeting
    // The device will attempt to get close to the target, but won't be exact
    info!("Setting all fans to 1000 RPM");
    set_all_fans(&mut devices, FanControl::Rpm(1000)).await;
    Timer::after_secs(10).await;
    print_all_fans(&mut devices).await;

    // Switch back to duty cycle mode at any point
    info!("Setting all fans to 35% duty cycle");
    set_all_fans(&mut devices, FanControl::DutyCycle(35)).await;
    Timer::after_secs(10).await;
    print_all_fans(&mut devices).await;

    // A simple loop to demonstrate sweeping the duty cycle
    let mut target_duty_cycle = 20_u8;
    loop {
        Timer::after_secs(2).await;
        if target_duty_cycle < 100 {
            target_duty_cycle += 1;
        } else {
            target_duty_cycle = 20;
        }
        info!("Setting all fans to {}% duty cycle", target_duty_cycle);
        set_all_fans(&mut devices, FanControl::DutyCycle(target_duty_cycle)).await;
        print_all_fans(&mut devices).await;
    }
}
