#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_rp::{
    bind_interrupts,
    i2c::{self, Async, I2c as EmbassyI2c, InterruptHandler},
    peripherals::I2C0,
};
use embassy_time::Timer;
use emc230x::{Emc230x, EMC2301_I2C_ADDR};
use fans::FanSelect;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    I2C0_IRQ => InterruptHandler<I2C0>;
});

async fn print_fan_info(dev: &mut Emc230x<EmbassyI2c<'_, I2C0, Async>>) {
    let fan_select = FanSelect(1);
    let duty_cycle = dev.duty_cycle(fan_select).await.unwrap();
    let rpm = dev.rpm(fan_select).await.unwrap();
    defmt::info!("Fan 1: Duty Cycle: {}%; RPM: {}", duty_cycle, rpm);
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let sda = p.PIN_0;
    let scl = p.PIN_1;

    let i2c = i2c::I2c::new_async(p.I2C0, scl, sda, Irqs, i2c::Config::default());
    let mut emc230x: Emc230x<EmbassyI2c<'_, I2C0, Async>> =
        Emc230x::new(i2c, EMC2301_I2C_ADDR).await.unwrap();

    defmt::info!("EMC2301 Fan Controller Example");

    // The device should start with the fans set to 100% duty cycle
    print_fan_info(&mut emc230x).await;

    // Control the fan via setting the duty cycle
    defmt::info!("Setting Fan 1 to 85% duty cycle");
    emc230x
        .set_mode(emc230x::FanSelect(1), emc230x::FanControl::DutyCycle(85))
        .await
        .unwrap();
    Timer::after_secs(5).await;
    print_fan_info(&mut emc230x).await;

    // Or control the fan via setting a RPM to target
    // The device will attempt to get close to the target, but won't be exact
    defmt::info!("Setting Fan 1 to 1000 RPM");
    emc230x
        .set_mode(emc230x::FanSelect(1), emc230x::FanControl::Rpm(1000))
        .await
        .unwrap();
    Timer::after_secs(10).await;
    print_fan_info(&mut emc230x).await;

    // You can switch back to duty cycle mode at any point
    defmt::info!("Setting Fan 1 to 35% duty cycle");
    emc230x
        .set_mode(emc230x::FanSelect(1), emc230x::FanControl::DutyCycle(35))
        .await
        .unwrap();
    Timer::after_secs(10).await;
    print_fan_info(&mut emc230x).await;

    // A simple example loop to demonstrate changing the duty cycle
    let mut target_duty_cycle = 20_u8;
    loop {
        Timer::after_secs(2).await;
        if target_duty_cycle < 100 {
            target_duty_cycle += 1;
        } else {
            target_duty_cycle = 20;
        }
        defmt::info!("Setting Fan 1 to {}% duty cycle", target_duty_cycle);
        emc230x
            .set_mode(emc230x::FanSelect(1), emc230x::FanControl::DutyCycle(target_duty_cycle))
            .await
            .unwrap();
        print_fan_info(&mut emc230x).await;
    }
}
