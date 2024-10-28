use super::RegisterAddress;
use emc230x_macros::RegisterAddress;

bitfield::bitfield! {
    #[derive(Clone, Copy, Default, RegisterAddress)]
    #[register(address = 0x2B)]
    pub struct PwmOutputConfig(u8);
    impl Debug;

    /// Fan 5 PWM Output Type
    ///
    /// 0: Open drain output.
    ///
    /// 1: Push-pull output.
    pub pmot5, set_pmot5: 4;

    /// Fan 4 PWM Output Type
    ///
    /// 0: Open drain output.
    ///
    /// 1: Push-pull output.
    pub pmot4, set_pmot4: 3;

    /// Fan 3 PWM Output Type
    ///
    /// 0: Open drain output.
    ///
    /// 1: Push-pull output.
    pub pmot3, set_pmot3: 2;

    /// Fan 2 PWM Output Type
    ///
    /// 0: Open drain output.
    ///
    /// 1: Push-pull output.
    pub pmot2, set_pmot2: 1;

    /// Fan 1 PWM Output Type
    ///
    /// 0: Open drain output.
    ///
    /// 1: Push-pull output.
    pub pmot1, set_pmot1: 0;
}

impl PwmOutputConfig {
    pub fn open_drain(&mut self, sel: u8) {
        match sel {
            1 => self.set_pmot1(false),
            2 => self.set_pmot2(false),
            3 => self.set_pmot3(false),
            4 => self.set_pmot4(false),
            5 => self.set_pmot5(false),
            _ => {}
        }
    }

    pub fn push_pull(&mut self, sel: u8) {
        match sel {
            1 => self.set_pmot1(true),
            2 => self.set_pmot2(true),
            3 => self.set_pmot3(true),
            4 => self.set_pmot4(true),
            5 => self.set_pmot5(true),
            _ => {}
        }
    }
}