// Copyright (c) 2024 Jake Swensen
// SPDX-License-Identifier: MPL-2.0
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

use bitfield::bitfield;

use super::RegisterOffset;
use crate::round_to;
use emc230x_macros::RegisterOffset;

bitfield! {
    #[derive(Clone, Copy, RegisterOffset)]
    #[register(offset = 0x08, default = 0x66)]
    pub struct FanMinimumDrive(u8);
    impl Debug;

    /// Minimum Drive
    ///
    /// The minimum PWM duty cycle that the device will output to the fan.
    pub min_drive, set_min_drive: 7, 0;
}

impl FanMinimumDrive {
    pub fn duty_cycle(&self) -> u8 {
        let duty = (self.0 as f64 / 255.0) * 100.0;
        round_to!(duty, u8)
    }

    pub fn from_duty_cycle(duty: u8) -> Self {
        let raw = (duty as f64 / 100.0) * 255.0;
        let raw = round_to!(raw, u8);
        FanMinimumDrive(raw)
    }
}
