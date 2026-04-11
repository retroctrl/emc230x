// Copyright (c) 2024 Jake Swensen
// SPDX-License-Identifier: MPL-2.0
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

use thiserror::Error;

#[derive(Clone, Copy, Debug, Error)]
pub enum Error {
    #[error("I2C bus error")]
    I2c,

    #[error("Invalid device identifier")]
    InvalidDeviceId,

    #[error("Invalid manufacturer identifier")]
    InvalidManufacturerId,

    #[error("Invalid fan number")]
    InvalidFan,

    #[error("Invalid I2C address")]
    InvalidI2cAddress,

    #[error("Failed to convert register value to specific type")]
    RegisterTypeConversion,
}

impl defmt::Format for Error {
    fn format(&self, f: defmt::Formatter) {
        match self {
            Error::I2c => defmt::write!(f, "I2c"),
            Error::InvalidDeviceId => defmt::write!(f, "InvalidDeviceId"),
            Error::InvalidManufacturerId => defmt::write!(f, "InvalidManufacturerId"),
            Error::InvalidFan => defmt::write!(f, "InvalidFan"),
            Error::InvalidI2cAddress => defmt::write!(f, "InvalidI2cAddress"),
            Error::RegisterTypeConversion => defmt::write!(f, "RegisterTypeConversion"),
        }
    }
}
