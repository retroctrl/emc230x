use thiserror::Error;

#[derive(Debug, Error)]
pub enum Error {
    #[error("I2C bus error")]
    I2c,

    #[error("Invalid device identifier")]
    InvalidDeviceId,

    #[error("Invalid fan number")]
    InvalidFan,

    #[error("Failed to convert register value to specific type")]
    RegisterTypeConversion,
}

impl defmt::Format for Error {
    fn format(&self, f: defmt::Formatter) {
        match self {
            Error::I2c => defmt::write!(f, "I2c"),
            Error::InvalidDeviceId => defmt::write!(f, "InvalidDeviceId"),
            Error::InvalidFan => defmt::write!(f, "InvalidFan"),
            Error::RegisterTypeConversion => defmt::write!(f, "RegisterTypeConversion"),
        }
    }
}
