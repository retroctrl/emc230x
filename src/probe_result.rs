// Copyright (c) 2026 Jake Swensen
// SPDX-License-Identifier: MPL-2.0
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

use crate::error::Error;
use crate::EMC230X_ADDRESSES;

/// The set of I2C addresses at which EMC230x devices were discovered during a bus probe.
///
/// Returned by [`crate::Emc230x::probe`]. Iterating yields only addresses where a device was found.
#[derive(Copy, Clone, Debug, Default)]
pub struct ProbeResult(pub(crate) [bool; 6]);

impl ProbeResult {
    /// Returns an iterator over found device addresses.
    pub fn iter(&self) -> impl Iterator<Item = u8> + '_ {
        EMC230X_ADDRESSES
            .iter()
            .zip(self.0.iter())
            .filter_map(|(&addr, &found)| found.then_some(addr))
    }

    /// Returns `true` if no devices were found.
    pub fn is_empty(&self) -> bool {
        self.0.iter().all(|&x| !x)
    }

    /// Returns the number of devices found.
    pub fn len(&self) -> usize {
        self.0.iter().filter(|&&x| x).count()
    }

    /// Returns whether a device was found at the given address.
    ///
    /// Returns an error if the address is not a valid EMC230x I2C address.
    pub fn contains(&self, address: u8) -> Result<bool, Error> {
        let index = EMC230X_ADDRESSES
            .iter()
            .position(|&a| a == address)
            .ok_or(Error::InvalidI2cAddress)?;
        Ok(self.0[index])
    }
}

impl IntoIterator for ProbeResult {
    type Item = u8;
    type IntoIter = core::iter::FilterMap<
        core::iter::Zip<core::array::IntoIter<u8, 6>, core::array::IntoIter<bool, 6>>,
        fn((u8, bool)) -> Option<u8>,
    >;

    fn into_iter(self) -> Self::IntoIter {
        EMC230X_ADDRESSES
            .into_iter()
            .zip(self.0)
            .filter_map((|(addr, found)| found.then_some(addr)) as fn((u8, bool)) -> Option<u8>)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::*;

    #[test]
    fn contains_found_address() {
        let result = ProbeResult([false, false, false, true, false, false]);
        assert_eq!(result.contains(EMC230X_I2C_ADDR_3).unwrap(), true);
    }

    #[test]
    fn contains_not_found_address() {
        let result = ProbeResult([false, false, false, true, false, false]);
        assert!(!result.contains(EMC230X_I2C_ADDR_0).unwrap());
    }

    #[test]
    fn contains_invalid_address() {
        let result = ProbeResult::default();
        assert!(result.contains(0xFF).is_err());
    }
}
