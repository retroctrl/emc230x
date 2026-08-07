// Copyright (c) 2026 Jake Swensen
// SPDX-License-Identifier: MPL-2.0
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

/// Round an `f64` to the nearest integer of type `$ty`.
///
/// Workaround for `core` not providing `round()` in `no_std`.
///
/// Set to `$ty::MAX` or `$ty::MIN` if the value is out of range for the type.
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

fn round_to_in_range(value: f64, min: f64, max: f64) -> Option<f64> {
    if !value.is_finite() || value < min || value > max {
        None
    } else {
        Some(value)
    }
}

pub(crate) fn round_to_u8(value: f64) -> u8 {
    let value = round_to_in_range(value, u8::MIN as f64, u8::MAX as f64)
        .unwrap_or_else(|| value.clamp(u8::MIN as f64, u8::MAX as f64));

    round_to!(value, u8)
}

pub(crate) fn round_to_u16(value: f64) -> u16 {
    let value = round_to_in_range(value, u16::MIN as f64, u16::MAX as f64)
        .unwrap_or_else(|| value.clamp(u16::MIN as f64, u16::MAX as f64));

    round_to!(value, u16)
}

#[cfg(test)]
mod tests {
    #[test]
    fn round_to_u8_rounds_values() {
        assert_eq!(super::round_to_u8(12.4), 12);
        assert_eq!(super::round_to_u8(12.5), 13);
    }

    #[test]
    fn round_to_u8_clamps_to_bounds() {
        assert_eq!(super::round_to_u8(999.0), u8::MAX);
        assert_eq!(super::round_to_u8(-999.0), u8::MIN);
    }

    #[test]
    fn round_to_u16_clamps_to_bounds() {
        assert_eq!(super::round_to_u16(42.4), 42);
        assert_eq!(super::round_to_u16(42.5), 43);
        assert_eq!(super::round_to_u16(-1.0), u16::MIN);
        assert_eq!(super::round_to_u16(f64::INFINITY), u16::MAX);
    }
}
