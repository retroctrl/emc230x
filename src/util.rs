// Copyright (c) 2026 Jake Swensen
// SPDX-License-Identifier: MPL-2.0
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

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
