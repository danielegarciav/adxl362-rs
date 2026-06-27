//! Activity / inactivity detection configuration types.
//!
//! # LINK/LOOP encoding — Rev. G verified
//!
//! `ACT_INACT_CTL[5:4]` from Table 13 (Rev. G p.31):
//! - `0b00` → Default (also `0b10`, since the field is listed as `X0`)
//! - `0b01` → Linked
//! - `0b11` → Loop
//!
//! The `0b10` case silently behaves as Default, **not** Linked or Loop.
//! This is a common porting mistake from pre-production datasheets.

/// Link/Loop mode for autonomous activity↔inactivity sequencing.
///
/// Both `ACT_EN` and `INACT_EN` must be set for Linked or Loop to engage.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum LinkLoopMode {
    /// Independent (default) — activity and inactivity fire independently.
    Default = 0b00,
    /// Linked — activity arms after inactivity fires, and vice-versa.
    Linked = 0b01,
    /// Loop — continuously alternates: inactivity → autosleep → activity → wake.
    Loop = 0b11,
}

/// Configuration for the activity detector.
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ActivityConfig {
    /// Detection threshold in raw codes (11-bit, 0..=2047).
    ///
    /// Use `Range::mg_to_threshold_code` to compute from milli-g.
    pub threshold_codes: u16,
    /// Number of consecutive samples above threshold before activity is declared.
    pub time_samples: u8,
    /// `true` → referenced mode (delta from a stored baseline);
    /// `false` → absolute mode (compared directly against threshold).
    pub referenced: bool,
}

impl ActivityConfig {
    /// Helper: referenced mode.
    pub fn referenced(threshold_codes: u16, time_samples: u8) -> Self {
        Self {
            threshold_codes,
            time_samples,
            referenced: true,
        }
    }
    /// Helper: absolute mode.
    pub fn absolute(threshold_codes: u16, time_samples: u8) -> Self {
        Self {
            threshold_codes,
            time_samples,
            referenced: false,
        }
    }
}

/// Configuration for the inactivity (no-motion) detector.
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct InactivityConfig {
    /// Detection threshold in raw codes (11-bit, 0..=2047).
    pub threshold_codes: u16,
    /// Number of consecutive samples below threshold before inactivity is declared (16-bit).
    pub time_samples: u16,
    /// `true` → referenced mode; `false` → absolute mode.
    pub referenced: bool,
}

impl InactivityConfig {
    /// Helper: referenced mode.
    pub fn referenced(threshold_codes: u16, time_samples: u16) -> Self {
        Self {
            threshold_codes,
            time_samples,
            referenced: true,
        }
    }
    /// Helper: absolute mode.
    pub fn absolute(threshold_codes: u16, time_samples: u16) -> Self {
        Self {
            threshold_codes,
            time_samples,
            referenced: false,
        }
    }
}
