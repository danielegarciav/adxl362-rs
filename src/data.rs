//! Acceleration and temperature data types.

use crate::config::filter::Range;

/// A raw 3-axis acceleration sample (12-bit sign-extended values).
///
/// Burst-read of registers 0x0E–0x13; the device already sign-extends each
/// axis value into the high register byte, so `i16::from_le_bytes([lo, hi])`
/// gives the signed value directly, in the range –2048..=2047.
#[derive(Clone, Copy, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct RawAccel {
    /// Raw X-axis value (12-bit, sign-extended to i16).
    pub x: i16,
    /// Raw Y-axis value (12-bit, sign-extended to i16).
    pub y: i16,
    /// Raw Z-axis value (12-bit, sign-extended to i16).
    pub z: i16,
}

impl RawAccel {
    /// Construct from a 6-byte little-endian burst read of registers 0x0E–0x13.
    pub fn from_bytes(buf: &[u8; 6]) -> Self {
        RawAccel {
            x: i16::from_le_bytes([buf[0], buf[1]]),
            y: i16::from_le_bytes([buf[2], buf[3]]),
            z: i16::from_le_bytes([buf[4], buf[5]]),
        }
    }

    /// Convert to milli-g using the driver's current range setting.
    #[inline]
    pub fn to_mg(self, range: Range) -> AccelMg {
        AccelMg {
            x: range.raw_to_mg(self.x),
            y: range.raw_to_mg(self.y),
            z: range.raw_to_mg(self.z),
        }
    }

    /// Convert to `f32` g values.  Requires the `float` feature.
    #[cfg(feature = "float")]
    #[inline]
    pub fn to_g(self, range: Range) -> AccelG {
        AccelG {
            x: range.raw_to_g(self.x),
            y: range.raw_to_g(self.y),
            z: range.raw_to_g(self.z),
        }
    }
}

/// 3-axis acceleration in integer milli-g.
#[derive(Clone, Copy, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AccelMg {
    /// X-axis acceleration in milli-g.
    pub x: i32,
    /// Y-axis acceleration in milli-g.
    pub y: i32,
    /// Z-axis acceleration in milli-g.
    pub z: i32,
}

/// 3-axis acceleration in `f32` g.  Requires the `float` feature.
/// 3-axis acceleration in `f32` g.  Requires the `float` feature.
#[cfg(feature = "float")]
#[derive(Clone, Copy, Debug, Default)]
pub struct AccelG {
    /// X-axis acceleration in g.
    pub x: f32,
    /// Y-axis acceleration in g.
    pub y: f32,
    /// Z-axis acceleration in g.
    pub z: f32,
}

/// 8-bit (MSB-only) 3-axis sample from registers 0x08–0x0A.
///
/// Each byte is the 8 MSBs of the 12-bit ADC value (`≈ raw >> 4`).
/// Useful for low-power, coarse-motion polling where saving one SPI byte
/// per axis matters.
#[derive(Clone, Copy, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct RawAccel8 {
    /// X-axis MSB (8 most significant bits of the 12-bit ADC value).
    pub x: i8,
    /// Y-axis MSB.
    pub y: i8,
    /// Z-axis MSB.
    pub z: i8,
}

/// Temperature data and conversion helpers.
///
/// # Accuracy
/// Rev. G specifies 0.065 °C/LSB and a nominal 25 °C bias of ~350 LSB, but the
/// bias has a large part-to-part spread (σ ≈ 290 LSB ≈ 19 °C).  Absolute
/// temperature is only meaningful after per-device calibration at a known
/// temperature.
pub struct Temperature {
    /// Raw 12-bit sign-extended ADC value.
    pub raw: i16,
}

impl Temperature {
    /// Convert to °C using a caller-supplied bias (default 350 LSB from Rev. G).
    ///
    /// `T(°C) ≈ 25 + (raw − bias) × 0.065`
    #[cfg(feature = "float")]
    #[inline]
    pub fn to_celsius(&self, bias: i16) -> f32 {
        25.0 + (self.raw - bias) as f32 * 0.065
    }

    /// Integer approximation: returns temperature in centi-°C (hundredths of a
    /// degree) without an FPU.  Accuracy matches the raw-integer path.
    #[inline]
    pub fn to_centi_celsius(&self, bias: i16) -> i32 {
        // 0.065 °C/LSB = 65 centi-°C / 1000 LSB
        let delta_lsb = (self.raw - bias) as i32;
        2500 + (delta_lsb * 65) / 1000
    }
}
