//! FILTER_CTL register types: measurement range, output data rate, filter bandwidth.

/// Measurement range (FILTER_CTL bits [7:6]).
///
/// Scale factors are from Rev. G Table 1.  The ±8 g row is **235 LSB/g**, not the
/// intuitively expected 250 — using 250 would introduce ~6 % error.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Range {
    /// ±2 g — 1000 LSB/g, 1 mg/LSB.
    G2 = 0b00,
    /// ±4 g — 500 LSB/g, 2 mg/LSB.
    G4 = 0b01,
    /// ±8 g — 235 LSB/g, 4.255 mg/LSB.
    G8 = 0b10,
}

impl Range {
    /// Scale factor in LSB/g (used for threshold ↔ code conversions).
    #[inline]
    pub fn lsb_per_g(self) -> u32 {
        match self {
            Range::G2 => 1000,
            Range::G4 => 500,
            Range::G8 => 235,
        }
    }

    /// Convert a raw 12-bit signed sample to integer milli-g (no FPU required).
    ///
    /// Resolution: 1 mg (G2), 2 mg (G4), ~4 mg (G8 — truncated from 4.255 mg/LSB).
    #[inline]
    pub fn raw_to_mg(self, raw: i16) -> i32 {
        match self {
            Range::G2 => raw as i32,
            Range::G4 => raw as i32 * 2,
            // 4.255 mg/LSB; use i32 intermediate to avoid overflow.
            Range::G8 => (raw as i32 * 4255) / 1000,
        }
    }

    /// Convert milli-g to the nearest threshold code for this range.
    #[inline]
    pub fn mg_to_threshold_code(self, mg: u32) -> u16 {
        let code = (mg * self.lsb_per_g()) / 1000;
        // threshold registers are 11-bit unsigned
        code.min(0x7FF) as u16
    }

    /// `f32` conversion, gated behind the `float` feature.
    #[cfg(feature = "float")]
    #[inline]
    pub fn raw_to_g(self, raw: i16) -> f32 {
        raw as f32 / self.lsb_per_g() as f32
    }
}

/// Output data rate (FILTER_CTL bits [2:0]).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum OutputDataRate {
    /// 12.5 Hz.
    Hz12_5 = 0b000,
    /// 25 Hz.
    Hz25 = 0b001,
    /// 50 Hz.
    Hz50 = 0b010,
    /// 100 Hz.
    Hz100 = 0b011,
    /// 200 Hz.
    Hz200 = 0b100,
    /// 400 Hz.
    Hz400 = 0b101,
}

impl OutputDataRate {
    /// Returns the ODR in units of mHz (milli-Hertz) to allow integer time math.
    #[inline]
    pub fn mhz(self) -> u32 {
        match self {
            OutputDataRate::Hz12_5 => 12_500,
            OutputDataRate::Hz25 => 25_000,
            OutputDataRate::Hz50 => 50_000,
            OutputDataRate::Hz100 => 100_000,
            OutputDataRate::Hz200 => 200_000,
            OutputDataRate::Hz400 => 400_000,
        }
    }

    /// Convert a sample count to milliseconds at this ODR (rounded up).
    #[inline]
    pub fn samples_to_ms(self, samples: u32) -> u32 {
        // samples * 1000 ms/s / (mhz / 1000 Hz/mHz) = samples * 1_000_000 / mhz
        samples * 1_000_000 / self.mhz()
    }

    /// Convert a duration in milliseconds to the nearest sample count.
    #[inline]
    pub fn ms_to_samples(self, ms: u32) -> u32 {
        (ms * self.mhz()) / 1_000_000
    }
}

/// Anti-aliasing filter bandwidth (FILTER_CTL bit 4, `HALF_BW`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FilterBandwidth {
    /// Bandwidth = ODR/2 (`HALF_BW` = 0, default).
    Half,
    /// Bandwidth = ODR/4 (`HALF_BW` = 1).
    Quarter,
}

impl FilterBandwidth {
    #[inline]
    pub(crate) fn half_bw_bit(self) -> bool {
        matches!(self, FilterBandwidth::Quarter)
    }
}
