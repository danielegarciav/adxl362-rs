//! POWER_CTL register types.

/// Noise control mode (POWER_CTL bits [5:4], `LOW_NOISE[1:0]`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum NoiseMode {
    /// Standard noise, lowest current consumption.
    Normal = 0b00,
    /// Reduced noise (~2× current vs. Normal).
    LowNoise = 0b01,
    /// Lowest noise (~4× current vs. Normal).
    UltraLowNoise = 0b10,
}

/// POWER_CTL `MEASURE[1:0]` field encoding (bits [1:0]).
///
/// The device powers up in `Standby`.  Wake-up mode shares the same MEASURE=10
/// bit-pattern as Measurement but additionally sets the WAKEUP bit (bit 3).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub(crate) enum MeasureBits {
    /// All sensing off (~10 nA).
    Standby = 0b00,
    /// Continuous measurement (or wake-up when WAKEUP bit is set).
    Measurement = 0b10,
}
