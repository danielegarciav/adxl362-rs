//! Interrupt mapping and STATUS register types.

/// Selects which hardware interrupt pin to configure.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum IntPin {
    /// Hardware interrupt pin 1 (also usable as external clock input).
    Int1,
    /// Hardware interrupt pin 2 (also usable as sync-sampling trigger).
    Int2,
}

/// Interrupt source map for INTMAP1 / INTMAP2.
///
/// Each boolean enables routing the corresponding event to the selected pin.
/// Multiple sources are OR-combined by the device.
#[derive(Clone, Copy, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct IntSources {
    /// Route DATA_READY to the pin.
    pub data_ready: bool,
    /// Route FIFO_READY to the pin.
    pub fifo_ready: bool,
    /// Route FIFO_WATERMARK to the pin.
    pub fifo_watermark: bool,
    /// Route FIFO_OVERRUN to the pin.
    pub fifo_overrun: bool,
    /// Route ACT (activity detected) to the pin.
    pub act: bool,
    /// Route INACT (inactivity detected) to the pin.
    pub inact: bool,
    /// Route AWAKE to the pin (valid only in linked/loop mode).
    pub awake: bool,
    /// When `true` the pin asserts low (active-low polarity).
    pub active_low: bool,
}

impl IntSources {
    /// Encode to the INTMAP register byte.
    #[inline]
    pub(crate) fn to_register_byte(self) -> u8 {
        (self.data_ready as u8)
            | ((self.fifo_ready as u8) << 1)
            | ((self.fifo_watermark as u8) << 2)
            | ((self.fifo_overrun as u8) << 3)
            | ((self.act as u8) << 4)
            | ((self.inact as u8) << 5)
            | ((self.awake as u8) << 6)
            | ((self.active_low as u8) << 7)
    }
}

/// Parsed STATUS register (0x0B).
///
/// # Clearing semantics
/// - `act` / `inact` clear on STATUS read.
/// - `data_ready` clears after reading the data registers.
/// - `fifo_ready` / `fifo_watermark` / `fifo_overrun` clear when enough FIFO entries are read.
/// - `awake` is live only in linked/loop mode; ignore it otherwise (see design doc §5).
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Status {
    /// Data is ready to read from the data registers.
    pub data_ready: bool,
    /// At least one valid FIFO sample is available.
    pub fifo_ready: bool,
    /// FIFO has reached (or exceeded) the watermark level.
    pub fifo_watermark: bool,
    /// FIFO has overflowed; oldest samples were discarded.
    pub fifo_overrun: bool,
    /// Activity event detected (cleared by reading STATUS).
    pub act: bool,
    /// Inactivity event detected (cleared by reading STATUS).
    pub inact: bool,
    /// Device is awake (valid only in linked/loop mode; ignore otherwise).
    pub awake: bool,
    /// User-register error flag; high on power-up until the first write.
    pub err_user_regs: bool,
}

impl Status {
    /// Parse from a raw STATUS byte.
    #[inline]
    pub fn from_byte(byte: u8) -> Self {
        Status {
            data_ready: byte & 0x01 != 0,
            fifo_ready: byte & 0x02 != 0,
            fifo_watermark: byte & 0x04 != 0,
            fifo_overrun: byte & 0x08 != 0,
            act: byte & 0x10 != 0,
            inact: byte & 0x20 != 0,
            awake: byte & 0x40 != 0,
            err_user_regs: byte & 0x80 != 0,
        }
    }
}
