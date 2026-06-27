//! Register map and SPI command constants for the ADXL362.
//!
//! All addresses and reset values are taken from the ADXL362 Data Sheet, Rev. G (May 2023).

/// SPI command byte: write register(s).
pub(crate) const CMD_WRITE: u8 = 0x0A;
/// SPI command byte: read register(s).
pub(crate) const CMD_READ: u8 = 0x0B;
/// SPI command byte: read FIFO (no address follows).
pub(crate) const CMD_READ_FIFO: u8 = 0x0D;

/// Value to write to `SOFT_RESET` to trigger a device reset.
pub(crate) const SOFT_RESET_KEY: u8 = 0x52;

/// Expected value of `DEVID_AD` (Analog Devices vendor ID).
pub(crate) const DEVID_AD_VALUE: u8 = 0xAD;
/// Expected value of `DEVID_MST` (MEMS vendor ID). Checked at init if desired.
#[allow(dead_code)]
pub(crate) const DEVID_MST_VALUE: u8 = 0x1D;
/// Expected value of `PARTID` (part number, 362 octal).
pub(crate) const PARTID_VALUE: u8 = 0xF2;

/// ADXL362 register addresses.
#[repr(u8)]
#[allow(dead_code)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Register {
    /// Analog Devices vendor ID (R, reset 0xAD).
    DevIdAd = 0x00,
    /// MEMS vendor ID (R, reset 0x1D).
    DevIdMst = 0x01,
    /// Part ID (R, reset 0xF2).
    PartId = 0x02,
    /// Silicon revision (R, reset 0x01).
    RevId = 0x03,
    /// 8-bit X-axis data, MSBs only (R).
    XData = 0x08,
    /// 8-bit Y-axis data, MSBs only (R).
    YData = 0x09,
    /// 8-bit Z-axis data, MSBs only (R).
    ZData = 0x0A,
    /// Status register (R, reset ~0xC0 on power-up).
    Status = 0x0B,
    /// FIFO entry count, low byte (R).
    FifoEntriesL = 0x0C,
    /// FIFO entry count, high byte (R).
    FifoEntriesH = 0x0D,
    /// X-axis low byte, 12-bit sign-extended (R). Burst from here reads X/Y/Z in one shot.
    XDataL = 0x0E,
    XDataH = 0x0F,
    YDataL = 0x10,
    YDataH = 0x11,
    ZDataL = 0x12,
    ZDataH = 0x13,
    /// Temperature low byte, 12-bit sign-extended (R).
    TempL = 0x14,
    TempH = 0x15,
    /// Write 0x52 to reset; should only be written while in Standby (W).
    SoftReset = 0x1F,
    /// Activity threshold low byte, 11-bit unsigned (RW).
    ThreshActL = 0x20,
    ThreshActH = 0x21,
    /// Activity time in samples, 8-bit (RW).
    TimeAct = 0x22,
    /// Inactivity threshold low byte, 11-bit unsigned (RW).
    ThreshInactL = 0x23,
    ThreshInactH = 0x24,
    /// Inactivity time in samples, 16-bit (RW).
    TimeInactL = 0x25,
    TimeInactH = 0x26,
    /// Activity/inactivity control (RW, reset 0x00).
    ActInactCtl = 0x27,
    /// FIFO control (RW, reset 0x00).
    FifoControl = 0x28,
    /// FIFO watermark sample count, lower 8 bits (RW, reset 0x80).
    FifoSamples = 0x29,
    /// INT1 source map (RW, reset 0x00).
    IntMap1 = 0x2A,
    /// INT2 source map (RW, reset 0x00).
    IntMap2 = 0x2B,
    /// Filter / output data rate control (RW, reset 0x13).
    FilterCtl = 0x2C,
    /// Power control (RW, reset 0x00).
    PowerCtl = 0x2D,
    /// Self-test (RW, reset 0x00).
    SelfTest = 0x2E,
}
