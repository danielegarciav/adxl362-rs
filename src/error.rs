//! Error types for the ADXL362 driver.

/// Driver error type, generic over the SPI bus error.
#[derive(Debug)]
pub enum Error<E> {
    /// SPI bus error.
    Spi(E),
    /// The device did not respond with the expected identity registers.
    InvalidDevice {
        /// The PARTID value actually read from the device (expected `0xF2`).
        found_part_id: u8,
    },
    /// A FIFO read was requested with an odd buffer length (samples are 16-bit).
    FifoLengthOdd,
}

impl<E: core::fmt::Debug> core::fmt::Display for Error<E> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Error::Spi(e) => write!(f, "SPI error: {:?}", e),
            Error::InvalidDevice { found_part_id } => {
                write!(
                    f,
                    "Invalid device: expected PARTID=0xF2, found {:#04x}",
                    found_part_id
                )
            }
            Error::FifoLengthOdd => write!(f, "FIFO read buffer must have even length"),
        }
    }
}
