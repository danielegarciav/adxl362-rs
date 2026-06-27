//! Low-level SPI transport primitives.
//!
//! All bus I/O goes through these four functions; nothing above this layer
//! touches the SPI device directly.  This keeps the transport seam clean for a
//! future migration to `device-driver` (see design doc §10).

use embedded_hal::spi::Operation;

// `SpiDevice` is the only thing that differs between sync and async surfaces.
// `Operation` always comes from embedded_hal (the async trait reuses the same type).
#[cfg(feature = "blocking")]
use embedded_hal::spi::SpiDevice;
#[cfg(not(feature = "blocking"))]
use embedded_hal_async::spi::SpiDevice;

use crate::error::Error;
use crate::register::{CMD_READ, CMD_READ_FIFO, CMD_WRITE, Register};

/// Read a single register.
#[maybe_async::maybe_async]
pub(crate) async fn read_register<SPI>(
    spi: &mut SPI,
    reg: Register,
) -> Result<u8, Error<SPI::Error>>
where
    SPI: SpiDevice,
{
    let mut buf = [0u8; 1];
    spi.transaction(&mut [
        Operation::Write(&[CMD_READ, reg as u8]),
        Operation::Read(&mut buf),
    ])
    .await
    .map_err(Error::Spi)?;
    Ok(buf[0])
}

/// Burst-read consecutive registers starting at `start` into `buf`.
///
/// The address pointer halts at 0x3F (no wrap-around).
#[maybe_async::maybe_async]
pub(crate) async fn read_registers<SPI>(
    spi: &mut SPI,
    start: Register,
    buf: &mut [u8],
) -> Result<(), Error<SPI::Error>>
where
    SPI: SpiDevice,
{
    spi.transaction(&mut [
        Operation::Write(&[CMD_READ, start as u8]),
        Operation::Read(buf),
    ])
    .await
    .map_err(Error::Spi)
}

/// Write a single register.
#[maybe_async::maybe_async]
pub(crate) async fn write_register<SPI>(
    spi: &mut SPI,
    reg: Register,
    val: u8,
) -> Result<(), Error<SPI::Error>>
where
    SPI: SpiDevice,
{
    spi.write(&[CMD_WRITE, reg as u8, val])
        .await
        .map_err(Error::Spi)
}

/// Read `buf.len()` bytes from the FIFO. `buf.len()` must be even (samples are 16-bit).
#[maybe_async::maybe_async]
pub(crate) async fn read_fifo<SPI>(spi: &mut SPI, buf: &mut [u8]) -> Result<(), Error<SPI::Error>>
where
    SPI: SpiDevice,
{
    if !buf.len().is_multiple_of(2) {
        return Err(Error::FifoLengthOdd);
    }
    spi.transaction(&mut [Operation::Write(&[CMD_READ_FIFO]), Operation::Read(buf)])
        .await
        .map_err(Error::Spi)
}
