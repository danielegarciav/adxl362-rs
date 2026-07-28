//! # ADXL362 raw register dump — Seeed XIAO nRF52840 + Embassy
//!
//! Reads DEVID_AD, DEVID_MST, PARTID and REVID directly over raw SPI,
//! bypassing `Adxl362::new()`'s identity check so you can see the actual
//! byte values instead of just a pass/fail. Useful once `spi_loopback` has
//! confirmed the MCU/SPI3 side is good, to see whether the sensor is
//! completely silent (all 0x00 / all 0xFF) or responding with something
//! unexpected (wrong mode/timing, partial response, etc).
//!
//! ## Wiring (Seeed XIAO nRF52840, plain — not Sense)
//!
//! | ADXL362 | XIAO pin | nRF52840 pin |
//! |---------|----------|--------------|
//! | SCK     | D8       | P1.13        |
//! | MISO    | D9       | P1.14        |
//! | MOSI    | D10      | P1.15        |
//! | CS      | D2       | P0.28        |
//! | VDD     | 3V3      | 1.6–3.5 V    |
//! | GND     | GND      |              |
//!
//! Expected values: `DEVID_AD=0xAD`, `DEVID_MST=0x1D`, `PARTID=0xF2`,
//! `REVID>=0x01`.
//!
//! ## Build
//!
//! ```sh
//! cargo run --release --bin id_dump
//! ```

#![no_std]
#![no_main]

use defmt::info;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_nrf::{
    bind_interrupts,
    gpio::{Level, Output, OutputDrive},
    peripherals,
    spim::{self, Spim},
};
use embassy_time::{Delay, Timer};
use embedded_hal_async::spi::SpiDevice;
use embedded_hal_bus::spi::ExclusiveDevice;
use panic_probe as _;

bind_interrupts!(struct Irqs {
    SPIM3 => spim::InterruptHandler<peripherals::SPI3>;
});

/// SPI command byte: read register(s) (ADXL362 Rev. G).
const CMD_READ: u8 = 0x0B;

async fn read_reg(spi: &mut impl SpiDevice, addr: u8) -> u8 {
    let mut rx = [0u8; 3];
    spi.transfer(&mut rx, &[CMD_READ, addr, 0x00])
        .await
        .unwrap();
    rx[2]
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_nrf::init(Default::default());

    let mut config = spim::Config::default();
    config.frequency = spim::Frequency::M4;
    config.mode = spim::MODE_0;

    let spim = Spim::new(p.SPI3, Irqs, p.P1_13, p.P1_14, p.P1_15, config);
    let cs = Output::new(p.P0_28, Level::High, OutputDrive::Standard);
    let mut spi = ExclusiveDevice::new(spim, cs, Delay).unwrap();

    loop {
        let devid_ad = read_reg(&mut spi, 0x00).await;
        let devid_mst = read_reg(&mut spi, 0x01).await;
        let partid = read_reg(&mut spi, 0x02).await;
        let revid = read_reg(&mut spi, 0x03).await;

        info!(
            "DEVID_AD={:x} (want ad)  DEVID_MST={:x} (want 1d)  PARTID={:x} (want f2)  REVID={:x}",
            devid_ad, devid_mst, partid, revid
        );

        if devid_ad == 0x00 && devid_mst == 0x00 && partid == 0x00 {
            info!(
                "all-zero: MISO idle-low — check VDD reaches the chip's pin, and that CS is \
                 actually toggling (scope/multimeter on P0.28)"
            );
        } else if devid_ad == 0xFF && devid_mst == 0xFF && partid == 0xFF {
            info!(
                "all-0xff: MISO idle-high (floating) — nothing is driving it; check the chip \
                 is seated/soldered and MISO is actually connected"
            );
        }

        Timer::after_millis(1000).await;
    }
}
