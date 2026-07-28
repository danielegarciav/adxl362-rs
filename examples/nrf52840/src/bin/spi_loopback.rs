//! # SPI3 loopback sanity check — Seeed XIAO nRF52840 + Embassy
//!
//! Proves the MCU's SPI3 peripheral and its two pins actually work,
//! **independent of the ADXL362**. Run this before trusting a bad part-ID
//! read from `basic` — if this test fails, the sensor isn't the problem yet.
//!
//! Note: PR #2's `spi_diag` already found P1.13–P1.15 stuck high under a
//! push-pull GPIO drive test on this board. This test re-checks P1.14/P1.15
//! independently, via the SPI peripheral itself rather than raw GPIO — if it
//! also fails, that corroborates the pins being the problem rather than the
//! ADXL362.
//!
//! ## Setup
//!
//! No sensor needed. Just jumper a wire directly between MOSI and MISO on the
//! XIAO board:
//!
//! | Signal | XIAO pin | nRF52840 pin |
//! |--------|----------|--------------|
//! | MOSI   | D10      | P1.15        |
//! | MISO   | D9       | P1.14        |
//!
//! ## Build
//!
//! ```sh
//! cargo run --release --bin spi_loopback
//! ```

#![no_std]
#![no_main]

use defmt::{error, info};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_nrf::{
    bind_interrupts,
    peripherals,
    spim::{self, Spim},
};
use embassy_time::Timer;
use panic_probe as _;

bind_interrupts!(struct Irqs {
    SPIM3 => spim::InterruptHandler<peripherals::SPI3>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_nrf::init(Default::default());

    let mut config = spim::Config::default();
    config.frequency = spim::Frequency::M4;
    config.mode = spim::MODE_0;

    // SCK (P1.13 / D8) is unused here but Spim::new requires a pin; it can be
    // left unconnected for this test.
    let mut spim = Spim::new(p.SPI3, Irqs, p.P1_13, p.P1_14, p.P1_15, config);

    let patterns: [[u8; 4]; 4] = [
        [0x00, 0x00, 0x00, 0x00],
        [0xFF, 0xFF, 0xFF, 0xFF],
        [0xAA, 0x55, 0xAA, 0x55],
        [0xDE, 0xAD, 0xBE, 0xEF],
    ];

    loop {
        let mut all_ok = true;
        for tx in &patterns {
            let mut rx = [0u8; 4];
            spim.transfer(&mut rx, tx).await.unwrap();
            if rx != *tx {
                all_ok = false;
                error!("loopback MISMATCH: sent {:x} got {:x}", tx, rx);
            }
        }

        if all_ok {
            info!("loopback OK — SPI3 peripheral + MOSI(D10)/MISO(D9) pins are good");
        } else {
            error!(
                "loopback FAILED — check the MOSI(D10)<->MISO(D9) jumper wire itself first; \
                 if the jumper is solid, this corroborates spi_diag's P1.13-15 stuck-high finding"
            );
        }

        Timer::after_millis(1000).await;
    }
}
