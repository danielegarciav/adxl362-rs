//! # ADXL362 FIFO stream with watermark interrupt — nRF52840 + Embassy
//!
//! Configures the ADXL362 FIFO in **Stream mode** with a 60-sample watermark.
//! When the watermark fires on INT1, the nRF52840 drains the FIFO and converts
//! each XYZ set to integer milli-g.
//!
//! ## FIFO details
//!
//! - Stream mode: newest sample overwrites oldest when the 512-sample FIFO fills.
//! - Watermark at 60 individual tagged samples (2 bytes each).
//!   60 ÷ 3 axes = 20 complete XYZ sets per interrupt at 100 Hz → ~200 ms batches.
//! - FIFO_OVERRUN is mapped alongside FIFO_WATERMARK so lost samples surface on INT1.
//! - Temperature is not stored in the FIFO.
//!
//! ## Wiring (nRF52840-DK)
//!
//! | ADXL362 | Pin   | Notes                                |
//! |---------|-------|--------------------------------------|
//! | SCK     | P0.29 |                                      |
//! | MISO    | P0.28 |                                      |
//! | MOSI    | P0.30 |                                      |
//! | CS      | P0.31 |                                      |
//! | INT1    | P0.02 | Active-high output; no MCU pull      |
//! | VDD     | VDD   | 1.6–3.5 V                            |
//! | GND     | GND   |                                      |
//!
//! ## Build
//!
//! ```sh
//! cargo build --bin fifo --release
//! ```

#![no_std]
#![no_main]

use defmt::{info, warn};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_nrf::{
    bind_interrupts,
    gpio::{Input, Level, Output, OutputDrive, Pull},
    peripherals,
    spim::{self, Spim},
};
use embassy_time::Timer;
use embedded_hal_bus::spi::ExclusiveDevice;
use panic_probe as _;

use adxl362::{Adxl362, FifoMode, IntPin, IntSources, OutputDataRate, Range};

bind_interrupts!(struct Irqs {
    SPIM3 => spim::InterruptHandler<peripherals::SPI3>;
});

// 60 individual axis samples per interrupt; each is 2 bytes.
const WATERMARK: u16 = 60;

// Buffer sized for 2× the watermark so brief back-to-back triggers don't truncate.
const BUF_LEN: usize = (WATERMARK as usize) * 2 * 2;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_nrf::init(Default::default());

    let mut config = spim::Config::default();
    config.frequency = spim::Frequency::M4;
    config.mode = spim::MODE_0;

    let spim = Spim::new(p.SPI3, Irqs, p.P0_29, p.P0_28, p.P0_30, config);
    let cs   = Output::new(p.P0_31, Level::High, OutputDrive::Standard);
    let spi  = ExclusiveDevice::new_no_delay(spim, cs).unwrap();

    let mut accel = Adxl362::new(spi).await.expect("ADXL362 not found");

    accel.soft_reset().await.unwrap();
    Timer::after_millis(1).await; // ≥ 0.5 ms post-reset (Rev. G §13)

    accel.set_range(Range::G4).await.unwrap();
    accel.set_output_data_rate(OutputDataRate::Hz100).await.unwrap();

    // Stream FIFO: newest sample overwrites oldest; fire INT1 at the watermark.
    accel.set_fifo(FifoMode::Stream, false, WATERMARK).await.unwrap();

    // Raise INT1 on watermark or overrun so missed samples are surfaced.
    accel.map_interrupts(
        IntPin::Int1,
        IntSources {
            fifo_watermark: true,
            fifo_overrun:   true,
            ..Default::default()
        },
    ).await.unwrap();

    accel.start_measurement().await.unwrap();
    Timer::after_millis(40).await; // 4/ODR settle

    // INT1 is driven by the ADXL362 — no MCU pull resistor.
    let mut int1 = Input::new(p.P0_02, Pull::None);
    let range    = accel.range(); // cache once; avoids a register read per iteration
    let mut buf  = [0u8; BUF_LEN];

    info!(
        "ADXL362 FIFO streaming at 100 Hz, watermark = {} samples",
        WATERMARK
    );

    loop {
        // Await rising edge on INT1 (watermark or overrun), then read STATUS.
        let status = accel.wait_for_event(&mut int1).await.unwrap();

        if status.fifo_overrun {
            warn!("FIFO overrun — oldest samples were discarded");
        }

        // Drain whatever the FIFO currently holds, up to our buffer.
        // Keep byte count even — the driver requires even-length FIFO reads.
        let entries    = accel.fifo_entries().await.unwrap();
        let byte_count = ((entries as usize) * 2).min(BUF_LEN) & !1;

        // read_fifo_sets fills buf[..byte_count] from the FIFO, then returns an
        // iterator of complete XYZ sets; partial sets at buffer edges are dropped.
        let sets = accel.read_fifo_sets(&mut buf[..byte_count]).await.unwrap();

        for set in sets {
            if let (Some(x), Some(y), Some(z)) = (set.x, set.y, set.z) {
                info!(
                    "x: {} mg  y: {} mg  z: {} mg",
                    range.raw_to_mg(x),
                    range.raw_to_mg(y),
                    range.raw_to_mg(z),
                );
            }
        }
    }
}
