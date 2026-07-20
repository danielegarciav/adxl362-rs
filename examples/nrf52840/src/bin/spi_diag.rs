//! # SPI bus diagnostic for the ADXL362 wiring
//!
//! Run this when `basic` reports `InvalidDevice { found_part_id: 255 }`.
//! `0xFF` on every byte means MISO was never driven low, so the question is
//! whether the part is on the bus at all — and if so, on which pins.
//!
//! Three phases:
//!
//! 1. **GPIO self-test** — drive each candidate pin push-pull low, then high,
//!    reading it back. A pin that will not follow its own output driver is not
//!    routed off-package on this module, or is shorted; nothing downstream can
//!    work until that is resolved.
//! 2. **Bus survey** — for both the pre-rewire P0 pin group and the current P1
//!    group, try all six assignments to (SCK, MISO, MOSI) against both CS
//!    candidates, dumping the ID block each time. This catches a swapped pair
//!    and also catches "the part is still wired where it used to be".
//! 3. **Verdict** — report the wiring that answered, if any.
//!
//! ```sh
//! cargo run --release --bin spi_diag
//! ```

#![no_std]
#![no_main]

use defmt::{info, warn};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_nrf::{
    bind_interrupts,
    gpio::{AnyPin, Flex, Level, Output, OutputDrive, Pull},
    peripherals,
    spim::{self, Spim},
    Peri,
};
use embassy_time::Timer;
use panic_probe as _;

bind_interrupts!(struct Irqs {
    SPIM3 => spim::InterruptHandler<peripherals::SPI3>;
});

/// Every pin either wiring has ever used. Indices below refer into this table.
const NAMES: [&str; 8] = [
    "P0_28", "P0_29", "P0_30", "P0_31", "P0_02", "P1_13", "P1_14", "P1_15",
];

/// Bus pin triples to survey, as (label, indices). Order within a triple is
/// irrelevant — every permutation gets tried.
const BUSES: [(&str, [usize; 3]); 2] = [
    ("P0 group (pre-rewire)", [1, 0, 2]), // P0_29 sck, P0_28 miso, P0_30 mosi
    ("P1 group (current)", [5, 6, 7]),    // P1_13 sck, P1_14 miso, P1_15 mosi
];

/// CS candidates: the current one and the one the P0 wiring used.
const CS_CANDIDATES: [usize; 2] = [4, 3]; // P0_02, P0_31

/// The six ways three wires can map onto (sck, miso, mosi).
const PERMS: [[usize; 3]; 6] = [
    [0, 1, 2],
    [0, 2, 1],
    [1, 0, 2],
    [1, 2, 0],
    [2, 0, 1],
    [2, 1, 0],
];

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut p = embassy_nrf::init(Default::default());

    let mut pins: [Peri<'_, AnyPin>; 8] = [
        p.P0_28.into(),
        p.P0_29.into(),
        p.P0_30.into(),
        p.P0_31.into(),
        p.P0_02.into(),
        p.P1_13.into(),
        p.P1_14.into(),
        p.P1_15.into(),
    ];

    // ── Phase 1: do these GPIOs work at all? ────────────────────────────────
    //
    // A push-pull driver beats any sane external pull, so readback that
    // disagrees with the level we just drove means the pin is not actually
    // reaching the outside world.
    info!("── phase 1: GPIO self-test ──");
    for i in 0..pins.len() {
        // `set_as_input_output` keeps the input buffer connected while the pad
        // is driven, so `is_high` reads the actual pad, not the output latch.
        let mut flex = Flex::new(pins[i].reborrow());
        flex.set_as_input_output(Pull::None, OutputDrive::Standard);

        flex.set_low();
        Timer::after_millis(1).await;
        let low_ok = flex.is_low();

        flex.set_high();
        Timer::after_millis(1).await;
        let high_ok = flex.is_high();

        flex.set_as_disconnected();

        match (low_ok, high_ok) {
            (true, true) => info!("{}: OK (follows both levels)", NAMES[i]),
            (false, true) => warn!("{}: stuck HIGH — shorted to VDD, or pull too strong", NAMES[i]),
            (true, false) => warn!("{}: stuck LOW — shorted to GND", NAMES[i]),
            (false, false) => warn!("{}: DEAD — not routed off-module, or damaged", NAMES[i]),
        }
    }

    // ── Phase 2: does the part answer anywhere? ─────────────────────────────
    //
    // 1 MHz keeps us clear of any signal-integrity question on jumper wire.
    info!("── phase 2: bus survey @ 1 MHz ──");
    let mut config = spim::Config::default();
    config.frequency = spim::Frequency::M1;
    config.mode = spim::MODE_0;

    let mut found: Option<(&str, usize, usize, usize, usize)> = None;

    for cs_idx in CS_CANDIDATES {
        for (bus_label, bus) in BUSES {
            info!("cs={} bus={}", NAMES[cs_idx], bus_label);

            for perm in PERMS {
                let (s, mi, mo) = (bus[perm[0]], bus[perm[1]], bus[perm[2]]);

                // Distinct by construction: CS is never a member of a bus triple.
                let [cs_pin, sck, miso, mosi] =
                    pins.get_disjoint_mut([cs_idx, s, mi, mo]).unwrap();

                let mut cs = Output::new(cs_pin.reborrow(), Level::High, OutputDrive::Standard);
                let mut spim = Spim::new(
                    p.SPI3.reborrow(),
                    Irqs,
                    sck.reborrow(),
                    miso.reborrow(),
                    mosi.reborrow(),
                    config.clone(),
                );

                // 0x0B = read command, 0x00 = start at DEVID_AD.
                // Reply lands in bytes 2.. : DEVID_AD, DEVID_MST, PARTID, REVID.
                let tx = [0x0B, 0x00, 0x00, 0x00, 0x00, 0x00];
                let mut rx = [0u8; 6];

                Timer::after_micros(50).await;
                cs.set_low();
                Timer::after_micros(50).await;
                let r = spim.transfer(&mut rx, &tx).await;
                Timer::after_micros(50).await;
                cs.set_high();

                if r.is_err() {
                    warn!("  sck={} miso={} mosi={}: SPI error", NAMES[s], NAMES[mi], NAMES[mo]);
                    continue;
                }

                let matched = rx[2] == 0xAD && rx[4] == 0xF2;
                if matched {
                    found = Some((bus_label, cs_idx, s, mi, mo));
                    info!(
                        "  sck={} miso={} mosi={}: MATCH  ad={:#04x} mst={:#04x} part={:#04x} rev={:#04x}",
                        NAMES[s], NAMES[mi], NAMES[mo], rx[2], rx[3], rx[4], rx[5]
                    );
                } else {
                    info!(
                        "  sck={} miso={} mosi={}:        ad={:#04x} mst={:#04x} part={:#04x} rev={:#04x}",
                        NAMES[s], NAMES[mi], NAMES[mo], rx[2], rx[3], rx[4], rx[5]
                    );
                }

                Timer::after_millis(2).await;
            }
        }
    }

    // ── Phase 3: verdict ────────────────────────────────────────────────────
    match found {
        Some((bus_label, cs_idx, s, mi, mo)) => {
            info!("── result: ADXL362 found on {} ──", bus_label);
            info!(
                "use: Spim::new(p.SPI3, Irqs, p.{}, p.{}, p.{}, config)  cs = p.{}",
                NAMES[s], NAMES[mi], NAMES[mo], NAMES[cs_idx]
            );
        }
        None => {
            warn!("── result: no response on any pin group, CS, or permutation ──");
            warn!("the bus is electrically idle-high everywhere, so the part is not");
            warn!("talking: check VDD/GND at the module, and that CS reaches its pin.");
        }
    }

    loop {
        Timer::after_millis(1000).await;
    }
}
