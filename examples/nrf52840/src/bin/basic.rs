//! # ADXL362 basic polling — nRF52840 + Embassy
//!
//! Reads X/Y/Z acceleration in integer milli-g at 100 Hz and logs over RTT.
//!
//! ## Wiring (nRF52840-DK)
//!
//! | ADXL362 | Pin   | Notes                               |
//! |---------|-------|-------------------------------------|
//! | SCK     | P0.29 | SPI3 default SCK                    |
//! | MISO    | P0.28 | SPI3 default MISO                   |
//! | MOSI    | P0.30 | SPI3 default MOSI                   |
//! | CS      | P0.31 | Driven low during transfers         |
//! | VDD     | VDD   | 1.6–3.5 V                           |
//! | GND     | GND   |                                     |
//!
//! Adjust the `P0_XX` constants to match your wiring.
//!
//! ## Build
//!
//! ```sh
//! cargo build --bin basic --release
//! # target is thumbv7em-none-eabi, set in .cargo/config.toml
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
use embassy_time::Timer;
use embedded_hal_bus::spi::ExclusiveDevice;
use panic_probe as _;

use adxl362::{Adxl362, NoiseMode, OutputDataRate, Range};

bind_interrupts!(struct Irqs {
    SPIM3 => spim::InterruptHandler<peripherals::SPI3>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_nrf::init(Default::default());

    // SPI Mode 0 (CPOL=0, CPHA=0), 4 MHz — well within the 8 MHz ADXL362 max.
    let mut config = spim::Config::default();
    config.frequency = spim::Frequency::M4;
    config.mode = spim::MODE_0;

    // Spim::new parameter order: (peripheral, irq, sck, miso, mosi, config)
    let spim = Spim::new(p.SPI3, Irqs, p.P0_29, p.P0_28, p.P0_30, config);
    let cs   = Output::new(p.P0_31, Level::High, OutputDrive::Standard);
    let spi  = ExclusiveDevice::new_no_delay(spim, cs).unwrap();

    let mut accel = Adxl362::new(spi).await.expect("ADXL362 not found — check wiring and VDD");

    // Soft-reset guarantees a clean register state after a warm reboot.
    accel.soft_reset().await.unwrap();
    Timer::after_millis(1).await; // ≥ 0.5 ms post-reset (Rev. G §13)

    // All configuration must happen while the device is in Standby (power-on default).
    accel.set_range(Range::G4).await.unwrap();
    accel.set_output_data_rate(OutputDataRate::Hz100).await.unwrap();
    accel.set_noise_mode(NoiseMode::LowNoise).await.unwrap();
    accel.start_measurement().await.unwrap();

    // Allow 4/ODR = 40 ms for the first valid sample to appear (Rev. G §7.2).
    Timer::after_millis(40).await;

    info!("ADXL362 ready, polling at 100 Hz");

    loop {
        // Spin until DATA_READY, then burst-read all three axes in one SPI transaction.
        while !accel.data_ready().await.unwrap() {}

        let mg = accel.read_accel_mg().await.unwrap();
        info!("x: {} mg  y: {} mg  z: {} mg", mg.x, mg.y, mg.z);
    }
}
