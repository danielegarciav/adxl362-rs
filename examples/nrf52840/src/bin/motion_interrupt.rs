//! # ADXL362 autonomous motion switch — nRF52840 + Embassy
//!
//! Configures activity and inactivity detection in **Loop mode** with autosleep.
//! The ADXL362 handles wake/sleep transitions entirely on its own; the nRF52840
//! simply observes the AWAKE level signal on INT2.
//!
//! ## AWAKE signal behaviour
//!
//! In Loop mode, INT2 (mapped to AWAKE) is a **level** output:
//! - **Rising edge** → device just woke up (activity threshold exceeded)
//! - **Falling edge** → device just went back to sleep (inactivity timeout)
//!
//! This example awaits the rising edge with the driver's `wait_for_event`, then
//! manually awaits the falling edge on the pin for the sleep transition.
//!
//! ## Motion thresholds
//!
//! - Activity:   > 250 mg for 2 consecutive samples (~20 ms at 100 Hz)
//! - Inactivity: < 150 mg for 300 samples (3 s at 100 Hz) → autosleep
//!
//! ## Wiring (nRF52840-DK)
//!
//! | ADXL362 | Pin   | Notes                                |
//! |---------|-------|--------------------------------------|
//! | SCK     | P0.29 |                                      |
//! | MISO    | P0.28 |                                      |
//! | MOSI    | P0.30 |                                      |
//! | CS      | P0.31 |                                      |
//! | INT2    | P0.03 | Active-high level; no MCU pull       |
//! | VDD     | VDD   | 1.6–3.5 V                            |
//! | GND     | GND   |                                      |
//!
//! ## Build
//!
//! ```sh
//! cargo build --bin motion_interrupt --release
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

use adxl362::{
    ActivityConfig, Adxl362, InactivityConfig, IntPin, IntSources, LinkLoopMode, NoiseMode,
    OutputDataRate, Range,
};

bind_interrupts!(struct Irqs {
    SPIM3 => spim::InterruptHandler<peripherals::SPI3>;
});

const ACT_THRESHOLD_MG: u32 = 250;
const ACT_TIME_SAMPLES: u8 = 2;
const INACT_THRESHOLD_MG: u32 = 150;
const INACT_TIME_SAMPLES: u16 = 300; // 3 s at 100 Hz

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_nrf::init(Default::default());

    let mut config = spim::Config::default();
    config.frequency = spim::Frequency::M4;
    config.mode = spim::MODE_0;

    let spim = Spim::new(p.SPI3, Irqs, p.P0_29, p.P0_28, p.P0_30, config);
    let cs = Output::new(p.P0_31, Level::High, OutputDrive::Standard);
    let spi = ExclusiveDevice::new_no_delay(spim, cs).unwrap();

    let mut accel = Adxl362::new(spi).await.expect("ADXL362 not found");

    accel.soft_reset().await.unwrap();
    Timer::after_millis(1).await; // ≥ 0.5 ms post-reset (Rev. G §13)

    // ±2 g gives the best sensitivity for sub-1 g motion detection.
    accel.set_range(Range::G2).await.unwrap();
    accel
        .set_output_data_rate(OutputDataRate::Hz100)
        .await
        .unwrap();
    accel.set_noise_mode(NoiseMode::LowNoise).await.unwrap();

    let act_thresh = Range::G2.mg_to_threshold_code(ACT_THRESHOLD_MG);
    let inact_thresh = Range::G2.mg_to_threshold_code(INACT_THRESHOLD_MG);

    accel
        .configure_activity(ActivityConfig::absolute(act_thresh, ACT_TIME_SAMPLES))
        .await
        .unwrap();
    accel
        .configure_inactivity(InactivityConfig::absolute(inact_thresh, INACT_TIME_SAMPLES))
        .await
        .unwrap();

    // Loop mode: ADXL362 alternates autonomously — activity → wake → inactivity → autosleep.
    // ACT_EN and INACT_EN are already set by the configure_* calls above.
    accel.set_link_loop(LinkLoopMode::Loop).await.unwrap();
    accel.set_autosleep(true).await.unwrap();

    // Route AWAKE (level) to INT2, active-high.
    // AWAKE is only meaningful in linked / loop mode (Rev. G §5).
    accel
        .map_interrupts(
            IntPin::Int2,
            IntSources {
                awake: true,
                ..Default::default()
            },
        )
        .await
        .unwrap();

    accel.start_measurement().await.unwrap();
    Timer::after_millis(40).await; // 4/ODR settle

    // INT2 is driven by the ADXL362 — no MCU pull resistor.
    let mut int2 = Input::new(p.P0_03, Pull::None);

    info!(
        "Motion switch ready — {}mg wake / {}mg sleep ({} s)",
        ACT_THRESHOLD_MG,
        INACT_THRESHOLD_MG,
        INACT_TIME_SAMPLES / 100,
    );

    loop {
        // Rising edge: ADXL362 just woke up.  wait_for_event reads STATUS
        // internally (clearing the ACT flag).
        let status = accel.wait_for_event(&mut int2).await.unwrap();
        if status.err_user_regs {
            warn!("ERR_USER_REGS on wake");
        }
        info!("AWAKE — motion detected");

        // Falling edge: ADXL362 entered autosleep after the inactivity timeout.
        int2.wait_for_falling_edge().await;
        // Read STATUS to clear the INACT flag so the next cycle arms cleanly.
        let _ = accel.read_status().await.unwrap();
        info!("ASLEEP — {} s of inactivity", INACT_TIME_SAMPLES / 100);
    }
}
