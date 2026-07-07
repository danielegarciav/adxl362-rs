//! # adxl362
//!
//! `no_std` embedded-hal 1.0 driver for the Analog Devices ADXL362 ultra-low-power
//! 3-axis MEMS accelerometer.
//!
//! ## Features
//!
//! | Feature | Default | Description |
//! |---------|---------|-------------|
//! | `async` | ✓ | Async surface via `embedded-hal-async`; disable for blocking-only. |
//! | `blocking` | | Sync surface; use with `--no-default-features --features blocking`. |
//! | `float` | | `f32` g / °C conversions via `read_accel_g` / `Temperature::to_celsius`. |
//! | `defmt` | | Derive `defmt::Format` on all public types. |
//!
//! ## Quick start
//!
//! ```rust,ignore
//! let mut accel = Adxl362::new(spi).await?;
//! accel.set_range(Range::G4).await?;
//! accel.set_output_data_rate(OutputDataRate::Hz100).await?;
//! accel.start_measurement().await?;
//!
//! let raw = accel.read_raw().await?;           // [i16; 3] — no FPU needed
//! let mg  = accel.read_accel_mg().await?;      // integer milli-g
//! ```
//!
//! ## SPI requirements
//!
//! Mode 0 (CPOL=0, CPHA=0), MSB-first, **1–8 MHz** (use ≥ 1 MHz when the FIFO
//! is active to sustain burst reads, Rev. G Table 10).

#![cfg_attr(not(test), no_std)]
#![deny(missing_docs)]

#[cfg(not(any(feature = "async", feature = "blocking")))]
compile_error!(
    "adxl362: no feature surface enabled. `async` is the default, so this usually means you set \
     `default-features = false` without also picking a surface — add `features = [\"blocking\"]` \
     for the sync API, or `features = [\"async\"]` to keep the async one."
);

#[cfg(all(feature = "async", feature = "blocking"))]
compile_error!(
    "`async` and `blocking` are mutually exclusive — enable exactly one (blocking wins silently \
     via maybe-async otherwise)"
);

pub mod config;
pub mod data;
pub mod error;

mod device;
mod register;
mod transport;

pub use config::{
    ActivityConfig, FifoMode, FifoSample, FifoSampleSet, FifoTag, FilterBandwidth,
    InactivityConfig, IntPin, IntSources, LinkLoopMode, NoiseMode, OutputDataRate, Range, Status,
};
#[cfg(feature = "float")]
pub use data::AccelG;
pub use data::{AccelMg, RawAccel, RawAccel8, Temperature};
pub use device::{Adxl362, SelfTestResult};
pub use error::Error;
