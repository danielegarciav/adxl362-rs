//! Configuration type modules (one per ADXL362 register group).

pub mod fifo;
pub mod filter;
pub mod interrupt;
pub mod motion;
pub mod power;

pub use fifo::{FifoMode, FifoSample, FifoSampleSet, FifoTag};
pub use filter::{FilterBandwidth, OutputDataRate, Range};
pub use interrupt::{IntPin, IntSources, Status};
pub use motion::{ActivityConfig, InactivityConfig, LinkLoopMode};
pub use power::NoiseMode;
