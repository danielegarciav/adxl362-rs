# adxl362-rs

`no_std` embedded-hal 1.0 driver for the Analog Devices ADXL362 ultra-low-power 3-axis MEMS accelerometer.

## Features

- Async by default via `embedded-hal-async`; blocking surface available with `--no-default-features --features blocking`
- Single SPI transaction for burst reads (all three axes in one call)
- Activity / inactivity detection with linked and loop modes, autosleep
- 512-sample FIFO with stream, FIFO, and triggered modes; watermark interrupt support
- Integer milli-g output with no FPU required; optional `f32` g output via the `float` feature
- `defmt::Format` derives on all public types via the `defmt` feature
- Built-in self-test sequence per Rev. G Table 22

| Feature   | Default | Description |
|-----------|---------|-------------|
| `async`   | ✓       | Async surface via `embedded-hal-async` |
| `blocking`|         | Sync surface; use with `--no-default-features --features blocking` |
| `float`   |         | `f32` g / °C conversions (`read_accel_g`, `Temperature::to_celsius`) |
| `defmt`   |         | `defmt::Format` on all public types |

## Installation

```toml
[dependencies]
adxl362 = { version = "0.1", features = ["async", "defmt"] }
```

For blocking use:

```toml
[dependencies]
adxl362 = { version = "0.1", default-features = false, features = ["blocking"] }
```

## SPI requirements

Mode 0 (CPOL=0, CPHA=0), MSB-first, **1–8 MHz**. Use ≥ 1 MHz when the FIFO is active to sustain burst reads (Rev. G Table 10).

## Quick start

```rust,ignore
// Async (Embassy / embedded-hal-async)
let mut accel = Adxl362::new(spi).await?;

accel.soft_reset().await?;
delay.delay_ms(1).await;               // ≥ 0.5 ms post-reset (Rev. G §13)

accel.set_range(Range::G4).await?;
accel.set_output_data_rate(OutputDataRate::Hz100).await?;
accel.set_noise_mode(NoiseMode::LowNoise).await?;
accel.start_measurement().await?;

delay.delay_ms(40).await;              // 4/ODR settle

while !accel.data_ready().await? {}
let mg = accel.read_accel_mg().await?; // AccelMg { x, y, z } in integer milli-g
```

## Examples

Examples follow the [Embassy](https://github.com/embassy-rs/embassy) repo convention — each MCU target is a standalone Cargo project with its own `Cargo.toml`, `.cargo/config.toml`, `memory.x`, and `build.rs`.

### nRF52840 (`examples/nrf52840/`)

Three binaries targeting the nRF52840-DK:

| Binary | Description |
|--------|-------------|
| `basic` | Poll accelerometer at 100 Hz, log X/Y/Z over RTT |
| `motion_interrupt` | Autonomous wake/sleep in Loop mode; observe AWAKE on INT2 |
| `fifo` | FIFO Stream mode with watermark interrupt on INT1; drain and convert per batch |

**Wiring (shared by all three binaries):**

| ADXL362 | nRF52840-DK pin | Notes |
|---------|-----------------|-------|
| SCK     | P0.29           | SPI3 SCK |
| MISO    | P0.28           | SPI3 MISO |
| MOSI    | P0.30           | SPI3 MOSI |
| CS      | P0.31           | Active-low chip select |
| VDD     | VDD             | 1.6–3.5 V |
| GND     | GND             | |

`motion_interrupt` additionally uses P0.03 for INT2; `fifo` uses P0.02 for INT1.

**Build:**

```sh
cd examples/nrf52840
cargo build --bin basic --release
cargo build --bin motion_interrupt --release
cargo build --bin fifo --release
```

The target (`thumbv7em-none-eabi`) is set in `.cargo/config.toml`; no `--target` flag needed.

## API overview

### Construction

```rust,ignore
let mut accel = Adxl362::new(spi).await?;   // verifies DEVID_AD + PARTID
```

`Adxl362::new` reads the identity registers and returns `Error::InvalidDevice` if the device is not detected.

### Power modes

```rust,ignore
accel.start_measurement().await?;   // continuous measurement
accel.standby().await?;             // ~10 nA, register writes must happen here
accel.enter_wakeup().await?;        // ~270 nA, ~6 Hz motion check
accel.set_autosleep(true).await?;   // autonomous sleep between activity events
```

### Data reads

```rust,ignore
let raw = accel.read_raw().await?;         // RawAccel { x, y, z: i16 }
let mg  = accel.read_accel_mg().await?;    // AccelMg  { x, y, z: i32 } (milli-g)
let g   = accel.read_accel_g().await?;     // AccelG   { x, y, z: f32 } (float feature)
let t   = accel.read_temperature_raw().await?; // Temperature { raw: i16 }
```

### Activity / inactivity detection

```rust,ignore
use adxl362::{ActivityConfig, InactivityConfig, LinkLoopMode};

accel.configure_activity(ActivityConfig::absolute(thresh, time_samples)).await?;
accel.configure_inactivity(InactivityConfig::absolute(thresh, time_samples)).await?;
accel.set_link_loop(LinkLoopMode::Loop).await?;
accel.set_autosleep(true).await?;
```

`Range::mg_to_threshold_code` and `OutputDataRate::ms_to_samples` convert human-friendly values to register codes.

### Interrupts

```rust,ignore
use adxl362::{IntPin, IntSources};

accel.map_interrupts(IntPin::Int1, IntSources { fifo_watermark: true, ..Default::default() }).await?;

// Async only — borrow a GPIO pin per call
let status = accel.wait_for_event(&mut int_pin).await?;
```

### FIFO

```rust,ignore
use adxl362::FifoMode;

accel.set_fifo(FifoMode::Stream, false, 60).await?; // watermark = 60 samples

let entries    = accel.fifo_entries().await?;
let byte_count = ((entries as usize) * 2).min(buf.len()) & !1;
let sets       = accel.read_fifo_sets(&mut buf[..byte_count]).await?;

for set in sets {
    // set.x / set.y / set.z are Option<i16>
}
```

## Testing

Tests use `embedded-hal-mock` and run on the host with the `blocking` feature:

```sh
cargo test --no-default-features --features blocking
```

## License

Licensed under either of [Apache License, Version 2.0](LICENSE-APACHE) or [MIT license](LICENSE-MIT) at your option.
