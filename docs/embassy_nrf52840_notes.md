# Embassy / nRF52840 Integration Notes

Reference for writing or updating the examples under `examples/nrf52840/`.

## Example project structure

Follows the embassy-rs repo convention — each target is a standalone Cargo project, not `[[example]]` entries in the library crate:

```
examples/nrf52840/
├── Cargo.toml          # standalone crate, path dep on ../../
├── .cargo/config.toml  # sets default target + probe-rs runner
├── memory.x            # nRF52840 flash/RAM layout
├── build.rs            # copies memory.x into linker search path
└── src/bin/            # one file per example binary
    ├── basic.rs
    ├── motion_interrupt.rs
    └── fifo.rs
```

Build any example from inside `examples/nrf52840/`:

```sh
cargo build --bin basic --release
```

## Dependency versions (verified 2026-06)

| Crate              | Version  | Notes                                                                             |
| ------------------ | -------- | --------------------------------------------------------------------------------- |
| `embassy-executor` | `0.10.0` | features: `platform-cortex-m`, `executor-thread`, `defmt`                         |
| `embassy-nrf`      | `0.11.0` | features: `nrf52840`, `time-driver-rtc1`, `gpiote`, `defmt`                       |
| `embassy-time`     | `0.5.1`  | features: `defmt`, `defmt-timestamp-uptime`                                       |
| `embedded-hal-bus` | `0.3`    | features: `async` — provides `ExclusiveDevice`                                    |
| `defmt`            | `1`      | **Breaking:** lib crate was on `"0.3"`, bumped to `"1"` to match embassy-nrf 0.11 |
| `defmt-rtt`        | `1`      |                                                                                   |
| `panic-probe`      | `1`      | features: `print-defmt`                                                           |
| `cortex-m`         | `0.7`    | features: `inline-asm`, `critical-section-single-core`                            |
| `cortex-m-rt`      | `0.7`    |                                                                                   |

## embassy-nrf 0.11 API specifics

### SPIM setup

```rust
bind_interrupts!(struct Irqs {
    // Interrupt name is just SPIM3; peripheral token is SPI3 (not SPIM3)
    SPIM3 => spim::InterruptHandler<peripherals::SPI3>;
});

let mut config = spim::Config::default();
config.frequency = spim::Frequency::M4;  // ≤ 8 MHz for ADXL362
config.mode = spim::MODE_0;              // CPOL=0, CPHA=0

// Parameter order: (peripheral, irq, sck, miso, mosi, config)
let spim = Spim::new(p.SPI3, Irqs, p.P0_29, p.P0_28, p.P0_30, config);
```

Default SPI3 pins on the nRF52840-DK: SCK=P0.29, MISO=P0.28, MOSI=P0.30, CS=P0.31.

### SpiDevice for single-device SPI

```rust
let cs  = Output::new(p.P0_31, Level::High, OutputDrive::Standard);
let spi = ExclusiveDevice::new_no_delay(spim, cs).unwrap();
// ExclusiveDevice<Spim, Output, NoDelay> implements embedded_hal_async::spi::SpiDevice
```

### GPIO async wait — inherent vs. trait methods

This so far seems to be **intentional embassy design**, not a version mismatch or deprecation.

The root tension: `embedded_hal_async::digital::Wait` must return `Result<(), Self::Error>` to
abstract over GPIO implementations that can genuinely fail (e.g. I²C-expander-backed pins).
But `embassy_nrf::gpio::Input` uses `type Error = Infallible` — it cannot fail — so the `Result`
is pure noise that forces callers to `.unwrap()` something that never returns `Err`.

Embassy's solution: provide ergonomic **inherent methods returning `()`** for direct use on the
concrete type, and keep the `Wait` **trait impl** (which just wraps each inherent call in
`Ok(...)`) for generic compatibility. The same pattern applies to `Output::set_high()` and
`set_low()` throughout embassy-nrf — it is consistent and deliberate.

`gpiote.rs` defines both in the same file:

1. **Inherent methods** (added by the `gpiote` feature) — return `()`:
   ```rust
   pub async fn wait_for_falling_edge(&mut self) { ... }
   ```
2. **`embedded_hal_async::digital::Wait` trait impl** — wraps the inherent call in `Ok(...)`,
   returns `Result<(), Infallible>`:
   ```rust
   async fn wait_for_falling_edge(&mut self) -> Result<(), Self::Error> {
       Ok(self.wait_for_falling_edge().await)
   }
   ```

Rust method resolution always picks inherent over trait, so `pin.wait_for_falling_edge().await`
returns `()`, **not** `Result`. Do not call `.unwrap()` on it.

The `Wait` trait import is only needed when passing the pin to a generic function bounded by
`P: Wait` (e.g. the driver's `wait_for_event`) — there, the trait method is what gets called and
it does return `Result<(), Infallible>`.

The reason docs.rs makes this look like it returns `Result`: the page lists the **trait
implementation's** signatures. The inherent methods live in a separate `impl` block gated behind
the `gpiote` feature and are easy to miss.

### No explicit GPIOTE bind_interrupts needed

`gpio::Input::wait_for_*` works without binding the GPIOTE interrupt in user code — the `gpiote` feature handles it internally. Only bind `SPIM3` for the SPI driver.

## defmt formatting in macros

`defmt` does **not** support `std::fmt` width/fill/alignment specifiers. Use plain `{}`:

```rust
info!("x: {} mg  y: {} mg  z: {} mg", mg.x, mg.y, mg.z);  // correct
info!("x: {:5} mg", mg.x);  // ERROR: unknown display hint "5"
```
