# ADXL362 Driver — Implementation Notes

Reference: `docs/ADXL362_driver_design_v5.md`

## Key decisions made during implementation

### 1. `maybe-async` vs `maybe-async-cfg`

**Decision:** Use `maybe-async` (not `maybe-async-cfg`).

The design doc says "single source, both surfaces via `maybe-async-cfg`." In practice,
`maybe-async-cfg` v0.2 renames types and functions by appending `Sync`/`Async` suffixes
in the generated code, which breaks the "one type, conditionally async methods" pattern.

`maybe-async` v0.2 does what we actually want: `#[maybe_async::maybe_async]` on an impl
block strips `async`/`.await` in sync mode without renaming anything.

### 2. Feature layout: `async` (default) + `blocking` (opt-in)

**Decision:** The sync/blocking surface is opt-in via `--features blocking`, not the default.

`maybe-async` uses its own `is_sync` feature internally. To enable it, `blocking` feature
activates `maybe-async/is_sync`. The async surface is the default (no extra feature needed
besides the package-level `async` feature which brings in `embedded-hal-async`).

Feature matrix:
- Default (`async`): async surface via `embedded-hal-async::spi::SpiDevice`
- `blocking` (+ `--no-default-features`): sync surface via `embedded-hal::spi::SpiDevice`
- `float`: adds `f32` g and °C helpers
- `defmt`: adds `defmt::Format` derives on all public types

To run tests: `cargo test --no-default-features --features blocking`

### 3. SpiDevice conditional import pattern

The `SpiDevice` trait in async vs sync mode resolves through:
```rust
#[cfg(feature = "blocking")]
use embedded_hal::spi::SpiDevice;
#[cfg(not(feature = "blocking"))]
use embedded_hal_async::spi::SpiDevice;
```

`embedded_hal::spi::Operation` is always imported from `embedded_hal` regardless of mode
(the async trait reuses the same `Operation` type).

### 4. POWER_CTL register encoding

Wake-up mode uses `MEASURE[1:0] = 0b10` (same as Measurement) **plus** bit 3 `WAKEUP = 1`.
It does NOT use a separate `MEASURE = 0b01` encoding.  
`standby()` → clears bits [1:0] and bit 3.  
`start_measurement()` → sets MEASURE = 0b10, clears bit 3.  
`enter_wakeup()` → sets MEASURE = 0b10 AND bit 3.

### 5. `delay` for `soft_reset` and `self_test`

`soft_reset()` does NOT apply the 0.5 ms wait itself; callers must wait after the call.
This keeps the API timer-agnostic and matches the design doc intent.

`self_test()` accepts `delay: &mut D` where `D: DelayNs` (embedded-hal / embedded-hal-async
depending on feature). It applies the 40 ms settle internally (4/ODR at 100 Hz).

### 6. `wait_for_event` pin error handling

Pin errors (from `Wait::wait_for_rising_edge`) are discarded. Most embedded GPIO
implementations use `Infallible` as the error type, making this a no-op in practice.
If pin error propagation is needed in the future, `Error` would need a second generic
parameter `Error<SpiE, PinE>`.

### 7. FIFO set iterator (heap-free)

`read_fifo_sets` returns a `FifoSetIter<'_>` that re-parses the raw byte buffer on-the-fly
rather than collecting `FifoSample` into a vec. This keeps the driver heap-free on no_std.

### 8. ACT_INACT_CTL bits [7:6] masking

The design doc warns that bits [7:6] are unused but RW. All read-modify-write operations on
this register mask off the high two bits to avoid relying on a fixed reset pattern:
```rust
let masked = cur & 0x3F;
// ... modify ...
write(reg, result & 0x3F);
```

### 9. ±8 g scale factor

Hardcoded as **235 LSB/g** (4.255 mg/LSB), NOT 250.  Using 250 would produce ~6% error.
Integer milli-g conversion: `(raw as i32 * 4255) / 1000` (truncates, no FPU required).

### 10. FIFO sign-extension

FIFO 16-bit words have tag in bits [15:14], 12-bit signed value in bits [11:0],
with bits [13:12] being the device-inserted sign extension of bit [11].

Sign-extend via i32 to avoid i16 overflow:
```rust
let raw12 = (word & 0x0FFF) as i32;
let value = ((raw12 << 20) >> 20) as i16;
```

### 11. `AccelG` (float) docs

`AccelG` struct (behind `float` feature) requires explicit doc comments on each field
because `#![deny(missing_docs)]` is crate-wide. `#[cfg(feature = "float")]` items are
not exempt.

### 12. Test strategy

Tests live in `tests/spi_transactions.rs` and use `embedded-hal-mock` v0.11 (eh1 feature).
They test byte-level SPI transactions, sign-extension correctness, FIFO parsing, and
threshold/ODR conversion helpers. They run only with `--features blocking` since
`embedded-hal-mock` sync SPI is used.

Future: async tests can use `embedded-hal-mock` async SPI mock + `tokio::test`.
