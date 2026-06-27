# ADXL362 `embedded-hal` Driver — Design Document

A `no_std` Rust driver for the Analog Devices **ADXL362** ultra-low-power 3-axis
MEMS accelerometer, built on **`embedded-hal` 1.0** traits and intended to be
first-class in **Embassy** (async) projects.

> Source: *ADXL362 Data Sheet, Rev. G (May 2023)* — the production datasheet.
> All timing/sensitivity constants below are taken from Rev. G. Encode them
> against Rev. G specifically: a couple of fields (notably `LINK/LOOP` in
> `ACT_INACT_CTL`) have an unusual encoding that is easy to get wrong if you
> work from memory or from a pre-production sheet. See
> [§13](#13-timing-constants--revision-notes) for the values to trust and why.

---

## 1. Goals & scope

**Primary goal.** Provide a safe, ergonomic, well-tested driver that exposes the
full feature set of the ADXL362 over SPI, usable in both blocking and async
(Embassy) firmware.

**Design principles.**

- `no_std`, no heap, no panics on the happy path.
- Generic over the HAL: depend only on `embedded-hal` / `embedded-hal-async`
  traits, never a concrete MCU HAL.
- Cheap to use wrong-free: the type system should make the common operations
  obvious and guard against the easy mistakes (reading data in Standby, mixing
  up axis bytes, forgetting to clear an interrupt).
- Layered: a thin register-access core that everything else is built on, so an
  advanced user can always drop down to raw registers.
- Float-optional: raw integer reads are always available; `g` / m/s²
  conversions sit behind a feature so the crate stays usable on cores without
  an FPU.

**Non-goals (v1).** Board-specific wiring, RTOS integration beyond Embassy's
trait surface, and calibration storage. These belong in user code or examples.

---

## 2. Target ecosystem & dependencies

| Concern | Choice | Notes |
|---|---|---|
| SPI abstraction | `embedded-hal::spi::SpiDevice` (blocking) / `embedded-hal-async::spi::SpiDevice` (async) | `SpiDevice` manages CS for us; we do **not** use raw `SpiBus` so the driver composes cleanly on a shared bus. |
| Interrupt pins | `embedded-hal::digital::InputPin` + `embedded-hal-async::digital::Wait` | Optional; see [§7.5](#75-interrupts-int1--int2) and the interrupt-pin decision in [§10](#10-design-decisions). |
| Logging | `defmt` behind a feature | Off by default. |
| Conversion | `libm` or core float behind a `float` feature | Raw `i16` always available without it. |
| Ecosystem trait | optional `accelerometer` crate trait impls behind a feature | Lets the driver slot into generic motion code. |
| Testing | `embedded-hal-mock` | Drive SPI transactions in unit tests with no hardware. |

Minimum SPI configuration the user must provide: **SPI mode 0** (CPOL=0,
CPHA=0), MSB-first, **1–8 MHz** clock (Rev. G recommends 1–8 MHz; Table 10 gives
an absolute `fCLK` max of **8 MHz**, with **12 pF** maximum bus loading). Keep
SCLK **≥ 1 MHz when using the FIFO** so it can sustain burst reads (p.21). Note
Table 10 also specifies an absolute `fCLK` **floor of 2.4 kHz**, which the
footnote says applies only when the FIFO is in use; the 1 MHz figure is the
practically relevant one for keeping up with burst reads.

---

## 3. Device summary (the facts the driver encodes)

- 3-axis, ±2 g / ±4 g / ±8 g, always sampled by a **12-bit ADC** (sensitivity
  scales with range; see [§6](#6-data-formats--conversions)).
- Two operating modes: **Measurement** (continuous, full bandwidth) and
  **Wake-up** (~6 Hz motion check, ~270 nA), plus **Standby** (~10 nA, all
  sensing off). **Powers up in Standby.**
- On-chip **temperature sensor** (12-bit).
- **512-sample FIFO** with disabled / oldest-saved / stream / triggered modes;
  each FIFO datum is self-describing (carries an axis tag).
- Autonomous **activity / inactivity** detection (absolute or referenced),
  linkable into Linked / Loop modes with **Autosleep** — enables a standalone
  motion switch with zero MCU involvement.
- Two interrupt pins **INT1 / INT2**, each freely mappable to any subset of 7
  status sources, active-high or active-low. INT1 doubles as an external-clock
  input; INT2 doubles as a sync-sampling trigger.
- **Self-test** (electrostatic actuation of all three axes).

---

## 4. SPI transport layer

All bus traffic is one of three command-prefixed transactions. CS is asserted
for the whole transaction (handled by `SpiDevice`).

| Command byte | Operation | Frame |
|---|---|---|
| `0x0A` | Write register(s) | `0x0A`, `addr`, `data…` |
| `0x0B` | Read register(s) | `0x0B`, `addr`, then read `data…` |
| `0x0D` | Read FIFO | `0x0D`, then read `data…` (no address) |

**Transport rules to honour:**

- **Auto-increment** on multi-byte register read/write; the address pointer
  **halts at `0x3F`** (no wraparound). Burst reads of the 6 data registers
  (`0x0E`–`0x13`) or 6+2 with temperature (`…0x15`) are the recommended way to
  get a coherent sample set.
- **FIFO reads must be an even number of bytes** (samples are 16-bit, LSB
  first). Reading past empty yields `0x00`. Unlimited burst length is allowed.
- MISO is high-Z except while the device is returning read data — harmless, but
  means writes return don't-care bytes.
- Registers `0x00`–`0x2C` should only be modified while in **Standby**; changes
  made in Measurement Mode may take effect mid-sample.

**Proposed transport API (private core):**

```rust
fn read_register(&mut self, reg: Register) -> Result<u8, Error<E>>;
fn read_registers(&mut self, start: Register, buf: &mut [u8]) -> Result<(), Error<E>>;
fn write_register(&mut self, reg: Register, val: u8) -> Result<(), Error<E>>;
fn read_fifo(&mut self, buf: &mut [u8]) -> Result<(), Error<E>>; // buf.len() must be even
```

---

## 5. Register map

| Addr | Name | Access | Reset | Purpose |
|---|---|---|---|---|
| 0x00 | DEVID_AD | R | 0xAD | Analog Devices ID |
| 0x01 | DEVID_MST | R | 0x1D | MEMS ID |
| 0x02 | PARTID | R | 0xF2 | Part ID (362 octal) |
| 0x03 | REVID | R | 0x01 | Silicon revision |
| 0x08–0x0A | XDATA / YDATA / ZDATA | R | 0x00 | 8-bit (MSB-only) data |
| 0x0B | STATUS | R | 0x40* | Status flags |
| 0x0C–0x0D | FIFO_ENTRIES_L/H | R | 0x00 | FIFO fill count (0–512) |
| 0x0E–0x13 | X/Y/ZDATA_L/H | R | 0x00 | 12-bit sign-extended data |
| 0x14–0x15 | TEMP_L/H | R | 0x00 | 12-bit sign-extended temperature |
| 0x1F | SOFT_RESET | W | 0x00 | Write `0x52` to reset |
| 0x20–0x21 | THRESH_ACT_L/H | RW | 0x00 | Activity threshold (11-bit unsigned) |
| 0x22 | TIME_ACT | RW | 0x00 | Activity time (samples, 8-bit) |
| 0x23–0x24 | THRESH_INACT_L/H | RW | 0x00 | Inactivity threshold (11-bit unsigned) |
| 0x25–0x26 | TIME_INACT_L/H | RW | 0x00 | Inactivity time (samples, 16-bit) |
| 0x27 | ACT_INACT_CTL | RW | 0x00† | Act/inact enable, ref/abs, link/loop |
| 0x28 | FIFO_CONTROL | RW | 0x00 | FIFO mode, temp-in-FIFO, AH bit |
| 0x29 | FIFO_SAMPLES | RW | 0x80 | FIFO watermark sample count |
| 0x2A | INTMAP1 | RW | 0x00 | INT1 source map + active-low |
| 0x2B | INTMAP2 | RW | 0x00 | INT2 source map + active-low |
| 0x2C | FILTER_CTL | RW | 0x13 | Range, half-BW, ext-sample, ODR |
| 0x2D | POWER_CTL | RW | 0x00 | Ext-clk, noise mode, wakeup, autosleep, measure |
| 0x2E | SELF_TEST | RW | 0x00 | Self-test enable |

\* In Rev. G, STATUS is **read-only** with a *stated* reset of `0x40` (bit 6,
`AWAKE`, set). Treat that reset value with suspicion: it is internally
inconsistent with the rest of the datasheet. The prose (p.19, p.27) says
`ERR_USER_REGS` (bit 7) is **high on power-up** until the first register write,
which makes the true power-up byte effectively `0xC0`, not `0x40`. Likewise bit 6
(`AWAKE`) defaults to 1 and must be **ignored unless** activity/inactivity are in
linked or loop mode. Conclusion for the driver: do **not** assert on any
power-up value of STATUS; model it as a read-only bitfield and read it for live
flags only.

† `ACT_INACT_CTL` resets to `0x00` per Table 11/13. Bits [7:6] are unused but
RW, so they read back as written; the driver should mask them off rather than
rely on a fixed reset pattern.

The driver will define `Register` as a `#[repr(u8)]` enum plus a typed bitfield
layer for the structured registers (STATUS, ACT_INACT_CTL, FIFO_CONTROL,
INTMAP*, FILTER_CTL, POWER_CTL).

---

## 6. Data formats & conversions

**Acceleration (12-bit).** Each axis is a 12-bit two's-complement value that the
device **already sign-extends** into the high register. So reading the two bytes
little-endian and reinterpreting gives the signed value directly:

```rust
let raw = i16::from_le_bytes([low, high]); // already in -2048..=2047
```

No manual masking/sign extension needed — a nice property worth a unit test to
lock in.

**Acceleration (8-bit).** `XDATA`/`YDATA`/`ZDATA` (0x08–0x0A) hold the **8 MSBs**
of the 12-bit value (≈ `raw >> 4`), for single-byte-per-axis low-power reads.
Expose as an `i8`-based fast path.

**Sensitivity / range:**

| Range | Scale factor | Resolution |
|---|---|---|
| ±2 g | 1000 LSB/g | 1 mg/LSB |
| ±4 g | 500 LSB/g | 2 mg/LSB |
| ±8 g | **235 LSB/g** | **4.255 mg/LSB** |

**Watch the ±8 g row:** it is *not* a clean 250 LSB/g. Rev. G specifies
**235 LSB/g** (4.255 mg/LSB), so the three scale factors are not a tidy
1000/500/250 progression. Hardcoding 250 would give ~6% error at ±8 g — use an
explicit per-range scale-factor table.

Corroboration from the datasheet itself: the free-fall start-up routine (p.39,
step 1) writes 150 codes and labels it "600 mg" — a figure that only holds at
250 LSB/g (150 / 250 = 0.6 g). Under the Table 1 value of 235 LSB/g those same
150 codes are ~638 mg. In other words ADI's *own* worked example was never
updated after the sensitivity spec changed, which is the clearest possible
argument for sourcing the scale factor from one authoritative per-range constant
and never inlining a literal.

Conversion to g: `g = raw / scale_factor(range)`. Provide both an integer
milli-g path (`raw` → `i32` mg, no FPU) and an `f32` g path behind the `float`
feature.

**Temperature (12-bit).** Sign-extended like the axes. Rev. G specifies
sensitivity **0.065 °C/LSB** and a 25 °C bias of **~350 LSB**, but the bias has a
large part-to-part spread (σ ≈ 290 LSB ≈ 19 °C). So
`T(°C) ≈ 25 + (raw − bias) × 0.065`, where `bias` should ideally be **measured
per device** at a known temperature. The driver should return **raw
temperature** plus a conversion helper taking a caller-supplied bias (defaulting
to 350 LSB), and the docs should warn that absolute temperature is only
trustworthy after calibration.

---

## 7. Feature areas to implement

### 7.1 Identity / bring-up
`new()` issues a soft reset (optional), reads `DEVID_AD`/`PARTID`, and returns
`Error::InvalidDevice` on mismatch. Provides `device_id()`, `part_id()`,
`revision()`.

### 7.2 Operating mode & power
`POWER_CTL` (0x2D): `standby()`, `start_measurement()`, `enter_wakeup()`,
`set_noise_mode(Normal|LowNoise|UltraLowNoise)`, `set_autosleep(bool)`,
`use_external_clock(bool)`. Enforce "configure in Standby" in the high-level API
(see typestate decision in [§10](#10-design-decisions)). Rev. G clarifies
that in wake-up mode every feature stays available **except the activity timer**
(one-sample activity detection is used); registers, live data, and the FIFO are
all still accessible.

### 7.3 Output config
`FILTER_CTL` (0x2C): `set_range(G2|G4|G8)`, `set_output_data_rate(...)`
(12.5/25/50/100/200/400 Hz), `set_filter_bandwidth(Half|Quarter)` (HALF_BW),
`set_external_sampling(bool)` (repurposes INT2). The driver caches the current
range so conversions don't need an extra read.

### 7.4 Reading samples
`read_raw() -> [i16; 3]` (burst of 0x0E–0x13), `read_accel()` (converted),
`read_raw_8bit()`, `read_temperature_raw()`, `data_ready()` (STATUS bit 0).
Note the up-to-**80 µs** latency between a data-register read and DATA_READY
clearing.

### 7.5 Activity / inactivity & free-fall
Threshold + time registers (0x20–0x26), `ACT_INACT_CTL` (0x27). Model:
- `ActivityConfig { threshold_codes, time_samples, reference: Referenced|Absolute }`
- same for inactivity (16-bit time)
- `LinkLoopMode { Default, Linked, Loop }` — **encode per Rev. G** in
  `ACT_INACT_CTL[5:4]`: Default `0b00`, **Linked `0b01`**, Loop `0b11` (and both
  `ACT_EN` + `INACT_EN` must be set for linked/loop to take effect). Note the
  encoding is asymmetric: Table 13 lists Default as `X0`, so bit 4 is really the
  "enable linking" bit and `0b10` also decodes to **Default**, not Loop. The
  practical trap: a value of `0b10` written in the belief that it means Linked
  (or Loop) silently produces *unlinked* default behavior with no error flag. If
  you ever port constants from a pre-production sheet or from memory, re-verify
  this field against Rev. G Table 13. See
  [§13](#13-timing-constants--revision-notes).
- a `free_fall(threshold, time)` convenience built on **absolute inactivity**
  (datasheet recommends 300–600 mg, 100–350 ms).

Thresholds are in **codes**; provide helpers to convert mg ↔ codes using the
current range's scale factor, and time helpers using the current ODR.

### 7.6 FIFO
`FIFO_CONTROL`/`FIFO_SAMPLES` (0x28–0x29), `FIFO_ENTRIES` (0x0C–0x0D), FIFO read
(0x0D). Model `FifoMode { Disabled, OldestSaved, Stream, Triggered }`,
`store_temperature: bool`, watermark sample count (note `AH` is the 9th bit of
the count, spanning two registers). **FIFO parsing is the subtle part:** each
16-bit datum tags its own content in bits [15:14] — `00`=X, `01`=Y, `10`=Z,
`11`=temp. Provide an iterator that yields tagged samples and a helper that
reassembles complete `{x,y,z(,temp)}` sets, tolerant of partial sets at buffer
boundaries.

### 7.7 Interrupts (INT1 / INT2)
`INTMAP1`/`INTMAP2` (0x2A–0x2B): map any of AWAKE, INACT, ACT, FIFO_OVERRUN,
FIFO_WATERMARK, FIFO_READY, DATA_READY to either pin (OR-combined), plus
`active_low`. Provide a `Status` struct parsed from 0x0B and document the
clearing semantics:
- read STATUS → clears ACT / INACT
- read data regs → clears DATA_READY
- read enough FIFO → clears FIFO_READY / WATERMARK / OVERRUN

Recommend disabling interrupts while reconfiguring thresholds to avoid spurious
triggers.

**Pin ownership — DECIDED:** the driver stays HAL-generic and never touches the
MCU interrupt system. Application code owns the GPIO that INT1/INT2 are wired to
and does the `bind_interrupts!` / `ExtiInput` / GPIOTE setup; the driver offers
an optional `async fn wait_for_event(&mut self, int: &mut impl Wait) -> Result<Status>`
that borrows the pin per call, awaits the edge (`wait_for_rising_edge` for the
default active-high config; falling if `INT_LOW` is set), and returns parsed
`Status`. This keeps `bind_interrupts!`/EXTI/GPIOTE entirely on the user side
(it's HAL-specific and not expressible through `embedded-hal`), and leaves INT1/
INT2 free to be repurposed as external-clock / sync-trigger inputs. The `Error`
enum gains a `Pin(E2)` variant since the pin error type differs from the SPI
error type.

### 7.8 Advanced I/O
External clock on INT1 (`EXT_CLK`): **25.6 kHz–51.2 kHz** (51.2 kHz nominal),
with `ODR_actual = ODR_selected × f_clk / 51.2 kHz`; synchronized sampling via
INT2 trigger (`EXT_SAMPLE`, active-high pulse ≥ 25 µs, ≥ 25 µs between pulses,
**max ~625 Hz**). A pin used for an alternate function can't also signal
interrupts — encode that mutual exclusion if practical.

### 7.9 Self-test & reset
`self_test()` implementing the Rev. G routine: configure **±8 g, 100 Hz ODR,
`HALF_BW = 0`**, read baseline → assert ST → **wait 4/ODR** → read → diff →
convert LSB→mg → compare against the per-supply limits (Table 22: X and Z
positive, **Y negative**) → deassert ST. Average 4–16 samples per read, and avoid
ODR < 100 Hz or 100 Hz-with-quarter-bandwidth (self-test response is bimodal
there). `soft_reset()` writes `0x52`, after which the driver must **wait ~0.5 ms**
before further access.

---

## 8. Proposed crate architecture

```
adxl362/
├── Cargo.toml          # features: async (default), float, defmt, accelerometer
├── src/
│   ├── lib.rs          # crate docs, re-exports
│   ├── error.rs        # Error<E> { Spi(E), InvalidDevice, ... }
│   ├── register.rs     # Register enum, command bytes, raw addresses
│   ├── transport.rs    # read/write/burst/fifo primitives (sync + async)
│   ├── config/         # typed bitfield structs per config register
│   │   ├── filter.rs   # Range, OutputDataRate, Bandwidth
│   │   ├── power.rs    # NoiseMode, mode control
│   │   ├── motion.rs   # Activity/Inactivity/LinkLoop
│   │   ├── fifo.rs     # FifoMode, FIFO parsing iterator
│   │   └── interrupt.rs# IntMap, Status, IntPin selection
│   ├── data.rs         # Acceleration, conversions, temperature
│   └── device.rs       # Adxl362<SPI> high-level API
├── examples/           # Embassy example(s): nRF52 or STM32
└── tests/              # embedded-hal-mock transaction tests
```

**Error type:**

```rust
pub enum Error<E> {
    Spi(E),
    InvalidDevice { found_part_id: u8 },
    // possibly: FifoLengthOdd, ConfigWhileMeasuring (if not prevented by types)
}
```

**Async/sync strategy — DECIDED:** single source, both surfaces generated via
`maybe-async-cfg`, `async` on by default. (If the register layer later moves to
`device-driver`, that crate emits both surfaces itself, shrinking what
`maybe-async-cfg` has to cover to just the hand-written high-level methods — see
[§10](#10-design-decisions).)

---

## 9. Public API sketch

```rust
let mut accel = Adxl362::new(spi)?;           // verifies PARTID
accel.soft_reset()?;
accel.set_range(Range::G4)?;                  // configured in Standby
accel.set_output_data_rate(Odr::Hz100)?;
accel.set_noise_mode(NoiseMode::LowNoise)?;
accel.start_measurement()?;

let raw = accel.read_raw()?;                  // [i16; 3]
let g   = accel.read_accel()?;                // feature = "float"

// motion switch, fully autonomous:
accel.configure_activity(ActivityConfig::referenced(threshold, time))?;
accel.configure_inactivity(InactivityConfig::referenced(threshold, time))?;
accel.set_link_loop(LinkLoopMode::Loop)?;
accel.set_autosleep(true)?;
accel.map_interrupts(IntPin::Int2, IntSources::AWAKE)?;

// FIFO drain:
accel.set_fifo(FifoMode::Stream, /*store_temp*/ false, /*watermark*/ 80)?;
for sample in accel.read_fifo_sets(&mut buf)? { /* {x,y,z} */ }
```

Async mirror (Embassy): the same methods are `async fn` under the `async`
feature; an interrupt-driven read might look like
`accel.wait_for_data_ready(&mut int1).await?`.

---

## 10. Design decisions

All five forks are settled.

1. **Async + blocking surface — DECIDED.** Both, single source via
   `maybe-async-cfg`, `async` on by default.

2. **Register layer: hand-rolled vs. `device-driver` — DECIDED (hand-rolled for
   v1).** ~20 registers, the bitfield code is mechanical and instructive, and it
   keeps full control over the burst/FIFO quirks with no extra dependency.
   `device-driver` (declarative manifest → generated typed accessors, emits both
   sync + async) remains a reasonable *later* migration target, but it is not
   adopted now: its manifest format has churned across versions (0.x DSL/YAML/
   JSON/TOML → 1.0 centring on KDL), so adopting early means tracking format
   churn for little v1 benefit. Its caveats for this chip — the **FIFO** falling
   outside the plain register model and **atomic XYZ bursts** needing the
   0x0E–0x13 block modeled as one wide register — are both handled by the crate's
   Buffer/Command and wide-register primitives, so they are migration details,
   not blockers.

   **Why deciding hand-rolled is low-risk:** a later swap to `device-driver` is
   estimated at ~3–5 engineer-days *provided the transport seam stays clean*,
   because device-driver has you implement the bus I/O yourself — the bytes on
   the wire stay identical, so the `embedded-hal-mock` transaction tests don't
   move and become the regression net that proves the migration is
   behavior-preserving. The high-level API, all conversion code, and FIFO parsing
   transfer untouched; only `register.rs` + the bitfield structs are replaced (by
   a manifest + an interface impl), and `maybe-async-cfg` drops away for the
   register layer (device-driver emits both surfaces). To keep that estimate
   valid, the v1 hand-rolled layer **must** observe these constraints:
   - all bus I/O goes through the four transport fns ([§4](#4-spi-transport-layer));
     nothing above transport touches the SPI device directly (these fns map ~1:1
     onto device-driver's interface traits);
   - the XYZ data block (0x0E–0x13) is read as a **single burst** from day one
     (maps to one wide register);
   - public enums (`Range`, `Odr`, …) stay separate from raw register layout,
     with the mapping in one place;
   - tests assert **byte-level transactions**, not internal bitfield-struct shape;
   - FIFO read transport is isolated from FIFO parse (parse stays, transport
     swaps).

   These are good layering regardless, so the constraints cost nothing extra.

3. **Mode safety: runtime vs. typestate — DECIDED.** Runtime for v1
   (`standby()` / `start_measurement()`); typestate
   (`Adxl362<SPI, Standby/Measuring>`) deferred as a possible later enhancement.

4. **Interrupt pin ownership — DECIDED.** Driver stays HAL-generic; application
   owns the GPIO + `bind_interrupts!`/EXTI/GPIOTE; driver offers an optional
   `wait_for_event(&mut self, int: &mut impl Wait)` that borrows the pin per
   call and returns parsed `Status`. See [§7.7](#77-interrupts-int1--int2).

5. **Conversion output — DECIDED.** Integer milli-g always; `f32` g behind the
   `float` feature; `fixed`-point deferred.

---

## 11. Implementation phases

- **Phase 1 — Core & reads.** Transport, `Register`/`Error`, device-ID check,
  range/ODR/bandwidth, Standby/Measurement, raw + 8-bit + converted reads,
  temperature (raw), soft reset. *Deliverable: blink-equivalent — read live g
  values over SPI.*
- **Phase 2 — Motion & interrupts.** Activity/inactivity (abs + ref),
  link/loop, autosleep, wake-up mode, STATUS, INTMAP1/2, free-fall helper.
- **Phase 3 — FIFO.** All four modes, entries count, even-length reads,
  tagged-sample parsing + set reassembly, watermark/overrun handling.
- **Phase 4 — Advanced & polish.** External clock, sync sampling, self-test,
  `accelerometer` trait impl, `defmt`, optional typestate, docs/examples.

Each phase ends with `embedded-hal-mock` tests and (from Phase 1) an Embassy
example that builds for a real target.

---

## 12. Testing & CI

- **Unit:** `embedded-hal-mock` SPI expectations for every register transaction,
  including the sign-extension and FIFO-tag parsing edge cases.
- **Build matrix:** host (`std` test build), plus `thumbv7em-none-eabihf`
  (Embassy STM32/nRF) with `--no-default-features` and with each feature.
- **Lints:** `cargo fmt --check`, `cargo clippy -- -D warnings`.
- **Docs:** `#![deny(missing_docs)]`, doctests where feasible.
- **Examples:** at least one Embassy example (suggest nRF52840 or STM32) read in
  CI as build-only.

---

## 13. Timing constants & revision notes

Built against **Rev. G (May 2023)**, the production datasheet — every value the
preliminary sheet left as TBD is now final. Concrete constants for the driver to
encode:

- **SPI:** mode 0; fCLK up to **8 MHz** (with a **1 MHz floor when using the
  FIFO**).
- **Power-up → Standby:** ~**5 ms**.
- **Measurement-mode instruction → first valid data:** **4 / ODR**.
- **Soft reset:** wait ~**0.5 ms** after writing `0x52`.
- **Data-ready clear latency:** up to **80 µs** after a data-register read.
- **±8 g scale factor:** **235 LSB/g** (4.255 mg/LSB), not 250 — see
  [§6](#6-data-formats--conversions).
- **Temperature:** 0.065 °C/LSB, ~350 LSB bias @ 25 °C, large part-to-part
  spread (calibrate per device).
- **Self-test:** ±8 g / 100 Hz / `HALF_BW=0`, settle **4/ODR**, Y-axis delta is
  negative.
- **External clock:** 25.6–51.2 kHz. **Sync sampling:** max ~625 Hz.

**`LINK/LOOP` encoding — verify against Rev. G (important).** The correct Rev. G
encoding of `ACT_INACT_CTL[5:4]` (Table 13, p.31) is Default `0b00`, **Linked
`0b01`**, Loop `0b11`, with Default formally listed as `X0` — so `0b10` *also*
decodes to Default. This is the field most likely to be miswritten: a `0b10`
intended as Linked or Loop silently yields unlinked default behavior with no
error. Both `ACT_EN` (bit 0) and `INACT_EN` (bit 2) must additionally be set for
linked/loop to engage.

> **Unverified cross-revision claim — confirm before relying on it.** Earlier
> drafts asserted that a *preliminary* datasheet encoded Linked as `0b10` and
> used `ACT_INACT_CTL = 0x0C` (which is *referenced* inactivity under Rev. G bit
> definitions) in its free-fall example, versus Rev. G's `0x04` (absolute
> inactivity, confirmed on p.39). The Rev. G values here are verified against the
> production datasheet; the statements about the preliminary sheet are **not**
> verifiable from Rev. G alone and should be cited to a specific earlier revision
> (and page) or dropped. They are *plausible* — the `X0`-decodes-to-Default quirk
> above makes a `0b10`-means-Linked port a real silent failure — but treat them
> as a caution, not an established fact, until the source revision is in hand.

The Rev. G free-fall example uses `ACT_INACT_CTL = 0x04` (absolute inactivity);
that value is confirmed on p.39 and is what the driver's `free_fall()` helper
should emit.

**Power-cycling note (only if firmware controls a supply-enable GPIO).** Rev. G
adds a hard `VRESET` requirement: to restart cleanly the supplies must be
discharged to ≤ 100 mV and held there ≥ 200 ms, then rise **linearly to 1.6 V
within 250 µs** (worst case at `VRESET` = 100 mV / 200 ms hold; a full discharge
to 0 V relaxes the rise budget to ≤ 600 µs). This is board/firmware-level, not
register-level, but relevant if the driver or its host ever software-power-cycles
the part.

---

## 14. References

- ADXL362 Data Sheet, Rev. G, Analog Devices (May 2023).
- AN-1025 — FIFO utilization in ADI digital accelerometers (FIFO patterns).
- `embedded-hal` / `embedded-hal-async` 1.0 SPI + digital traits.
- `embedded-hal-mock`, `maybe-async-cfg`, `device-driver`, `accelerometer`
  crates (ecosystem options referenced above).
