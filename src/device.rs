//! High-level `Adxl362` driver struct.

#[cfg(feature = "blocking")]
use embedded_hal::delay::DelayNs;
#[cfg(feature = "blocking")]
use embedded_hal::spi::SpiDevice;

#[cfg(not(feature = "blocking"))]
use embedded_hal_async::delay::DelayNs;
#[cfg(not(feature = "blocking"))]
use embedded_hal_async::spi::SpiDevice;

use crate::config::fifo::{self, FifoMode, FifoSample, FifoSampleSet};
use crate::config::filter::{FilterBandwidth, OutputDataRate, Range};
use crate::config::interrupt::{IntPin, IntSources, Status};
use crate::config::motion::{ActivityConfig, InactivityConfig, LinkLoopMode};
use crate::config::power::{MeasureBits, NoiseMode};
use crate::data::{AccelMg, RawAccel, RawAccel8, Temperature};
use crate::error::Error;
use crate::register::{DEVID_AD_VALUE, PARTID_VALUE, Register, SOFT_RESET_KEY};
use crate::transport;

/// ADXL362 driver, generic over the SPI device.
///
/// Instantiate with [`Adxl362::new`].  The device powers up in Standby; call
/// [`start_measurement`](Self::start_measurement) before reading samples.
#[derive(Debug)]
pub struct Adxl362<SPI> {
    spi: SPI,
    /// Cached measurement range — avoids an extra SPI read on every conversion.
    range: Range,
    /// Cached ODR — used by threshold/time helper conversions.
    odr: OutputDataRate,
}

// ── Pure (non-SPI) accessors ─────────────────────────────────────────────────

impl<SPI> Adxl362<SPI> {
    /// Current measurement range cached from the last `set_range` call.
    #[inline]
    pub fn range(&self) -> Range {
        self.range
    }

    /// Current output data rate cached from the last `set_output_data_rate` call.
    #[inline]
    pub fn odr(&self) -> OutputDataRate {
        self.odr
    }
}

// ── SPI methods — single source for sync + async via maybe-async-cfg ─────────

#[maybe_async::maybe_async]
impl<SPI> Adxl362<SPI>
where
    SPI: SpiDevice,
{
    // ── Construction & identity ─────────────────────────────────────────────

    /// Create a new driver instance and verify the device identity.
    ///
    /// Reads `DEVID_AD` (must be `0xAD`) and `PARTID` (must be `0xF2`).
    /// Does **not** issue a soft-reset; call [`soft_reset`](Self::soft_reset)
    /// first if you need a clean-slate start.
    pub async fn new(spi: SPI) -> Result<Self, Error<SPI::Error>> {
        let mut dev = Adxl362 {
            spi,
            range: Range::G2,
            odr: OutputDataRate::Hz100,
        };
        let devid = transport::read_register(&mut dev.spi, Register::DevIdAd).await?;
        let partid = transport::read_register(&mut dev.spi, Register::PartId).await?;
        if devid != DEVID_AD_VALUE || partid != PARTID_VALUE {
            return Err(Error::InvalidDevice {
                found_part_id: partid,
            });
        }
        Ok(dev)
    }

    /// Read the silicon revision register (`REVID`, 0x03).
    pub async fn revision(&mut self) -> Result<u8, Error<SPI::Error>> {
        transport::read_register(&mut self.spi, Register::RevId).await
    }

    // ── Soft reset ──────────────────────────────────────────────────────────

    /// Write the soft-reset key (`0x52`) to `SOFT_RESET`.
    ///
    /// The caller **must** wait ≥ 0.5 ms before any further register access
    /// (Rev. G §13).  The driver does not apply the delay itself to remain
    /// timer-agnostic; use `delay.delay_ms(1)` after this call.
    pub async fn soft_reset(&mut self) -> Result<(), Error<SPI::Error>> {
        transport::write_register(&mut self.spi, Register::SoftReset, SOFT_RESET_KEY).await
    }

    // ── Power control ───────────────────────────────────────────────────────

    /// Put the device into Standby (~10 nA, all sensing off).
    ///
    /// Configuration registers must only be modified in Standby.
    pub async fn standby(&mut self) -> Result<(), Error<SPI::Error>> {
        self.modify_power_ctl(|v| (v & !0x03) | MeasureBits::Standby as u8)
            .await
    }

    /// Start continuous measurement mode.
    ///
    /// Allow 4/ODR after this call before the first valid sample appears.
    pub async fn start_measurement(&mut self) -> Result<(), Error<SPI::Error>> {
        self.modify_power_ctl(|v| (v & !0x0B) | MeasureBits::Measurement as u8)
            .await
    }

    /// Enter wake-up mode (~6 Hz motion check, ~270 nA).
    ///
    /// Sets MEASURE = 0b10 **and** WAKEUP bit.  The activity timer is not
    /// available in wake-up mode (one-sample detection is used); live data
    /// registers and the FIFO remain accessible.
    pub async fn enter_wakeup(&mut self) -> Result<(), Error<SPI::Error>> {
        self.modify_power_ctl(|v| (v & !0x0B) | MeasureBits::Measurement as u8 | 0x08)
            .await
    }

    /// Set the noise / current trade-off mode.
    pub async fn set_noise_mode(&mut self, mode: NoiseMode) -> Result<(), Error<SPI::Error>> {
        self.modify_power_ctl(|v| (v & !0x30) | ((mode as u8) << 4))
            .await
    }

    /// Enable or disable autosleep between activity and inactivity events.
    ///
    /// Requires linked or loop mode with both `ACT_EN` and `INACT_EN` set.
    pub async fn set_autosleep(&mut self, enable: bool) -> Result<(), Error<SPI::Error>> {
        self.modify_power_ctl(|v| if enable { v | 0x04 } else { v & !0x04 })
            .await
    }

    /// Enable or disable the external clock on INT1 (25.6–51.2 kHz nominal).
    ///
    /// When enabled, INT1 becomes a clock input and cannot simultaneously
    /// assert interrupts.
    pub async fn use_external_clock(&mut self, enable: bool) -> Result<(), Error<SPI::Error>> {
        self.modify_power_ctl(|v| if enable { v | 0x40 } else { v & !0x40 })
            .await
    }

    // ── Output configuration ────────────────────────────────────────────────

    /// Set the measurement range.  Updates the internal cache for conversions.
    ///
    /// Call while in Standby.
    pub async fn set_range(&mut self, range: Range) -> Result<(), Error<SPI::Error>> {
        self.modify_filter_ctl(|v| (v & !0xC0) | ((range as u8) << 6))
            .await?;
        self.range = range;
        Ok(())
    }

    /// Set the output data rate.  Updates the internal cache.
    ///
    /// Call while in Standby.
    pub async fn set_output_data_rate(
        &mut self,
        odr: OutputDataRate,
    ) -> Result<(), Error<SPI::Error>> {
        self.modify_filter_ctl(|v| (v & !0x07) | odr as u8).await?;
        self.odr = odr;
        Ok(())
    }

    /// Set the anti-aliasing filter bandwidth.
    ///
    /// `Quarter` (HALF_BW = 1) halves the bandwidth to ODR/4.  Avoid
    /// `Quarter` at ODR < 100 Hz during self-test.
    pub async fn set_filter_bandwidth(
        &mut self,
        bw: FilterBandwidth,
    ) -> Result<(), Error<SPI::Error>> {
        self.modify_filter_ctl(|v| {
            if bw.half_bw_bit() {
                v | 0x10
            } else {
                v & !0x10
            }
        })
        .await
    }

    /// Enable or disable synchronized external sampling on INT2.
    ///
    /// When enabled, INT2 acts as an active-high sample-trigger input (pulse
    /// ≥ 25 µs, ≥ 25 µs between pulses, max ~625 Hz) and cannot simultaneously
    /// signal interrupts.
    pub async fn set_external_sampling(&mut self, enable: bool) -> Result<(), Error<SPI::Error>> {
        self.modify_filter_ctl(|v| if enable { v | 0x08 } else { v & !0x08 })
            .await
    }

    // ── Status & data reads ─────────────────────────────────────────────────

    /// Read and parse the STATUS register.
    ///
    /// Reading STATUS clears the `act` and `inact` flags.  See [`Status`] for
    /// the clearing semantics of all flags.
    pub async fn read_status(&mut self) -> Result<Status, Error<SPI::Error>> {
        let byte = transport::read_register(&mut self.spi, Register::Status).await?;
        Ok(Status::from_byte(byte))
    }

    /// Return `true` when DATA_READY is set in STATUS.
    ///
    /// DATA_READY clears up to 80 µs after reading the data registers (Rev. G).
    pub async fn data_ready(&mut self) -> Result<bool, Error<SPI::Error>> {
        Ok(self.read_status().await?.data_ready)
    }

    /// Burst-read all six 12-bit data registers (0x0E–0x13) in one SPI transaction.
    ///
    /// The device already sign-extends each axis into the high byte, so
    /// `i16::from_le_bytes` gives the signed value directly.
    pub async fn read_raw(&mut self) -> Result<RawAccel, Error<SPI::Error>> {
        let mut buf = [0u8; 6];
        transport::read_registers(&mut self.spi, Register::XDataL, &mut buf).await?;
        Ok(RawAccel::from_bytes(&buf))
    }

    /// Read and convert to integer milli-g using the cached range.
    pub async fn read_accel_mg(&mut self) -> Result<AccelMg, Error<SPI::Error>> {
        Ok(self.read_raw().await?.to_mg(self.range))
    }

    /// Read and convert to `f32` g.  Requires the `float` feature.
    #[cfg(feature = "float")]
    pub async fn read_accel_g(&mut self) -> Result<crate::data::AccelG, Error<SPI::Error>> {
        Ok(self.read_raw().await?.to_g(self.range))
    }

    /// Fast 8-bit-per-axis read from 0x08–0x0A (MSBs only, ≈ `raw >> 4`).
    pub async fn read_raw_8bit(&mut self) -> Result<RawAccel8, Error<SPI::Error>> {
        let mut buf = [0u8; 3];
        transport::read_registers(&mut self.spi, Register::XData, &mut buf).await?;
        Ok(RawAccel8 {
            x: buf[0] as i8,
            y: buf[1] as i8,
            z: buf[2] as i8,
        })
    }

    /// Read the raw 12-bit temperature value.
    ///
    /// See [`Temperature`] for conversion helpers.  Absolute accuracy requires
    /// per-device calibration (bias σ ≈ 19 °C, Rev. G).
    pub async fn read_temperature_raw(&mut self) -> Result<Temperature, Error<SPI::Error>> {
        let mut buf = [0u8; 2];
        transport::read_registers(&mut self.spi, Register::TempL, &mut buf).await?;
        Ok(Temperature {
            raw: i16::from_le_bytes([buf[0], buf[1]]),
        })
    }

    // ── Activity / inactivity detection ─────────────────────────────────────

    /// Configure the activity detector.
    pub async fn configure_activity(
        &mut self,
        cfg: ActivityConfig,
    ) -> Result<(), Error<SPI::Error>> {
        let thresh = cfg.threshold_codes & 0x7FF;
        transport::write_register(&mut self.spi, Register::ThreshActL, (thresh & 0xFF) as u8)
            .await?;
        transport::write_register(&mut self.spi, Register::ThreshActH, (thresh >> 8) as u8).await?;
        transport::write_register(&mut self.spi, Register::TimeAct, cfg.time_samples).await?;
        self.modify_act_inact_ctl(|v| {
            let v = v | 0x01; // ACT_EN
            if cfg.referenced { v | 0x02 } else { v & !0x02 }
        })
        .await
    }

    /// Configure the inactivity (no-motion) detector.
    pub async fn configure_inactivity(
        &mut self,
        cfg: InactivityConfig,
    ) -> Result<(), Error<SPI::Error>> {
        let thresh = cfg.threshold_codes & 0x7FF;
        transport::write_register(&mut self.spi, Register::ThreshInactL, (thresh & 0xFF) as u8)
            .await?;
        transport::write_register(&mut self.spi, Register::ThreshInactH, (thresh >> 8) as u8)
            .await?;
        transport::write_register(
            &mut self.spi,
            Register::TimeInactL,
            (cfg.time_samples & 0xFF) as u8,
        )
        .await?;
        transport::write_register(
            &mut self.spi,
            Register::TimeInactH,
            (cfg.time_samples >> 8) as u8,
        )
        .await?;
        self.modify_act_inact_ctl(|v| {
            let v = v | 0x04; // INACT_EN
            if cfg.referenced { v | 0x08 } else { v & !0x08 }
        })
        .await
    }

    /// Set the link/loop sequencing mode.
    ///
    /// Encoding (Rev. G Table 13): Default `0b00`, Linked `0b01`, Loop `0b11`.
    /// Value `0b10` silently decodes as Default — never write it intentionally.
    /// Both `ACT_EN` and `INACT_EN` must be set for Linked/Loop to engage.
    pub async fn set_link_loop(&mut self, mode: LinkLoopMode) -> Result<(), Error<SPI::Error>> {
        self.modify_act_inact_ctl(|v| (v & !0x30) | ((mode as u8) << 4))
            .await
    }

    /// Configure free-fall detection via absolute inactivity.
    ///
    /// Rev. G p.39 recommends 300–600 mg threshold, 100–350 ms time.
    /// Sets `ACT_INACT_CTL = 0x04` (absolute inactivity only, per p.39).
    pub async fn configure_free_fall(
        &mut self,
        threshold_mg: u32,
        time_ms: u32,
    ) -> Result<(), Error<SPI::Error>> {
        let thresh = self.range.mg_to_threshold_code(threshold_mg);
        let time_samples = self.odr.ms_to_samples(time_ms).min(0xFFFF) as u16;
        self.configure_inactivity(InactivityConfig::absolute(thresh, time_samples))
            .await?;
        // Clear ACT_EN and ACT_REF to match the Rev. G reference value of 0x04.
        self.modify_act_inact_ctl(|v| v & !0x03).await
    }

    // ── FIFO ────────────────────────────────────────────────────────────────

    /// Configure the FIFO.
    ///
    /// `watermark` is a 9-bit sample count (0–512); the 9th bit (AH) is stored
    /// in FIFO_CONTROL bit 3.
    pub async fn set_fifo(
        &mut self,
        mode: FifoMode,
        store_temp: bool,
        watermark: u16,
    ) -> Result<(), Error<SPI::Error>> {
        let watermark = watermark.min(512);
        let ah = ((watermark >> 8) & 1) as u8;
        let fifo_samples = (watermark & 0xFF) as u8;
        let fifo_ctl = (mode as u8) | ((store_temp as u8) << 2) | (ah << 3);
        transport::write_register(&mut self.spi, Register::FifoControl, fifo_ctl).await?;
        transport::write_register(&mut self.spi, Register::FifoSamples, fifo_samples).await
    }

    /// Read the current FIFO entry count (0–512).
    pub async fn fifo_entries(&mut self) -> Result<u16, Error<SPI::Error>> {
        let mut buf = [0u8; 2];
        transport::read_registers(&mut self.spi, Register::FifoEntriesL, &mut buf).await?;
        Ok(u16::from_le_bytes([buf[0], buf[1]]))
    }

    /// Read raw bytes from the FIFO into `buf`.  `buf.len()` must be even.
    pub async fn read_fifo_raw(&mut self, buf: &mut [u8]) -> Result<(), Error<SPI::Error>> {
        transport::read_fifo(&mut self.spi, buf).await
    }

    /// Read FIFO data and return a tagged-sample iterator over `buf`.
    ///
    /// `buf` must be even-length.  Iterate with [`FifoSample`] and its `tag`/
    /// `value` fields, or pass to [`fifo::reassemble_sample_sets`].
    pub async fn read_fifo_samples<'b>(
        &mut self,
        buf: &'b mut [u8],
    ) -> Result<impl Iterator<Item = FifoSample> + 'b, Error<SPI::Error>> {
        transport::read_fifo(&mut self.spi, buf).await?;
        Ok(fifo::parse_fifo_samples(buf))
    }

    /// Read FIFO data and return an iterator of complete XYZ(+temp) sets.
    ///
    /// Partial sets at buffer boundaries are silently dropped.
    pub async fn read_fifo_sets<'b>(
        &mut self,
        buf: &'b mut [u8],
    ) -> Result<impl Iterator<Item = FifoSampleSet> + 'b, Error<SPI::Error>> {
        transport::read_fifo(&mut self.spi, buf).await?;
        Ok(FifoSetIter::new(buf))
    }

    // ── Interrupt mapping ───────────────────────────────────────────────────

    /// Map interrupt sources to a pin.
    pub async fn map_interrupts(
        &mut self,
        pin: IntPin,
        sources: IntSources,
    ) -> Result<(), Error<SPI::Error>> {
        let reg = match pin {
            IntPin::Int1 => Register::IntMap1,
            IntPin::Int2 => Register::IntMap2,
        };
        transport::write_register(&mut self.spi, reg, sources.to_register_byte()).await
    }

    // ── Self-test ───────────────────────────────────────────────────────────

    /// Run the Rev. G self-test sequence.
    ///
    /// Temporarily reconfigures to ±8 g / 100 Hz / full bandwidth, measures a
    /// baseline, asserts the self-test bit, waits 4/ODR = 40 ms via `delay`,
    /// measures deflection, then restores original range and ODR.
    ///
    /// **Accept/reject limits** are in Rev. G Table 22: X and Z deltas must be
    /// positive, Y delta must be **negative**.
    pub async fn self_test<D: DelayNs>(
        &mut self,
        delay: &mut D,
        samples: u8,
    ) -> Result<SelfTestResult, Error<SPI::Error>> {
        let samples = samples.clamp(4, 16) as u32;

        let saved_range = self.range;
        let saved_odr = self.odr;

        self.set_range(Range::G8).await?;
        self.set_output_data_rate(OutputDataRate::Hz100).await?;
        self.set_filter_bandwidth(FilterBandwidth::Half).await?; // HALF_BW = 0

        let baseline = self.average_raw(samples).await?;

        transport::write_register(&mut self.spi, Register::SelfTest, 0x01).await?;
        delay.delay_ms(40).await; // 4 / ODR = 40 ms at 100 Hz (Rev. G §7.9)
        let deflected = self.average_raw(samples).await?;

        transport::write_register(&mut self.spi, Register::SelfTest, 0x00).await?;

        self.set_range(saved_range).await?;
        self.set_output_data_rate(saved_odr).await?;

        Ok(SelfTestResult {
            dx_mg: Range::G8.raw_to_mg(deflected.x) - Range::G8.raw_to_mg(baseline.x),
            dy_mg: Range::G8.raw_to_mg(deflected.y) - Range::G8.raw_to_mg(baseline.y),
            dz_mg: Range::G8.raw_to_mg(deflected.z) - Range::G8.raw_to_mg(baseline.z),
        })
    }

    // ── Private helpers ─────────────────────────────────────────────────────

    async fn modify_power_ctl<F>(&mut self, f: F) -> Result<(), Error<SPI::Error>>
    where
        F: FnOnce(u8) -> u8,
    {
        let cur = transport::read_register(&mut self.spi, Register::PowerCtl).await?;
        transport::write_register(&mut self.spi, Register::PowerCtl, f(cur)).await
    }

    async fn modify_filter_ctl<F>(&mut self, f: F) -> Result<(), Error<SPI::Error>>
    where
        F: FnOnce(u8) -> u8,
    {
        let cur = transport::read_register(&mut self.spi, Register::FilterCtl).await?;
        transport::write_register(&mut self.spi, Register::FilterCtl, f(cur)).await
    }

    async fn modify_act_inact_ctl<F>(&mut self, f: F) -> Result<(), Error<SPI::Error>>
    where
        F: FnOnce(u8) -> u8,
    {
        let cur = transport::read_register(&mut self.spi, Register::ActInactCtl).await?;
        // Mask off bits [7:6] (unused RW) to avoid relying on their reset state.
        let masked = cur & 0x3F;
        transport::write_register(&mut self.spi, Register::ActInactCtl, f(masked) & 0x3F).await
    }

    async fn average_raw(&mut self, samples: u32) -> Result<RawAccel, Error<SPI::Error>> {
        let (mut sx, mut sy, mut sz) = (0i32, 0i32, 0i32);
        for _ in 0..samples {
            let r = self.read_raw().await?;
            sx += r.x as i32;
            sy += r.y as i32;
            sz += r.z as i32;
        }
        Ok(RawAccel {
            x: (sx / samples as i32) as i16,
            y: (sy / samples as i32) as i16,
            z: (sz / samples as i32) as i16,
        })
    }
}

// ── Async-only extras ─────────────────────────────────────────────────────────

#[cfg(not(feature = "blocking"))]
impl<SPI> Adxl362<SPI>
where
    SPI: embedded_hal_async::spi::SpiDevice,
{
    /// Await an edge on a GPIO pin wired to INT1 or INT2, then return parsed STATUS.
    ///
    /// The application owns the GPIO (and does its own `bind_interrupts!` / EXTI /
    /// GPIOTE setup); the driver borrows the pin per call only.  For default
    /// active-high polarity the pin should be awaited on a rising edge; configure
    /// active-low polarity via [`IntSources::active_low`] and await a falling edge.
    pub async fn wait_for_event<P>(&mut self, pin: &mut P) -> Result<Status, Error<SPI::Error>>
    where
        P: embedded_hal_async::digital::Wait,
    {
        let _ = pin.wait_for_rising_edge().await; // pin errors discarded (usually Infallible)
        self.read_status().await
    }
}

// ── Return types ─────────────────────────────────────────────────────────────

/// Milli-g deltas returned by [`Adxl362::self_test`].
///
/// Per Rev. G Table 22: `dx_mg` and `dz_mg` must be positive, `dy_mg` negative.
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SelfTestResult {
    /// X-axis delta in milli-g (must be positive per Rev. G Table 22).
    pub dx_mg: i32,
    /// Y-axis delta in milli-g (must be **negative** per Rev. G Table 22).
    pub dy_mg: i32,
    /// Z-axis delta in milli-g (must be positive per Rev. G Table 22).
    pub dz_mg: i32,
}

// ── FIFO set iterator (stack-allocated, no heap) ──────────────────────────────

/// Iterates over complete [`FifoSampleSet`]s from a raw FIFO byte buffer.
///
/// Parses on-the-fly without heap allocation.  Partial sets at buffer
/// boundaries are dropped.
struct FifoSetIter<'a> {
    buf: &'a [u8],
    pos: usize,
    pending: FifoSampleSet,
    has_x: bool,
}

impl<'a> FifoSetIter<'a> {
    fn new(buf: &'a [u8]) -> Self {
        Self {
            buf,
            pos: 0,
            pending: FifoSampleSet::default(),
            has_x: false,
        }
    }
}

impl<'a> Iterator for FifoSetIter<'a> {
    type Item = FifoSampleSet;

    fn next(&mut self) -> Option<Self::Item> {
        use crate::config::fifo::{FifoSample, FifoTag};

        loop {
            if self.pos + 2 > self.buf.len() {
                return if self.has_x && self.pending.is_complete() {
                    self.has_x = false;
                    Some(self.pending)
                } else {
                    None
                };
            }

            let word = u16::from_le_bytes([self.buf[self.pos], self.buf[self.pos + 1]]);
            self.pos += 2;
            let s = FifoSample::from_raw_word(word);

            match s.tag {
                FifoTag::X => {
                    if self.has_x && self.pending.is_complete() {
                        // Emit previous complete set; back up to re-process this X.
                        self.pos -= 2;
                        self.has_x = false;
                        return Some(self.pending);
                    }
                    self.pending = FifoSampleSet::default();
                    self.pending.x = Some(s.value);
                    self.has_x = true;
                }
                FifoTag::Y => {
                    if self.has_x {
                        self.pending.y = Some(s.value);
                    }
                }
                FifoTag::Z => {
                    if self.has_x {
                        self.pending.z = Some(s.value);
                    }
                }
                FifoTag::Temp => {
                    if self.has_x {
                        self.pending.temp = Some(s.value);
                    }
                }
            }

            if self.has_x && self.pending.is_complete() {
                // Peek: if next word is temp, absorb it before emitting.
                let next_tag = self
                    .buf
                    .get(self.pos..self.pos + 2)
                    .map(|c| (u16::from_le_bytes([c[0], c[1]]) >> 14) & 0x03);
                if next_tag != Some(0x03) {
                    // 0x03 = Temp tag bits
                    self.has_x = false;
                    return Some(self.pending);
                }
            }
        }
    }
}
