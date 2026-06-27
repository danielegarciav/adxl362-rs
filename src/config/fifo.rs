//! FIFO configuration and data parsing.

/// FIFO operating mode (FIFO_CONTROL bits `[1:0]`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FifoMode {
    /// FIFO disabled.
    Disabled = 0b00,
    /// Oldest-saved mode: stops collecting when full; oldest samples preserved.
    OldestSaved = 0b01,
    /// Stream mode: newest sample overwrites oldest when full.
    Stream = 0b10,
    /// Triggered mode: collects samples around a trigger event.
    Triggered = 0b11,
}

/// Axis/source tag embedded in each 16-bit FIFO word (bits `[15:14]`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FifoTag {
    /// X-axis sample.
    X = 0b00,
    /// Y-axis sample.
    Y = 0b01,
    /// Z-axis sample.
    Z = 0b10,
    /// Temperature sample.
    Temp = 0b11,
}

/// One parsed FIFO sample: a tag and its 12-bit signed value.
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FifoSample {
    /// Axis or source that produced this sample.
    pub tag: FifoTag,
    /// Sign-extended 12-bit value from the FIFO word.
    pub value: i16,
}

impl FifoSample {
    /// Parse one 16-bit FIFO word (LSB-first as read from the bus).
    ///
    /// Bits `[15:14]` = tag; bits `[11:0]` = 12-bit signed value (bits `[13:12]` are
    /// sign extension of bit 11 inserted by the device).
    #[inline]
    pub fn from_raw_word(word: u16) -> Self {
        let tag_bits = (word >> 14) & 0x03;
        let tag = match tag_bits {
            0b00 => FifoTag::X,
            0b01 => FifoTag::Y,
            0b10 => FifoTag::Z,
            _ => FifoTag::Temp,
        };
        // Sign-extend the 12-bit value to i16 via i32 to avoid i16 overflow.
        let raw12 = (word & 0x0FFF) as i32;
        let value = ((raw12 << 20) >> 20) as i16;
        FifoSample { tag, value }
    }
}

/// A complete XYZ (and optional temperature) sample set reassembled from FIFO words.
#[derive(Clone, Copy, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FifoSampleSet {
    /// X-axis raw value, or `None` if not yet received.
    pub x: Option<i16>,
    /// Y-axis raw value, or `None` if not yet received.
    pub y: Option<i16>,
    /// Z-axis raw value, or `None` if not yet received.
    pub z: Option<i16>,
    /// Temperature raw value, or `None` if not stored or not yet received.
    pub temp: Option<i16>,
}

impl FifoSampleSet {
    /// Returns `true` when at least X, Y, and Z are populated.
    #[inline]
    pub fn is_complete(&self) -> bool {
        self.x.is_some() && self.y.is_some() && self.z.is_some()
    }
}

/// Parse a raw byte slice from a FIFO burst read into an iterator of [`FifoSample`]s.
///
/// `buf` must be even-length (the driver enforces this at the transport layer).
pub fn parse_fifo_samples(buf: &[u8]) -> impl Iterator<Item = FifoSample> + '_ {
    buf.chunks_exact(2).map(|chunk| {
        let word = u16::from_le_bytes([chunk[0], chunk[1]]);
        FifoSample::from_raw_word(word)
    })
}

/// Reassemble a slice of [`FifoSample`]s into complete [`FifoSampleSet`]s.
///
/// Partial sets at buffer boundaries are dropped; a new set starts on the next
/// X sample after a complete set is emitted.  If no X sample is seen before a
/// complete set of the same axis appears, the later value overwrites the earlier.
pub fn reassemble_sample_sets(samples: &[FifoSample]) -> impl Iterator<Item = FifoSampleSet> + '_ {
    SampleSetIter { samples, pos: 0 }
}

struct SampleSetIter<'a> {
    samples: &'a [FifoSample],
    pos: usize,
}

impl<'a> Iterator for SampleSetIter<'a> {
    type Item = FifoSampleSet;

    fn next(&mut self) -> Option<Self::Item> {
        // Skip until we find an X sample or exhaust the buffer.
        while self.pos < self.samples.len() && self.samples[self.pos].tag != FifoTag::X {
            self.pos += 1;
        }
        if self.pos >= self.samples.len() {
            return None;
        }

        let mut set = FifoSampleSet::default();
        while self.pos < self.samples.len() {
            let s = self.samples[self.pos];
            match s.tag {
                FifoTag::X => {
                    if set.x.is_some() {
                        // Started a new X without completing previous set — emit what we have.
                        if set.is_complete() {
                            return Some(set);
                        }
                        set = FifoSampleSet::default();
                    }
                    set.x = Some(s.value);
                }
                FifoTag::Y => set.y = Some(s.value),
                FifoTag::Z => set.z = Some(s.value),
                FifoTag::Temp => set.temp = Some(s.value),
            }
            self.pos += 1;
            if set.is_complete() {
                // Peek ahead: if the next sample is not temp, emit now.
                let next_is_temp = self
                    .samples
                    .get(self.pos)
                    .map(|s| s.tag == FifoTag::Temp)
                    .unwrap_or(false);
                if !next_is_temp {
                    return Some(set);
                }
            }
        }
        if set.is_complete() { Some(set) } else { None }
    }
}
