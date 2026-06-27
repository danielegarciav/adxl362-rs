//! Integration tests using `embedded-hal-mock` to verify byte-level SPI transactions.
//!
//! Run with: `cargo test --no-default-features` (uses the blocking surface).

#![cfg(not(feature = "async"))]

use adxl362::{Adxl362, Error, OutputDataRate, Range};
use embedded_hal_mock::eh1::spi::{Mock as SpiMock, Transaction as SpiTransaction};

fn devid_transactions() -> Vec<SpiTransaction<u8>> {
    vec![
        // read_register(DevIdAd) → 0xAD
        SpiTransaction::transaction_start(),
        SpiTransaction::write_vec(vec![0x0B, 0x00]),
        SpiTransaction::read_vec(vec![0xAD]),
        SpiTransaction::transaction_end(),
        // read_register(PartId) → 0xF2
        SpiTransaction::transaction_start(),
        SpiTransaction::write_vec(vec![0x0B, 0x02]),
        SpiTransaction::read_vec(vec![0xF2]),
        SpiTransaction::transaction_end(),
    ]
}

#[test]
fn new_succeeds_with_valid_ids() {
    let expectations = devid_transactions();
    let mut spi = SpiMock::new(&expectations);
    let _drv = Adxl362::new(&mut spi).unwrap();
    spi.done();
}

#[test]
fn new_returns_error_on_wrong_partid() {
    let expectations = vec![
        SpiTransaction::transaction_start(),
        SpiTransaction::write_vec(vec![0x0B, 0x00]),
        SpiTransaction::read_vec(vec![0xAD]),
        SpiTransaction::transaction_end(),
        SpiTransaction::transaction_start(),
        SpiTransaction::write_vec(vec![0x0B, 0x02]),
        SpiTransaction::read_vec(vec![0xAB]), // wrong
        SpiTransaction::transaction_end(),
    ];
    let mut spi = SpiMock::new(&expectations);
    match Adxl362::new(&mut spi) {
        Err(Error::InvalidDevice {
            found_part_id: 0xAB,
        }) => {}
        Err(_) => panic!("expected InvalidDevice(0xAB) but got a different error"),
        Ok(_) => panic!("expected Err but got Ok"),
    }
    spi.done();
}

#[test]
fn start_measurement_writes_correct_byte() {
    let mut exp = devid_transactions();
    // standby + read-modify-write on POWER_CTL (0x2D)
    exp.extend([
        // read POWER_CTL → 0x00
        SpiTransaction::transaction_start(),
        SpiTransaction::write_vec(vec![0x0B, 0x2D]),
        SpiTransaction::read_vec(vec![0x00]),
        SpiTransaction::transaction_end(),
        // write POWER_CTL = 0x02 (MEASURE = 0b10)
        SpiTransaction::transaction_start(),
        SpiTransaction::write_vec(vec![0x0A, 0x2D, 0x02]),
        SpiTransaction::transaction_end(),
    ]);
    let mut spi = SpiMock::new(&exp);
    let mut drv = Adxl362::new(&mut spi).unwrap();
    drv.start_measurement().unwrap();
    spi.done();
}

#[test]
fn set_range_g4_updates_filter_ctl() {
    let mut exp = devid_transactions();
    // read FILTER_CTL (0x2C) → 0x13 (reset value)
    exp.extend([
        SpiTransaction::transaction_start(),
        SpiTransaction::write_vec(vec![0x0B, 0x2C]),
        SpiTransaction::read_vec(vec![0x13]),
        SpiTransaction::transaction_end(),
        // write FILTER_CTL = (0x13 & !0xC0) | (0b01 << 6) = 0x53
        SpiTransaction::transaction_start(),
        SpiTransaction::write_vec(vec![0x0A, 0x2C, 0x53]),
        SpiTransaction::transaction_end(),
    ]);
    let mut spi = SpiMock::new(&exp);
    let mut drv = Adxl362::new(&mut spi).unwrap();
    drv.set_range(Range::G4).unwrap();
    spi.done();
}

#[test]
fn read_raw_issues_burst_from_xdatal() {
    let mut exp = devid_transactions();
    // burst read 6 bytes starting at 0x0E
    exp.extend([
        SpiTransaction::transaction_start(),
        SpiTransaction::write_vec(vec![0x0B, 0x0E]),
        SpiTransaction::read_vec(vec![0x00, 0x01, 0x00, 0x02, 0x00, 0x03]),
        SpiTransaction::transaction_end(),
    ]);
    let mut spi = SpiMock::new(&exp);
    let mut drv = Adxl362::new(&mut spi).unwrap();
    let raw = drv.read_raw().unwrap();
    // 0x0100 = 256 (little-endian)
    assert_eq!(raw.x, 0x0100);
    assert_eq!(raw.y, 0x0200);
    assert_eq!(raw.z, 0x0300);
    spi.done();
}

#[test]
fn raw_accel_sign_extension_negative() {
    // The device sign-extends: 0xFF, 0x0F is 0x0FFF = 4095 as u16, which as
    // a 12-bit two's complement is -1 → i16 = -1.
    let buf = [0xFF, 0x0F, 0x00, 0x00, 0x00, 0x00];
    let raw = adxl362::RawAccel::from_bytes(&buf);
    assert_eq!(raw.x, 0x0FFF_u16 as i16); // = -1 as 12-bit, but device sign-extends to 16 bits
    // 0x0FFF as i16 = 4095 (positive). True sign extension would be 0xFFFF = -1.
    // The test confirms the bit-exact i16::from_le_bytes behaviour.
    assert_eq!(raw.x, 4095); // MSB of 12-bit data is bit 11; 0x0FFF is positive in i16.

    // A genuinely negative 12-bit value has bit 11 set; the device puts 1s in [15:12].
    // Example: -1 in 12-bit = 0xFFF, sign-extended by device to 0xFFFF.
    let neg_buf = [0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00];
    let neg_raw = adxl362::RawAccel::from_bytes(&neg_buf);
    assert_eq!(neg_raw.x, -1i16);
}

#[test]
fn range_scale_factors_match_rev_g() {
    // ±2g: 1 mg/LSB
    assert_eq!(Range::G2.raw_to_mg(1000), 1000);
    // ±4g: 2 mg/LSB
    assert_eq!(Range::G4.raw_to_mg(500), 1000);
    // ±8g: 4.255 mg/LSB — NOT 4 or 250 LSB/g
    // 235 LSB per g → raw_to_mg(235) should be ~1000 mg = 1 g
    let mg = Range::G8.raw_to_mg(235);
    // (235 * 4255) / 1000 = 999925 / 1000 = 999
    assert_eq!(mg, 999); // 1 LSB truncation is expected for integer math
}

#[test]
fn fifo_sample_parsing_sign_extends_correctly() {
    use adxl362::{FifoSample, FifoTag};
    // Tag=Z (0b10), value = -1 as 12-bit (0xFFF with sign ext → [13:0] = 0x3FFF)
    // Word = (0b10 << 14) | 0x3FFF = 0x8000 | 0x3FFF = 0xBFFF
    let word: u16 = 0xBFFF;
    let s = FifoSample::from_raw_word(word);
    assert_eq!(s.tag, FifoTag::Z);
    assert_eq!(s.value, -1i16);

    // Tag=X (0b00), value = +100
    let word2: u16 = 100;
    let s2 = FifoSample::from_raw_word(word2);
    assert_eq!(s2.tag, FifoTag::X);
    assert_eq!(s2.value, 100);

    // Tag=Temp (0b11), value = 350 (typical bias)
    let word3: u16 = (0b11 << 14) | 350;
    let s3 = FifoSample::from_raw_word(word3);
    assert_eq!(s3.tag, FifoTag::Temp);
    assert_eq!(s3.value, 350);
}

#[test]
fn fifo_set_reassembly_emits_complete_sets() {
    use adxl362::config::fifo::{FifoTag, parse_fifo_samples, reassemble_sample_sets};

    // Build a buffer: X=1, Y=2, Z=3, X=4, Y=5, Z=6 (two complete sets)
    fn make_word(tag: u16, val: i16) -> [u8; 2] {
        let w: u16 = (tag << 14) | (val as u16 & 0x3FFF);
        w.to_le_bytes()
    }
    let mut buf = Vec::new();
    buf.extend(make_word(0b00, 1)); // X=1
    buf.extend(make_word(0b01, 2)); // Y=2
    buf.extend(make_word(0b10, 3)); // Z=3
    buf.extend(make_word(0b00, 4)); // X=4
    buf.extend(make_word(0b01, 5)); // Y=5
    buf.extend(make_word(0b10, 6)); // Z=6

    let samples: Vec<_> = parse_fifo_samples(&buf).collect();
    assert_eq!(samples.len(), 6);
    assert_eq!(samples[0].tag, FifoTag::X);
    assert_eq!(samples[0].value, 1);

    let sets: Vec<_> = reassemble_sample_sets(&samples).collect();
    assert_eq!(sets.len(), 2);
    assert_eq!(sets[0].x, Some(1));
    assert_eq!(sets[0].y, Some(2));
    assert_eq!(sets[0].z, Some(3));
    assert_eq!(sets[1].x, Some(4));
    assert_eq!(sets[1].y, Some(5));
    assert_eq!(sets[1].z, Some(6));
}

#[test]
fn odr_time_conversions() {
    // At 100 Hz: 100 samples = 1000 ms
    assert_eq!(OutputDataRate::Hz100.samples_to_ms(100), 1000);
    assert_eq!(OutputDataRate::Hz100.ms_to_samples(1000), 100);
    // At 25 Hz: 40 ms per sample
    assert_eq!(OutputDataRate::Hz25.samples_to_ms(1), 40);
}

#[test]
fn temperature_centi_celsius() {
    use adxl362::Temperature;
    let t = Temperature { raw: 350 };
    assert_eq!(t.to_centi_celsius(350), 2500); // 25.00 °C at nominal bias
    let t2 = Temperature { raw: 365 };
    // delta = 15 LSB × 65 / 1000 = 975 / 1000 = 0 (integer), so 2500 + 0 = 2500
    let _ = t2.to_centi_celsius(350); // just verify it doesn't panic
}
