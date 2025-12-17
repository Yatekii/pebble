//! GPS driver for MAX-M8Q module
//!
//! Datasheet: https://www.u-blox.com/sites/default/files/MAX-M8-FW3_DataSheet_%28UBX-15031506%29.pdf
//!
//! The GPS communicates via UART at 9600 baud (default).
//! GPIO8 = RX (from GPS TXD), GPIO18 = TX (to GPS RXD).
//! This module handles both UART communication and NMEA sentence parsing.

use defmt::Format;
use esp_hal::uart::{Config as UartConfig, RxConfig, TxConfig, Uart};

use esp_hal::Blocking;

/// GPS driver using FIFO polling
pub struct Gps<'d> {
    // Keep UART instance alive so it doesn't get disabled on drop
    _uart: Uart<'d, Blocking>,
    parser: GpsParser,
    ubx_parser: UbxParser,
}

impl<'d> Gps<'d> {
    /// Poll FIFO and parse any available GPS data
    /// Returns Some(GpsData) when a complete GGA sentence is parsed
    pub fn poll(&mut self) -> Option<GpsData> {
        let uart1 = unsafe { &*esp_hal::peripherals::UART1::ptr() };

        // Read all available bytes from FIFO
        let mut result = None;
        while uart1.status().read().rxfifo_cnt().bits() > 0 {
            let byte = uart1.fifo().read().rxfifo_rd_byte().bits();
            // Try NMEA parsing
            if let Some(data) = self.parser.process(&[byte]) {
                result = Some(data);
            }
            // Also check for UBX responses
            self.ubx_parser.process(byte);
        }

        result
    }

    /// Get the current GPS data (even if no new fix available)
    pub fn data(&self) -> GpsData {
        self.parser.data()
    }
}

/// Initialize GPS UART on GPIO8 (RX) and GPIO18 (TX)
pub fn init(
    uart: esp_hal::peripherals::UART1<'static>,
    rx: esp_hal::peripherals::GPIO8<'static>,
    tx: esp_hal::peripherals::GPIO18<'static>,
) -> Gps<'static> {
    defmt::info!("GPS: configuring UART1 at 9600 baud");
    let config = UartConfig::default()
        .with_baudrate(9600)
        .with_rx(RxConfig::default())
        .with_tx(TxConfig::default());

    let mut uart = Uart::new(uart, config).unwrap().with_rx(rx).with_tx(tx);

    // Configure antenna supervisor via CFG-ANT
    // The MAX-M8Q has a DEDICATED LNA_EN pin (physical pin 13) which is NOT a PIO.
    // Try enabling all antenna supervisor features.
    // flags bits:
    //   bit 0: svcs - enable antenna supply voltage control
    //   bit 1: scd - enable short circuit detection
    //   bit 2: ocd - enable open circuit detection
    //   bit 3: pdwnOnSCD - power down on short circuit
    //   bit 4: recovery - enable recovery from short
    let flags: u16 = 0x0001; // svcs only - antenna voltage control
    let pins: u16 = 0x0000; // Use dedicated LNA_EN pin

    let mut cfg_ant: [u8; 12] = [
        0xB5,
        0x62,
        0x06,
        0x13,
        0x04,
        0x00,
        (flags & 0xFF) as u8,
        ((flags >> 8) & 0xFF) as u8,
        (pins & 0xFF) as u8,
        ((pins >> 8) & 0xFF) as u8,
        0x00,
        0x00,
    ];
    let (ck_a, ck_b) = ubx_checksum(&cfg_ant[2..10]);
    cfg_ant[10] = ck_a;
    cfg_ant[11] = ck_b;

    defmt::info!("GPS: CFG-ANT flags={:#06x} pins={:#06x}", flags, pins);
    if let Err(e) = embedded_io::Write::write_all(&mut uart, &cfg_ant) {
        defmt::warn!("GPS: CFG-ANT failed: {:?}", e);
    }

    // Small delay to let the command be processed
    // (blocking delay since we're in init)
    for _ in 0..100000 {
        core::hint::spin_loop();
    }

    // Save config to flash with CFG-CFG
    // clearMask=0, saveMask=0x0000001F (all sections), loadMask=0
    // deviceMask=0x17 (BBR + Flash + EEPROM + SPI Flash)
    let mut cfg_cfg: [u8; 21] = [
        0xB5, 0x62, // sync
        0x06, 0x09, // class=CFG, id=CFG
        0x0D, 0x00, // length = 13
        0x00, 0x00, 0x00, 0x00, // clearMask = 0 (don't clear)
        0x1F, 0x00, 0x00, 0x00, // saveMask = 0x1F (all sections)
        0x00, 0x00, 0x00, 0x00, // loadMask = 0 (don't load)
        0x17, // deviceMask = BBR + Flash + EEPROM + SPI Flash
        0x00, 0x00, // checksum placeholder
    ];
    let (ck_a, ck_b) = ubx_checksum(&cfg_cfg[2..19]);
    cfg_cfg[19] = ck_a;
    cfg_cfg[20] = ck_b;

    defmt::info!("GPS: saving config to flash");
    if let Err(e) = embedded_io::Write::write_all(&mut uart, &cfg_cfg) {
        defmt::warn!("GPS: CFG-CFG failed: {:?}", e);
    }

    // Also poll current antenna status to see what the module reports
    let cfg_ant_poll: [u8; 8] = [
        0xB5, 0x62, // sync
        0x06, 0x13, // class, id (CFG-ANT)
        0x00, 0x00, // length = 0 (poll request)
        0x19, 0x51, // checksum
    ];
    if let Err(e) = embedded_io::Write::write_all(&mut uart, &cfg_ant_poll) {
        defmt::warn!("GPS: CFG-ANT poll failed: {:?}", e);
    }

    // Poll MON-HW to check hardware status including antenna
    let mon_hw_poll: [u8; 8] = [
        0xB5, 0x62, // sync
        0x0A, 0x09, // class=MON, id=HW
        0x00, 0x00, // length = 0 (poll request)
        0x13, 0x43, // checksum
    ];
    if let Err(e) = embedded_io::Write::write_all(&mut uart, &mon_hw_poll) {
        defmt::warn!("GPS: MON-HW poll failed: {:?}", e);
    }

    defmt::info!("GPS: init complete");

    Gps {
        _uart: uart,
        parser: GpsParser::new(),
        ubx_parser: UbxParser::new(),
    }
}

/// UBX protocol parser for decoding binary responses
struct UbxParser {
    state: UbxState,
    class: u8,
    id: u8,
    length: u16,
    payload: [u8; 64],
    payload_idx: usize,
    ck_a: u8,
    ck_b: u8,
}

#[derive(Clone, Copy, PartialEq)]
enum UbxState {
    Sync1,
    Sync2,
    Class,
    Id,
    LengthLow,
    LengthHigh,
    Payload,
    ChecksumA,
    ChecksumB,
}

impl UbxParser {
    fn new() -> Self {
        Self {
            state: UbxState::Sync1,
            class: 0,
            id: 0,
            length: 0,
            payload: [0; 64],
            payload_idx: 0,
            ck_a: 0,
            ck_b: 0,
        }
    }

    fn process(&mut self, byte: u8) {
        match self.state {
            UbxState::Sync1 => {
                if byte == 0xB5 {
                    self.state = UbxState::Sync2;
                }
            }
            UbxState::Sync2 => {
                if byte == 0x62 {
                    self.state = UbxState::Class;
                    self.ck_a = 0;
                    self.ck_b = 0;
                } else {
                    self.state = UbxState::Sync1;
                }
            }
            UbxState::Class => {
                self.class = byte;
                self.ck_a = self.ck_a.wrapping_add(byte);
                self.ck_b = self.ck_b.wrapping_add(self.ck_a);
                self.state = UbxState::Id;
            }
            UbxState::Id => {
                self.id = byte;
                self.ck_a = self.ck_a.wrapping_add(byte);
                self.ck_b = self.ck_b.wrapping_add(self.ck_a);
                self.state = UbxState::LengthLow;
            }
            UbxState::LengthLow => {
                self.length = byte as u16;
                self.ck_a = self.ck_a.wrapping_add(byte);
                self.ck_b = self.ck_b.wrapping_add(self.ck_a);
                self.state = UbxState::LengthHigh;
            }
            UbxState::LengthHigh => {
                self.length |= (byte as u16) << 8;
                self.ck_a = self.ck_a.wrapping_add(byte);
                self.ck_b = self.ck_b.wrapping_add(self.ck_a);
                self.payload_idx = 0;
                if self.length == 0 {
                    self.state = UbxState::ChecksumA;
                } else {
                    self.state = UbxState::Payload;
                }
            }
            UbxState::Payload => {
                if self.payload_idx < self.payload.len() {
                    self.payload[self.payload_idx] = byte;
                }
                self.payload_idx += 1;
                self.ck_a = self.ck_a.wrapping_add(byte);
                self.ck_b = self.ck_b.wrapping_add(self.ck_a);
                if self.payload_idx >= self.length as usize {
                    self.state = UbxState::ChecksumA;
                }
            }
            UbxState::ChecksumA => {
                if byte == self.ck_a {
                    self.state = UbxState::ChecksumB;
                } else {
                    defmt::warn!("UBX checksum A mismatch");
                    self.state = UbxState::Sync1;
                }
            }
            UbxState::ChecksumB => {
                if byte == self.ck_b {
                    self.handle_message();
                } else {
                    defmt::warn!("UBX checksum B mismatch");
                }
                self.state = UbxState::Sync1;
            }
        }
    }

    fn handle_message(&self) {
        match (self.class, self.id) {
            (0x06, 0x13) => {
                // CFG-ANT response
                if self.length >= 4 {
                    let flags = self.payload[0] as u16 | ((self.payload[1] as u16) << 8);
                    let pins = self.payload[2] as u16 | ((self.payload[3] as u16) << 8);
                    defmt::info!(
                        "UBX CFG-ANT response: flags={:#06x} pins={:#06x}",
                        flags,
                        pins
                    );
                    defmt::info!(
                        "  svcs={} scd={} ocd={} pdwnOnSCD={} recovery={}",
                        (flags & 0x01) != 0,
                        (flags & 0x02) != 0,
                        (flags & 0x04) != 0,
                        (flags & 0x08) != 0,
                        (flags & 0x10) != 0
                    );
                    defmt::info!(
                        "  pinSwitch={} pinSCD={} pinOCD={} reconfig={}",
                        pins & 0x1F,
                        (pins >> 5) & 0x1F,
                        (pins >> 10) & 0x1F,
                        (pins >> 15) & 0x01
                    );
                }
            }
            (0x05, 0x01) => {
                // ACK-ACK
                if self.length >= 2 {
                    defmt::info!(
                        "UBX ACK for class={:#04x} id={:#04x}",
                        self.payload[0],
                        self.payload[1]
                    );
                }
            }
            (0x05, 0x00) => {
                // ACK-NAK
                if self.length >= 2 {
                    defmt::warn!(
                        "UBX NAK for class={:#04x} id={:#04x}",
                        self.payload[0],
                        self.payload[1]
                    );
                }
            }
            (0x0A, 0x09) => {
                // MON-HW response - hardware status
                if self.length >= 60 {
                    // Antenna status is at offset 20 (aStatus)
                    let a_status = self.payload[20];
                    // Antenna power is at offset 21 (aPower)
                    let a_power = self.payload[21];
                    // flags at offset 22-25
                    let flags = u32::from_le_bytes([
                        self.payload[22],
                        self.payload[23],
                        self.payload[24],
                        self.payload[25],
                    ]);
                    let status_str = match a_status {
                        0 => "INIT",
                        1 => "DONTKNOW",
                        2 => "OK",
                        3 => "SHORT",
                        4 => "OPEN",
                        _ => "UNKNOWN",
                    };
                    let power_str = match a_power {
                        0 => "OFF",
                        1 => "ON",
                        2 => "DONTKNOW",
                        _ => "UNKNOWN",
                    };
                    defmt::info!(
                        "UBX MON-HW: aStatus={} ({}) aPower={} ({}) flags={:#010x}",
                        a_status,
                        status_str,
                        a_power,
                        power_str,
                        flags
                    );
                }
            }
            _ => {
                defmt::debug!(
                    "UBX message class={:#04x} id={:#04x} len={}",
                    self.class,
                    self.id,
                    self.length
                );
            }
        }
    }
}

/// GPS fix quality
#[derive(Debug, Clone, Copy, Default, Format, PartialEq, Eq)]
pub enum FixQuality {
    #[default]
    Invalid = 0,
    GpsFix = 1,
    DgpsFix = 2,
    PpsFix = 3,
    RtkFixed = 4,
    RtkFloat = 5,
    Estimated = 6,
    Manual = 7,
    Simulation = 8,
}

impl From<u8> for FixQuality {
    fn from(value: u8) -> Self {
        match value {
            1 => FixQuality::GpsFix,
            2 => FixQuality::DgpsFix,
            3 => FixQuality::PpsFix,
            4 => FixQuality::RtkFixed,
            5 => FixQuality::RtkFloat,
            6 => FixQuality::Estimated,
            7 => FixQuality::Manual,
            8 => FixQuality::Simulation,
            _ => FixQuality::Invalid,
        }
    }
}

/// GPS fix type for GSA sentences
#[derive(Debug, Clone, Copy, Default, Format, PartialEq, Eq)]
pub enum FixType {
    #[default]
    NoFix = 1,
    Fix2D = 2,
    Fix3D = 3,
}

impl From<u8> for FixType {
    fn from(value: u8) -> Self {
        match value {
            2 => FixType::Fix2D,
            3 => FixType::Fix3D,
            _ => FixType::NoFix,
        }
    }
}

/// GPS position data
#[derive(Debug, Clone, Copy, Default)]
pub struct Position {
    /// Latitude in degrees (positive = North, negative = South)
    pub latitude: f32,
    /// Longitude in degrees (positive = East, negative = West)
    pub longitude: f32,
    /// Altitude above sea level in meters
    pub altitude: f32,
    /// Fix quality
    pub fix_quality: FixQuality,
    /// Number of satellites used
    pub satellites: u8,
    /// Horizontal dilution of precision
    pub hdop: f32,
}

impl Position {
    /// Check if position has a valid fix
    pub fn has_fix(&self) -> bool {
        self.fix_quality != FixQuality::Invalid
    }
}

/// GPS time data
#[derive(Debug, Clone, Copy, Default)]
pub struct Time {
    pub hours: u8,
    pub minutes: u8,
    pub seconds: u8,
    pub milliseconds: u16,
}

/// GPS date data
#[derive(Debug, Clone, Copy, Default)]
pub struct Date {
    pub day: u8,
    pub month: u8,
    pub year: u16,
}

/// Complete GPS data from latest fix
#[derive(Debug, Clone, Copy, Default)]
pub struct GpsData {
    pub position: Position,
    pub time: Time,
    pub date: Date,
    /// Speed over ground in knots
    pub speed_knots: f32,
    /// Course over ground in degrees
    pub course: f32,
    /// Fix type (2D/3D)
    pub fix_type: FixType,
}

/// NMEA sentence buffer size (max NMEA sentence is 82 chars including CR LF)
const NMEA_BUFFER_SIZE: usize = 128;

/// GPS NMEA parser
pub struct GpsParser {
    buffer: [u8; NMEA_BUFFER_SIZE],
    buffer_pos: usize,
    data: GpsData,
}

impl GpsParser {
    /// Create a new GPS parser
    pub fn new() -> Self {
        Self {
            buffer: [0u8; NMEA_BUFFER_SIZE],
            buffer_pos: 0,
            data: GpsData::default(),
        }
    }

    /// Process incoming bytes from GPS
    /// Returns Some(GpsData) when a complete GGA sentence with fix is parsed
    pub fn process(&mut self, bytes: &[u8]) -> Option<GpsData> {
        let mut result = None;

        for &b in bytes {
            // Skip 0xFF bytes (no data marker from u-blox I2C)
            if b == 0xFF {
                continue;
            }

            // Check for sentence start
            if b == b'$' {
                self.buffer_pos = 0;
                self.buffer[0] = b;
                self.buffer_pos = 1;
                continue;
            }

            // Check for sentence end
            if b == b'\n' {
                if self.buffer_pos > 0 && self.buffer[self.buffer_pos - 1] == b'\r' {
                    // Complete sentence received - parse in place
                    let sentence_len = self.buffer_pos - 1; // Exclude \r
                    if self.parse_nmea_len(sentence_len) {
                        result = Some(self.data);
                    }
                }
                self.buffer_pos = 0;
                continue;
            }

            // Add byte to buffer
            if self.buffer_pos < NMEA_BUFFER_SIZE {
                self.buffer[self.buffer_pos] = b;
                self.buffer_pos += 1;
            } else {
                // Buffer overflow, reset
                self.buffer_pos = 0;
            }
        }

        result
    }

    /// Get the current GPS data
    pub fn data(&self) -> GpsData {
        self.data
    }

    /// Parse an NMEA sentence from the internal buffer
    /// Returns true if position data was updated (GGA sentence parsed successfully)
    fn parse_nmea_len(&mut self, len: usize) -> bool {
        // Verify checksum
        if !verify_checksum(&self.buffer[..len]) {
            return false;
        }

        // Get sentence type (chars 1-5 after $)
        if len < 6 {
            return false;
        }

        // Copy sentence type to avoid borrow issues
        let mut sentence_type = [0u8; 5];
        sentence_type.copy_from_slice(&self.buffer[1..6]);

        match &sentence_type {
            b"GPGGA" | b"GNGGA" => self.parse_gga_len(len),
            b"GPRMC" | b"GNRMC" => {
                self.parse_rmc_len(len);
                false // RMC doesn't contain altitude, wait for GGA
            }
            b"GPGSA" | b"GNGSA" => {
                self.parse_gsa_len(len);
                false
            }
            b"GNTXT" | b"GPTXT" => {
                // Log TXT messages (antenna status, etc)
                if let Ok(s) = core::str::from_utf8(&self.buffer[..len]) {
                    defmt::info!("GPS TXT: {}", s);
                }
                false
            }
            _ => false,
        }
    }

    /// Parse GGA sentence (Global Positioning System Fix Data)
    fn parse_gga_len(&mut self, len: usize) -> bool {
        let fields: heapless::Vec<&[u8], 16> =
            self.buffer[..len].split(|&b| b == b',').take(16).collect();

        if fields.len() < 10 {
            return false;
        }

        // Field 1: Time (hhmmss.ss)
        if let Some(time) = parse_time(fields[1]) {
            self.data.time = time;
        }

        // Field 2-3: Latitude (ddmm.mmm,N/S)
        if let Some(lat) = parse_coordinate(fields[2], fields[3]) {
            self.data.position.latitude = lat;
        }

        // Field 4-5: Longitude (dddmm.mmm,E/W)
        if let Some(lon) = parse_coordinate(fields[4], fields[5]) {
            self.data.position.longitude = lon;
        }

        // Field 6: Fix quality
        if !fields[6].is_empty() {
            if let Some(q) = parse_u8(fields[6]) {
                self.data.position.fix_quality = FixQuality::from(q);
            }
        }

        // Field 7: Number of satellites
        if !fields[7].is_empty() {
            if let Some(n) = parse_u8(fields[7]) {
                self.data.position.satellites = n;
            }
        }

        // Field 8: HDOP
        if !fields[8].is_empty() {
            if let Some(h) = parse_f32(fields[8]) {
                self.data.position.hdop = h;
            }
        }

        // Field 9: Altitude
        if !fields[9].is_empty() {
            if let Some(alt) = parse_f32(fields[9]) {
                self.data.position.altitude = alt;
            }
        }

        // Return true to report data even without fix (shows satellites)
        true
    }

    /// Parse RMC sentence (Recommended Minimum Navigation Information)
    fn parse_rmc_len(&mut self, len: usize) {
        let fields: heapless::Vec<&[u8], 14> =
            self.buffer[..len].split(|&b| b == b',').take(14).collect();

        if fields.len() < 10 {
            return;
        }

        // Field 1: Time
        if let Some(time) = parse_time(fields[1]) {
            self.data.time = time;
        }

        // Field 7: Speed over ground (knots)
        if fields.len() > 7 && !fields[7].is_empty() {
            if let Some(speed) = parse_f32(fields[7]) {
                self.data.speed_knots = speed;
            }
        }

        // Field 8: Course over ground
        if fields.len() > 8 && !fields[8].is_empty() {
            if let Some(course) = parse_f32(fields[8]) {
                self.data.course = course;
            }
        }

        // Field 9: Date (ddmmyy)
        if fields.len() > 9 && !fields[9].is_empty() {
            if let Some(date) = parse_date(fields[9]) {
                self.data.date = date;
            }
        }
    }

    /// Parse GSA sentence (GPS DOP and active satellites)
    fn parse_gsa_len(&mut self, len: usize) {
        let fields: heapless::Vec<&[u8], 18> =
            self.buffer[..len].split(|&b| b == b',').take(18).collect();

        if fields.len() < 3 {
            return;
        }

        // Field 2: Fix type (1=no fix, 2=2D, 3=3D)
        if !fields[2].is_empty() {
            if let Some(fix) = parse_u8(fields[2]) {
                self.data.fix_type = FixType::from(fix);
            }
        }
    }
}

impl Default for GpsParser {
    fn default() -> Self {
        Self::new()
    }
}

/// Calculate UBX protocol checksum (8-bit Fletcher)
fn ubx_checksum(data: &[u8]) -> (u8, u8) {
    let mut ck_a: u8 = 0;
    let mut ck_b: u8 = 0;
    for &byte in data {
        ck_a = ck_a.wrapping_add(byte);
        ck_b = ck_b.wrapping_add(ck_a);
    }
    (ck_a, ck_b)
}

/// Verify NMEA checksum
fn verify_checksum(sentence: &[u8]) -> bool {
    // Find the asterisk
    let asterisk_pos = sentence.iter().position(|&b| b == b'*');
    let asterisk_pos = match asterisk_pos {
        Some(pos) => pos,
        None => return false,
    };

    // Calculate checksum (XOR of all bytes between $ and *)
    let mut checksum: u8 = 0;
    for &b in &sentence[1..asterisk_pos] {
        checksum ^= b;
    }

    // Parse expected checksum (2 hex digits after *)
    if sentence.len() < asterisk_pos + 3 {
        return false;
    }

    let expected = parse_hex_byte(&sentence[asterisk_pos + 1..asterisk_pos + 3]);
    expected == Some(checksum)
}

/// Parse a hex byte from 2 ASCII hex characters
fn parse_hex_byte(s: &[u8]) -> Option<u8> {
    if s.len() < 2 {
        return None;
    }

    let high = hex_digit(s[0])?;
    let low = hex_digit(s[1])?;
    Some((high << 4) | low)
}

fn hex_digit(c: u8) -> Option<u8> {
    match c {
        b'0'..=b'9' => Some(c - b'0'),
        b'A'..=b'F' => Some(c - b'A' + 10),
        b'a'..=b'f' => Some(c - b'a' + 10),
        _ => None,
    }
}

/// Parse time from NMEA format (hhmmss.ss)
fn parse_time(s: &[u8]) -> Option<Time> {
    if s.len() < 6 {
        return None;
    }

    let hours = parse_2digits(&s[0..2])?;
    let minutes = parse_2digits(&s[2..4])?;
    let seconds = parse_2digits(&s[4..6])?;

    let milliseconds = if s.len() > 7 && s[6] == b'.' {
        // Parse fractional seconds
        let frac = &s[7..];
        let mut ms = 0u16;
        for (i, &d) in frac.iter().take(3).enumerate() {
            if d.is_ascii_digit() {
                let mult = match i {
                    0 => 100,
                    1 => 10,
                    2 => 1,
                    _ => 0,
                };
                ms += (d - b'0') as u16 * mult;
            }
        }
        ms
    } else {
        0
    };

    Some(Time {
        hours,
        minutes,
        seconds,
        milliseconds,
    })
}

/// Parse date from NMEA format (ddmmyy)
fn parse_date(s: &[u8]) -> Option<Date> {
    if s.len() < 6 {
        return None;
    }

    let day = parse_2digits(&s[0..2])?;
    let month = parse_2digits(&s[2..4])?;
    let year = parse_2digits(&s[4..6])? as u16 + 2000;

    Some(Date { day, month, year })
}

/// Parse coordinate from NMEA format
fn parse_coordinate(coord: &[u8], dir: &[u8]) -> Option<f32> {
    if coord.is_empty() || dir.is_empty() {
        return None;
    }

    // Find the decimal point
    let dot_pos = coord.iter().position(|&b| b == b'.')?;

    // Degrees are before the last 2 digits of the integer part
    let deg_end = if dot_pos > 2 { dot_pos - 2 } else { 0 };
    let degrees = parse_int(&coord[..deg_end])? as f32;

    // Minutes are the last 2 digits plus fractional part
    let minutes = parse_f32(&coord[deg_end..])?;

    let mut result = degrees + minutes / 60.0;

    // Apply direction
    if dir[0] == b'S' || dir[0] == b'W' {
        result = -result;
    }

    Some(result)
}

/// Parse 2 ASCII digits as u8
fn parse_2digits(s: &[u8]) -> Option<u8> {
    if s.len() < 2 {
        return None;
    }
    let d1 = s[0].checked_sub(b'0')?;
    let d2 = s[1].checked_sub(b'0')?;
    if d1 > 9 || d2 > 9 {
        return None;
    }
    Some(d1 * 10 + d2)
}

/// Parse integer from ASCII digits
fn parse_int(s: &[u8]) -> Option<u32> {
    if s.is_empty() {
        return None;
    }
    let mut result = 0u32;
    for &b in s {
        let d = b.checked_sub(b'0')?;
        if d > 9 {
            return None;
        }
        result = result.checked_mul(10)?.checked_add(d as u32)?;
    }
    Some(result)
}

/// Parse u8 from ASCII digits
fn parse_u8(s: &[u8]) -> Option<u8> {
    let v = parse_int(s)?;
    if v > 255 {
        return None;
    }
    Some(v as u8)
}

/// Parse f32 from ASCII (simple implementation without std)
fn parse_f32(s: &[u8]) -> Option<f32> {
    if s.is_empty() {
        return None;
    }

    let mut negative = false;
    let mut idx = 0;

    // Check for negative sign
    if s[0] == b'-' {
        negative = true;
        idx = 1;
    }

    // Parse integer part
    let mut int_part = 0u32;
    while idx < s.len() && s[idx] != b'.' {
        let d = s[idx].checked_sub(b'0')?;
        if d > 9 {
            return None;
        }
        int_part = int_part.saturating_mul(10).saturating_add(d as u32);
        idx += 1;
    }

    // Parse fractional part
    let mut frac_part = 0u32;
    let mut frac_divisor = 1u32;
    if idx < s.len() && s[idx] == b'.' {
        idx += 1;
        while idx < s.len() {
            let d = s[idx].checked_sub(b'0')?;
            if d > 9 {
                break; // Stop at non-digit (e.g., checksum marker)
            }
            frac_part = frac_part.saturating_mul(10).saturating_add(d as u32);
            frac_divisor = frac_divisor.saturating_mul(10);
            idx += 1;
        }
    }

    let result = int_part as f32 + (frac_part as f32 / frac_divisor as f32);
    Some(if negative { -result } else { result })
}
