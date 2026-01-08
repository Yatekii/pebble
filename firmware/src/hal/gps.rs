//! GPS driver for MAX-M8Q module
//!
//! Datasheet: https://www.u-blox.com/sites/default/files/MAX-M8-FW3_DataSheet_%28UBX-15031506%29.pdf
//!
//! The GPS communicates via UART at 9600 baud (default).
//! GPIO8 = RX (from GPS TXD), GPIO18 = TX (to GPS RXD).
//! This module handles UART communication and uses the `nmea` crate for parsing.

use chrono::{Datelike, Timelike};
use defmt::Format;
use esp_hal::Blocking;
use esp_hal::uart::{Config as UartConfig, RxConfig, TxConfig, Uart};
use nmea::Nmea;
use nmea::SentenceType;
use nmea::sentences::FixType as NmeaFixType;

/// GPS driver using FIFO polling
pub struct Gps<'d> {
    _uart: Uart<'d, Blocking>,
    nmea: Nmea,
    ubx_parser: UbxParser,
    line_buffer: [u8; 128],
    line_pos: usize,
}

impl<'d> Gps<'d> {
    /// Poll FIFO and parse any available GPS data
    /// Returns Some(GpsData) when a complete sentence updates the fix
    pub fn poll(&mut self) -> Option<GpsData> {
        let uart1 = unsafe { &*esp_hal::peripherals::UART1::ptr() };

        let mut result = None;
        while uart1.status().read().rxfifo_cnt().bits() > 0 {
            let byte = uart1.fifo().read().rxfifo_rd_byte().bits();

            // Try UBX parsing for binary responses
            self.ubx_parser.process(byte);

            // Buffer NMEA sentences
            if byte == b'$' {
                self.line_pos = 0;
            }

            if self.line_pos < self.line_buffer.len() {
                self.line_buffer[self.line_pos] = byte;
                self.line_pos += 1;
            }

            // Check for end of NMEA sentence
            if byte == b'\n' && self.line_pos > 1 {
                if let Ok(sentence) = core::str::from_utf8(&self.line_buffer[..self.line_pos]) {
                    let sentence = sentence.trim();
                    // Log raw NMEA for debugging
                    defmt::debug!("NMEA: {}", sentence);
                    if let Err(e) = self.nmea.parse(sentence) {
                        defmt::warn!("NMEA parse error: {:?}", defmt::Debug2Format(&e));
                    } else {
                        result = Some(self.to_gps_data());
                    }
                }
                self.line_pos = 0;
            }
        }

        result
    }

    /// Get the current GPS data
    pub fn data(&self) -> GpsData {
        self.to_gps_data()
    }

    /// Convert nmea state to GpsData
    fn to_gps_data(&self) -> GpsData {
        let fix_quality = match self.nmea.fix_type {
            Some(NmeaFixType::Invalid) | None => FixQuality::Invalid,
            Some(NmeaFixType::Gps) => FixQuality::GpsFix,
            Some(NmeaFixType::DGps) => FixQuality::DgpsFix,
            Some(NmeaFixType::Pps) => FixQuality::PpsFix,
            Some(NmeaFixType::Rtk) => FixQuality::RtkFixed,
            Some(NmeaFixType::FloatRtk) => FixQuality::RtkFloat,
            Some(NmeaFixType::Estimated) => FixQuality::Estimated,
            Some(NmeaFixType::Manual) => FixQuality::Manual,
            Some(NmeaFixType::Simulation) => FixQuality::Simulation,
        };

        // Determine fix type from HDOP/altitude availability as proxy for 2D/3D
        let fix_type = if self.nmea.hdop.is_some() && self.nmea.altitude.is_some() {
            FixType::Fix3D
        } else if self.nmea.hdop.is_some() {
            FixType::Fix2D
        } else {
            FixType::NoFix
        };

        // Use satellites in view if no fix satellites reported
        let satellites = self
            .nmea
            .num_of_fix_satellites
            .map(|n| n as u8)
            .unwrap_or_else(|| {
                // Count satellites from GSV data
                self.nmea.satellites().len() as u8
            });

        GpsData {
            position: Position {
                latitude: self.nmea.latitude.map(|l| l as f32).unwrap_or(0.0),
                longitude: self.nmea.longitude.map(|l| l as f32).unwrap_or(0.0),
                altitude: self.nmea.altitude.unwrap_or(0.0),
                fix_quality,
                satellites,
                hdop: self.nmea.hdop.unwrap_or(99.9),
            },
            time: self
                .nmea
                .fix_time
                .map(|t| Time {
                    hours: t.hour() as u8,
                    minutes: t.minute() as u8,
                    seconds: t.second() as u8,
                    milliseconds: (t.nanosecond() / 1_000_000) as u16,
                })
                .unwrap_or_default(),
            date: self
                .nmea
                .fix_date
                .map(|d| Date {
                    day: d.day() as u8,
                    month: d.month() as u8,
                    year: d.year() as u16,
                })
                .unwrap_or_default(),
            speed_knots: self.nmea.speed_over_ground.unwrap_or(0.0),
            course: self.nmea.true_course.unwrap_or(0.0),
            fix_type,
        }
    }
}

/// Initialize GPS UART on GPIO8 (RX) and GPIO18 (TX)
pub fn init(
    uart: esp_hal::peripherals::UART1<'static>,
    rx: esp_hal::peripherals::GPIO8<'static>,
    tx: esp_hal::peripherals::GPIO18<'static>,
) -> Result<Gps<'static>, esp_hal::uart::ConfigError> {
    defmt::info!("GPS: configuring UART1 at 9600 baud");
    let config = UartConfig::default()
        .with_baudrate(9600)
        .with_rx(RxConfig::default())
        .with_tx(TxConfig::default());

    let mut uart = Uart::new(uart, config)?.with_rx(rx).with_tx(tx);

    // Configure antenna supervisor via CFG-ANT
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
    let _ = embedded_io::Write::write_all(&mut uart, &cfg_ant);

    // Small delay
    for _ in 0..100000 {
        core::hint::spin_loop();
    }

    // Enable multiple GNSS constellations via CFG-GNSS
    // This configures GPS + GLONASS + Galileo for better satellite coverage
    // Message structure: CFG-GNSS (0x06 0x3E)
    // Payload: 4 bytes header + 4 config blocks × 8 bytes = 36 bytes
    // Total: sync(2) + class/id(2) + len(2) + payload(36) + checksum(2) = 44 bytes
    #[rustfmt::skip]
    let mut cfg_gnss: [u8; 44] = [
        0xB5, 0x62,             // Sync
        0x06, 0x3E,             // Class/ID: CFG-GNSS
        0x24, 0x00,             // Length: 36 bytes

        // Header (4 bytes)
        0x00,                   // msgVer: 0
        0x00,                   // numTrkChHw: read-only, set to 0
        0x20,                   // numTrkChUse: 32 channels
        0x04,                   // numConfigBlocks: 4

        // GPS (gnssId=0): enabled, 8-16 channels
        0x00,                   // gnssId: GPS
        0x08,                   // resTrkCh: 8
        0x10,                   // maxTrkCh: 16
        0x00,                   // reserved
        0x01, 0x00, 0x01, 0x01, // flags: enabled

        // SBAS (gnssId=1): disabled
        0x01,                   // gnssId: SBAS
        0x01,                   // resTrkCh: 1
        0x03,                   // maxTrkCh: 3
        0x00,                   // reserved
        0x00, 0x00, 0x01, 0x01, // flags: disabled

        // Galileo (gnssId=2): enabled, 4-8 channels
        0x02,                   // gnssId: Galileo
        0x04,                   // resTrkCh: 4
        0x08,                   // maxTrkCh: 8
        0x00,                   // reserved
        0x01, 0x00, 0x01, 0x01, // flags: enabled

        // GLONASS (gnssId=6): enabled, 4-8 channels
        0x06,                   // gnssId: GLONASS
        0x04,                   // resTrkCh: 4
        0x08,                   // maxTrkCh: 8
        0x00,                   // reserved
        0x01, 0x00, 0x01, 0x01, // flags: enabled

        0x00, 0x00,             // Checksum placeholder
    ];
    let (ck_a, ck_b) = ubx_checksum(&cfg_gnss[2..42]);
    cfg_gnss[42] = ck_a;
    cfg_gnss[43] = ck_b;

    defmt::info!("GPS: enabling GPS + Galileo + GLONASS constellations");
    let _ = embedded_io::Write::write_all(&mut uart, &cfg_gnss);

    // Small delay for config to apply
    for _ in 0..100000 {
        core::hint::spin_loop();
    }

    // Save config to flash with CFG-CFG
    let mut cfg_cfg: [u8; 21] = [
        0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x17, 0x00, 0x00,
    ];
    let (ck_a, ck_b) = ubx_checksum(&cfg_cfg[2..19]);
    cfg_cfg[19] = ck_a;
    cfg_cfg[20] = ck_b;

    defmt::info!("GPS: saving config to flash");
    let _ = embedded_io::Write::write_all(&mut uart, &cfg_cfg);

    // Poll antenna status
    let cfg_ant_poll: [u8; 8] = [0xB5, 0x62, 0x06, 0x13, 0x00, 0x00, 0x19, 0x51];
    let _ = embedded_io::Write::write_all(&mut uart, &cfg_ant_poll);

    // Poll MON-HW
    let mon_hw_poll: [u8; 8] = [0xB5, 0x62, 0x0A, 0x09, 0x00, 0x00, 0x13, 0x43];
    let _ = embedded_io::Write::write_all(&mut uart, &mon_hw_poll);

    defmt::info!("GPS: init complete");

    // Create NMEA parser with required sentence types enabled
    let nmea = Nmea::create_for_navigation(&[
        SentenceType::GGA, // Position fix data
        SentenceType::GSA, // DOP and active satellites
        SentenceType::GSV, // Satellites in view
        SentenceType::RMC, // Recommended minimum data
        SentenceType::VTG, // Course over ground
        SentenceType::GLL, // Geographic position
    ])
    .unwrap_or_default();

    Ok(Gps {
        _uart: uart,
        nmea,
        ubx_parser: UbxParser::new(),
        line_buffer: [0; 128],
        line_pos: 0,
    })
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
                    defmt::info!("UBX CFG-ANT: flags={:#06x} pins={:#06x}", flags, pins);
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
                // MON-HW response
                if self.length >= 60 {
                    let a_status = self.payload[20];
                    let a_power = self.payload[21];
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
                    defmt::info!("UBX MON-HW: aStatus={} aPower={}", status_str, power_str);
                }
            }
            _ => {
                defmt::trace!(
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

/// GPS fix type for GSA sentences
#[derive(Debug, Clone, Copy, Default, Format, PartialEq, Eq)]
pub enum FixType {
    #[default]
    NoFix = 1,
    Fix2D = 2,
    Fix3D = 3,
}

/// GPS position data
#[derive(Debug, Clone, Copy, Default)]
pub struct Position {
    pub latitude: f32,
    pub longitude: f32,
    pub altitude: f32,
    pub fix_quality: FixQuality,
    pub satellites: u8,
    pub hdop: f32,
}

impl Position {
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
    pub speed_knots: f32,
    pub course: f32,
    pub fix_type: FixType,
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
