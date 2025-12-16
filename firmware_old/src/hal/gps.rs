// GPS driver wrapper using the nmea crate
// Datasheet: https://www.u-blox.com/sites/default/files/MAX-M8-FW3_DataSheet_%28UBX-15031506%29.pdf

use esp_hal::peripherals::{GPIO18, GPIO8, UART1};
use esp_hal::uart::{Config as UartConfig, Uart};
use esp_hal::Blocking;
use nmea::Nmea;

/// GPS position data
#[derive(Debug, Clone, Copy, Default)]
pub struct Position {
    /// Latitude in degrees (positive = North, negative = South)
    pub latitude: f64,
    /// Longitude in degrees (positive = East, negative = West)
    pub longitude: f64,
    /// Whether the position fix is valid
    pub valid: bool,
}

/// GPS driver error
#[derive(Debug)]
pub enum GpsError {
    UartCreate,
    Uart,
    Parse,
    NoFix,
    BufferOverflow,
}

/// Initialize the GPS with UART1 peripheral and GPIO8 (RX) / GPIO18 (TX)
pub fn init(
    uart1: UART1<'static>,
    rx: GPIO8<'static>,
    tx: GPIO18<'static>,
) -> Result<Gps<'static>, GpsError> {
    let uart = Uart::new(uart1, UartConfig::default().with_baudrate(9600))
        .map_err(|_| GpsError::UartCreate)?
        .with_rx(rx)
        .with_tx(tx);

    Ok(Gps::new(uart))
}

/// MAX-M8Q GPS driver using nmea crate for parsing
pub struct Gps<'d> {
    uart: Uart<'d, Blocking>,
    nmea: Nmea,
    buffer: [u8; 128],
    buffer_pos: usize,
}

impl<'d> Gps<'d> {
    /// Create a new GPS driver instance from an already-configured UART
    pub fn new(uart: Uart<'d, Blocking>) -> Self {
        Self {
            uart,
            nmea: Nmea::default(),
            buffer: [0u8; 128],
            buffer_pos: 0,
        }
    }

    /// Process incoming UART data and update position if a valid NMEA sentence is received
    /// Call this regularly to process GPS data
    pub fn update(&mut self) -> Result<(), GpsError> {
        let mut byte = [0u8];

        // Read available bytes from UART
        while self.uart.read_ready() {
            if self.uart.read(&mut byte).is_err() {
                break;
            }

            // Look for end of NMEA sentence
            if byte[0] == b'\n' {
                if self.buffer_pos > 0 {
                    // Try to parse the sentence
                    if let Ok(sentence) = core::str::from_utf8(&self.buffer[..self.buffer_pos]) {
                        // Remove trailing CR if present
                        let sentence = sentence.trim_end_matches('\r');
                        let _ = self.nmea.parse(sentence);
                    }
                }
                self.buffer_pos = 0;
            } else if byte[0] != b'\r' {
                // Add byte to buffer (skip CR)
                if self.buffer_pos < self.buffer.len() {
                    self.buffer[self.buffer_pos] = byte[0];
                    self.buffer_pos += 1;
                } else {
                    // Buffer overflow, reset
                    self.buffer_pos = 0;
                }
            }
        }

        Ok(())
    }

    /// Get the current position
    pub fn position(&self) -> Position {
        match (self.nmea.latitude, self.nmea.longitude) {
            (Some(lat), Some(lon)) => Position {
                latitude: lat,
                longitude: lon,
                valid: true,
            },
            _ => Position::default(),
        }
    }

    /// Check if we have a valid GPS fix
    pub fn has_fix(&self) -> bool {
        self.nmea.latitude.is_some() && self.nmea.longitude.is_some()
    }

    /// Get the number of satellites in view
    pub fn satellites(&self) -> Option<u32> {
        self.nmea.num_of_fix_satellites
    }

    /// Get the altitude in meters
    pub fn altitude(&self) -> Option<f32> {
        self.nmea.altitude
    }

    /// Get access to the underlying NMEA parser for advanced queries
    pub fn nmea(&self) -> &Nmea {
        &self.nmea
    }
}
