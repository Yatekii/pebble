//! BLE GATT server for sensor data
//!
//! Provides a custom GATT service for accelerometer, gyroscope, and magnetometer data.

use trouble_host::prelude::*;

/// LED color chunk for 24 LEDs (72 bytes: 24 LEDs * 3 bytes RGB each)
/// Using a wrapper struct to implement Default for [u8; 72]
#[derive(Clone, Copy)]
#[repr(transparent)]
pub struct LedColorChunk(pub [u8; 72]);

impl Default for LedColorChunk {
    fn default() -> Self {
        Self([0u8; 72])
    }
}

impl AsRef<[u8]> for LedColorChunk {
    fn as_ref(&self) -> &[u8] {
        &self.0
    }
}

// Implement the GATT traits manually
impl trouble_host::types::gatt_traits::AsGatt for LedColorChunk {
    const MIN_SIZE: usize = 0;
    const MAX_SIZE: usize = 72;

    fn as_gatt(&self) -> &[u8] {
        &self.0
    }
}

impl trouble_host::types::gatt_traits::FromGatt for LedColorChunk {
    fn from_gatt(data: &[u8]) -> Result<Self, trouble_host::types::gatt_traits::FromGattError> {
        if data.len() > 72 {
            return Err(trouble_host::types::gatt_traits::FromGattError::InvalidLength);
        }
        let mut arr = [0u8; 72];
        arr[..data.len()].copy_from_slice(data);
        Ok(Self(arr))
    }
}

/// Accelerometer data packed for BLE transmission (6 bytes)
#[derive(Clone, Copy, Default)]
pub struct AccBleData {
    pub x: i16,
    pub y: i16,
    pub z: i16,
}

impl AccBleData {
    pub fn to_bytes(&self) -> [u8; 6] {
        let mut buf = [0u8; 6];
        buf[0..2].copy_from_slice(&self.x.to_le_bytes());
        buf[2..4].copy_from_slice(&self.y.to_le_bytes());
        buf[4..6].copy_from_slice(&self.z.to_le_bytes());
        buf
    }
}

/// Gyroscope data packed for BLE transmission (6 bytes)
#[derive(Clone, Copy, Default)]
pub struct GyroBleData {
    pub x: i16,
    pub y: i16,
    pub z: i16,
}

impl GyroBleData {
    pub fn to_bytes(&self) -> [u8; 6] {
        let mut buf = [0u8; 6];
        buf[0..2].copy_from_slice(&self.x.to_le_bytes());
        buf[2..4].copy_from_slice(&self.y.to_le_bytes());
        buf[4..6].copy_from_slice(&self.z.to_le_bytes());
        buf
    }
}

/// Magnetometer data packed for BLE transmission (12 bytes)
#[derive(Clone, Copy, Default)]
pub struct MagBleData {
    pub x: i32,
    pub y: i32,
    pub z: i32,
}

impl MagBleData {
    pub fn to_bytes(&self) -> [u8; 12] {
        let mut buf = [0u8; 12];
        buf[0..4].copy_from_slice(&self.x.to_le_bytes());
        buf[4..8].copy_from_slice(&self.y.to_le_bytes());
        buf[8..12].copy_from_slice(&self.z.to_le_bytes());
        buf
    }
}

/// AHRS orientation data packed for BLE transmission (12 bytes)
/// Roll, pitch, yaw as f32 in degrees
#[derive(Clone, Copy, Default)]
pub struct AhrsBleData {
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
}

impl AhrsBleData {
    pub fn to_bytes(&self) -> [u8; 12] {
        let mut buf = [0u8; 12];
        buf[0..4].copy_from_slice(&self.roll.to_le_bytes());
        buf[4..8].copy_from_slice(&self.pitch.to_le_bytes());
        buf[8..12].copy_from_slice(&self.yaw.to_le_bytes());
        buf
    }
}

/// GPS data packed for BLE transmission (15 bytes)
/// latitude: f32, longitude: f32, altitude: f32, satellites: u8, fix_quality: u8, has_fix: bool
#[derive(Clone, Copy, Default)]
pub struct GpsBleData {
    pub latitude: f32,
    pub longitude: f32,
    pub altitude: f32,
    pub satellites: u8,
    pub fix_quality: u8,
    pub has_fix: bool,
}

impl GpsBleData {
    pub fn to_bytes(&self) -> [u8; 15] {
        let mut buf = [0u8; 15];
        buf[0..4].copy_from_slice(&self.latitude.to_le_bytes());
        buf[4..8].copy_from_slice(&self.longitude.to_le_bytes());
        buf[8..12].copy_from_slice(&self.altitude.to_le_bytes());
        buf[12] = self.satellites;
        buf[13] = self.fix_quality;
        buf[14] = if self.has_fix { 1 } else { 0 };
        buf
    }
}

/// Maximum satellites per BLE chunk (to fit in BLE MTU)
/// Each satellite: gnss_type (1) + prn (1) + elevation (1) + azimuth (2) + snr (1) = 6 bytes
/// With 1 byte count header: 1 + 12*6 = 73 bytes (fits in typical 185+ byte MTU)
pub const SATS_PER_CHUNK: usize = 12;

/// Satellite info chunk for BLE transmission
/// Format: [count, (gnss_type, prn, elevation, azimuth_lo, azimuth_hi, snr) * 12]
#[derive(Clone, Copy)]
pub struct SatelliteChunk(pub [u8; 73]);

impl Default for SatelliteChunk {
    fn default() -> Self {
        Self([0u8; 73])
    }
}

impl AsRef<[u8]> for SatelliteChunk {
    fn as_ref(&self) -> &[u8] {
        &self.0
    }
}

impl trouble_host::types::gatt_traits::AsGatt for SatelliteChunk {
    const MIN_SIZE: usize = 0;
    const MAX_SIZE: usize = 73;

    fn as_gatt(&self) -> &[u8] {
        &self.0
    }
}

impl trouble_host::types::gatt_traits::FromGatt for SatelliteChunk {
    fn from_gatt(data: &[u8]) -> Result<Self, trouble_host::types::gatt_traits::FromGattError> {
        if data.len() > 73 {
            return Err(trouble_host::types::gatt_traits::FromGattError::InvalidLength);
        }
        let mut arr = [0u8; 73];
        arr[..data.len()].copy_from_slice(data);
        Ok(Self(arr))
    }
}

/// Satellite BLE data with satellite details
#[derive(Clone, Copy, Default)]
pub struct SatelliteBleData {
    /// GNSS type (0=Unknown, 1=GPS, 2=GLONASS, 3=Galileo, 4=BeiDou, 5=QZSS, 6=SBAS)
    pub gnss_type: u8,
    /// Satellite PRN/ID
    pub prn: u8,
    /// Elevation in degrees (0-90)
    pub elevation: u8,
    /// Azimuth in degrees (0-359)
    pub azimuth: u16,
    /// Signal-to-noise ratio in dB-Hz (0-99)
    pub snr: u8,
}

impl SatelliteBleData {
    /// Serialize satellite data to 6 bytes
    pub fn to_bytes(&self) -> [u8; 6] {
        [
            self.gnss_type,
            self.prn,
            self.elevation,
            (self.azimuth & 0xFF) as u8,
            ((self.azimuth >> 8) & 0xFF) as u8,
            self.snr,
        ]
    }
}

/// Serialize a chunk of satellites (up to 12) into BLE format
/// Returns the chunk data with count in first byte
pub fn satellites_to_chunk(satellites: &[SatelliteBleData]) -> SatelliteChunk {
    let mut chunk = SatelliteChunk::default();
    let count = satellites.len().min(SATS_PER_CHUNK);
    chunk.0[0] = count as u8;

    for (i, sat) in satellites.iter().take(SATS_PER_CHUNK).enumerate() {
        let offset = 1 + i * 6;
        let bytes = sat.to_bytes();
        chunk.0[offset..offset + 6].copy_from_slice(&bytes);
    }

    chunk
}

/// Pebble Sensor Service GATT definition
#[gatt_service(uuid = "12345678-1234-5678-1234-56789abcdef0")]
pub struct SensorService {
    /// Accelerometer data characteristic
    /// 6 bytes: x, y, z (all i16, little-endian)
    #[descriptor(uuid = "2901", read, value = "Accelerometer")]
    #[characteristic(uuid = "12345678-1234-5678-1234-56789abcdef1", read, notify)]
    pub acc_data: [u8; 6],

    /// Gyroscope data characteristic
    /// 6 bytes: x, y, z (all i16, little-endian)
    #[descriptor(uuid = "2901", read, value = "Gyroscope")]
    #[characteristic(uuid = "12345678-1234-5678-1234-56789abcdef2", read, notify)]
    pub gyro_data: [u8; 6],

    /// Magnetometer data characteristic
    /// 12 bytes: x, y, z (all i32, little-endian)
    #[descriptor(uuid = "2901", read, value = "Magnetometer")]
    #[characteristic(uuid = "12345678-1234-5678-1234-56789abcdef3", read, notify)]
    pub mag_data: [u8; 12],

    /// AHRS orientation characteristic
    /// 12 bytes: roll, pitch, yaw (all f32, little-endian, degrees)
    #[descriptor(uuid = "2901", read, value = "Orientation")]
    #[characteristic(uuid = "12345678-1234-5678-1234-56789abcdef9", read, notify)]
    pub orientation: [u8; 12],

    /// LED control characteristic
    /// Write format: [brightness, led_index, r, g, b] or [brightness, 0xFF, r, g, b] to set all LEDs
    /// Read/notify returns current state: [brightness, last_led_index, r, g, b]
    #[descriptor(uuid = "2901", read, value = "LED Control")]
    #[characteristic(uuid = "12345678-1234-5678-1234-56789abcdef4", read, write, notify)]
    pub led_control: [u8; 5],

    /// LED brightness characteristic
    /// Single byte: global brightness 0-255
    #[descriptor(uuid = "2901", read, value = "LED Brightness")]
    #[characteristic(uuid = "12345678-1234-5678-1234-56789abcdef5", read, write, notify)]
    pub led_brightness: u8,

    /// LED colors chunk 1 (LEDs 0-23)
    /// 72 bytes: 24 LEDs * 3 bytes (R, G, B) each
    #[descriptor(uuid = "2901", read, value = "LED Colors 0-23")]
    #[characteristic(uuid = "12345678-1234-5678-1234-56789abcdef6", read, write, notify)]
    pub led_colors_0: LedColorChunk,

    /// LED colors chunk 2 (LEDs 24-47)
    /// 72 bytes: 24 LEDs * 3 bytes (R, G, B) each
    #[descriptor(uuid = "2901", read, value = "LED Colors 24-47")]
    #[characteristic(uuid = "12345678-1234-5678-1234-56789abcdef7", read, write, notify)]
    pub led_colors_1: LedColorChunk,

    /// LED colors chunk 3 (LEDs 48-71)
    /// 72 bytes: 24 LEDs * 3 bytes (R, G, B) each
    #[descriptor(uuid = "2901", read, value = "LED Colors 48-71")]
    #[characteristic(uuid = "12345678-1234-5678-1234-56789abcdef8", read, write, notify)]
    pub led_colors_2: LedColorChunk,

    /// GPS data characteristic
    /// 15 bytes: latitude (f32), longitude (f32), altitude (f32), satellites (u8), fix_quality (u8), has_fix (u8)
    #[descriptor(uuid = "2901", read, value = "GPS Position")]
    #[characteristic(uuid = "12345678-1234-5678-1234-56789abcdefa", read, notify)]
    pub gps_data: [u8; 15],

    /// Satellite info chunk 0 (satellites 0-11)
    /// 73 bytes: count (u8), then 12x [gnss_type (u8), prn (u8), elevation (u8), azimuth (u16 LE), snr (u8)]
    #[descriptor(uuid = "2901", read, value = "Satellites 0-11")]
    #[characteristic(uuid = "12345678-1234-5678-1234-56789abcdefc", read, notify)]
    pub satellites_0: SatelliteChunk,

    /// Satellite info chunk 1 (satellites 12-23)
    /// 73 bytes: count (u8), then 12x [gnss_type (u8), prn (u8), elevation (u8), azimuth (u16 LE), snr (u8)]
    #[descriptor(uuid = "2901", read, value = "Satellites 12-23")]
    #[characteristic(uuid = "12345678-1234-5678-1234-56789abcdefd", read, notify)]
    pub satellites_1: SatelliteChunk,

    /// Device status characteristic
    /// 16 bytes: [status, error] pairs for LEDs, GPS, Servo, IMU, Magnetometer + 6 reserved
    /// Status: 0 = NotInitialized, 1 = Ok, 2 = Error
    /// Error codes: 0 = None, 1 = InitFailed, 2 = I2cError, 3 = UartError, 4 = TimerError,
    ///              5 = ChannelError, 6 = ChipIdMismatch, 7 = Timeout, 8 = OtpError
    #[descriptor(uuid = "2901", read, value = "Device Status")]
    #[characteristic(uuid = "12345678-1234-5678-1234-56789abcdefb", read, notify)]
    pub device_status: [u8; 16],
}

/// GATT Server with Sensor Service
/// attribute_table_size: 11 characteristics × 4 attrs + service + GAP = ~52
#[gatt_server(attribute_table_size = 128)]
pub struct SensorServer {
    pub sensor_service: SensorService,
}
