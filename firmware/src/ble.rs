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
}

/// GATT Server with Sensor Service
/// attribute_table_size: 8 characteristics × 3 attrs + 8 descriptors + service + GAP = ~40
#[gatt_server(attribute_table_size = 64)]
pub struct SensorServer {
    pub sensor_service: SensorService,
}
