//! BLE GATT server for sensor data
//!
//! Provides a custom GATT service for accelerometer, gyroscope, and magnetometer data.

use trouble_host::prelude::*;

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
}

/// GATT Server with Sensor Service
#[gatt_server]
pub struct SensorServer {
    pub sensor_service: SensorService,
}
