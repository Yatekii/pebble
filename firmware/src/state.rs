//! Global state and inter-task communication.
//!
//! This module contains all the shared state watches used for communication
//! between async tasks.

use core::sync::atomic::AtomicU8;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::watch::Watch;

use crate::comms::ble::{AccBleData, AhrsBleData, GpsBleData, GyroBleData, MagBleData};

/// Sensor data aggregated from IMU and AHRS filter.
#[derive(Clone, Copy, Default)]
pub struct SensorData {
    pub acc: AccBleData,
    pub gyro: GyroBleData,
    pub mag: MagBleData,
    pub orientation: AhrsBleData,
    pub valid: bool,
}

/// LED command received from BLE.
///
/// Special values for `led_index`:
/// - `0xFF`: Apply to all LEDs
/// - `0xFE`: Brightness only update
/// - `0xF0`: Colors chunk 0 update
/// - `0xF1`: Colors chunk 1 update
/// - `0xF2`: Colors chunk 2 update
#[derive(Clone, Copy, Default)]
pub struct LedCommand {
    pub brightness: u8,
    pub led_index: u8,
    pub r: u8,
    pub g: u8,
    pub b: u8,
}

/// Watch for broadcasting sensor data to multiple connection handlers.
pub static SENSOR_DATA: Watch<CriticalSectionRawMutex, SensorData, 2> = Watch::new();

/// Watch for broadcasting GPS data to multiple connection handlers.
pub static GPS_DATA: Watch<CriticalSectionRawMutex, GpsBleData, 2> = Watch::new();

/// Watch for LED commands from BLE.
pub static LED_COMMAND: Watch<CriticalSectionRawMutex, LedCommand, 2> = Watch::new();

/// LED color chunks (24 LEDs * 3 bytes = 72 bytes each).
pub static LED_COLORS_0: Watch<CriticalSectionRawMutex, [u8; 72], 2> = Watch::new();
pub static LED_COLORS_1: Watch<CriticalSectionRawMutex, [u8; 72], 2> = Watch::new();
pub static LED_COLORS_2: Watch<CriticalSectionRawMutex, [u8; 72], 2> = Watch::new();

/// Track number of active BLE connections.
pub static ACTIVE_CONNECTIONS: AtomicU8 = AtomicU8::new(0);

/// Compass heading in degrees (0-359), broadcast from IMU task to LED task.
pub static COMPASS_HEADING: Watch<CriticalSectionRawMutex, u16, 2> = Watch::new();

/// Watch for broadcasting device status to BLE clients.
pub static DEVICE_STATUS: Watch<CriticalSectionRawMutex, DeviceStatus, 2> = Watch::new();

/// Device initialization status for all peripherals.
#[derive(Clone, Copy, Default)]
pub struct DeviceStatus {
    pub leds: PeripheralStatus,
    pub gps: PeripheralStatus,
    pub servo: PeripheralStatus,
    pub imu: PeripheralStatus,
    pub magnetometer: PeripheralStatus,
}

impl DeviceStatus {
    /// Serialize to 16 bytes for BLE transmission.
    /// Format: [status, error] pairs for each peripheral, then 6 reserved bytes.
    pub fn to_bytes(&self) -> [u8; 16] {
        let mut bytes = [0u8; 16];
        let (s, e) = self.leds.to_bytes();
        bytes[0] = s;
        bytes[1] = e;
        let (s, e) = self.gps.to_bytes();
        bytes[2] = s;
        bytes[3] = e;
        let (s, e) = self.servo.to_bytes();
        bytes[4] = s;
        bytes[5] = e;
        let (s, e) = self.imu.to_bytes();
        bytes[6] = s;
        bytes[7] = e;
        let (s, e) = self.magnetometer.to_bytes();
        bytes[8] = s;
        bytes[9] = e;
        // bytes[10..16] are reserved (already 0)
        bytes
    }
}

/// Status of a single peripheral.
#[derive(Clone, Copy, Default)]
pub enum PeripheralStatus {
    #[default]
    NotInitialized,
    Ok,
    Error(PeripheralError),
}

impl PeripheralStatus {
    /// Convert to (status_code, error_code) pair.
    pub fn to_bytes(&self) -> (u8, u8) {
        match self {
            PeripheralStatus::NotInitialized => (0, 0),
            PeripheralStatus::Ok => (1, 0),
            PeripheralStatus::Error(e) => (2, e.to_code()),
        }
    }
}

/// Error codes for peripheral initialization failures.
#[derive(Clone, Copy)]
pub enum PeripheralError {
    InitFailed = 1,
    I2cError = 2,
    UartError = 3,
    TimerError = 4,
    ChannelError = 5,
    ChipIdMismatch = 6,
    Timeout = 7,
    OtpError = 8,
}

impl PeripheralError {
    pub fn to_code(&self) -> u8 {
        *self as u8
    }
}
