//! Inter-layer communication using Watch channels.
//!
//! This module defines all the Watch channels used for communication between
//! firmware layers. Watches provide a pub/sub mechanism where one sender can
//! broadcast to multiple receivers.
//!
//! # Channel Categories
//!
//! ## Sensor Data (HAL → Filter → Comms)
//! Processed sensor readings from the filter layer.
//!
//! ## Device State (Filter → Control/Comms)
//! High-level device state from sensor fusion.
//!
//! ## Commands (Control/Comms → HAL)
//! Commands for actuators (LEDs, servo).
//!
//! ## Status (All layers → Comms)
//! Device health and initialization status.

use core::sync::atomic::AtomicU8;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::watch::Watch;

use crate::comms::ble::{
    AccBleData, AhrsBleData, GpsBleData, GyroBleData, MagBleData, SatelliteChunk,
};

// =============================================================================
// Sensor Data Types
// =============================================================================

/// Aggregated sensor data from IMU and AHRS filter.
///
/// Contains raw accelerometer, gyroscope, and magnetometer readings,
/// plus the computed orientation from the AHRS filter.
#[derive(Clone, Copy, Default)]
pub struct SensorData {
    /// Accelerometer data (x, y, z as i16).
    pub acc: AccBleData,
    /// Gyroscope data (x, y, z as i16).
    pub gyro: GyroBleData,
    /// Magnetometer data (x, y, z as i32, microtesla * 100).
    pub mag: MagBleData,
    /// Orientation from AHRS (roll, pitch, yaw in degrees).
    pub orientation: AhrsBleData,
    /// True if the data is valid.
    pub valid: bool,
}

// =============================================================================
// Command Types
// =============================================================================

/// LED command received from BLE or control layer.
///
/// Special values for `led_index`:
/// - `0xFF`: Apply to all LEDs
/// - `0xFE`: Brightness only update
/// - `0xF0`: Colors chunk 0 update
/// - `0xF1`: Colors chunk 1 update
/// - `0xF2`: Colors chunk 2 update
#[derive(Clone, Copy, Default)]
pub struct LedCommand {
    /// Global brightness (0-255).
    pub brightness: u8,
    /// LED index or special command.
    pub led_index: u8,
    /// Red component (0-255).
    pub r: u8,
    /// Green component (0-255).
    pub g: u8,
    /// Blue component (0-255).
    pub b: u8,
}

/// Servo command.
#[derive(Clone, Copy, Default)]
pub struct ServoCommand {
    /// Target angle in degrees (0-180).
    pub angle: u8,
}

// =============================================================================
// Status Types
// =============================================================================

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

// =============================================================================
// Watch Channels
// =============================================================================

// --- Sensor Data (Filter → Comms) ---

/// Watch for broadcasting sensor data to BLE and other consumers.
pub static SENSOR_DATA: Watch<CriticalSectionRawMutex, SensorData, 2> = Watch::new();

/// Watch for broadcasting GPS data.
pub static GPS_DATA: Watch<CriticalSectionRawMutex, GpsBleData, 2> = Watch::new();

/// Watch for broadcasting satellite info chunk 0 (satellites 0-11).
pub static SATELLITES_0: Watch<CriticalSectionRawMutex, SatelliteChunk, 2> = Watch::new();

/// Watch for broadcasting satellite info chunk 1 (satellites 12-23).
pub static SATELLITES_1: Watch<CriticalSectionRawMutex, SatelliteChunk, 2> = Watch::new();

// --- Device State (Filter → Control/Comms) ---

/// Compass heading in degrees (0-359), broadcast from filter to LED task.
pub static COMPASS_HEADING: Watch<CriticalSectionRawMutex, u16, 2> = Watch::new();

// --- Commands (Control/Comms → HAL) ---

/// Watch for LED commands from BLE or control layer.
pub static LED_COMMAND: Watch<CriticalSectionRawMutex, LedCommand, 2> = Watch::new();

/// LED color chunks (24 LEDs * 3 bytes = 72 bytes each).
pub static LED_COLORS_0: Watch<CriticalSectionRawMutex, [u8; 72], 2> = Watch::new();
pub static LED_COLORS_1: Watch<CriticalSectionRawMutex, [u8; 72], 2> = Watch::new();
pub static LED_COLORS_2: Watch<CriticalSectionRawMutex, [u8; 72], 2> = Watch::new();

/// Watch for servo commands.
pub static SERVO_COMMAND: Watch<CriticalSectionRawMutex, ServoCommand, 2> = Watch::new();

// --- Status ---

/// Watch for broadcasting device status to BLE clients.
pub static DEVICE_STATUS: Watch<CriticalSectionRawMutex, DeviceStatus, 2> = Watch::new();

// --- Connection State ---

/// Track number of active BLE connections.
pub static ACTIVE_CONNECTIONS: AtomicU8 = AtomicU8::new(0);
