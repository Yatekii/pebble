//! BLE client for connecting to the Pebble sensor service.

use std::sync::Arc;
use std::sync::mpsc;
use std::time::Duration;

use btleplug::api::{Central, Manager as _, Peripheral as _, ScanFilter};
use btleplug::platform::{Adapter, Manager, Peripheral};
use futures::StreamExt;
use parking_lot::Mutex;
use uuid::Uuid;

use crate::data::{ImuHistory, ImuReading};

/// Custom UUID for the Pebble Sensor Service
const SENSOR_SERVICE_UUID: Uuid = Uuid::from_bytes([
    0x12, 0x34, 0x56, 0x78, 0x12, 0x34, 0x56, 0x78, 0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc, 0xde, 0xf0,
]);

/// UUID for accelerometer data characteristic (6 bytes: 3x i16)
const ACC_DATA_UUID: Uuid = Uuid::from_bytes([
    0x12, 0x34, 0x56, 0x78, 0x12, 0x34, 0x56, 0x78, 0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc, 0xde, 0xf1,
]);

/// UUID for gyroscope data characteristic (6 bytes: 3x i16)
const GYRO_DATA_UUID: Uuid = Uuid::from_bytes([
    0x12, 0x34, 0x56, 0x78, 0x12, 0x34, 0x56, 0x78, 0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc, 0xde, 0xf2,
]);

/// UUID for magnetometer data characteristic (12 bytes: 3x i32)
const MAG_DATA_UUID: Uuid = Uuid::from_bytes([
    0x12, 0x34, 0x56, 0x78, 0x12, 0x34, 0x56, 0x78, 0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc, 0xde, 0xf3,
]);

/// UUID for LED colors chunk 0 (LEDs 0-23, 72 bytes: 24 LEDs × 3 RGB bytes)
const LED_COLORS_0_UUID: Uuid = Uuid::from_bytes([
    0x12, 0x34, 0x56, 0x78, 0x12, 0x34, 0x56, 0x78, 0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc, 0xde, 0xf6,
]);

/// UUID for LED colors chunk 1 (LEDs 24-47, 72 bytes: 24 LEDs × 3 RGB bytes)
const LED_COLORS_1_UUID: Uuid = Uuid::from_bytes([
    0x12, 0x34, 0x56, 0x78, 0x12, 0x34, 0x56, 0x78, 0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc, 0xde, 0xf7,
]);

/// UUID for LED colors chunk 2 (LEDs 48-71, 72 bytes: 24 LEDs × 3 RGB bytes)
const LED_COLORS_2_UUID: Uuid = Uuid::from_bytes([
    0x12, 0x34, 0x56, 0x78, 0x12, 0x34, 0x56, 0x78, 0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc, 0xde, 0xf8,
]);

/// UUID for AHRS orientation data characteristic (12 bytes: 3x f32 roll, pitch, yaw in degrees)
const AHRS_DATA_UUID: Uuid = Uuid::from_bytes([
    0x12, 0x34, 0x56, 0x78, 0x12, 0x34, 0x56, 0x78, 0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc, 0xde, 0xf9,
]);

/// UUID for GPS data characteristic (15 bytes: lat, lon, alt as f32, satellites, fix_quality, has_fix as u8)
const GPS_DATA_UUID: Uuid = Uuid::from_bytes([
    0x12, 0x34, 0x56, 0x78, 0x12, 0x34, 0x56, 0x78, 0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc, 0xde, 0xfa,
]);

/// UUID for device status characteristic (16 bytes: status/error pairs for each peripheral)
const DEVICE_STATUS_UUID: Uuid = Uuid::from_bytes([
    0x12, 0x34, 0x56, 0x78, 0x12, 0x34, 0x56, 0x78, 0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc, 0xde, 0xfb,
]);

/// UUID for satellite info chunk 0 (satellites 0-11)
const SATELLITES_0_UUID: Uuid = Uuid::from_bytes([
    0x12, 0x34, 0x56, 0x78, 0x12, 0x34, 0x56, 0x78, 0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc, 0xde, 0xfc,
]);

/// UUID for satellite info chunk 1 (satellites 12-23)
const SATELLITES_1_UUID: Uuid = Uuid::from_bytes([
    0x12, 0x34, 0x56, 0x78, 0x12, 0x34, 0x56, 0x78, 0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc, 0xde, 0xfd,
]);

/// BLE connection state
#[derive(Clone, Debug, PartialEq)]
pub enum ConnectionState {
    Disconnected,
    Scanning,
    Connecting,
    Connected,
    Error(String),
}

/// RGB color for a single LED
pub type LedColor = [u8; 3];

/// All 72 LED colors
pub type LedColors = [LedColor; 72];

/// AHRS orientation reading (roll, pitch, yaw in degrees)
#[derive(Clone, Copy, Debug, Default)]
pub struct AhrsReading {
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
}

/// GPS reading
#[derive(Clone, Copy, Debug, Default)]
pub struct GpsReading {
    pub latitude: f32,
    pub longitude: f32,
    pub altitude: f32,
    pub satellites: u8,
    #[allow(dead_code)]
    pub fix_quality: u8,
    pub has_fix: bool,
}

/// GNSS constellation type
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(u8)]
pub enum GnssType {
    #[default]
    Unknown = 0,
    Gps = 1,
    Glonass = 2,
    Galileo = 3,
    BeiDou = 4,
    Qzss = 5,
    Sbas = 6,
}

impl GnssType {
    pub fn from_u8(v: u8) -> Self {
        match v {
            1 => GnssType::Gps,
            2 => GnssType::Glonass,
            3 => GnssType::Galileo,
            4 => GnssType::BeiDou,
            5 => GnssType::Qzss,
            6 => GnssType::Sbas,
            _ => GnssType::Unknown,
        }
    }
}

/// Individual satellite info
#[derive(Clone, Copy, Debug, Default)]
pub struct SatelliteInfo {
    /// GNSS constellation type
    pub gnss_type: GnssType,
    /// Satellite PRN/ID
    pub prn: u8,
    /// Elevation in degrees (0-90)
    #[expect(unused)]
    pub elevation: u8,
    /// Azimuth in degrees (0-359)
    #[expect(unused)]
    pub azimuth: u16,
    /// Signal-to-noise ratio in dB-Hz (0-99)
    pub snr: u8,
}

/// Parse satellite chunk from BLE data (73 bytes: count + 12 * 6 bytes)
fn parse_satellite_chunk(data: &[u8], _chunk_index: u8) -> Vec<SatelliteInfo> {
    if data.is_empty() {
        return Vec::new();
    }
    let count = data[0] as usize;
    let mut satellites = Vec::with_capacity(count.min(12));

    for i in 0..count.min(12) {
        let offset = 1 + i * 6;
        if offset + 6 > data.len() {
            break;
        }
        satellites.push(SatelliteInfo {
            gnss_type: GnssType::from_u8(data[offset]),
            prn: data[offset + 1],
            elevation: data[offset + 2],
            azimuth: u16::from_le_bytes([data[offset + 3], data[offset + 4]]),
            snr: data[offset + 5],
        });
    }

    satellites
}

/// Device peripheral status
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct PeripheralStatusData {
    /// 0 = NotInitialized, 1 = Ok, 2 = Error
    pub status: u8,
    /// Error code (0 = None, 1 = InitFailed, 2 = I2cError, etc.)
    pub error: u8,
}

impl PeripheralStatusData {
    pub fn status_text(&self) -> &'static str {
        match self.status {
            0 => "Not Initialized",
            1 => "OK",
            2 => "Error",
            _ => "Unknown",
        }
    }

    pub fn error_text(&self) -> Option<&'static str> {
        if self.status != 2 {
            return None;
        }
        Some(match self.error {
            1 => "Init Failed",
            2 => "I2C Error",
            3 => "UART Error",
            4 => "Timer Error",
            5 => "Channel Error",
            6 => "Chip ID Mismatch",
            7 => "Timeout",
            8 => "OTP Error",
            _ => "Unknown Error",
        })
    }
}

/// Device status for all peripherals
#[derive(Clone, Copy, Debug, Default)]
pub struct DeviceStatusData {
    pub leds: PeripheralStatusData,
    pub gps: PeripheralStatusData,
    pub servo: PeripheralStatusData,
    pub imu: PeripheralStatusData,
    pub magnetometer: PeripheralStatusData,
}

impl DeviceStatusData {
    /// Parse from 16-byte BLE data
    pub fn from_bytes(data: &[u8]) -> Option<Self> {
        if data.len() < 10 {
            return None;
        }
        Some(Self {
            leds: PeripheralStatusData {
                status: data[0],
                error: data[1],
            },
            gps: PeripheralStatusData {
                status: data[2],
                error: data[3],
            },
            servo: PeripheralStatusData {
                status: data[4],
                error: data[5],
            },
            imu: PeripheralStatusData {
                status: data[6],
                error: data[7],
            },
            magnetometer: PeripheralStatusData {
                status: data[8],
                error: data[9],
            },
        })
    }
}

/// Message sent from BLE task to the UI
pub enum BleMessage {
    StateChanged(ConnectionState),
    AccelData(ImuReading),
    GyroData(ImuReading),
    MagData(ImuReading),
    AhrsData(AhrsReading),
    GpsData(GpsReading),
    /// LED colors update: chunk index (0, 1, or 2) and 24 LED colors
    LedColorsChunk(u8, [LedColor; 24]),
    /// Device status update
    DeviceStatus(DeviceStatusData),
    /// Satellite info chunk (0 or 1)
    SatelliteChunk(u8, Vec<SatelliteInfo>),
}

/// Shared state between BLE task and UI
pub struct BleState {
    pub connection_state: ConnectionState,
    pub imu_history: ImuHistory,
}

impl BleState {
    pub fn new() -> Self {
        Self {
            connection_state: ConnectionState::Disconnected,
            imu_history: ImuHistory::new(200),
        }
    }
}

/// Start the BLE client in a background tokio runtime.
/// Returns a receiver for BLE messages (std::sync::mpsc for cross-runtime compatibility).
pub fn start_ble_client(state: Arc<Mutex<BleState>>) -> mpsc::Receiver<BleMessage> {
    let (tx, rx) = mpsc::channel();

    std::thread::spawn(move || {
        let rt = match tokio::runtime::Runtime::new() {
            Ok(rt) => rt,
            Err(e) => {
                let _ = tx.send(BleMessage::StateChanged(ConnectionState::Error(format!(
                    "Failed to create tokio runtime: {e}"
                ))));
                return;
            }
        };
        rt.block_on(async move {
            if let Err(e) = run_ble_client(tx.clone(), state).await {
                let _ = tx.send(BleMessage::StateChanged(ConnectionState::Error(
                    e.to_string(),
                )));
            }
        });
    });

    rx
}

async fn run_ble_client(
    tx: mpsc::Sender<BleMessage>,
    state: Arc<Mutex<BleState>>,
) -> anyhow::Result<()> {
    // Track the last connected device ID to detect stale peripherals
    let mut last_device_id: Option<String> = None;

    loop {
        // Re-create the manager and adapter on each connection attempt.
        // This is a workaround for a btleplug bug on macOS where peripheral objects
        // become stale after disconnect - their notification streams stop working.
        // By recreating the manager, we ensure we get fresh peripheral instances.
        // See: https://github.com/deviceplug/btleplug/issues/413
        let manager = Manager::new().await?;
        let adapters = manager.adapters().await?;
        let central = adapters
            .into_iter()
            .next()
            .ok_or_else(|| anyhow::anyhow!("No Bluetooth adapters found"))?;

        // Start scanning
        eprintln!("Starting BLE scan...");
        let _ = tx.send(BleMessage::StateChanged(ConnectionState::Scanning));
        state.lock().connection_state = ConnectionState::Scanning;

        // Clear any cached peripherals by restarting the scan
        let _ = central.stop_scan().await;
        central.start_scan(ScanFilter::default()).await?;

        // Wait for device discovery
        let device = find_pebble_device(&central, last_device_id.as_deref()).await;

        let _ = central.stop_scan().await;

        let Some(device) = device else {
            tokio::time::sleep(Duration::from_secs(1)).await;
            continue;
        };

        // Store the device ID for next iteration
        last_device_id = Some(device.id().to_string());

        // Connect to device
        let _ = tx.send(BleMessage::StateChanged(ConnectionState::Connecting));
        state.lock().connection_state = ConnectionState::Connecting;

        // connect() can hang forever on macOS if the peripheral is wedged (e.g.
        // ESP holding a stale connection); bound it so the loop retries instead.
        match tokio::time::timeout(Duration::from_secs(10), device.connect()).await {
            Ok(Ok(())) => {}
            Ok(Err(e)) => {
                eprintln!("Failed to connect: {:?}", e);
                let _ = tx.send(BleMessage::StateChanged(ConnectionState::Disconnected));
                state.lock().connection_state = ConnectionState::Disconnected;
                tokio::time::sleep(Duration::from_secs(1)).await;
                continue;
            }
            Err(_) => {
                eprintln!("connect() timed out after 10s, retrying");
                let _ = tx.send(BleMessage::StateChanged(ConnectionState::Disconnected));
                state.lock().connection_state = ConnectionState::Disconnected;
                tokio::time::sleep(Duration::from_secs(1)).await;
                continue;
            }
        }

        // Verify connection is actually established
        match device.is_connected().await {
            Ok(true) => eprintln!("Connection verified"),
            Ok(false) => {
                eprintln!("Device reports not connected after connect() succeeded");
                let _ = tx.send(BleMessage::StateChanged(ConnectionState::Disconnected));
                state.lock().connection_state = ConnectionState::Disconnected;
                tokio::time::sleep(Duration::from_secs(1)).await;
                continue;
            }
            Err(e) => {
                eprintln!("Failed to verify connection: {:?}", e);
                let _ = tx.send(BleMessage::StateChanged(ConnectionState::Disconnected));
                state.lock().connection_state = ConnectionState::Disconnected;
                tokio::time::sleep(Duration::from_secs(1)).await;
                continue;
            }
        }

        // Discover services
        if let Err(e) = device.discover_services().await {
            eprintln!("Failed to discover services: {:?}", e);
            let _ = device.disconnect().await;
            let _ = tx.send(BleMessage::StateChanged(ConnectionState::Disconnected));
            state.lock().connection_state = ConnectionState::Disconnected;
            tokio::time::sleep(Duration::from_secs(1)).await;
            continue;
        }

        // Check we actually found characteristics before declaring connected
        let characteristics = device.characteristics();
        if characteristics.is_empty() {
            eprintln!("No characteristics found, connection may be stale");
            let _ = device.disconnect().await;
            let _ = tx.send(BleMessage::StateChanged(ConnectionState::Disconnected));
            state.lock().connection_state = ConnectionState::Disconnected;
            tokio::time::sleep(Duration::from_secs(1)).await;
            continue;
        }

        let _ = tx.send(BleMessage::StateChanged(ConnectionState::Connected));
        state.lock().connection_state = ConnectionState::Connected;

        // Log characteristics
        eprintln!("Found {} characteristics:", characteristics.len());
        for c in &characteristics {
            eprintln!("  - {} (props: {:?})", c.uuid, c.properties);
        }

        // Debug: print expected UUIDs
        eprintln!("Looking for AHRS: {}", AHRS_DATA_UUID);
        eprintln!("Looking for LED0: {}", LED_COLORS_0_UUID);
        eprintln!("Looking for LED1: {}", LED_COLORS_1_UUID);
        eprintln!("Looking for LED2: {}", LED_COLORS_2_UUID);

        let acc_char = characteristics.iter().find(|c| c.uuid == ACC_DATA_UUID);
        let gyro_char = characteristics.iter().find(|c| c.uuid == GYRO_DATA_UUID);
        let mag_char = characteristics.iter().find(|c| c.uuid == MAG_DATA_UUID);
        let ahrs_char = characteristics.iter().find(|c| c.uuid == AHRS_DATA_UUID);
        let gps_char = characteristics.iter().find(|c| c.uuid == GPS_DATA_UUID);
        let sats0_char = characteristics.iter().find(|c| c.uuid == SATELLITES_0_UUID);
        let sats1_char = characteristics.iter().find(|c| c.uuid == SATELLITES_1_UUID);
        let led0_char = characteristics.iter().find(|c| c.uuid == LED_COLORS_0_UUID);
        let led1_char = characteristics.iter().find(|c| c.uuid == LED_COLORS_1_UUID);
        let led2_char = characteristics.iter().find(|c| c.uuid == LED_COLORS_2_UUID);
        let status_char = characteristics
            .iter()
            .find(|c| c.uuid == DEVICE_STATUS_UUID);

        // Subscribe to notifications.
        for (name, ch) in [
            ("ACC", acc_char),
            ("GYRO", gyro_char),
            ("MAG", mag_char),
            ("AHRS", ahrs_char),
            ("GPS", gps_char),
            ("Satellites0", sats0_char),
            ("Satellites1", sats1_char),
            ("LED0", led0_char),
            ("LED1", led1_char),
            ("LED2", led2_char),
            ("DeviceStatus", status_char),
        ] {
            match ch {
                Some(ch) => match device.subscribe(ch).await {
                    Ok(_) => eprintln!("  Subscribed to {name}"),
                    Err(e) => eprintln!("  Failed to subscribe to {name}: {e:?}"),
                },
                None => eprintln!("{name} characteristic not found!"),
            }
        }

        // Read LED colors once on connect to get initial state
        read_led_colors(&device, led0_char, led1_char, led2_char, &tx).await;

        // Read device status once on connect
        if let Some(status_char) = status_char {
            match device.read(status_char).await {
                Ok(data) => {
                    eprintln!("DeviceStatus read: {} bytes", data.len());
                    if let Some(status) = DeviceStatusData::from_bytes(&data) {
                        eprintln!(
                            "  LEDs={}, GPS={}, Servo={}, IMU={}, Mag={}",
                            status.leds.status_text(),
                            status.gps.status_text(),
                            status.servo.status_text(),
                            status.imu.status_text(),
                            status.magnetometer.status_text()
                        );
                        let _ = tx.send(BleMessage::DeviceStatus(status));
                    }
                }
                Err(e) => eprintln!("DeviceStatus read error: {:?}", e),
            }
        }

        // Listen for notifications
        eprintln!("Waiting for notifications...");
        let mut notification_stream = device.notifications().await?;
        let mut notification_count = 0u64;
        let mut last_data_time = std::time::Instant::now();
        // Force reconnect if no data for 2 seconds - macOS is_connected() is unreliable
        let max_silence = Duration::from_secs(2);

        loop {
            // Check for timeout - if no notifications for 500ms, check connection
            let timeout_result =
                tokio::time::timeout(Duration::from_millis(500), notification_stream.next()).await;

            let notification = match timeout_result {
                Ok(Some(n)) => {
                    last_data_time = std::time::Instant::now();
                    n
                }
                Ok(None) => {
                    eprintln!("Notification stream ended (returned None)");
                    break;
                }
                Err(_) => {
                    // Timeout fired - check if we've been silent too long
                    let silence_duration = last_data_time.elapsed();
                    if silence_duration > max_silence {
                        eprintln!("No data for {:?}, assuming disconnected", silence_duration);
                        break;
                    }
                    // Don't trust is_connected() on macOS - it often returns true for dead connections
                    // Just continue waiting until max_silence is reached
                    continue;
                }
            };

            notification_count += 1;
            if notification_count <= 10 || notification_count.is_multiple_of(500) {
                eprintln!(
                    "Notification #{}: uuid={}, len={}",
                    notification_count,
                    notification.uuid,
                    notification.value.len()
                );
            }

            if notification.uuid == ACC_DATA_UUID && notification.value.len() >= 6 {
                let data = &notification.value;
                let accel = ImuReading {
                    x: i16::from_le_bytes([data[0], data[1]]) as f64,
                    y: i16::from_le_bytes([data[2], data[3]]) as f64,
                    z: i16::from_le_bytes([data[4], data[5]]) as f64,
                };
                if tx.send(BleMessage::AccelData(accel)).is_err() {
                    eprintln!("Channel closed, UI receiver dropped");
                    break;
                }
            } else if notification.uuid == GYRO_DATA_UUID && notification.value.len() >= 6 {
                let data = &notification.value;
                let gyro = ImuReading {
                    x: i16::from_le_bytes([data[0], data[1]]) as f64,
                    y: i16::from_le_bytes([data[2], data[3]]) as f64,
                    z: i16::from_le_bytes([data[4], data[5]]) as f64,
                };
                let _ = tx.send(BleMessage::GyroData(gyro));
            } else if notification.uuid == MAG_DATA_UUID && notification.value.len() >= 12 {
                let data = &notification.value;
                let mag = ImuReading {
                    x: i32::from_le_bytes([data[0], data[1], data[2], data[3]]) as f64,
                    y: i32::from_le_bytes([data[4], data[5], data[6], data[7]]) as f64,
                    z: i32::from_le_bytes([data[8], data[9], data[10], data[11]]) as f64,
                };
                let _ = tx.send(BleMessage::MagData(mag));
            } else if notification.uuid == AHRS_DATA_UUID && notification.value.len() >= 12 {
                let data = &notification.value;
                let ahrs = AhrsReading {
                    roll: f32::from_le_bytes([data[0], data[1], data[2], data[3]]),
                    pitch: f32::from_le_bytes([data[4], data[5], data[6], data[7]]),
                    yaw: f32::from_le_bytes([data[8], data[9], data[10], data[11]]),
                };
                if notification_count <= 20 || notification_count % 100 == 0 {
                    eprintln!(
                        "AHRS received: roll={:.1} pitch={:.1} yaw={:.1}",
                        ahrs.roll, ahrs.pitch, ahrs.yaw
                    );
                }
                let _ = tx.send(BleMessage::AhrsData(ahrs));
            } else if notification.uuid == GPS_DATA_UUID && notification.value.len() >= 15 {
                let data = &notification.value;
                let gps = GpsReading {
                    latitude: f32::from_le_bytes([data[0], data[1], data[2], data[3]]),
                    longitude: f32::from_le_bytes([data[4], data[5], data[6], data[7]]),
                    altitude: f32::from_le_bytes([data[8], data[9], data[10], data[11]]),
                    satellites: data[12],
                    fix_quality: data[13],
                    has_fix: data[14] != 0,
                };
                if notification_count <= 20 || notification_count % 100 == 0 {
                    eprintln!(
                        "GPS received: lat={:.6} lon={:.6} alt={:.1} sats={} fix={}",
                        gps.latitude, gps.longitude, gps.altitude, gps.satellites, gps.has_fix
                    );
                }
                let _ = tx.send(BleMessage::GpsData(gps));
            } else if notification.uuid == LED_COLORS_0_UUID && notification.value.len() >= 72 {
                if notification_count <= 20 || notification_count % 100 == 0 {
                    eprintln!(
                        "LED0 notification: first 9 bytes = {:?}",
                        &notification.value[..9]
                    );
                }
                if let Some(colors) = parse_led_colors(&notification.value) {
                    let _ = tx.send(BleMessage::LedColorsChunk(0, colors));
                }
            } else if notification.uuid == LED_COLORS_1_UUID && notification.value.len() >= 72 {
                if notification_count <= 20 || notification_count % 100 == 0 {
                    eprintln!(
                        "LED1 notification: first 9 bytes = {:?}",
                        &notification.value[..9]
                    );
                }
                if let Some(colors) = parse_led_colors(&notification.value) {
                    let _ = tx.send(BleMessage::LedColorsChunk(1, colors));
                }
            } else if notification.uuid == LED_COLORS_2_UUID && notification.value.len() >= 72 {
                if notification_count <= 20 || notification_count % 100 == 0 {
                    eprintln!(
                        "LED2 notification: first 9 bytes = {:?}",
                        &notification.value[..9]
                    );
                }
                if let Some(colors) = parse_led_colors(&notification.value) {
                    let _ = tx.send(BleMessage::LedColorsChunk(2, colors));
                }
            } else if notification.uuid == DEVICE_STATUS_UUID && notification.value.len() >= 10 {
                if let Some(status) = DeviceStatusData::from_bytes(&notification.value) {
                    eprintln!(
                        "DeviceStatus notification: LEDs={}, GPS={}, Servo={}, IMU={}, Mag={}",
                        status.leds.status_text(),
                        status.gps.status_text(),
                        status.servo.status_text(),
                        status.imu.status_text(),
                        status.magnetometer.status_text()
                    );
                    let _ = tx.send(BleMessage::DeviceStatus(status));
                }
            } else if notification.uuid == SATELLITES_0_UUID && !notification.value.is_empty() {
                let satellites = parse_satellite_chunk(&notification.value, 0);
                if notification_count <= 20 || notification_count % 100 == 0 {
                    eprintln!("Satellites0 received: {} satellites", satellites.len());
                }
                let _ = tx.send(BleMessage::SatelliteChunk(0, satellites));
            } else if notification.uuid == SATELLITES_1_UUID && !notification.value.is_empty() {
                let satellites = parse_satellite_chunk(&notification.value, 1);
                if notification_count <= 20 || notification_count % 100 == 0 {
                    eprintln!("Satellites1 received: {} satellites", satellites.len());
                }
                let _ = tx.send(BleMessage::SatelliteChunk(1, satellites));
            }
        }

        eprintln!(
            "Notification stream ended after {} notifications",
            notification_count
        );

        // Update UI state immediately - don't wait for disconnect to complete
        eprintln!("Connection lost, updating UI state...");
        let _ = tx.send(BleMessage::StateChanged(ConnectionState::Disconnected));
        state.lock().connection_state = ConnectionState::Disconnected;

        // Try to disconnect cleanly, but with a timeout since it can hang on macOS
        eprintln!("Disconnecting device...");
        match tokio::time::timeout(Duration::from_secs(2), device.disconnect()).await {
            Ok(Ok(_)) => eprintln!("Device disconnected successfully"),
            Ok(Err(e)) => eprintln!("Device disconnect error (expected): {:?}", e),
            Err(_) => eprintln!("Device disconnect timed out (ignoring)"),
        }

        // Will automatically reconnect
        eprintln!("Will reconnect in 1 second...");
        tokio::time::sleep(Duration::from_secs(1)).await;
        eprintln!("Looping back to scan...");
    }
}

/// Parse 72 bytes into 24 LED RGB colors
fn parse_led_colors(data: &[u8]) -> Option<[LedColor; 24]> {
    if data.len() < 72 {
        return None;
    }
    let mut colors = [[0u8; 3]; 24];
    for i in 0..24 {
        colors[i] = [data[i * 3], data[i * 3 + 1], data[i * 3 + 2]];
    }
    Some(colors)
}

/// Read LED colors from characteristics (for polling since firmware doesn't notify)
async fn read_led_colors(
    device: &Peripheral,
    led0_char: Option<&btleplug::api::Characteristic>,
    led1_char: Option<&btleplug::api::Characteristic>,
    led2_char: Option<&btleplug::api::Characteristic>,
    tx: &mpsc::Sender<BleMessage>,
) {
    if let Some(char) = led0_char {
        match device.read(char).await {
            Ok(data) => {
                eprintln!(
                    "LED0 read: {} bytes, first 9: {:?}",
                    data.len(),
                    &data[..data.len().min(9)]
                );
                if let Some(colors) = parse_led_colors(&data) {
                    let _ = tx.send(BleMessage::LedColorsChunk(0, colors));
                }
            }
            Err(e) => eprintln!("LED0 read error: {:?}", e),
        }
    }
    if let Some(char) = led1_char {
        match device.read(char).await {
            Ok(data) => {
                eprintln!(
                    "LED1 read: {} bytes, first 9: {:?}",
                    data.len(),
                    &data[..data.len().min(9)]
                );
                if let Some(colors) = parse_led_colors(&data) {
                    let _ = tx.send(BleMessage::LedColorsChunk(1, colors));
                }
            }
            Err(e) => eprintln!("LED1 read error: {:?}", e),
        }
    }
    if let Some(char) = led2_char {
        match device.read(char).await {
            Ok(data) => {
                eprintln!(
                    "LED2 read: {} bytes, first 9: {:?}",
                    data.len(),
                    &data[..data.len().min(9)]
                );
                if let Some(colors) = parse_led_colors(&data) {
                    let _ = tx.send(BleMessage::LedColorsChunk(2, colors));
                }
            }
            Err(e) => eprintln!("LED2 read error: {:?}", e),
        }
    }
}

async fn find_pebble_device(central: &Adapter, last_device_id: Option<&str>) -> Option<Peripheral> {
    // Wait a bit for scanning
    tokio::time::sleep(Duration::from_secs(3)).await;

    let peripherals = central.peripherals().await.ok()?;

    eprintln!(
        "Found {} BLE peripherals (last_device_id: {:?})",
        peripherals.len(),
        last_device_id
    );

    for peripheral in &peripherals {
        let id = peripheral.id().to_string();

        // Try to get properties
        if let Ok(Some(props)) = peripheral.properties().await {
            let name = props.local_name.as_deref().unwrap_or("<no name>");
            let services: Vec<_> = props.services.iter().map(|u| u.to_string()).collect();
            eprintln!(
                "  Device: {} [{}] (services: {:?})",
                name,
                id,
                if services.is_empty() {
                    vec!["none".to_string()]
                } else {
                    services
                }
            );

            // Check if device name contains "Pebble", "ESP", or common ESP32 names
            if let Some(name) = &props.local_name {
                let name_lower = name.to_lowercase();
                if name_lower.contains("pebble")
                    || name_lower.contains("esp")
                    || name_lower.contains("nimble")
                {
                    eprintln!("  -> Found Pebble by name!");
                    return Some(peripheral.clone());
                }
            }

            // Check advertised services
            if props.services.contains(&SENSOR_SERVICE_UUID) {
                eprintln!("  -> Found Pebble by service UUID!");
                return Some(peripheral.clone());
            }
        } else {
            eprintln!("  Device: <no props> [{}]", id);
        }
    }

    eprintln!("Pebble device not found, retrying...");
    None
}
