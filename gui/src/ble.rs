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

/// Message sent from BLE task to the UI
pub enum BleMessage {
    StateChanged(ConnectionState),
    AccelData(ImuReading),
    GyroData(ImuReading),
    MagData(ImuReading),
    /// LED colors update: chunk index (0, 1, or 2) and 24 LED colors
    LedColorsChunk(u8, [LedColor; 24]),
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
        let rt = tokio::runtime::Runtime::new().expect("Failed to create tokio runtime");
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
    let manager = Manager::new().await?;
    let adapters = manager.adapters().await?;
    let central = adapters
        .into_iter()
        .next()
        .ok_or_else(|| anyhow::anyhow!("No Bluetooth adapters found"))?;

    loop {
        // Start scanning
        let _ = tx.send(BleMessage::StateChanged(ConnectionState::Scanning));
        state.lock().connection_state = ConnectionState::Scanning;

        central.start_scan(ScanFilter::default()).await?;

        // Wait for device discovery
        let device = find_pebble_device(&central).await;

        central.stop_scan().await?;

        let Some(device) = device else {
            tokio::time::sleep(Duration::from_secs(1)).await;
            continue;
        };

        // Connect to device
        let _ = tx.send(BleMessage::StateChanged(ConnectionState::Connecting));
        state.lock().connection_state = ConnectionState::Connecting;

        if device.connect().await.is_err() {
            let _ = tx.send(BleMessage::StateChanged(ConnectionState::Error(
                "Failed to connect".into(),
            )));
            tokio::time::sleep(Duration::from_secs(1)).await;
            continue;
        }

        // Discover services
        if device.discover_services().await.is_err() {
            let _ = device.disconnect().await;
            continue;
        }

        let _ = tx.send(BleMessage::StateChanged(ConnectionState::Connected));
        state.lock().connection_state = ConnectionState::Connected;

        // Find our characteristics
        let characteristics = device.characteristics();
        eprintln!("Found {} characteristics:", characteristics.len());
        for c in &characteristics {
            eprintln!("  - {} (props: {:?})", c.uuid, c.properties);
        }

        // Debug: print expected UUIDs
        eprintln!("Looking for LED0: {}", LED_COLORS_0_UUID);
        eprintln!("Looking for LED1: {}", LED_COLORS_1_UUID);
        eprintln!("Looking for LED2: {}", LED_COLORS_2_UUID);

        let acc_char = characteristics.iter().find(|c| c.uuid == ACC_DATA_UUID);
        let gyro_char = characteristics.iter().find(|c| c.uuid == GYRO_DATA_UUID);
        let mag_char = characteristics.iter().find(|c| c.uuid == MAG_DATA_UUID);
        let led0_char = characteristics.iter().find(|c| c.uuid == LED_COLORS_0_UUID);
        let led1_char = characteristics.iter().find(|c| c.uuid == LED_COLORS_1_UUID);
        let led2_char = characteristics.iter().find(|c| c.uuid == LED_COLORS_2_UUID);

        // Subscribe to notifications
        if let Some(acc_char) = acc_char {
            eprintln!("Subscribing to ACC characteristic...");
            match device.subscribe(acc_char).await {
                Ok(_) => eprintln!("  Subscribed to ACC notifications"),
                Err(e) => eprintln!("  Failed to subscribe to ACC: {:?}", e),
            }
        } else {
            eprintln!("ACC characteristic not found!");
        }

        if let Some(gyro_char) = gyro_char {
            eprintln!("Subscribing to GYRO characteristic...");
            match device.subscribe(gyro_char).await {
                Ok(_) => eprintln!("  Subscribed to GYRO notifications"),
                Err(e) => eprintln!("  Failed to subscribe to GYRO: {:?}", e),
            }
        } else {
            eprintln!("GYRO characteristic not found!");
        }

        if let Some(mag_char) = mag_char {
            eprintln!("Subscribing to MAG characteristic...");
            match device.subscribe(mag_char).await {
                Ok(_) => eprintln!("  Subscribed to MAG notifications"),
                Err(e) => eprintln!("  Failed to subscribe to MAG: {:?}", e),
            }
        } else {
            eprintln!("MAG characteristic not found!");
        }

        if let Some(led0_char) = led0_char {
            eprintln!("Subscribing to LED0 characteristic...");
            match device.subscribe(led0_char).await {
                Ok(_) => eprintln!("  Subscribed to LED0 notifications"),
                Err(e) => eprintln!("  Failed to subscribe to LED0: {:?}", e),
            }
        } else {
            eprintln!("LED0 characteristic not found!");
        }

        if let Some(led1_char) = led1_char {
            eprintln!("Subscribing to LED1 characteristic...");
            match device.subscribe(led1_char).await {
                Ok(_) => eprintln!("  Subscribed to LED1 notifications"),
                Err(e) => eprintln!("  Failed to subscribe to LED1: {:?}", e),
            }
        } else {
            eprintln!("LED1 characteristic not found!");
        }

        if let Some(led2_char) = led2_char {
            eprintln!("Subscribing to LED2 characteristic...");
            match device.subscribe(led2_char).await {
                Ok(_) => eprintln!("  Subscribed to LED2 notifications"),
                Err(e) => eprintln!("  Failed to subscribe to LED2: {:?}", e),
            }
        } else {
            eprintln!("LED2 characteristic not found!");
        }

        // Read LED colors initially (firmware uses .set() not .notify())
        read_led_colors(&device, led0_char, led1_char, led2_char, &tx).await;

        // Spawn a task to periodically poll LED colors
        let device_for_poll = device.clone();
        let tx_for_poll = tx.clone();
        let led0_char_clone = led0_char.cloned();
        let led1_char_clone = led1_char.cloned();
        let led2_char_clone = led2_char.cloned();
        tokio::spawn(async move {
            loop {
                tokio::time::sleep(Duration::from_millis(500)).await;
                read_led_colors(
                    &device_for_poll,
                    led0_char_clone.as_ref(),
                    led1_char_clone.as_ref(),
                    led2_char_clone.as_ref(),
                    &tx_for_poll,
                )
                .await;
            }
        });

        // Listen for notifications
        eprintln!("Waiting for notifications...");
        let mut notification_stream = device.notifications().await?;
        let mut notification_count = 0u64;

        while let Some(notification) = notification_stream.next().await {
            notification_count += 1;
            if notification_count <= 5 || notification_count.is_multiple_of(100) {
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
                let _ = tx.send(BleMessage::AccelData(accel));
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
            } else if notification.uuid == LED_COLORS_0_UUID && notification.value.len() >= 72 {
                if let Some(colors) = parse_led_colors(&notification.value) {
                    let _ = tx.send(BleMessage::LedColorsChunk(0, colors));
                }
            } else if notification.uuid == LED_COLORS_1_UUID && notification.value.len() >= 72 {
                if let Some(colors) = parse_led_colors(&notification.value) {
                    let _ = tx.send(BleMessage::LedColorsChunk(1, colors));
                }
            } else if notification.uuid == LED_COLORS_2_UUID && notification.value.len() >= 72 {
                if let Some(colors) = parse_led_colors(&notification.value) {
                    let _ = tx.send(BleMessage::LedColorsChunk(2, colors));
                }
            }
        }

        eprintln!(
            "Notification stream ended after {} notifications",
            notification_count
        );

        // Disconnected
        let _ = tx.send(BleMessage::StateChanged(ConnectionState::Disconnected));
        state.lock().connection_state = ConnectionState::Disconnected;
        tokio::time::sleep(Duration::from_secs(1)).await;
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

async fn find_pebble_device(central: &Adapter) -> Option<Peripheral> {
    // Wait a bit for scanning
    tokio::time::sleep(Duration::from_secs(3)).await;

    let peripherals = central.peripherals().await.ok()?;

    eprintln!("Found {} BLE peripherals", peripherals.len());

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
