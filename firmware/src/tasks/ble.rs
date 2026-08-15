//! BLE peripheral task and LED command processing.

use core::sync::atomic::Ordering;

use defmt::{error, info, warn};
use embassy_time::{Duration, Timer};
use trouble_host::prelude::*;

use crate::comms::ble::{LedColorChunk, SensorServer};
use crate::hal::led::{LED_STATE, LedState, NUM_LEDS};
use crate::state::{
    ACTIVE_CONNECTIONS, BATTERY_VOLTAGE, DEVICE_STATUS, GPS_DATA, LED_COLORS_0, LED_COLORS_1,
    LED_COLORS_2, LED_COMMAND, LedCommand, SATELLITES_0, SATELLITES_1, SENSOR_DATA,
};

/// Advertising data for the BLE peripheral: flags + complete local name "Pebble".
#[rustfmt::skip]
const ADV_DATA: [u8; 11] = [
    0x02, 0x01, 0x06,
    0x07, 0x09, b'P', b'e', b'b', b'b', b'l', b'e',
];

/// Pack the `led_control` notification/characteristic payload:
/// `[brightness, 0xFF marker, first LED r, g, b]`.
fn led_control_bytes(state: &LedState) -> [u8; 5] {
    [
        state.brightness,
        0xFF,
        state.chunk0[0],
        state.chunk0[1],
        state.chunk0[2],
    ]
}

/// Run the BLE peripheral: drives the host stack, advertises, and for each
/// connection fans out sensor/LED/GPS/status notifications until it drops.
/// Also mirrors LED state into the GATT characteristics so fresh reads are current.
pub async fn run<'srv, 'stack, C: Controller>(
    server: &'srv SensorServer<'srv>,
    mut peripheral: Peripheral<'stack, C, DefaultPacketPool>,
    mut runner: Runner<'stack, C, DefaultPacketPool>,
) {
    let runner_task = async {
        let _ = runner.run().await;
    };

    let ble_task = async {
        loop {
            info!("Starting BLE advertising...");

            let advertiser = match peripheral
                .advertise(
                    &Default::default(),
                    Advertisement::ConnectableScannableUndirected {
                        adv_data: &ADV_DATA,
                        scan_data: &[],
                    },
                )
                .await
            {
                Ok(advertiser) => advertiser,
                Err(_e) => {
                    info!("Advertising error");
                    Timer::after(Duration::from_secs(1)).await;
                    continue;
                }
            };

            info!("Waiting for connection...");

            let conn = match advertiser.accept().await {
                Ok(conn) => match conn.with_attribute_server(server) {
                    Ok(gatt_conn) => gatt_conn,
                    Err(_e) => {
                        info!("Failed to create GATT connection");
                        continue;
                    }
                },
                Err(_e) => {
                    info!("Connection accept error");
                    continue;
                }
            };

            let conn_num = ACTIVE_CONNECTIONS.fetch_add(1, Ordering::Relaxed) + 1;
            info!("Client connected! ({} active)", conn_num);

            // Handle inbound GATT writes (LED control).
            let gatt_events = async {
                loop {
                    match conn.next().await {
                        GattConnectionEvent::Disconnected { reason } => {
                            info!("GATT disconnected: {:?}", reason);
                            break;
                        }
                        GattConnectionEvent::Gatt {
                            event: GattEvent::Write(write_event),
                        } => {
                            handle_led_write(write_event.handle(), write_event.data(), server);
                        }
                        _ => {}
                    }
                }
            };

            // Sensor notifications.
            let sensor_notify = async {
                let Some(mut receiver) = SENSOR_DATA.receiver() else {
                    error!("No sensor data receiver slot available");
                    return;
                };
                loop {
                    let data = receiver.changed().await;
                    // Throttle + coalesce to ~50Hz. The IMU publishes at 100Hz;
                    // notifying all 4 characteristics per sample floods the BLE
                    // link (~400 pkt/s), starving CCCD-write responses so new
                    // subscriptions get dropped mid-connect.
                    Timer::after(Duration::from_millis(20)).await;
                    let data = receiver.try_get().unwrap_or(data);
                    if !data.valid {
                        continue;
                    }

                    if server
                        .sensor_service
                        .acc_data
                        .notify(&conn, &data.acc.to_bytes())
                        .await
                        .is_err()
                    {
                        break;
                    }
                    if server
                        .sensor_service
                        .gyro_data
                        .notify(&conn, &data.gyro.to_bytes())
                        .await
                        .is_err()
                    {
                        break;
                    }
                    if server
                        .sensor_service
                        .mag_data
                        .notify(&conn, &data.mag.to_bytes())
                        .await
                        .is_err()
                    {
                        break;
                    }
                    if server
                        .sensor_service
                        .orientation
                        .notify(&conn, &data.orientation.to_bytes())
                        .await
                        .is_err()
                    {
                        break;
                    }
                }
            };

            // LED notifications (throttled to ~10Hz).
            let led_notify = async {
                let Some(mut receiver) = LED_STATE.receiver() else {
                    error!("No LED state receiver slot available");
                    return;
                };
                loop {
                    let state = receiver.changed().await;
                    Timer::after(Duration::from_millis(100)).await;
                    let state = receiver.try_get().unwrap_or(state);

                    if server
                        .sensor_service
                        .led_brightness
                        .notify(&conn, &state.brightness)
                        .await
                        .is_err()
                    {
                        break;
                    }
                    if server
                        .sensor_service
                        .led_colors_0
                        .notify(&conn, &LedColorChunk(state.chunk0))
                        .await
                        .is_err()
                    {
                        break;
                    }
                    if server
                        .sensor_service
                        .led_colors_1
                        .notify(&conn, &LedColorChunk(state.chunk1))
                        .await
                        .is_err()
                    {
                        break;
                    }
                    if server
                        .sensor_service
                        .led_colors_2
                        .notify(&conn, &LedColorChunk(state.chunk2))
                        .await
                        .is_err()
                    {
                        break;
                    }
                    if server
                        .sensor_service
                        .led_control
                        .notify(&conn, &led_control_bytes(&state))
                        .await
                        .is_err()
                    {
                        break;
                    }
                }
            };

            // GPS notifications.
            let gps_notify = async {
                let Some(mut receiver) = GPS_DATA.receiver() else {
                    error!("No GPS data receiver slot available");
                    return;
                };
                loop {
                    let data = receiver.changed().await;
                    if server
                        .sensor_service
                        .gps_data
                        .notify(&conn, &data.to_bytes())
                        .await
                        .is_err()
                    {
                        break;
                    }
                }
            };

            // Satellite info notifications (two chunks).
            let satellites_0_notify = async {
                let Some(mut receiver) = SATELLITES_0.receiver() else {
                    error!("No satellites_0 receiver slot available");
                    return;
                };
                loop {
                    let data = receiver.changed().await;
                    if server
                        .sensor_service
                        .satellites_0
                        .notify(&conn, &data)
                        .await
                        .is_err()
                    {
                        break;
                    }
                }
            };
            let satellites_1_notify = async {
                let Some(mut receiver) = SATELLITES_1.receiver() else {
                    error!("No satellites_1 receiver slot available");
                    return;
                };
                loop {
                    let data = receiver.changed().await;
                    if server
                        .sensor_service
                        .satellites_1
                        .notify(&conn, &data)
                        .await
                        .is_err()
                    {
                        break;
                    }
                }
            };

            // Device status: send once on connect, then on change.
            let status_notify = async {
                let Some(mut receiver) = DEVICE_STATUS.receiver() else {
                    error!("No device status receiver slot available");
                    return;
                };
                let status = receiver.get().await;
                let _ = server
                    .sensor_service
                    .device_status
                    .notify(&conn, &status.to_bytes())
                    .await;
                loop {
                    let status = receiver.changed().await;
                    if server
                        .sensor_service
                        .device_status
                        .notify(&conn, &status.to_bytes())
                        .await
                        .is_err()
                    {
                        break;
                    }
                }
            };

            // Battery voltage notifications.
            let battery_notify = async {
                let Some(mut receiver) = BATTERY_VOLTAGE.receiver() else {
                    error!("No battery voltage receiver slot available");
                    return;
                };
                loop {
                    let mv = receiver.changed().await;
                    if server
                        .sensor_service
                        .battery_voltage
                        .notify(&conn, &mv.to_le_bytes())
                        .await
                        .is_err()
                    {
                        break;
                    }
                }
            };

            // Any future completing/failing ends the connection.
            // Nested selects because `select` supports at most 5 futures.
            embassy_futures::select::select(
                embassy_futures::select::select5(
                    gatt_events,
                    sensor_notify,
                    led_notify,
                    gps_notify,
                    status_notify,
                ),
                embassy_futures::select::select3(
                    satellites_0_notify,
                    satellites_1_notify,
                    battery_notify,
                ),
            )
            .await;

            let remaining = ACTIVE_CONNECTIONS.fetch_sub(1, Ordering::Relaxed) - 1;
            info!("Client disconnected ({} remaining)", remaining);
        }
    };

    // Mirror LED state into the GATT characteristics so reads reflect current state.
    let led_ble_sync_task = async {
        let Some(mut receiver) = LED_STATE.receiver() else {
            error!("No LED state receiver slot available for BLE sync");
            return;
        };
        loop {
            let state = receiver.changed().await;

            let _ = server
                .sensor_service
                .led_brightness
                .set(server, &state.brightness);
            let _ = server
                .sensor_service
                .led_colors_0
                .set(server, &LedColorChunk(state.chunk0));
            let _ = server
                .sensor_service
                .led_colors_1
                .set(server, &LedColorChunk(state.chunk1));
            let _ = server
                .sensor_service
                .led_colors_2
                .set(server, &LedColorChunk(state.chunk2));
            let _ = server
                .sensor_service
                .led_control
                .set(server, &led_control_bytes(&state));
        }
    };

    embassy_futures::join::join3(runner_task, ble_task, led_ble_sync_task).await;
}

/// Special LED index values:
/// - 0x00-0x47 (0-71): Individual LED index
/// - 0xFE: Brightness-only command (no LED color change)
/// - 0xFF: Set all LEDs to the same color
/// - 0xF0-0xF2: Color chunk update trigger (internal use)
const LED_INDEX_BRIGHTNESS_ONLY: u8 = 0xFE;
const LED_INDEX_ALL_LEDS: u8 = 0xFF;
const LED_INDEX_CHUNK_0: u8 = 0xF0;
const LED_INDEX_CHUNK_1: u8 = 0xF1;
const LED_INDEX_CHUNK_2: u8 = 0xF2;

/// Check if an LED index is valid (individual LED, all LEDs, or special command).
fn is_valid_led_index(index: u8) -> bool {
    (index as usize) < NUM_LEDS
        || index == LED_INDEX_BRIGHTNESS_ONLY
        || index == LED_INDEX_ALL_LEDS
        || index == LED_INDEX_CHUNK_0
        || index == LED_INDEX_CHUNK_1
        || index == LED_INDEX_CHUNK_2
}

/// Process a GATT write to LED-related characteristics.
///
/// Returns true if the write was handled.
pub fn handle_led_write(handle: u16, value: &[u8], server: &SensorServer) -> bool {
    // LED control: [brightness, led_index, r, g, b]
    if handle == server.sensor_service.led_control.handle {
        if value.len() < 5 {
            warn!(
                "LED control command too short: {} bytes (expected 5)",
                value.len()
            );
            return true;
        }

        let led_index = value[1];
        if !is_valid_led_index(led_index) {
            warn!(
                "Invalid LED index: {} (valid: 0-{} or special values)",
                led_index,
                NUM_LEDS - 1
            );
            return true;
        }

        let cmd = LedCommand {
            brightness: value[0],
            led_index,
            r: value[2],
            g: value[3],
            b: value[4],
        };
        LED_COMMAND.sender().send(cmd);
        info!("Received LED control command via BLE");
        return true;
    }

    // LED brightness: single byte
    if handle == server.sensor_service.led_brightness.handle {
        if value.is_empty() {
            warn!("LED brightness command empty");
            return true;
        }

        let cmd = LedCommand {
            brightness: value[0],
            led_index: LED_INDEX_BRIGHTNESS_ONLY,
            r: 0,
            g: 0,
            b: 0,
        };
        LED_COMMAND.sender().send(cmd);
        info!("Received LED brightness command via BLE");
        return true;
    }

    // LED colors chunk 0 (LEDs 0-23)
    if handle == server.sensor_service.led_colors_0.handle {
        if value.len() < 72 {
            warn!(
                "LED colors chunk 0 too short: {} bytes (expected 72)",
                value.len()
            );
            return true;
        }

        let mut colors = [0u8; 72];
        colors.copy_from_slice(&value[..72]);
        LED_COLORS_0.sender().send(colors);
        LED_COMMAND.sender().send(LedCommand {
            brightness: 0xFF,
            led_index: LED_INDEX_CHUNK_0,
            r: 0,
            g: 0,
            b: 0,
        });
        info!("Received LED colors chunk 0 via BLE");
        return true;
    }

    // LED colors chunk 1 (LEDs 24-47)
    if handle == server.sensor_service.led_colors_1.handle {
        if value.len() < 72 {
            warn!(
                "LED colors chunk 1 too short: {} bytes (expected 72)",
                value.len()
            );
            return true;
        }

        let mut colors = [0u8; 72];
        colors.copy_from_slice(&value[..72]);
        LED_COLORS_1.sender().send(colors);
        LED_COMMAND.sender().send(LedCommand {
            brightness: 0xFF,
            led_index: LED_INDEX_CHUNK_1,
            r: 0,
            g: 0,
            b: 0,
        });
        info!("Received LED colors chunk 1 via BLE");
        return true;
    }

    // LED colors chunk 2 (LEDs 48-71)
    if handle == server.sensor_service.led_colors_2.handle {
        if value.len() < 72 {
            warn!(
                "LED colors chunk 2 too short: {} bytes (expected 72)",
                value.len()
            );
            return true;
        }

        let mut colors = [0u8; 72];
        colors.copy_from_slice(&value[..72]);
        LED_COLORS_2.sender().send(colors);
        LED_COMMAND.sender().send(LedCommand {
            brightness: 0xFF,
            led_index: LED_INDEX_CHUNK_2,
            r: 0,
            g: 0,
            b: 0,
        });
        info!("Received LED colors chunk 2 via BLE");
        return true;
    }

    false
}
