#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use bt_hci::controller::ExternalController;
use core::sync::atomic::{AtomicU8, Ordering};
use defmt::info;
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::watch::Watch;
use embassy_time::{Duration, Timer};
use esp_hal::clock::CpuClock;
use esp_hal::delay::Delay;

use esp_hal::timer::timg::TimerGroup;
use esp_radio::ble::controller::BleConnector;
use panic_rtt_target as _;
use pebble::ble::{AccBleData, GyroBleData, MagBleData, SensorServer};
use pebble::hal::{imu, led};
use static_cell::StaticCell;
use trouble_host::prelude::*;

extern crate alloc;

const CONNECTIONS_MAX: usize = 2;
const L2CAP_CHANNELS_MAX: usize = 4;

/// Sensor data to be broadcast to all connections
#[derive(Clone, Copy, Default)]
struct SensorData {
    acc: AccBleData,
    gyro: GyroBleData,
    mag: MagBleData,
    valid: bool,
}

/// Watch for broadcasting sensor data to multiple connection handlers
static SENSOR_DATA: Watch<CriticalSectionRawMutex, SensorData, 2> = Watch::new();

/// LED command: [brightness, led_index (0xFF = all), r, g, b]
#[derive(Clone, Copy, Default)]
struct LedCommand {
    brightness: u8,
    led_index: u8,
    r: u8,
    g: u8,
    b: u8,
}

/// Watch for LED commands from BLE
static LED_COMMAND: Watch<CriticalSectionRawMutex, LedCommand, 2> = Watch::new();

/// LED color chunks (24 LEDs * 3 bytes = 72 bytes each)
static LED_COLORS_0: Watch<CriticalSectionRawMutex, [u8; 72], 2> = Watch::new();
static LED_COLORS_1: Watch<CriticalSectionRawMutex, [u8; 72], 2> = Watch::new();
static LED_COLORS_2: Watch<CriticalSectionRawMutex, [u8; 72], 2> = Watch::new();

/// Track number of active connections
static ACTIVE_CONNECTIONS: AtomicU8 = AtomicU8::new(0);

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[esp_rtos::main]
async fn main(_spawner: Spawner) -> ! {
    rtt_target::rtt_init_defmt!();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 65536);
    esp_alloc::heap_allocator!(size: 64 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let sw_interrupt =
        esp_hal::interrupt::software::SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_interrupt.software_interrupt0);

    info!("Embassy initialized!");

    // Initialize LEDs
    static LED_BUFFER: static_cell::StaticCell<[esp_hal::rmt::PulseCode; led::LED_BUFFER_SIZE]> =
        static_cell::StaticCell::new();
    let led_buffer = LED_BUFFER.init(esp_hal_smartled::smart_led_buffer!(led::NUM_LEDS));
    let mut leds = match led::init(peripherals.RMT, peripherals.GPIO15, led_buffer) {
        Ok(leds) => {
            info!("LEDs initialized successfully");
            leds
        }
        Err(e) => {
            defmt::panic!("Failed to initialize LEDs: {:?}", e);
        }
    };

    // Set initial LED state: all green at 50% brightness
    leds.set_brightness(128);
    leds.set_all(led::Color::green());
    if let Err(_e) = leds.show() {
        info!("Failed to update LEDs");
    } else {
        info!("LEDs initialized: all green at 50% brightness");
    }

    // Send initial LED command state
    LED_COMMAND.sender().send(LedCommand {
        brightness: 128,
        led_index: 0xFF,
        r: 0,
        g: 255,
        b: 0,
    });

    // Initialize IMU
    let mut delay = Delay::new();
    let mut imu = match imu::init(
        peripherals.I2C0,
        peripherals.GPIO2,
        peripherals.GPIO3,
        &mut delay,
    ) {
        Ok(mut imu) => {
            info!("IMU initialized successfully");
            if let Err(e) = imu.init_magnetometer(&mut delay) {
                info!("Failed to initialize magnetometer: {:?}", e);
            }
            imu
        }
        Err(e) => {
            defmt::panic!("Failed to initialize IMU: {:?}", e);
        }
    };

    // Initialize BLE
    let radio_init = esp_radio::init().expect("Failed to initialize Wi-Fi/BLE controller");

    let transport = BleConnector::new(&radio_init, peripherals.BT, Default::default()).unwrap();
    let ble_controller = ExternalController::<_, 1>::new(transport);

    static HOST_RESOURCES: StaticCell<
        HostResources<DefaultPacketPool, CONNECTIONS_MAX, L2CAP_CHANNELS_MAX>,
    > = StaticCell::new();
    let host_resources = HOST_RESOURCES.init(HostResources::new());

    let stack = trouble_host::new(ble_controller, host_resources);
    let Host {
        mut peripheral,
        mut runner,
        ..
    } = stack.build();

    info!("BLE stack initialized");

    // Create the GATT server
    static SERVER: StaticCell<SensorServer> = StaticCell::new();
    let server = SERVER.init(
        SensorServer::new_with_config(GapConfig::Peripheral(PeripheralConfig {
            name: "Pebble",
            appearance: &appearance::sensor::GENERIC_SENSOR,
        }))
        .expect("Failed to create GATT server"),
    );

    info!("GATT server created");

    // Task to run the BLE stack
    let runner_task = async {
        let _ = runner.run().await;
    };

    // Task to read IMU and broadcast sensor data
    let imu_task = async {
        loop {
            let imu_result = imu.read();
            let mag_result = imu.read_mag();

            let mut data = SensorData::default();

            if let Ok(imu_data) = &imu_result {
                data.acc = AccBleData {
                    x: imu_data.acc_x,
                    y: imu_data.acc_y,
                    z: imu_data.acc_z,
                };
                data.gyro = GyroBleData {
                    x: imu_data.gyr_x,
                    y: imu_data.gyr_y,
                    z: imu_data.gyr_z,
                };
                data.valid = true;
            }

            if let Ok(mag_data) = &mag_result {
                data.mag = MagBleData {
                    x: mag_data.x,
                    y: mag_data.y,
                    z: mag_data.z,
                };
            }

            // Broadcast to all waiting connections
            SENSOR_DATA.sender().send(data);

            // Log data locally
            if let (Ok(imu_data), Ok(mag_data)) = (&imu_result, &mag_result) {
                info!(
                    "acc=({}, {}, {}) gyr=({}, {}, {}) mag=({}, {}, {})",
                    imu_data.acc_x,
                    imu_data.acc_y,
                    imu_data.acc_z,
                    imu_data.gyr_x,
                    imu_data.gyr_y,
                    imu_data.gyr_z,
                    mag_data.x,
                    mag_data.y,
                    mag_data.z
                );
            }

            Timer::after(Duration::from_millis(100)).await;
        }
    };

    // BLE characteristics are synced automatically via LED_STATE when show() is called

    // Task to run LED fade animation: green -> red -> blue -> green
    let led_task = async {
        let mut receiver = LED_COMMAND.receiver().unwrap();

        // Fade parameters
        const FADE_STEPS: u16 = 128; // Steps per transition
        const FADE_DELAY_MS: u64 = 20; // Delay between steps (total ~2.5s per transition)

        // Color keyframes: green -> red -> blue -> green
        let keyframes: [(u8, u8, u8); 4] = [
            (0, 255, 0), // Green
            (255, 0, 0), // Red
            (0, 0, 255), // Blue
            (0, 255, 0), // Green (back to start)
        ];

        let mut keyframe_idx: usize = 0;
        let mut step: u16 = 0;
        let mut led_pos: usize = 0; // Current LED position for circle animation

        loop {
            // Check for BLE command (non-blocking)
            if let Some(cmd) = receiver.try_changed() {
                // Handle BLE command - BLE sync is automatic via LED_STATE
                match cmd.led_index {
                    0xFE => {
                        leds.set_brightness(cmd.brightness);
                    }
                    0xF0 => {
                        let colors = LED_COLORS_0
                            .receiver()
                            .unwrap()
                            .try_get()
                            .unwrap_or([0u8; 72]);
                        for i in 0..24 {
                            leds.set(
                                i,
                                led::Color::new(
                                    colors[i * 3],
                                    colors[i * 3 + 1],
                                    colors[i * 3 + 2],
                                ),
                            );
                        }
                    }
                    0xF1 => {
                        let colors = LED_COLORS_1
                            .receiver()
                            .unwrap()
                            .try_get()
                            .unwrap_or([0u8; 72]);
                        for i in 0..24 {
                            leds.set(
                                24 + i,
                                led::Color::new(
                                    colors[i * 3],
                                    colors[i * 3 + 1],
                                    colors[i * 3 + 2],
                                ),
                            );
                        }
                    }
                    0xF2 => {
                        let colors = LED_COLORS_2
                            .receiver()
                            .unwrap()
                            .try_get()
                            .unwrap_or([0u8; 72]);
                        for i in 0..24 {
                            leds.set(
                                48 + i,
                                led::Color::new(
                                    colors[i * 3],
                                    colors[i * 3 + 1],
                                    colors[i * 3 + 2],
                                ),
                            );
                        }
                    }
                    0xFF => {
                        if cmd.brightness != 0xFF {
                            leds.set_brightness(cmd.brightness);
                        }
                        leds.set_all(led::Color::new(cmd.r, cmd.g, cmd.b));
                    }
                    idx => {
                        if cmd.brightness != 0xFF {
                            leds.set_brightness(cmd.brightness);
                        }
                        leds.set(idx as usize, led::Color::new(cmd.r, cmd.g, cmd.b));
                    }
                }
                let _ = leds.show(); // This broadcasts to LED_STATE automatically
                continue;
            }

            // Run fade animation with single LED going in a circle
            let from = keyframes[keyframe_idx];
            let to = keyframes[(keyframe_idx + 1) % keyframes.len()];

            // Linear interpolation for color
            let t = step as u32;
            let r = ((from.0 as u32 * (FADE_STEPS as u32 - t) + to.0 as u32 * t)
                / FADE_STEPS as u32) as u8;
            let g = ((from.1 as u32 * (FADE_STEPS as u32 - t) + to.1 as u32 * t)
                / FADE_STEPS as u32) as u8;
            let b = ((from.2 as u32 * (FADE_STEPS as u32 - t) + to.2 as u32 * t)
                / FADE_STEPS as u32) as u8;

            // Clear all LEDs and set only the current position
            leds.clear();
            leds.set(led_pos, led::Color::new(r, g, b));
            let _ = leds.show();

            // Advance position around the circle
            led_pos = (led_pos + 1) % led::NUM_LEDS;

            step += 1;
            if step >= FADE_STEPS {
                step = 0;
                keyframe_idx = (keyframe_idx + 1) % (keyframes.len() - 1);
            }

            Timer::after(Duration::from_millis(FADE_DELAY_MS)).await;
        }
    };

    // Main BLE peripheral loop - accepts connections and handles them concurrently
    let ble_task = async {
        // Advertising data: Flags + Complete Local Name "Pebble"
        #[rustfmt::skip]
        let adv_data: [u8; 11] = [
            0x02, 0x01, 0x06,                              // Flags: LE General Discoverable + BR/EDR Not Supported
            0x07, 0x09, b'P', b'e', b'b', b'b', b'l', b'e' // Complete Local Name: "Pebble" (length 7 = 6 chars + type)
        ];

        loop {
            info!("Starting BLE advertising...");

            let advertiser = match peripheral
                .advertise(
                    &Default::default(),
                    Advertisement::ConnectableScannableUndirected {
                        adv_data: &adv_data,
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

            // Handle this connection - GATT events and notifications
            // Use select to handle both concurrently, and immediately restart advertising
            // when either completes (connection ends)

            // Task to handle GATT events
            let gatt_events = async {
                loop {
                    match conn.next().await {
                        GattConnectionEvent::Disconnected { reason } => {
                            info!("GATT disconnected: {:?}", reason);
                            break;
                        }
                        GattConnectionEvent::Gatt { event } => {
                            if let GattEvent::Write(write_event) = event {
                                let handle = write_event.handle();
                                let value = write_event.data();

                                // LED control: [brightness, led_index, r, g, b]
                                if handle == server.sensor_service.led_control.handle {
                                    if value.len() >= 5 {
                                        let cmd = LedCommand {
                                            brightness: value[0],
                                            led_index: value[1],
                                            r: value[2],
                                            g: value[3],
                                            b: value[4],
                                        };
                                        LED_COMMAND.sender().send(cmd);
                                        info!("Received LED control command via BLE");
                                    }
                                }
                                // LED brightness: single byte
                                else if handle == server.sensor_service.led_brightness.handle {
                                    if !value.is_empty() {
                                        let cmd = LedCommand {
                                            brightness: value[0],
                                            led_index: 0xFE, // Special: brightness only
                                            r: 0,
                                            g: 0,
                                            b: 0,
                                        };
                                        LED_COMMAND.sender().send(cmd);
                                        info!("Received LED brightness command via BLE");
                                    }
                                }
                                // LED colors chunk 0 (LEDs 0-23)
                                else if handle == server.sensor_service.led_colors_0.handle {
                                    if value.len() >= 72 {
                                        let mut colors = [0u8; 72];
                                        colors.copy_from_slice(&value[..72]);
                                        LED_COLORS_0.sender().send(colors);
                                        LED_COMMAND.sender().send(LedCommand {
                                            brightness: 0xFF,
                                            led_index: 0xF0,
                                            r: 0,
                                            g: 0,
                                            b: 0,
                                        });
                                        info!("Received LED colors chunk 0 via BLE");
                                    }
                                }
                                // LED colors chunk 1 (LEDs 24-47)
                                else if handle == server.sensor_service.led_colors_1.handle {
                                    if value.len() >= 72 {
                                        let mut colors = [0u8; 72];
                                        colors.copy_from_slice(&value[..72]);
                                        LED_COLORS_1.sender().send(colors);
                                        LED_COMMAND.sender().send(LedCommand {
                                            brightness: 0xFF,
                                            led_index: 0xF1,
                                            r: 0,
                                            g: 0,
                                            b: 0,
                                        });
                                        info!("Received LED colors chunk 1 via BLE");
                                    }
                                }
                                // LED colors chunk 2 (LEDs 48-71)
                                else if handle == server.sensor_service.led_colors_2.handle {
                                    if value.len() >= 72 {
                                        let mut colors = [0u8; 72];
                                        colors.copy_from_slice(&value[..72]);
                                        LED_COLORS_2.sender().send(colors);
                                        LED_COMMAND.sender().send(LedCommand {
                                            brightness: 0xFF,
                                            led_index: 0xF2,
                                            r: 0,
                                            g: 0,
                                            b: 0,
                                        });
                                        info!("Received LED colors chunk 2 via BLE");
                                    }
                                }
                            }
                        }
                        _ => {}
                    }
                }
            };

            // Task to send sensor notifications to this connection
            let sensor_notify = async {
                let mut receiver = SENSOR_DATA.receiver().unwrap();
                loop {
                    let data = receiver.changed().await;

                    if !data.valid {
                        continue;
                    }

                    // Send accelerometer
                    if server
                        .sensor_service
                        .acc_data
                        .notify(&conn, &data.acc.to_bytes())
                        .await
                        .is_err()
                    {
                        break;
                    }

                    // Send gyroscope
                    if server
                        .sensor_service
                        .gyro_data
                        .notify(&conn, &data.gyro.to_bytes())
                        .await
                        .is_err()
                    {
                        break;
                    }

                    // Send magnetometer
                    if server
                        .sensor_service
                        .mag_data
                        .notify(&conn, &data.mag.to_bytes())
                        .await
                        .is_err()
                    {
                        break;
                    }
                }
            };

            // Task to send LED notifications to this connection (throttled to 10Hz)
            let led_notify = async {
                let mut receiver = led::LED_STATE.receiver().unwrap();
                loop {
                    let state = receiver.changed().await;

                    // Throttle notifications to avoid overwhelming BLE
                    Timer::after(Duration::from_millis(100)).await;

                    // Get latest state (skip intermediate updates)
                    let state = receiver.try_get().unwrap_or(state);

                    // Notify LED brightness
                    if server
                        .sensor_service
                        .led_brightness
                        .notify(&conn, &state.brightness)
                        .await
                        .is_err()
                    {
                        break;
                    }

                    // Notify LED color chunks
                    if server
                        .sensor_service
                        .led_colors_0
                        .notify(&conn, &pebble::ble::LedColorChunk(state.chunk0))
                        .await
                        .is_err()
                    {
                        break;
                    }

                    if server
                        .sensor_service
                        .led_colors_1
                        .notify(&conn, &pebble::ble::LedColorChunk(state.chunk1))
                        .await
                        .is_err()
                    {
                        break;
                    }

                    if server
                        .sensor_service
                        .led_colors_2
                        .notify(&conn, &pebble::ble::LedColorChunk(state.chunk2))
                        .await
                        .is_err()
                    {
                        break;
                    }

                    // Notify led_control with current color
                    let ctrl = [
                        state.brightness,
                        0xFF,
                        state.chunk0[0],
                        state.chunk0[1],
                        state.chunk0[2],
                    ];
                    if server
                        .sensor_service
                        .led_control
                        .notify(&conn, &ctrl)
                        .await
                        .is_err()
                    {
                        break;
                    }
                }
            };

            embassy_futures::select::select3(gatt_events, sensor_notify, led_notify).await;

            let remaining = ACTIVE_CONNECTIONS.fetch_sub(1, Ordering::Relaxed) - 1;
            info!("Client disconnected ({} remaining)", remaining);
        }
    };

    // Task to sync LED state to BLE characteristics
    let led_ble_sync_task = async {
        let mut receiver = led::LED_STATE.receiver().unwrap();
        loop {
            let state = receiver.changed().await;

            // Update BLE characteristics
            let _ = server
                .sensor_service
                .led_brightness
                .set(server, &state.brightness);
            let _ = server
                .sensor_service
                .led_colors_0
                .set(server, &pebble::ble::LedColorChunk(state.chunk0));
            let _ = server
                .sensor_service
                .led_colors_1
                .set(server, &pebble::ble::LedColorChunk(state.chunk1));
            let _ = server
                .sensor_service
                .led_colors_2
                .set(server, &pebble::ble::LedColorChunk(state.chunk2));

            // Update led_control with current color (use first LED's color)
            let ctrl = [
                state.brightness,
                0xFF,
                state.chunk0[0],
                state.chunk0[1],
                state.chunk0[2],
            ];
            let _ = server.sensor_service.led_control.set(server, &ctrl);
        }
    };

    // Run all tasks concurrently
    embassy_futures::join::join5(runner_task, imu_task, ble_task, led_task, led_ble_sync_task)
        .await;

    loop {
        Timer::after(Duration::from_secs(1)).await;
    }
}
