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
use pebble::ble::{AccBleData, AhrsBleData, GpsBleData, GyroBleData, MagBleData, SensorServer};
use pebble::hal::{ahrs::AhrsFilter, gps, imu, led, servo};
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
    orientation: AhrsBleData,
    valid: bool,
}

/// Watch for broadcasting sensor data to multiple connection handlers
static SENSOR_DATA: Watch<CriticalSectionRawMutex, SensorData, 2> = Watch::new();

/// Watch for broadcasting GPS data to multiple connection handlers
static GPS_DATA: Watch<CriticalSectionRawMutex, GpsBleData, 2> = Watch::new();

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

/// Compass heading in degrees (0-359), broadcast from IMU task to LED task
static COMPASS_HEADING: Watch<CriticalSectionRawMutex, u16, 2> = Watch::new();

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

    // Initialize shared I2C bus for IMU and magnetometer (bodged to same bus)
    let mut delay = Delay::new();
    static SHARED_I2C: static_cell::StaticCell<
        imu::SharedI2c<esp_hal::i2c::master::I2c<'static, esp_hal::Blocking>>,
    > = static_cell::StaticCell::new();
    let shared_i2c = SHARED_I2C.init(
        imu::init_shared(peripherals.I2C0, peripherals.GPIO2, peripherals.GPIO3)
            .expect("Failed to initialize I2C"),
    );

    // Initialize IMU on shared bus
    let imu = match imu::SharedImu::new(shared_i2c, &mut delay) {
        Ok(imu) => {
            info!("IMU initialized successfully");
            imu
        }
        Err(e) => {
            defmt::panic!("Failed to initialize IMU: {:?}", e);
        }
    };

    // Initialize magnetometer directly on shared I2C bus
    let mag = match imu::SharedBmm350::new(shared_i2c, &mut delay) {
        Ok(mag) => {
            info!("Magnetometer initialized successfully");
            Some(mag)
        }
        Err(e) => {
            info!("Failed to initialize magnetometer: {:?}", e);
            None
        }
    };

    // Initialize GPS on UART1 (GPIO8=RX, GPIO18=TX)
    let mut gps = gps::init(peripherals.UART1, peripherals.GPIO8, peripherals.GPIO18);
    info!("GPS initialized on UART1");

    // Initialize servo on GPIO1
    // Timer and Ledc must be stored in static cells to satisfy lifetime requirements
    static LEDC: StaticCell<esp_hal::ledc::Ledc<'static>> = StaticCell::new();
    static SERVO_TIMER: StaticCell<esp_hal::ledc::timer::Timer<'static, esp_hal::ledc::LowSpeed>> =
        StaticCell::new();

    let mut ledc = esp_hal::ledc::Ledc::new(peripherals.LEDC);
    ledc.set_global_slow_clock(esp_hal::ledc::LSGlobalClkSource::APBClk);
    let ledc = LEDC.init(ledc);
    let servo_timer = SERVO_TIMER.init(servo::init_timer(ledc));
    let mut servo = match servo::init_channel(ledc, peripherals.GPIO1, servo_timer) {
        Ok(s) => {
            info!("Servo initialized successfully");
            Some(s)
        }
        Err(e) => {
            info!("Failed to initialize servo: {:?}", e);
            None
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

    // Task to read IMU data, apply AHRS filter, and broadcast sensor data
    let imu_task = async {
        // Create AHRS filter (Madgwick algorithm)
        // Sample period = 10ms (100Hz), beta = 0.1 (moderate responsiveness)
        let mut ahrs = AhrsFilter::new();
        let mut sample_count: u32 = 0;

        // Hard iron calibration - track min/max to find center
        let mut mag_x_min: i32 = i32::MAX;
        let mut mag_x_max: i32 = i32::MIN;
        let mut mag_y_min: i32 = i32::MAX;
        let mut mag_y_max: i32 = i32::MIN;

        loop {
            // Read IMU data
            let imu_result = imu.read();
            let mag_result = mag.as_ref().map(|m| m.read());

            let mut data = SensorData::default();

            if let Ok(imu_data) = &imu_result {
                // Convert to physical units
                let accel_raw = imu_data.accel_g();
                let gyro = imu_data.gyro_rads();

                // Correct accelerometer for sensor offset from center of rotation
                // The IMU is 45mm (0.045m) to the right (+X) of the PCB center
                // Formula: a_corrected = a_measured - ω × (ω × r)
                // For r = (rx, 0, 0), the centripetal acceleration is:
                //   a_cent_x = -rx * (ωy² + ωz²)
                //   a_cent_y = rx * ωx * ωy
                //   a_cent_z = rx * ωx * ωz
                const IMU_OFFSET_X: f32 = 0.045; // 45mm in meters
                const G: f32 = 9.81; // m/s² per g

                let (wx, wy, wz) = gyro; // rad/s
                let a_cent_x = -IMU_OFFSET_X * (wy * wy + wz * wz) / G; // Convert to g
                let a_cent_y = IMU_OFFSET_X * wx * wy / G;
                let a_cent_z = IMU_OFFSET_X * wx * wz / G;

                let accel = (
                    accel_raw.0 - a_cent_x,
                    accel_raw.1 - a_cent_y,
                    accel_raw.2 - a_cent_z,
                );

                // Update AHRS filter with corrected IMU data
                let orientation = ahrs.update_imu(accel, gyro);

                // Log orientation periodically (every 50 samples = 5 seconds)
                sample_count += 1;
                if sample_count % 50 == 0 {
                    info!(
                        "AHRS: roll={} pitch={} yaw={}",
                        orientation.roll as i32, orientation.pitch as i32, orientation.yaw as i32
                    );
                }

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
                data.orientation = AhrsBleData {
                    roll: orientation.roll,
                    pitch: orientation.pitch,
                    yaw: orientation.yaw,
                };
                data.valid = true;
            }

            if let Some(Ok(mag_data)) = &mag_result {
                data.mag = MagBleData {
                    x: mag_data.x,
                    y: mag_data.y,
                    z: mag_data.z,
                };

                // Update calibration min/max
                if mag_data.x < mag_x_min {
                    mag_x_min = mag_data.x;
                }
                if mag_data.x > mag_x_max {
                    mag_x_max = mag_data.x;
                }
                if mag_data.y < mag_y_min {
                    mag_y_min = mag_data.y;
                }
                if mag_data.y > mag_y_max {
                    mag_y_max = mag_data.y;
                }

                // Calculate hard iron offset (center of min/max)
                let x_offset = (mag_x_min + mag_x_max) / 2;
                let y_offset = (mag_y_min + mag_y_max) / 2;

                // Apply hard iron correction
                let x = (mag_data.x - x_offset) as f32;
                let y = (mag_data.y - y_offset) as f32;

                // Calculate compass heading from magnetometer X and Y
                let heading_rad = libm::atan2f(y, x);
                let mut heading_deg = heading_rad * 180.0 / core::f32::consts::PI;
                if heading_deg < 0.0 {
                    heading_deg += 360.0;
                }
                info!(
                    "MAG: x={} y={} heading={} (cal: x_off={} y_off={})",
                    mag_data.x, mag_data.y, heading_deg as i32, x_offset, y_offset
                );
                COMPASS_HEADING.sender().send(heading_deg as u16);
            }

            // Broadcast to all waiting connections
            SENSOR_DATA.sender().send(data);

            Timer::after(Duration::from_millis(100)).await;
        }
    };

    // Task to read GPS data by polling FIFO
    let gps_task = async {
        info!("GPS task started");

        loop {
            if let Some(gps_data) = gps.poll() {
                if gps_data.position.has_fix() {
                    info!(
                        "GPS: lat={} lon={} alt={}m sats={} fix={:?}",
                        gps_data.position.latitude,
                        gps_data.position.longitude,
                        gps_data.position.altitude,
                        gps_data.position.satellites,
                        gps_data.position.fix_quality
                    );
                } else {
                    info!(
                        "GPS: waiting for fix (sats={})",
                        gps_data.position.satellites
                    );
                }

                // Broadcast GPS data for BLE
                GPS_DATA.sender().send(GpsBleData {
                    latitude: gps_data.position.latitude,
                    longitude: gps_data.position.longitude,
                    altitude: gps_data.position.altitude,
                    satellites: gps_data.position.satellites,
                    fix_quality: gps_data.position.fix_quality as u8,
                    has_fix: gps_data.position.has_fix(),
                });
            }
            // Poll every 100ms - FIFO is 128 bytes, at 9600 baud we get ~960 bytes/sec
            Timer::after(Duration::from_millis(100)).await;
        }
    };

    // BLE characteristics are synced automatically via LED_STATE when show() is called

    // Task to display compass heading - one LED pointing north
    let led_task = async {
        let mut heading_receiver = COMPASS_HEADING.receiver().unwrap();

        // Set brightness
        leds.set_brightness(64);

        loop {
            // Wait for new heading
            let heading = heading_receiver.changed().await;

            // Convert heading (0-359 degrees) to LED index (0-71)
            // LED 0 is at some physical position, heading 0 = magnetic north
            // 72 LEDs in a circle = 5 degrees per LED
            // Invert direction so LED points north (not rotates with heading)
            // Subtract 90 degree offset to correct for physical LED position
            let inverted_heading = (360 - heading + 270) % 360;
            let led_index = ((inverted_heading as u32 * led::NUM_LEDS as u32) / 360) as usize;

            // Clear all LEDs and light only the north-pointing one
            leds.clear();
            leds.set(led_index, led::Color::red());
            let _ = leds.show();
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

                    // Send orientation (AHRS)
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

            // Task to send GPS notifications to this connection
            let gps_notify = async {
                let mut receiver = GPS_DATA.receiver().unwrap();
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

            embassy_futures::select::select4(gatt_events, sensor_notify, led_notify, gps_notify)
                .await;

            let remaining = ACTIVE_CONNECTIONS.fetch_sub(1, Ordering::Relaxed) - 1;
            info!("Client disconnected ({} remaining)", remaining);
        }
    };

    // Task to move servo in 90-degree steps: 0 -> 90 -> 180 -> 90 -> 0 -> ...
    let servo_task = async {
        if let Some(ref mut servo) = servo {
            let positions: [u8; 4] = [0, 90, 180, 90];
            let mut idx = 0;

            loop {
                let angle = positions[idx];
                if let Err(e) = servo.set_angle(angle) {
                    info!("Servo error: {:?}", e);
                }
                info!("Servo moved to {} degrees", angle);

                idx = (idx + 1) % positions.len();
                Timer::after(Duration::from_secs(2)).await;
            }
        } else {
            // No servo, just wait forever
            loop {
                Timer::after(Duration::from_secs(60)).await;
            }
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
    embassy_futures::join::join5(
        runner_task,
        embassy_futures::join::join3(imu_task, gps_task, servo_task),
        ble_task,
        led_task,
        led_ble_sync_task,
    )
    .await;

    loop {
        Timer::after(Duration::from_secs(1)).await;
    }
}
