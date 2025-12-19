#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use bt_hci::controller::ExternalController;
use core::sync::atomic::Ordering;
use defmt::{error, info};
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_hal::clock::CpuClock;
use esp_hal::delay::Delay;
use esp_hal::timer::timg::TimerGroup;
use esp_radio::ble::controller::BleConnector;
use panic_rtt_target as _;
use pebble::comms::ble::{LedColorChunk, SensorServer};
use pebble::hal::led::LED_STATE;
use pebble::hal::{gps, imu, led, servo};
use pebble::state::{ACTIVE_CONNECTIONS, GPS_DATA, LED_COMMAND, LedCommand, SENSOR_DATA};
use pebble::tasks;
use static_cell::StaticCell;
use trouble_host::prelude::*;

extern crate alloc;

const CONNECTIONS_MAX: usize = 2;
const L2CAP_CHANNELS_MAX: usize = 4;

/// Advertising data for BLE peripheral.
#[rustfmt::skip]
const ADV_DATA: [u8; 11] = [
    0x02, 0x01, 0x06,                              // Flags: LE General Discoverable + BR/EDR Not Supported
    0x07, 0x09, b'P', b'e', b'b', b'b', b'l', b'e' // Complete Local Name: "Pebble"
];

// This creates a default app-descriptor required by the esp-idf bootloader.
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
    static LED_BUFFER: StaticCell<[esp_hal::rmt::PulseCode; led::LED_BUFFER_SIZE]> =
        StaticCell::new();
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

    // Initialize shared I2C bus for IMU and magnetometer
    let mut delay = Delay::new();
    static SHARED_I2C: StaticCell<
        pebble::hal::peripherals::i2c::SharedI2c<
            esp_hal::i2c::master::I2c<'static, esp_hal::Blocking>,
        >,
    > = StaticCell::new();
    let shared_i2c = SHARED_I2C.init(
        imu::init_shared(peripherals.I2C0, peripherals.GPIO2, peripherals.GPIO3)
            .expect("Failed to initialize I2C"),
    );

    // Initialize IMU on shared bus
    let imu = match imu::SharedBmi270::new(shared_i2c, &mut delay) {
        Ok(imu) => {
            info!("IMU initialized successfully");
            imu
        }
        Err(e) => {
            defmt::panic!("Failed to initialize IMU: {:?}", e);
        }
    };

    // Initialize magnetometer directly on shared I2C bus
    let magnetometer = match imu::SharedBmm350::new(shared_i2c, &mut delay) {
        Ok(magnetometer) => {
            info!("Magnetometer initialized successfully");
            Some(magnetometer)
        }
        Err(error) => {
            info!("Failed to initialize magnetometer: {:?}", error);
            None
        }
    };

    // Initialize GPS on UART1 (GPIO8=RX, GPIO18=TX)
    let mut gps = gps::init(peripherals.UART1, peripherals.GPIO8, peripherals.GPIO18);
    info!("GPS initialized on UART1");

    // Initialize servo on GPIO1
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
            error!("Failed to initialize servo: {:?}", e);
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

    // Main BLE peripheral loop - accepts connections and handles them concurrently
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
                                tasks::ble::handle_led_write(handle, value, server);
                            }
                        }
                        _ => {}
                    }
                }
            };

            // Task to send sensor notifications
            let sensor_notify = async {
                let mut receiver = SENSOR_DATA.receiver().unwrap();
                loop {
                    let data = receiver.changed().await;
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

            // Task to send LED notifications (throttled to 10Hz)
            let led_notify = async {
                let mut receiver = LED_STATE.receiver().unwrap();
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

            // Task to send GPS notifications
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

    // Task to sync LED state to BLE characteristics
    let led_ble_sync_task = async {
        let mut receiver = LED_STATE.receiver().unwrap();
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
        embassy_futures::join::join3(
            tasks::imu::run(&imu, magnetometer.as_ref()),
            tasks::gps::run(&mut gps),
            tasks::servo::run(&mut servo),
        ),
        ble_task,
        tasks::led::run_compass(&mut leds),
        led_ble_sync_task,
    )
    .await;

    #[allow(clippy::empty_loop)]
    loop {}
}
