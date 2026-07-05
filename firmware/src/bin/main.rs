#![no_std]
#![no_main]
#![deny(clippy::all)]

use bt_hci::controller::ExternalController;
use core::sync::atomic::Ordering;
use defmt::{error, info};
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use embedded_hal::delay::DelayNs;
use esp_hal::clock::CpuClock;
use esp_hal::delay::Delay;
use esp_hal::timer::timg::TimerGroup;
use esp_radio::ble::controller::BleConnector;
use panic_rtt_target as _;
use pebble::comms::ble::{LedColorChunk, SensorServer};
use pebble::hal::led::LED_STATE;
use pebble::hal::{gps, imu, led, servo};
use pebble::state::{
    ACTIVE_CONNECTIONS, DEVICE_STATUS, DeviceStatus, GPS_DATA, LED_COMMAND, LedCommand,
    PeripheralError, PeripheralStatus, SATELLITES_0, SATELLITES_1, SENSOR_DATA,
};
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

    // Track device initialization status
    let mut device_status = DeviceStatus::default();

    // Initialize LEDs
    static LED_BUFFER: StaticCell<[esp_hal::rmt::PulseCode; led::LED_BUFFER_SIZE]> =
        StaticCell::new();
    let led_buffer = LED_BUFFER.init(esp_hal_smartled::smart_led_buffer!(led::NUM_LEDS));
    let mut leds = match led::init(peripherals.RMT, peripherals.GPIO15, led_buffer) {
        Ok(leds) => {
            info!("LEDs initialized successfully");
            device_status.leds = PeripheralStatus::Ok;
            Some(leds)
        }
        Err(e) => {
            error!(
                "Failed to initialize LEDs: {:?} - continuing without LED support",
                e
            );
            device_status.leds = PeripheralStatus::Error(PeripheralError::InitFailed);
            None
        }
    };

    // Set initial LED state: all green at 2% brightness
    if let Some(ref mut leds) = leds {
        leds.set_brightness(5);
        leds.set_all(led::Color::green());
        if let Err(_e) = leds.show() {
            info!("Failed to update LEDs");
        } else {
            info!("LEDs initialized: all green at 2% brightness");
        }

        // Send initial LED command state
        LED_COMMAND.sender().send(LedCommand {
            brightness: 5,
            led_index: 0xFF,
            r: 0,
            g: 255,
            b: 0,
        });
    }

    // Initialize shared I2C bus for IMU and magnetometer
    let mut delay = Delay::new();
    static SHARED_I2C: StaticCell<
        pebble::hal::peripherals::i2c::SharedI2c<
            esp_hal::i2c::master::I2c<'static, esp_hal::Blocking>,
        >,
    > = StaticCell::new();
    let shared_i2c = match imu::init_shared(peripherals.I2C0, peripherals.GPIO2, peripherals.GPIO3)
    {
        Ok(i2c) => SHARED_I2C.init(i2c),
        Err(e) => {
            defmt::panic!("Failed to initialize I2C: {:?}", e);
        }
    };

    // Initialize IMU on shared bus
    let imu = match imu::bmi270::SharedBmi270::new(shared_i2c, &mut delay) {
        Ok(imu) => {
            info!("IMU initialized successfully");
            device_status.imu = PeripheralStatus::Ok;
            imu
        }
        Err(e) => {
            defmt::panic!("Failed to initialize IMU: {:?}", e);
        }
    };

    // Initialize magnetometer directly on shared I2C bus (with retries)
    let magnetometer = {
        let mut result = None;
        for attempt in 1..=5 {
            match imu::bmm350::SharedBmm350::new(shared_i2c, &mut delay) {
                Ok(magnetometer) => {
                    info!("Magnetometer initialized successfully");
                    device_status.magnetometer = PeripheralStatus::Ok;
                    result = Some(magnetometer);
                    break;
                }
                Err(error) => {
                    info!(
                        "Failed to initialize magnetometer (attempt {}): {:?}",
                        attempt, error
                    );
                    // Wait before retry
                    delay.delay_ms(100);
                }
            }
        }
        if result.is_none() {
            device_status.magnetometer = PeripheralStatus::Error(PeripheralError::I2cError);
        }
        result
    };

    // Initialize GPS on UART1 (GPIO8=RX, GPIO18=TX)
    let mut gps = match gps::init(peripherals.UART1, peripherals.GPIO8, peripherals.GPIO18) {
        Ok(gps) => {
            info!("GPS initialized on UART1");
            device_status.gps = PeripheralStatus::Ok;
            Some(gps)
        }
        Err(e) => {
            error!(
                "Failed to initialize GPS: {:?} - continuing without GPS support",
                e
            );
            device_status.gps = PeripheralStatus::Error(PeripheralError::UartError);
            None
        }
    };

    // Initialize servo on GPIO1
    static LEDC: StaticCell<esp_hal::ledc::Ledc<'static>> = StaticCell::new();
    static SERVO_TIMER: StaticCell<esp_hal::ledc::timer::Timer<'static, esp_hal::ledc::LowSpeed>> =
        StaticCell::new();

    let mut ledc = esp_hal::ledc::Ledc::new(peripherals.LEDC);
    ledc.set_global_slow_clock(esp_hal::ledc::LSGlobalClkSource::APBClk);
    let ledc = LEDC.init(ledc);
    let mut servo = match servo::init_timer(ledc) {
        Ok(timer) => {
            let servo_timer = SERVO_TIMER.init(timer);
            match servo::init_channel(ledc, peripherals.GPIO1, servo_timer) {
                Ok(s) => {
                    info!("Servo initialized successfully");
                    device_status.servo = PeripheralStatus::Ok;
                    Some(s)
                }
                Err(e) => {
                    error!("Failed to initialize servo channel: {:?}", e);
                    device_status.servo = PeripheralStatus::Error(PeripheralError::ChannelError);
                    None
                }
            }
        }
        Err(e) => {
            error!("Failed to initialize servo timer: {:?}", e);
            device_status.servo = PeripheralStatus::Error(PeripheralError::TimerError);
            None
        }
    };

    // Initialize BLE
    let radio_init = match esp_radio::init() {
        Ok(init) => init,
        Err(e) => {
            defmt::panic!("Failed to initialize Wi-Fi/BLE controller: {:?}", e);
        }
    };
    let transport = match BleConnector::new(&radio_init, peripherals.BT, Default::default()) {
        Ok(t) => t,
        Err(e) => {
            defmt::panic!("Failed to create BLE connector: {:?}", e);
        }
    };
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
    let server = match SensorServer::new_with_config(GapConfig::Peripheral(PeripheralConfig {
        name: "Pebble",
        appearance: &appearance::sensor::GENERIC_SENSOR,
    })) {
        Ok(s) => SERVER.init(s),
        Err(e) => {
            defmt::panic!("Failed to create GATT server: {:?}", e);
        }
    };

    info!("GATT server created");

    // Broadcast device status and set initial BLE characteristic value
    DEVICE_STATUS.sender().send(device_status);
    let status_bytes = device_status.to_bytes();
    info!("Device status bytes: {:?}", &status_bytes[..10]);
    match server
        .sensor_service
        .device_status
        .set(server, &status_bytes)
    {
        Ok(_) => info!("Device status characteristic set successfully"),
        Err(_e) => error!("Failed to set device status characteristic"),
    }
    info!(
        "Device status broadcast: LEDs={}, GPS={}, Servo={}, IMU={}, Mag={}",
        device_status.leds.to_bytes().0,
        device_status.gps.to_bytes().0,
        device_status.servo.to_bytes().0,
        device_status.imu.to_bytes().0,
        device_status.magnetometer.to_bytes().0,
    );

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
                        GattConnectionEvent::Gatt {
                            event: GattEvent::Write(write_event),
                        } => {
                            let handle = write_event.handle();
                            let value = write_event.data();
                            tasks::ble::handle_led_write(handle, value, server);
                        }
                        _ => {}
                    }
                }
            };

            // Task to send sensor notifications
            let sensor_notify = async {
                let Some(mut receiver) = SENSOR_DATA.receiver() else {
                    error!("No sensor data receiver slot available");
                    return;
                };
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

            // Task to send satellite info notifications (chunk 0)
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

            // Task to send satellite info notifications (chunk 1)
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

            // Task to send device status on connect (one-shot, then watches for changes)
            let status_notify = async {
                let Some(mut receiver) = DEVICE_STATUS.receiver() else {
                    error!("No device status receiver slot available");
                    return;
                };
                // Send current status immediately on connect
                let status = receiver.get().await;
                let _ = server
                    .sensor_service
                    .device_status
                    .notify(&conn, &status.to_bytes())
                    .await;

                // Watch for any status changes (unlikely after init, but supported)
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

            // Run all notification tasks - any one completing/failing will end the connection
            // Use nested selects since select only supports up to 5 futures
            embassy_futures::select::select(
                embassy_futures::select::select5(
                    gatt_events,
                    sensor_notify,
                    led_notify,
                    gps_notify,
                    status_notify,
                ),
                embassy_futures::select::select(satellites_0_notify, satellites_1_notify),
            )
            .await;

            let remaining = ACTIVE_CONNECTIONS.fetch_sub(1, Ordering::Relaxed) - 1;
            info!("Client disconnected ({} remaining)", remaining);
        }
    };

    // Task to sync LED state to BLE characteristics
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
