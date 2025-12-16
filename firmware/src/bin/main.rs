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
use esp_hal::i2c::master::{Config as I2cConfig, I2c};
use esp_hal::time::Rate;
use esp_hal::timer::timg::TimerGroup;
use esp_radio::ble::controller::BleConnector;
use panic_rtt_target as _;
use pebble::ble::{AccBleData, GyroBleData, MagBleData, SensorServer};
use pebble::hal::imu;
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
                        GattConnectionEvent::Gatt { event: _ } => {}
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

            embassy_futures::select::select(gatt_events, sensor_notify).await;

            let remaining = ACTIVE_CONNECTIONS.fetch_sub(1, Ordering::Relaxed) - 1;
            info!("Client disconnected ({} remaining)", remaining);
        }
    };

    // Run all tasks concurrently
    embassy_futures::join::join3(runner_task, imu_task, ble_task).await;

    loop {
        Timer::after(Duration::from_secs(1)).await;
    }
}
