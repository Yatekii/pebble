#![no_std]
#![no_main]
#![deny(clippy::all)]

use bt_hci::controller::ExternalController;
use defmt::{error, info};
use embassy_executor::Spawner;
use embedded_hal::delay::DelayNs;
use esp_hal::clock::CpuClock;
use esp_hal::delay::Delay;
use esp_hal::timer::timg::TimerGroup;
use esp_radio::ble::controller::BleConnector;
use panic_rtt_target as _;
use pebble::comms::ble::SensorServer;
use pebble::hal::{gps, imu, led, servo};
use pebble::state::{
    DEVICE_STATUS, DeviceStatus, LED_COMMAND, LedCommand, PeripheralError, PeripheralStatus,
};
use pebble::tasks;
use static_cell::StaticCell;
use trouble_host::prelude::*;

extern crate alloc;

const CONNECTIONS_MAX: usize = 2;
const L2CAP_CHANNELS_MAX: usize = 4;

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

    // Initialize IMU on shared bus (with retries; the bus can time out on a
    // cold boot, so reinitialize a few times before giving up).
    let imu = {
        let mut imu = None;
        for attempt in 1..=5 {
            match imu::bmi270::SharedBmi270::new(shared_i2c, &mut delay) {
                Ok(dev) => {
                    info!("IMU initialized successfully");
                    device_status.imu = PeripheralStatus::Ok;
                    imu = Some(dev);
                    break;
                }
                Err(e) => {
                    info!("Failed to initialize IMU (attempt {}): {:?}", attempt, e);
                    delay.delay_ms(100);
                }
            }
        }
        match imu {
            Some(imu) => imu,
            None => defmt::panic!("Failed to initialize IMU after retries"),
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
        peripheral, runner, ..
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

    // Run the BLE peripheral (host stack, connections, notifications, LED mirror)
    // alongside the sensor/actuator/puzzle tasks.
    embassy_futures::join::join3(
        tasks::ble::run(server, peripheral, runner),
        embassy_futures::join::join4(
            tasks::imu::run(&imu, magnetometer.as_ref()),
            tasks::gps::run(&mut gps),
            tasks::servo::run(&mut servo),
            tasks::puzzle::run(),
        ),
        tasks::led::run_compass(&mut leds),
    )
    .await;

    #[allow(clippy::empty_loop)]
    loop {}
}
