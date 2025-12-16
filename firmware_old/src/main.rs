#![no_std]
#![no_main]

use esp_backtrace as _;

mod hal;
mod puzzle;
mod state;

use hal::leds::{self, Rgb, LED_COUNT};

#[esp_hal::main]
fn main() -> ! {
    rtt_target::rtt_init_defmt!();

    defmt::info!("Pebble firmware initialized");

    let peripherals = esp_hal::init(esp_hal::Config::default());

    defmt::info!("Pebble firmware initialized");

    // // Initialize LED strip
    // let mut leds = match leds::init(peripherals.RMT, peripherals.GPIO15) {
    //     Ok(leds) => {
    //         defmt::info!("WS2812B LEDs initialized ({} LEDs)", LED_COUNT);
    //         leds
    //     }
    //     Err(e) => {
    //         defmt::error!("Failed to initialize LEDs: {:?}", defmt::Debug2Format(&e));
    //         panic!("LED initialization failed");
    //     }
    // };

    // // Turn on LED 0 to indicate boot
    // let mut colors = [Rgb::BLACK; LED_COUNT];
    // colors[0] = Rgb::GREEN;
    // if let Err(e) = leds.write(&colors) {
    //     defmt::error!("Failed to write LEDs: {:?}", defmt::Debug2Format(&e));
    // }

    defmt::info!("LED 0 turned on (green)");

    // Initialize IMU
    // let _imu = match hal::imu::init(peripherals.I2C0, peripherals.GPIO2, peripherals.GPIO3) {
    //     Ok(imu) => {
    //         defmt::info!("BMI270 IMU initialized successfully");
    //         Some(imu)
    //     }
    //     Err(e) => {
    //         defmt::error!("Failed to initialize IMU: {:?}", defmt::Debug2Format(&e));
    //         None
    //     }
    // };

    // Initialize GPS
    // let _gps = match hal::gps::init(peripherals.UART1, peripherals.GPIO8, peripherals.GPIO18) {
    //     Ok(gps) => {
    //         defmt::info!("GPS initialized");
    //         Some(gps)
    //     }
    //     Err(e) => {
    //         defmt::error!("Failed to initialize GPS: {:?}", defmt::Debug2Format(&e));
    //         None
    //     }
    // };

    // Initialize Servo
    // let _servo = match hal::servo::init(peripherals.LEDC, peripherals.GPIO1) {
    //     Ok(servo) => {
    //         defmt::info!("Servo initialized");
    //         Some(servo)
    //     }
    //     Err(e) => {
    //         defmt::error!("Failed to initialize Servo: {:?}", defmt::Debug2Format(&e));
    //         None
    //     }
    // };

    loop {
        // Main loop - currently idle
        defmt::error!("HELP!");
    }
}
