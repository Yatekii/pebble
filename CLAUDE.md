# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

I am building a reverse geocache. A reverse geocache (in further writing I will just call it geocache or cache or puzzlebox) is a box, that opens when you bring it to the correct location.
Since that alone would be boring, the box has a couple of puzzles built in.
The electronic design contain an IMU (acc, gyro, compass) (9 axes) , a GPS, 72 Neopixel LEDs arranged in a circle, an ESP C6 with WIfi and Bluetooth and a USB charger. Additionally there is a reed switch which just connects the battery as a nice hidden on/off switch. There is also a Servo which has 180 degree range of motion and is controlled via a PWM signal.

The GPS is controlled via UART, the IMU (acc & gyro directly via I2C) and the Compass is attached to the IMU via I2c and uses the relay functionality.

This is the firmware component of theproject. The project consists of:

- **firmware/** - Rust firmware (this directory)
- **hardware/** - KiCad PCB designs including main board and USB fast charger schematic

The puzzlebox contains a number of puzzles that use the different devices in different ways.

# Architecture notes

The firmware should be structured as such that there is a puzzle module that then contains a module for each puzzle.
There should also be a HAL module which makes it easy to read sensor data or talk to the servo.
There should also be a state module which is an FSM that advances when a further puzzle is unlocked. The state should be persisted to Flash!

We use the Rust esp-hal from Github master and embassy to talk to peripherals.

Make sure to write idiomatic Rust code, DO NOT allocate and always fix warnings before commiting.

We build for the ESP32-C6 platform and use probe-rs to flash and debug and defmt for logging.

## Hardware Components

### Main Board

| Component           | Part Number     | Description                                                     | Datasheet                                                                                                         |
| ------------------- | --------------- | --------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------- |
| MCU                 | ESP32-C6-MINI-1 | WiFi 802.11ax + Bluetooth 5 + Zigbee/Thread module              | [Datasheet](https://www.espressif.com/sites/default/files/documentation/esp32-c6-mini-1_mini-1u_datasheet_en.pdf) |
| GPS                 | MAX-M8Q         | u-blox GNSS module, 2.7-3.6V                                    | [Datasheet](https://www.u-blox.com/sites/default/files/MAX-M8-FW3_DataSheet_%28UBX-15031506%29.pdf)               |
| IMU (Accel/Gyro)    | BMI270          | Bosch 6-axis IMU with I2C interface                             | [Datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi270-ds000.pdf)       |
| Magnetometer        | BMM350          | Bosch 3-axis magnetometer, connected via BMI270 I2C passthrough | [Datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmm350-ds001.pdf)       |
| LEDs                | WS2812B-2020    | Addressable RGB LEDs, 2.0x2.0mm, 72 units in a circle           | [Datasheet](https://cdn-shop.adafruit.com/product-files/4684/4684_WS2812B-2020_V1.3_EN.pdf)                       |
| 5V Boost Converter  | TPS61023DRLR    | Texas Instruments boost converter for LED power                 | [Datasheet](https://www.ti.com/lit/gpn/tps61023)                                                                  |
| 3.3V Buck Converter | TPS563200       | Texas Instruments 3A synchronous step-down regulator            | [Datasheet](http://www.ti.com/lit/ds/symlink/tps563200.pdf)                                                       |
| N-Channel MOSFET    | BSS123          | Logic-level N-channel MOSFET                                    | -                                                                                                                 |
| P-Channel MOSFET    | BSS84           | Logic-level P-channel MOSFET                                    | -                                                                                                                 |

### USB Charger Board

| Component       | Part Number   | Description                                     | Datasheet                                                                                     |
| --------------- | ------------- | ----------------------------------------------- | --------------------------------------------------------------------------------------------- |
| Battery Charger | MAX77751CEFG+ | 3.15A USB Type-C autonomous Li+ battery charger | [Datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/MAX77751.pdf) |
| Op-Amp          | LPV521        | Ultra-low power single operational amplifier    | [Datasheet](https://www.ti.com/lit/ds/symlink/lpv521.pdf)                                     |
| USB Switch      | MAX4906ELB+T  | USB 2.0 high-speed switch                       | [Datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/MAX4906E.pdf) |
| Reed Switch     | -             | Magnetic reed switch for power on/off           | -                                                                                             |
| USB Connector   | USB-C 16P     | USB 2.0 Type-C receptacle                       | -                                                                                             |

### Pin Assignments (ESP32-C6)

| GPIO | Signal    | Function                   |
| ---- | --------- | -------------------------- |
| IO0  | MAG_INT   | Compass Interrupt          |
| IO1  | PWM       | Servo PWM                  |
| IO2  | SDA2      | I2C data (BMI270/BMM350)   |
| IO3  | SCL2      | I2C clock (BMI270/BMM350)  |
| IO4  | IMU_INT2  | BMI270 interrupt 1         |
| IO5  | IMU_INT1  | BMI270 interrupt 2         |
| IO6  | VBAT      | Battery voltage ADC input  |
| IO7  | -         | Not connected              |
| IO8  | RX        | GPS UART RX (from GPS TXD) |
| IO9  | BOOT      | Boot mode pin              |
| IO12 | DN        | USB D-                     |
| IO13 | DP        | USB D+                     |
| IO14 | -         | Not connected              |
| IO15 | DIN       | WS2812B NeoPixel data      |
| IO18 | TX        | GPS UART TX (to GPS RXD)   |
| IO19 | TIME      | GPS time pulse             |
| IO20 | nGPS_RST  | GPS reset (active low)     |
| IO21 | SDA       | Secondary I2C data         |
| IO22 | SCL       | Secondary I2C clock        |
| IO23 | nSAFEBOOT | GPS safeboot (active low)  |
| TXD0 | TXD0      | Bootloader UART TX         |
| RXD0 | RXD0      | Bootloader UART RX         |

## Development Commands

### Building

```bash
cargo build
cargo build --release
```

### Running

```bash
cargo run
cargo run --release
```

### Testing

```bash
cargo test
cargo test <test_name>  # Run a specific test
```

### Linting

```bash
cargo clippy
cargo fmt --check      # Check formatting
cargo fmt              # Apply formatting
```
