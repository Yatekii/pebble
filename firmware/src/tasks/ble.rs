//! BLE helper functions for LED command processing.

use defmt::{info, warn};

use crate::comms::ble::SensorServer;
use crate::hal::led::NUM_LEDS;
use crate::state::{LED_COLORS_0, LED_COLORS_1, LED_COLORS_2, LED_COMMAND, LedCommand};

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
