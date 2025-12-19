//! BLE helper functions for LED command processing.

use defmt::info;

use crate::comms::ble::SensorServer;
use crate::state::{LED_COLORS_0, LED_COLORS_1, LED_COLORS_2, LED_COMMAND, LedCommand};

/// Process a GATT write to LED-related characteristics.
///
/// Returns true if the write was handled.
pub fn handle_led_write(handle: u16, value: &[u8], server: &SensorServer) -> bool {
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
        return true;
    }

    // LED brightness: single byte
    if handle == server.sensor_service.led_brightness.handle {
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
        return true;
    }

    // LED colors chunk 0 (LEDs 0-23)
    if handle == server.sensor_service.led_colors_0.handle {
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
        return true;
    }

    // LED colors chunk 1 (LEDs 24-47)
    if handle == server.sensor_service.led_colors_1.handle {
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
        return true;
    }

    // LED colors chunk 2 (LEDs 48-71)
    if handle == server.sensor_service.led_colors_2.handle {
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
        return true;
    }

    false
}
