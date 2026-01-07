//! LED task: displays patterns on the NeoPixel ring.
//!
//! This task receives LED commands from the control layer or BLE
//! and updates the LED ring accordingly.

use defmt::{error, info};

use crate::hal::led::{Color, LedStrip, NUM_LEDS};
use crate::state::{COMPASS_HEADING, LED_COLORS_0, LED_COLORS_1, LED_COLORS_2, LED_COMMAND};

/// Special LED index values.
const LED_INDEX_BRIGHTNESS_ONLY: u8 = 0xFE;
const LED_INDEX_ALL_LEDS: u8 = 0xFF;
const LED_INDEX_CHUNK_0: u8 = 0xF0;
const LED_INDEX_CHUNK_1: u8 = 0xF1;
const LED_INDEX_CHUNK_2: u8 = 0xF2;

/// Run the LED command handler task.
///
/// Listens for LED commands and updates the LED ring accordingly.
/// Supports individual LED control, all-LED control, brightness,
/// and chunk-based color updates.
pub async fn run(leds: &mut Option<LedStrip<'_>>) -> ! {
    let Some(leds) = leds.as_mut() else {
        info!("LED task disabled (no LED hardware)");
        loop {
            embassy_futures::yield_now().await;
        }
    };

    // Initialize to off
    leds.set_brightness(0);
    leds.clear();
    let _ = leds.show();

    let Some(mut cmd_receiver) = LED_COMMAND.receiver() else {
        error!("No LED command receiver slot available");
        loop {
            embassy_futures::yield_now().await;
        }
    };

    info!("LED task started");

    loop {
        let cmd = cmd_receiver.changed().await;

        match cmd.led_index {
            LED_INDEX_BRIGHTNESS_ONLY => {
                leds.set_brightness(cmd.brightness);
            }
            LED_INDEX_ALL_LEDS => {
                leds.set_brightness(cmd.brightness);
                leds.set_all(Color::new(cmd.r, cmd.g, cmd.b));
            }
            LED_INDEX_CHUNK_0 => {
                if let Some(colors) = LED_COLORS_0.try_get() {
                    apply_color_chunk(leds, &colors, 0);
                }
            }
            LED_INDEX_CHUNK_1 => {
                if let Some(colors) = LED_COLORS_1.try_get() {
                    apply_color_chunk(leds, &colors, 24);
                }
            }
            LED_INDEX_CHUNK_2 => {
                if let Some(colors) = LED_COLORS_2.try_get() {
                    apply_color_chunk(leds, &colors, 48);
                }
            }
            index if (index as usize) < NUM_LEDS => {
                leds.set_brightness(cmd.brightness);
                leds.set(index as usize, Color::new(cmd.r, cmd.g, cmd.b));
            }
            _ => {
                // Invalid index, ignore
            }
        }

        if let Err(_e) = leds.show() {
            error!("Failed to update LEDs");
        }
    }
}

/// Apply a 72-byte color chunk to LEDs starting at the given offset.
fn apply_color_chunk(leds: &mut LedStrip<'_>, colors: &[u8; 72], start_led: usize) {
    for i in 0..24 {
        let led_idx = start_led + i;
        if led_idx < NUM_LEDS {
            let color_idx = i * 3;
            leds.set(
                led_idx,
                Color::new(
                    colors[color_idx],
                    colors[color_idx + 1],
                    colors[color_idx + 2],
                ),
            );
        }
    }
}

/// Run the LED compass display task.
///
/// Shows a compass indicator pointing to north by subscribing
/// to the compass heading watch. The LED at the heading position
/// is lit red.
pub async fn run_compass(leds: &mut Option<LedStrip<'_>>) -> ! {
    let Some(leds) = leds.as_mut() else {
        info!("LED compass task disabled (no LED hardware)");
        loop {
            embassy_futures::yield_now().await;
        }
    };

    let Some(mut heading_receiver) = COMPASS_HEADING.receiver() else {
        error!("No compass heading receiver slot available");
        loop {
            embassy_futures::yield_now().await;
        }
    };

    // Initialize compass display
    leds.set_brightness(64);
    leds.clear();
    let _ = leds.show();

    info!("LED compass task started");

    loop {
        let heading = heading_receiver.changed().await;

        // Convert heading (0-359) to LED index (0-71)
        // LED 0 is at north (0 degrees), LEDs go clockwise
        let led_index = ((heading as usize * NUM_LEDS) / 360) % NUM_LEDS;

        // Clear all and light the north indicator
        leds.clear();
        leds.set(led_index, Color::red());

        if let Err(_e) = leds.show() {
            error!("Failed to update compass LEDs");
        }
    }
}
