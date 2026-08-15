//! LED task: displays patterns on the NeoPixel ring.
//!
//! This task receives LED commands from the control layer or BLE
//! and updates the LED ring accordingly.

use defmt::{error, info};
use embassy_futures::select::{Either3, select3};
use embassy_time::{Duration, Timer};

use crate::hal::led::{Color, LedStrip, NUM_LEDS};
use crate::state::{
    COMPASS_HEADING, DOUBLE_TAP_EVENT, IMMEDIATE_TAP_EVENT, LED_COLORS_0, LED_COLORS_1,
    LED_COLORS_2, LED_COMMAND,
};

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

/// Dead zone (degrees from the lit LED's center) before the compass indicator
/// re-targets. Half an LED sector is 2.5°; a bit more absorbs heading jitter.
const COMPASS_HYSTERESIS_DEG: usize = 4;

/// Run the LED compass display task.
///
/// Shows a compass indicator pointing to north and flashes the ring on
/// tap events: once for a single tap, twice for a double tap.
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
    let Some(mut tap_receiver) = IMMEDIATE_TAP_EVENT.receiver() else {
        error!("No IMMEDIATE_TAP_EVENT receiver slot available");
        loop {
            embassy_futures::yield_now().await;
        }
    };
    let Some(mut double_tap_receiver) = DOUBLE_TAP_EVENT.receiver() else {
        error!("No DOUBLE_TAP_EVENT receiver slot available");
        loop {
            embassy_futures::yield_now().await;
        }
    };
    // Initialize compass display
    leds.set_brightness(255);
    leds.clear();
    let _ = leds.show();

    info!("LED compass task started");

    let mut compass_led: usize = 0;

    loop {
        match select3(
            heading_receiver.changed(),
            tap_receiver.changed(),
            double_tap_receiver.changed(),
        )
        .await
        {
            Either3::First(heading) => {
                // Heading is CW from north. LED ring is numbered CCW (increasing index = CCW).
                // heading=0 → index 71 (LED 72, front), heading=90 → index 17 (LED 18, left side).
                //
                // COMPASS_HEADING is the raw tilt-compensated mag heading with no
                // temporal smoothing, so it jitters ±1-2°. Each LED spans 5°, so a
                // heading sitting near a sector boundary would flip between two
                // adjacent LEDs. Hysteresis: only re-target once the heading moves a
                // dead zone (> half a sector) past the currently-lit LED's center.
                let heading = heading as usize % 360;
                let current_center = ((compass_led + 1) % NUM_LEDS) * 360 / NUM_LEDS;
                let off = {
                    let d = heading.abs_diff(current_center);
                    if d > 180 { 360 - d } else { d }
                };
                if off > COMPASS_HYSTERESIS_DEG {
                    compass_led = (NUM_LEDS - 1 + heading * NUM_LEDS / 360) % NUM_LEDS;
                }
                leds.set_brightness(255);
                leds.clear();
                leds.set(compass_led, Color::white());
                if let Err(_e) = leds.show() {
                    error!("Failed to update compass LEDs");
                }
            }

            Either3::Second(_) | Either3::Third(_) => {
                flash(leds).await;
                restore_compass(leds, compass_led);
            }
        }
    }
}

/// Flash all LEDs white once (80 ms on / 80 ms off).
async fn flash(leds: &mut LedStrip<'_>) {
    leds.set_brightness(255);
    leds.set_all(Color::white());
    let _ = leds.show();
    Timer::after(Duration::from_millis(80)).await;

    leds.clear();
    let _ = leds.show();
    Timer::after(Duration::from_millis(80)).await;
}

/// Restore the single compass LED after a flash.
fn restore_compass(leds: &mut LedStrip<'_>, compass_led: usize) {
    leds.set_brightness(255);
    leds.clear();
    leds.set(compass_led, Color::white());
    let _ = leds.show();
}
