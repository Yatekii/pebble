//! LED task: displays patterns on the NeoPixel ring.
//!
//! This task receives LED commands from the control layer or BLE
//! and updates the LED ring accordingly.

use defmt::{error, info};
use embassy_time::{Duration, Timer};

use crate::hal::led::{Color, LedStrip, NUM_LEDS};
use crate::puzzle::events::LedPattern;
use crate::state::{LED_COLORS_0, LED_COLORS_1, LED_COLORS_2, LED_COMMAND, PUZZLE_LED};

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

/// Run the puzzle LED task.
///
/// Renders the active puzzle's ring feedback published on [`PUZZLE_LED`]:
/// a blue fill arc while a direction is being held, and green/red flashes on
/// correct/wrong. Replaces the old standalone north-compass demo.
pub async fn run_puzzle(leds: &mut Option<LedStrip<'_>>) -> ! {
    let Some(leds) = leds.as_mut() else {
        info!("LED puzzle task disabled (no LED hardware)");
        loop {
            embassy_futures::yield_now().await;
        }
    };

    let Some(mut rx) = PUZZLE_LED.receiver() else {
        error!("No PUZZLE_LED receiver slot available");
        loop {
            embassy_futures::yield_now().await;
        }
    };

    leds.set_brightness(0);
    leds.clear();
    let _ = leds.show();

    info!("LED puzzle task started");

    loop {
        let pattern = rx.changed().await;
        render(leds, pattern).await;
    }
}

/// Render one puzzle [`LedPattern`] to the ring.
async fn render(leds: &mut LedStrip<'_>, pattern: LedPattern) {
    match pattern {
        LedPattern::Off => {
            leds.set_brightness(0);
            leds.clear();
        }
        LedPattern::Solid { r, g, b } | LedPattern::Pulse { r, g, b, .. } => {
            leds.set_brightness(255);
            leds.set_all(Color::new(r, g, b));
        }
        LedPattern::Progress { percent, r, g, b } => {
            let lit = (percent.min(100) as usize * NUM_LEDS).div_ceil(100);
            leds.set_brightness(255);
            leds.clear();
            for i in 0..lit {
                leds.set(i, Color::new(r, g, b));
            }
        }
        LedPattern::Success => return flash_twice(leds, Color::green()).await,
        LedPattern::Error => return flash_twice(leds, Color::red()).await,
        // Compass rendering arrives with the waypoint puzzle.
        LedPattern::Compass { .. } => return,
    }
    if leds.show().is_err() {
        error!("Failed to update puzzle LEDs");
    }
}

/// Flash the whole ring `color` twice (120 ms on / 120 ms off), then clear.
async fn flash_twice(leds: &mut LedStrip<'_>, color: Color) {
    leds.set_brightness(255);
    for _ in 0..2 {
        leds.set_all(color);
        let _ = leds.show();
        Timer::after(Duration::from_millis(120)).await;
        leds.clear();
        let _ = leds.show();
        Timer::after(Duration::from_millis(120)).await;
    }
}
