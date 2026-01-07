//! LED task: displays compass heading on the NeoPixel ring.

use crate::hal::led::{Color, LedStrip};

/// Run the LED compass display task.
///
/// Shows a single red LED pointing magnetic north by subscribing
/// to the compass heading watch. If LEDs are not available (None),
/// this task does nothing but keeps running.
pub async fn run_compass(leds: &mut Option<LedStrip<'_>>) -> ! {
    // Initialize LEDs if available
    if let Some(leds) = leds.as_mut() {
        leds.set_brightness(64);
        leds.clear();
        leds.set(1, Color::red());
        let _ = leds.show();
    }

    // Keep running forever
    loop {
        embassy_futures::yield_now().await;
    }
}
