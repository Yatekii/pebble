//! LED task: displays compass heading on the NeoPixel ring.

use crate::hal::led::{Color, LedStrip, NUM_LEDS};
use crate::state::COMPASS_HEADING;

/// Run the LED compass display task.
///
/// Shows a single red LED pointing magnetic north by subscribing
/// to the compass heading watch.
pub async fn run_compass(leds: &mut LedStrip<'_>) -> ! {
    // Test mode: only LED 1 on
    leds.set_brightness(64);
    leds.clear();
    leds.set(1, Color::red());
    let _ = leds.show();

    // Keep running forever
    loop {
        embassy_futures::yield_now().await;
    }
}
