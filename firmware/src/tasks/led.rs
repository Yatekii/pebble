//! LED task: displays compass heading on the NeoPixel ring.

use crate::hal::led::{Color, LedStrip, NUM_LEDS};
use crate::state::COMPASS_HEADING;

/// Run the LED compass display task.
///
/// Shows a single red LED pointing magnetic north by subscribing
/// to the compass heading watch.
pub async fn run_compass(leds: &mut LedStrip<'_>) -> ! {
    let mut heading_receiver = COMPASS_HEADING.receiver().unwrap();

    leds.set_brightness(64);

    loop {
        let heading = heading_receiver.changed().await;

        // Convert heading (0-359 degrees) to LED index (0-71)
        // LED 0 is at some physical position, heading 0 = magnetic north
        // 72 LEDs in a circle = 5 degrees per LED
        // Invert direction so LED points north (not rotates with heading)
        // Subtract 90 degree offset to correct for physical LED position
        let inverted_heading = (360 - heading + 270) % 360;
        let led_index = ((inverted_heading as u32 * NUM_LEDS as u32) / 360) as usize;

        leds.clear();
        leds.set(led_index, Color::red());
        let _ = leds.show();
    }
}
