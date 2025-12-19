//! Servo task: moves the servo through a demonstration sequence.

use defmt::{error, info};
use embassy_time::{Duration, Timer};

use crate::hal::servo::Servo;

/// Run the servo demonstration task.
///
/// Moves the servo through positions: 0° → 90° → 180° → 90° → 0° in a loop.
pub async fn run(servo: &mut Option<Servo<'_>>) -> ! {
    if let Some(servo) = servo {
        let positions: [u8; 4] = [0, 90, 180, 90];
        let mut idx = 0;

        loop {
            let angle = positions[idx];
            if let Err(e) = servo.set_angle(angle) {
                error!("Servo error: {:?}", e);
            } else {
                info!("Servo moved to {} degrees", angle);
            }

            idx = (idx + 1) % positions.len();
            Timer::after(Duration::from_secs(2)).await;
        }
    } else {
        // No servo, just wait forever
        loop {
            Timer::after(Duration::from_secs(60)).await;
        }
    }
}
