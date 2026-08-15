//! Servo task: controls the servo motor based on commands.
//!
//! This task receives servo commands from the control layer and
//! moves the servo to the requested position.

use defmt::{error, info};
use embassy_time::{Duration, Timer};

use crate::hal::servo::Servo;
use crate::state::SERVO_COMMAND;

/// Run the servo command handler task.
///
/// Listens for servo commands and moves the servo to the requested
/// angle. If no servo hardware is available, the task does nothing.
pub async fn run(servo: &mut Option<Servo<'_>>) -> ! {
    let Some(servo) = servo.as_mut() else {
        info!("Servo task disabled (no servo hardware)");
        loop {
            Timer::after(Duration::from_secs(60)).await;
        }
    };

    let Some(mut cmd_receiver) = SERVO_COMMAND.receiver() else {
        error!("No servo command receiver slot available");
        loop {
            Timer::after(Duration::from_secs(60)).await;
        }
    };

    // Initialize to locked position (0 degrees)
    if let Err(e) = servo.set_angle(0) {
        error!("Failed to initialize servo: {:?}", e);
    }

    info!("Servo task started");

    loop {
        let cmd = cmd_receiver.changed().await;

        if let Err(e) = servo.set_angle(cmd.angle) {
            error!("Servo error: {:?}", e);
        } else {
            info!("Servo moved to {} degrees", cmd.angle);
        }
    }
}

/// Close angle for the servo (0° = box closed).
const CLOSE_ANGLE: u8 = 0;

/// Run the servo demonstration task.
///
/// Holds the servo at the closed position. For testing/demo purposes only.
pub async fn run_demo(servo: &mut Option<Servo<'_>>) -> ! {
    let Some(servo) = servo.as_mut() else {
        info!("Servo demo task disabled (no servo hardware)");
        loop {
            Timer::after(Duration::from_secs(60)).await;
        }
    };

    if let Err(e) = servo.set_angle(CLOSE_ANGLE) {
        error!("Servo error: {:?}", e);
    } else {
        info!("Servo set to close ({} degrees)", CLOSE_ANGLE);
    }

    loop {
        Timer::after(Duration::from_secs(60)).await;
    }
}
