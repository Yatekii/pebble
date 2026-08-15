//! Event types for puzzle layer communication.
//!
//! This module defines the events that puzzles can receive and the actions
//! they can emit.

use crate::filter::device_state::DeviceState;

/// Events that can be sent to puzzles.
#[derive(Clone, Copy)]
pub enum PuzzleEvent {
    /// Device state has been updated (orientation, GPS, etc.).
    DeviceStateChanged(DeviceState),

    /// A single tap/knock on the PCB was detected via the accelerometer.
    Tap,

    /// Two taps in quick succession were detected via the accelerometer.
    DoubleTap,

    /// A timeout occurred (for time-based puzzles).
    Timeout,

    /// External command received (e.g., from BLE).
    ExternalCommand(Command),
}

/// External commands that can trigger puzzle actions.
#[derive(Clone, Copy)]
pub enum Command {
    /// Skip the current puzzle (debug/admin only).
    Skip,
    /// Reset all puzzle progress.
    Reset,
    /// Hint request.
    Hint,
}

/// Actions that puzzles can emit.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Action {
    /// Set LED pattern.
    SetLeds(LedPattern),

    /// Move servo to position.
    MoveServo(ServoPosition),

    /// Puzzle completed, advance to next.
    Complete,

    /// Show a hint to the user.
    ShowHint(HintType),
}

/// LED pattern types for puzzle feedback.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum LedPattern {
    /// All LEDs off.
    Off,

    /// All LEDs solid color.
    Solid { r: u8, g: u8, b: u8 },

    /// Point the ring at a target: direction to it in degrees, clockwise from
    /// the box front (0 = straight ahead). Used to steer toward a waypoint.
    Compass { bearing_deg: u16 },

    /// Pulsing animation.
    Pulse { r: u8, g: u8, b: u8, speed_ms: u16 },

    /// Success animation (green sweep).
    Success,

    /// Error/wrong animation (red flash).
    Error,

    /// Progress indicator (partial ring lit).
    Progress { percent: u8, r: u8, g: u8, b: u8 },
}

/// Servo positions.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum ServoPosition {
    /// Locked position (0 degrees).
    Locked,
    /// Unlocked position (90 degrees).
    Unlocked,
    /// Custom angle (0-180 degrees).
    Angle(u8),
}

/// Hint types for user feedback.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum HintType {
    /// Direction hint (for navigation puzzles).
    Direction { heading: u16 },
    /// Distance hint (for GPS puzzles).
    Distance { meters: u32 },
    /// Tilt hint (for orientation puzzles).
    Tilt { direction: TiltDirection },
}

/// Tilt directions for orientation puzzles.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum TiltDirection {
    Forward,
    Back,
    Left,
    Right,
    LevelOut,
}
