//! Puzzle runtime task: the control-layer keystone.
//!
//! Owns the [`PuzzleStateMachine`], turns sensor/tap watch updates into
//! [`PuzzleEvent`]s, feeds them to the active puzzle, and bridges the returned
//! [`Action`]s onto the hardware command watches (`SERVO_COMMAND`, `LED_COMMAND`).
//!
//! Scope note: only [`CompassAlignPuzzle`] exists today, so only that arm is
//! wired. `LocationFind`/`TiltSequence` + proper enum dispatch land in #7, GPS
//! folding into `DeviceState` in #5, and the real LED ring renderer in #8/#10.

use defmt::{error, info};
use embassy_futures::select::{Either3, select3};

use crate::filter::device_state::DeviceState;
use crate::puzzle::events::{Action, LedPattern, PuzzleEvent, ServoPosition};
use crate::puzzle::puzzles::compass_align::CompassAlignPuzzle;
use crate::puzzle::state_machine::{PuzzleId, PuzzleStateMachine};
use crate::state::{
    COMPASS_HEADING, DOUBLE_TAP_EVENT, LED_COMMAND, LedCommand, SERVO_COMMAND, ServoCommand,
    TAP_EVENT,
};

/// Apply to all LEDs (mirrors `tasks::led::LED_INDEX_ALL_LEDS`).
const LED_INDEX_ALL_LEDS: u8 = 0xFF;

/// Target heading for the compass-alignment puzzle, in degrees (0-359).
// ponytail: hard-coded secret; move to flash/config if puzzles need per-box targets.
const COMPASS_TARGET_HEADING: u16 = 0;

/// Run the puzzle state machine, dispatching events and bridging actions.
pub async fn run() -> ! {
    let Some(mut heading_rx) = COMPASS_HEADING.receiver() else {
        error!("No COMPASS_HEADING receiver slot for puzzle task");
        idle().await
    };
    let Some(mut tap_rx) = TAP_EVENT.receiver() else {
        error!("No TAP_EVENT receiver slot for puzzle task");
        idle().await
    };
    let Some(mut double_tap_rx) = DOUBLE_TAP_EVENT.receiver() else {
        error!("No DOUBLE_TAP_EVENT receiver slot for puzzle task");
        idle().await
    };

    let mut machine = PuzzleStateMachine::new();
    let mut compass = CompassAlignPuzzle::new(COMPASS_TARGET_HEADING);

    // ponytail: single tap is a global open/close toggle, so it never reaches the
    // FSM as puzzle input. If puzzles ever need single-tap input, gate this on
    // machine.current_puzzle() == PuzzleId::Unlocked instead.
    let mut box_open = false;

    info!(
        "Puzzle task started (puzzle {})",
        machine.current_puzzle() as u8
    );

    loop {
        let event = match select3(
            heading_rx.changed(),
            tap_rx.changed(),
            double_tap_rx.changed(),
        )
        .await
        {
            Either3::First(heading) => PuzzleEvent::DeviceStateChanged(DeviceState {
                heading,
                ..Default::default()
            }),
            Either3::Second(_) => {
                box_open = !box_open;
                let pos = if box_open {
                    ServoPosition::Unlocked
                } else {
                    ServoPosition::Locked
                };
                info!("Tap toggle: box {}", if box_open { "open" } else { "closed" });
                apply_action(Action::MoveServo(pos));
                continue;
            }
            Either3::Third(_) => PuzzleEvent::DoubleTap,
        };

        // Dispatch to the active puzzle. Only CompassAlign is implemented; the
        // rest arrive with #7's enum dispatch.
        let action = match machine.current_puzzle() {
            PuzzleId::CompassAlign => machine.handle_event(&mut compass, event),
            PuzzleId::LocationFind | PuzzleId::TiltSequence | PuzzleId::Unlocked => None,
        };

        if let Some(action) = action {
            apply_action(action);
        }
    }
}

/// Bridge a puzzle [`Action`] onto the hardware command watches.
fn apply_action(action: Action) {
    match action {
        Action::MoveServo(pos) => {
            SERVO_COMMAND.sender().send(ServoCommand { angle: servo_angle(pos) });
        }
        Action::SetLeds(pattern) => {
            if let Some(cmd) = led_command(pattern) {
                LED_COMMAND.sender().send(cmd);
            }
        }
        // Completion is turned into servo/LED actions inside the state machine;
        // hints are speculative (no consumer yet).
        Action::Complete | Action::ShowHint(_) => {}
    }
}

/// Map a [`ServoPosition`] to a raw angle in degrees.
fn servo_angle(pos: ServoPosition) -> u8 {
    match pos {
        ServoPosition::Locked => 0,
        ServoPosition::Unlocked => 180,
        ServoPosition::Angle(a) => a,
    }
}

/// Map the whole-ring LED patterns to an all-LEDs command. Ring-relative
/// patterns (`Compass`, `Progress`) need the dedicated renderer from #8/#10 and
/// are skipped here.
fn led_command(pattern: LedPattern) -> Option<LedCommand> {
    let (r, g, b) = match pattern {
        LedPattern::Off => (0, 0, 0),
        LedPattern::Solid { r, g, b } => (r, g, b),
        LedPattern::Pulse { r, g, b, .. } => (r, g, b),
        LedPattern::Success => (0, 255, 0),
        LedPattern::Error => (255, 0, 0),
        LedPattern::Compass { .. } | LedPattern::Progress { .. } => return None,
    };
    Some(LedCommand {
        brightness: 255,
        led_index: LED_INDEX_ALL_LEDS,
        r,
        g,
        b,
    })
}

/// Park forever when a required watch slot is unavailable.
async fn idle() -> ! {
    loop {
        embassy_futures::yield_now().await;
    }
}
