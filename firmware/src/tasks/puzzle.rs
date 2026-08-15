//! Puzzle runtime task: the control-layer keystone.
//!
//! Owns the [`PuzzleStateMachine`], turns sensor/tap watch updates into
//! [`PuzzleEvent`]s, feeds them to the active puzzle, and bridges the returned
//! [`Action`]s onto the hardware watches (`SERVO_COMMAND` for the lock,
//! `PUZZLE_LED` for the ring).
//!
//! Scope note: only [`DirectionsPuzzle`] is wired today. `Waypoints` (GPS
//! geofences) and `Knock` (rhythm) land in their own PRs; until then those
//! states are inert.

use defmt::{error, info};
use embassy_futures::select::{Either3, select3};

use crate::filter::device_state::DeviceState;
use crate::puzzle::events::{Action, PuzzleEvent, ServoPosition};
use crate::puzzle::puzzles::directions::{self, DirectionsPuzzle};
use crate::puzzle::state_machine::{PuzzleId, PuzzleStateMachine};
use crate::state::{
    COMPASS_HEADING, DOUBLE_TAP_EVENT, PUZZLE_LED, SERVO_COMMAND, ServoCommand, TAP_EVENT,
};

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
    let mut directions = DirectionsPuzzle::new(&directions::TEST_SEQUENCE);

    // ponytail: single tap is a global open/close toggle, so it never reaches the
    // FSM as puzzle input. The Knock puzzle PR will gate this on the active state.
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

        // Dispatch to the active puzzle. Only Directions is wired today.
        let action = match machine.current_puzzle() {
            PuzzleId::Directions => machine.handle_event(&mut directions, event),
            PuzzleId::Waypoints | PuzzleId::Knock | PuzzleId::Unlocked => None,
        };

        if let Some(action) = action {
            apply_action(action);
        }
    }
}

/// Bridge a puzzle [`Action`] onto the hardware watches.
fn apply_action(action: Action) {
    match action {
        Action::MoveServo(pos) => {
            SERVO_COMMAND.sender().send(ServoCommand { angle: servo_angle(pos) });
        }
        Action::SetLeds(pattern) => {
            PUZZLE_LED.sender().send(pattern);
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

/// Park forever when a required watch slot is unavailable.
async fn idle() -> ! {
    loop {
        embassy_futures::yield_now().await;
    }
}
