//! Puzzle runtime task: the control-layer keystone.
//!
//! Owns the [`PuzzleStateMachine`], turns sensor/tap watch updates into
//! [`PuzzleEvent`]s, feeds them to the active puzzle, and bridges the returned
//! [`Action`]s onto the hardware watches (`SERVO_COMMAND` for the lock,
//! `PUZZLE_LED` for the ring).
//!
//! Knock timing lives here: the knock puzzle wants every spike, so it reads the
//! raw `IMMEDIATE_TAP_EVENT` stream (not single/double-tap classification) and
//! this task attaches inter-knock gaps plus a quiet-period finalize timeout.

use defmt::{error, info};
use embassy_futures::select::{Either, Either4, select, select4};
use embassy_time::{Duration, Instant, Timer};

use crate::filter::device_state::{DeviceState, GpsPosition};
use crate::puzzle::events::{Action, PuzzleEvent, ServoPosition};
use crate::puzzle::puzzles::directions::{self, DirectionsPuzzle};
use crate::puzzle::puzzles::knock::{self, KnockPuzzle, START_GAP};
use crate::puzzle::puzzles::waypoints::{self, WaypointsPuzzle};
use crate::puzzle::state_machine::{PuzzleId, PuzzleStateMachine};
use crate::state::{
    COMPASS_HEADING, GPS_DATA, IMMEDIATE_TAP_EVENT, PUZZLE_LED, SERVO_COMMAND, ServoCommand,
    TAP_EVENT,
};

/// Quiet time after the last knock that ends a knock attempt.
const KNOCK_FINALIZE: Duration = Duration::from_millis(2000);

/// Run the puzzle state machine, dispatching events and bridging actions.
pub async fn run() -> ! {
    let Some(mut heading_rx) = COMPASS_HEADING.receiver() else {
        error!("No COMPASS_HEADING receiver slot for puzzle task");
        idle().await
    };
    let Some(mut gps_rx) = GPS_DATA.receiver() else {
        error!("No GPS_DATA receiver slot for puzzle task");
        idle().await
    };
    let Some(mut tap_rx) = TAP_EVENT.receiver() else {
        error!("No TAP_EVENT receiver slot for puzzle task");
        idle().await
    };
    let Some(mut knock_rx) = IMMEDIATE_TAP_EVENT.receiver() else {
        error!("No IMMEDIATE_TAP_EVENT receiver slot for puzzle task");
        idle().await
    };

    let mut machine = PuzzleStateMachine::new();
    let mut directions = DirectionsPuzzle::new(&directions::TEST_SEQUENCE);
    let mut waypoints = WaypointsPuzzle::new(&waypoints::TEST_WAYPOINTS);
    let mut knock = KnockPuzzle::new(&knock::TEST_PATTERN);

    // Latest sensor values, so every event carries a full device state.
    let mut heading = 0u16;
    let mut position: Option<GpsPosition> = None;

    // Knock timing.
    let mut last_knock: Option<Instant> = None;
    let mut knock_deadline: Option<Instant> = None;

    // ponytail: single tap is a debug open/close toggle, gated off while the
    // knock puzzle is active so knocks don't fling the servo around.
    let mut box_open = false;

    info!(
        "Puzzle task started (puzzle {})",
        machine.current_puzzle() as u8
    );

    loop {
        // Fires only while a knock attempt is in progress.
        let finalize = async {
            match knock_deadline {
                Some(d) => Timer::at(d).await,
                None => core::future::pending::<()>().await,
            }
        };

        let event = match select(
            select4(
                heading_rx.changed(),
                gps_rx.changed(),
                tap_rx.changed(),
                knock_rx.changed(),
            ),
            finalize,
        )
        .await
        {
            Either::First(Either4::First(h)) => {
                heading = h;
                device_state_event(heading, position)
            }
            Either::First(Either4::Second(gps)) => {
                // Only trust a position with a fix; drop it otherwise.
                position = gps.has_fix.then_some(GpsPosition {
                    latitude: gps.latitude as f64,
                    longitude: gps.longitude as f64,
                    altitude: gps.altitude,
                    satellites: gps.satellites,
                    has_fix: true,
                });
                device_state_event(heading, position)
            }
            Either::First(Either4::Third(_)) => {
                // Single tap toggles the box, except mid-knock-puzzle.
                if machine.current_puzzle() == PuzzleId::Knock {
                    continue;
                }
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
            Either::First(Either4::Fourth(_)) => {
                // Raw knock spike — only the knock puzzle cares.
                if machine.current_puzzle() != PuzzleId::Knock {
                    continue;
                }
                let now = Instant::now();
                let gap = match last_knock {
                    Some(prev) => (now - prev).as_millis().min(START_GAP as u64 - 1) as u16,
                    None => START_GAP,
                };
                last_knock = Some(now);
                knock_deadline = Some(now + KNOCK_FINALIZE);
                PuzzleEvent::Tap { gap_ms: gap }
            }
            Either::Second(_) => {
                // Quiet period elapsed: end the knock attempt.
                knock_deadline = None;
                last_knock = None;
                PuzzleEvent::Timeout
            }
        };

        let action = match machine.current_puzzle() {
            PuzzleId::Directions => machine.handle_event(&mut directions, event),
            PuzzleId::Waypoints => machine.handle_event(&mut waypoints, event),
            PuzzleId::Knock => machine.handle_event(&mut knock, event),
            PuzzleId::Unlocked => None,
        };

        if let Some(action) = action {
            apply_action(action);
        }
    }
}

/// Build a device-state event from the latest heading and position.
fn device_state_event(heading: u16, position: Option<GpsPosition>) -> PuzzleEvent {
    PuzzleEvent::DeviceStateChanged(DeviceState {
        heading,
        position,
        ..Default::default()
    })
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
