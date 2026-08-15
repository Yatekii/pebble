//! Directions puzzle: point the box through a secret compass sequence.
//!
//! An external clue (e.g. a crossword) gives a sequence of cardinal directions,
//! encoded as the letters N / E / S / W. The player enters it by physically
//! pointing the box:
//!
//! - Point within [`TOLERANCE_DEG`] of a cardinal and the ring fills blue over
//!   ~5 s ([`FILL_TICKS`]).
//! - Full fill on the *correct* next direction → green flash, advance the
//!   sequence. Full fill on the *wrong* direction → red flash, reset to the
//!   start.
//! - Drifting off the cardinal (or switching to another) before the ring fills
//!   resets the fill.

use crate::filter::device_state::DeviceState;
use crate::puzzle::events::{Action, LedPattern, PuzzleEvent};
use crate::puzzle::state_machine::PuzzleState;

use super::Puzzle;

/// How close (in degrees) the heading must be to a cardinal to count.
const TOLERANCE_DEG: u16 = 5;

/// Ticks of sustained pointing to fill the ring. Heading arrives at ~100 Hz
/// (see `tasks::imu`), so 500 ticks ≈ 5 s.
const FILL_TICKS: u16 = 500;

/// The four cardinal directions the puzzle accepts.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Cardinal {
    North,
    East,
    South,
    West,
}

impl Cardinal {
    /// Compass heading of this cardinal in degrees.
    const fn heading(self) -> u16 {
        match self {
            Cardinal::North => 0,
            Cardinal::East => 90,
            Cardinal::South => 180,
            Cardinal::West => 270,
        }
    }
}

/// Test sequence: N W S E S W. Swap for the real per-box secret later.
pub const TEST_SEQUENCE: [Cardinal; 6] = [
    Cardinal::North,
    Cardinal::West,
    Cardinal::South,
    Cardinal::East,
    Cardinal::South,
    Cardinal::West,
];

/// Directions puzzle state.
pub struct DirectionsPuzzle {
    /// The secret sequence to enter.
    sequence: &'static [Cardinal],
    /// How many leading directions have been entered correctly.
    index: usize,
    /// Cardinal currently being filled, if the box points at one.
    filling: Option<Cardinal>,
    /// Fill progress toward [`FILL_TICKS`] for the current cardinal.
    fill: u16,
    /// True once the current hold has been evaluated; blocks re-filling until
    /// the box leaves the cardinal, so one hold can't count twice.
    committed: bool,
}

impl DirectionsPuzzle {
    /// Create a new directions puzzle for the given secret sequence.
    pub fn new(sequence: &'static [Cardinal]) -> Self {
        Self {
            sequence,
            index: 0,
            filling: None,
            fill: 0,
            committed: false,
        }
    }

    /// Number of directions entered correctly so far.
    pub fn solved_steps(&self) -> usize {
        self.index
    }

    /// Abandon any in-progress fill (heading left all cardinals).
    fn reset_fill(&mut self) {
        self.filling = None;
        self.fill = 0;
        self.committed = false;
    }
}

impl Puzzle for DirectionsPuzzle {
    fn handle_event(&mut self, event: PuzzleEvent, state: &mut PuzzleState) -> Option<Action> {
        let PuzzleEvent::DeviceStateChanged(DeviceState { heading, .. }) = event else {
            return None;
        };

        let Some(here) = cardinal_at(heading) else {
            self.reset_fill();
            state.progress = self.index as u8;
            return Some(Action::SetLeds(LedPattern::Off));
        };

        // Entering a different cardinal restarts the fill.
        if self.filling != Some(here) {
            self.filling = Some(here);
            self.fill = 0;
            self.committed = false;
        }

        // Already scored this hold — wait until the box leaves before refilling.
        if self.committed {
            return Some(Action::SetLeds(LedPattern::Off));
        }

        self.fill = self.fill.saturating_add(1);
        if self.fill < FILL_TICKS {
            let percent = (self.fill as u32 * 100 / FILL_TICKS as u32) as u8;
            return Some(Action::SetLeds(LedPattern::Progress {
                percent,
                r: 0,
                g: 0,
                b: 255,
            }));
        }

        // Ring full: score this direction.
        self.committed = true;
        if here == self.sequence[self.index] {
            self.index += 1;
            state.progress = self.index as u8;
            if self.index >= self.sequence.len() {
                return Some(Action::Complete);
            }
            Some(Action::SetLeds(LedPattern::Success))
        } else {
            self.index = 0;
            state.progress = 0;
            Some(Action::SetLeds(LedPattern::Error))
        }
    }

    fn initial_pattern(&self) -> LedPattern {
        LedPattern::Off
    }

    fn description(&self) -> &'static str {
        "Point the box through the secret compass sequence"
    }
}

/// The cardinal the heading points at, if within [`TOLERANCE_DEG`]. Cardinals
/// are 90° apart, so at a 5° tolerance at most one ever matches.
fn cardinal_at(heading: u16) -> Option<Cardinal> {
    const ALL: [Cardinal; 4] = [
        Cardinal::North,
        Cardinal::East,
        Cardinal::South,
        Cardinal::West,
    ];
    ALL.into_iter()
        .find(|c| heading_difference(heading % 360, c.heading()) <= TOLERANCE_DEG)
}

/// Smallest absolute difference between two headings, handling wrap-around.
fn heading_difference(a: u16, b: u16) -> u16 {
    let diff = a.abs_diff(b);
    if diff > 180 { 360 - diff } else { diff }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Feed `ticks` heading updates at `heading` and return the last action.
    fn hold(
        p: &mut DirectionsPuzzle,
        state: &mut PuzzleState,
        heading: u16,
        ticks: u16,
    ) -> Option<Action> {
        let mut last = None;
        for _ in 0..ticks {
            last = p.handle_event(
                PuzzleEvent::DeviceStateChanged(DeviceState {
                    heading,
                    ..Default::default()
                }),
                state,
            );
        }
        last
    }

    fn full(p: &mut DirectionsPuzzle, state: &mut PuzzleState, heading: u16) -> Option<Action> {
        hold(p, state, heading, FILL_TICKS)
    }

    #[test]
    fn cardinal_detection() {
        assert_eq!(cardinal_at(0), Some(Cardinal::North));
        assert_eq!(cardinal_at(3), Some(Cardinal::North));
        assert_eq!(cardinal_at(357), Some(Cardinal::North));
        assert_eq!(cardinal_at(90), Some(Cardinal::East));
        assert_eq!(cardinal_at(180), Some(Cardinal::South));
        assert_eq!(cardinal_at(270), Some(Cardinal::West));
        assert_eq!(cardinal_at(45), None);
        assert_eq!(cardinal_at(6), None);
    }

    #[test]
    fn correct_sequence_completes() {
        let mut p = DirectionsPuzzle::new(&TEST_SEQUENCE);
        let mut st = PuzzleState::default();
        for (i, c) in TEST_SEQUENCE.iter().enumerate() {
            let action = full(&mut p, &mut st, c.heading());
            if i + 1 == TEST_SEQUENCE.len() {
                assert!(matches!(action, Some(Action::Complete)));
            } else {
                assert!(matches!(action, Some(Action::SetLeds(LedPattern::Success))));
            }
        }
        assert_eq!(p.solved_steps(), TEST_SEQUENCE.len());
    }

    #[test]
    fn wrong_direction_resets() {
        let mut p = DirectionsPuzzle::new(&TEST_SEQUENCE);
        let mut st = PuzzleState::default();

        // First step correct (N).
        assert!(matches!(
            full(&mut p, &mut st, Cardinal::North.heading()),
            Some(Action::SetLeds(LedPattern::Success))
        ));
        assert_eq!(p.solved_steps(), 1);

        // Second step should be W; give E instead.
        assert!(matches!(
            full(&mut p, &mut st, Cardinal::East.heading()),
            Some(Action::SetLeds(LedPattern::Error))
        ));
        assert_eq!(p.solved_steps(), 0);

        // Sequence restarts: N is accepted again.
        assert!(matches!(
            full(&mut p, &mut st, Cardinal::North.heading()),
            Some(Action::SetLeds(LedPattern::Success))
        ));
    }

    #[test]
    fn leaving_before_full_resets_fill() {
        let mut p = DirectionsPuzzle::new(&TEST_SEQUENCE);
        let mut st = PuzzleState::default();

        // Partial fill on N, then wander off-cardinal.
        let a = hold(&mut p, &mut st, Cardinal::North.heading(), FILL_TICKS - 1);
        assert!(matches!(a, Some(Action::SetLeds(LedPattern::Progress { .. }))));
        assert!(matches!(
            hold(&mut p, &mut st, 45, 1),
            Some(Action::SetLeds(LedPattern::Off))
        ));

        // One more tick on N is nowhere near full — no completion yet.
        assert!(matches!(
            hold(&mut p, &mut st, Cardinal::North.heading(), 1),
            Some(Action::SetLeds(LedPattern::Progress { .. }))
        ));
        assert_eq!(p.solved_steps(), 0);
    }

    #[test]
    fn holding_past_full_counts_once() {
        let mut p = DirectionsPuzzle::new(&TEST_SEQUENCE);
        let mut st = PuzzleState::default();

        assert!(matches!(
            full(&mut p, &mut st, Cardinal::North.heading()),
            Some(Action::SetLeds(LedPattern::Success))
        ));
        // Keep holding N: committed, so it stays off and does not advance again.
        assert!(matches!(
            hold(&mut p, &mut st, Cardinal::North.heading(), FILL_TICKS),
            Some(Action::SetLeds(LedPattern::Off))
        ));
        assert_eq!(p.solved_steps(), 1);
    }
}
