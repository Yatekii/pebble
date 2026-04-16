//! Compass alignment puzzle.
//!
//! The user must rotate the device to align the compass with a target heading.
//! When the heading is within tolerance for a sustained duration, the puzzle
//! is solved.

use crate::filter::device_state::DeviceState;
use crate::puzzle::events::{Action, LedPattern, PuzzleEvent};
use crate::puzzle::state_machine::PuzzleState;

use super::Puzzle;

/// Tolerance in degrees for heading alignment.
const HEADING_TOLERANCE: u16 = 10;

/// Number of consecutive aligned readings required.
const REQUIRED_ALIGNED_COUNT: u8 = 50; // ~1 second at 50Hz

/// Compass alignment puzzle.
pub struct CompassAlignPuzzle {
    /// Target heading in degrees (0-359).
    target_heading: u16,
    /// Count of consecutive aligned readings.
    aligned_count: u8,
}

impl CompassAlignPuzzle {
    /// Create a new compass alignment puzzle with the given target heading.
    pub fn new(target_heading: u16) -> Self {
        Self {
            target_heading: target_heading % 360,
            aligned_count: 0,
        }
    }

    /// Check if the current heading is within tolerance of the target.
    fn is_aligned(&self, current_heading: u16) -> bool {
        let diff = heading_difference(current_heading, self.target_heading);
        diff <= HEADING_TOLERANCE
    }
}

impl Puzzle for CompassAlignPuzzle {
    fn handle_event(&mut self, event: PuzzleEvent, state: &mut PuzzleState) -> Option<Action> {
        match event {
            PuzzleEvent::DeviceStateChanged(device_state) => {
                let DeviceState { heading, .. } = device_state;

                if self.is_aligned(heading) {
                    self.aligned_count = self.aligned_count.saturating_add(1);

                    // Update progress percentage
                    state.progress =
                        ((self.aligned_count as u16 * 100) / REQUIRED_ALIGNED_COUNT as u16) as u8;

                    if self.aligned_count >= REQUIRED_ALIGNED_COUNT {
                        // Puzzle solved!
                        return Some(Action::Complete);
                    }

                    // Show progress
                    Some(Action::SetLeds(LedPattern::Progress {
                        percent: state.progress,
                        r: 0,
                        g: 255,
                        b: 0,
                    }))
                } else {
                    // Reset alignment counter
                    self.aligned_count = 0;
                    state.progress = 0;

                    // Show compass pointing to target
                    Some(Action::SetLeds(LedPattern::Compass {
                        target_heading: self.target_heading,
                    }))
                }
            }
            PuzzleEvent::Tap => None,
            PuzzleEvent::Timeout => None,
            PuzzleEvent::ExternalCommand(_) => None,
        }
    }

    fn initial_pattern(&self) -> LedPattern {
        LedPattern::Compass {
            target_heading: self.target_heading,
        }
    }

    fn description(&self) -> &'static str {
        "Align the compass to the target heading"
    }
}

/// Calculate the smallest difference between two headings.
fn heading_difference(a: u16, b: u16) -> u16 {
    let diff = if a > b { a - b } else { b - a };
    if diff > 180 { 360 - diff } else { diff }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_heading_difference() {
        assert_eq!(heading_difference(0, 10), 10);
        assert_eq!(heading_difference(10, 0), 10);
        assert_eq!(heading_difference(350, 10), 20);
        assert_eq!(heading_difference(10, 350), 20);
        assert_eq!(heading_difference(180, 0), 180);
        assert_eq!(heading_difference(0, 180), 180);
    }

    #[test]
    fn test_is_aligned() {
        let puzzle = CompassAlignPuzzle::new(90);
        assert!(puzzle.is_aligned(90));
        assert!(puzzle.is_aligned(85));
        assert!(puzzle.is_aligned(95));
        assert!(puzzle.is_aligned(100));
        assert!(!puzzle.is_aligned(101));
        assert!(!puzzle.is_aligned(79));
    }

    #[test]
    fn test_alignment_wrap_around() {
        let puzzle = CompassAlignPuzzle::new(5);
        assert!(puzzle.is_aligned(0));
        assert!(puzzle.is_aligned(355));
        assert!(puzzle.is_aligned(15));
        assert!(!puzzle.is_aligned(20));
        assert!(!puzzle.is_aligned(340));
    }
}
