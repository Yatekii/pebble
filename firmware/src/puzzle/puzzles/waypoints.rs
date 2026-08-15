//! Waypoints puzzle: navigate the box to a series of GPS locations.
//!
//! The ring points toward the current waypoint (a single lit LED). Getting
//! within a waypoint's geofence advances to the next one. The last waypoint
//! has a tight fence — reaching it completes the puzzle and opens the box.

use crate::filter::device_state::DeviceState;
use crate::puzzle::events::{Action, LedPattern, PuzzleEvent};
use crate::puzzle::state_machine::PuzzleState;

use super::Puzzle;

/// A GPS waypoint with the geofence radius that counts as "reached".
pub struct Waypoint {
    pub lat: f64,
    pub lon: f64,
    pub radius_m: f32,
}

/// Test route: two placeholder waypoints (Zürich). Wide fence first, tight
/// 10 m fence on the final one. Replace with the real route per box.
pub const TEST_WAYPOINTS: [Waypoint; 2] = [
    Waypoint {
        lat: 47.376888,
        lon: 8.541694,
        radius_m: 100.0,
    },
    Waypoint {
        lat: 47.365000,
        lon: 8.525000,
        radius_m: 10.0,
    },
];

/// Waypoints puzzle state.
pub struct WaypointsPuzzle {
    waypoints: &'static [Waypoint],
    index: usize,
}

impl WaypointsPuzzle {
    /// Create a new waypoints puzzle for the given route.
    pub fn new(waypoints: &'static [Waypoint]) -> Self {
        Self {
            waypoints,
            index: 0,
        }
    }

    /// Number of waypoints reached so far.
    pub fn reached(&self) -> usize {
        self.index
    }
}

impl Puzzle for WaypointsPuzzle {
    fn handle_event(&mut self, event: PuzzleEvent, state: &mut PuzzleState) -> Option<Action> {
        let PuzzleEvent::DeviceStateChanged(DeviceState {
            heading, position, ..
        }) = event
        else {
            return None;
        };

        // No fix yet — ring stays dark until we know where we are.
        let Some(pos) = position else {
            return Some(Action::SetLeds(LedPattern::Off));
        };

        let wp = &self.waypoints[self.index];
        if distance_m(pos.latitude, pos.longitude, wp.lat, wp.lon) <= wp.radius_m as f64 {
            self.index += 1;
            state.progress = self.index as u8;
            if self.index >= self.waypoints.len() {
                return Some(Action::Complete);
            }
            return Some(Action::SetLeds(LedPattern::Success));
        }

        // Point the ring at the waypoint, relative to where the box faces.
        let brg = bearing_deg(pos.latitude, pos.longitude, wp.lat, wp.lon);
        let rel = wrap360(brg - heading as f64) as u16;
        Some(Action::SetLeds(LedPattern::Compass { bearing_deg: rel }))
    }

    fn initial_pattern(&self) -> LedPattern {
        LedPattern::Off
    }

    fn description(&self) -> &'static str {
        "Navigate the box to the waypoints"
    }
}

// --- Geo helpers (equirectangular approximation; plenty for geofence ranges) ---

use core::f64::consts::PI;

/// Mean Earth radius in metres.
const EARTH_RADIUS_M: f64 = 6_371_000.0;

fn to_rad(deg: f64) -> f64 {
    deg * PI / 180.0
}

/// Wrap an angle in degrees to `[0, 360)`.
fn wrap360(mut deg: f64) -> f64 {
    deg %= 360.0;
    if deg < 0.0 {
        deg += 360.0;
    }
    deg
}

/// Great-ish-circle distance between two lat/lon points, in metres.
fn distance_m(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
    let x = to_rad(lon2 - lon1) * libm::cos(to_rad((lat1 + lat2) / 2.0));
    let y = to_rad(lat2 - lat1);
    EARTH_RADIUS_M * libm::sqrt(x * x + y * y)
}

/// Initial bearing from point 1 to point 2, degrees clockwise from north.
fn bearing_deg(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
    let x = to_rad(lon2 - lon1) * libm::cos(to_rad((lat1 + lat2) / 2.0));
    let y = to_rad(lat2 - lat1);
    wrap360(libm::atan2(x, y) * 180.0 / PI)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::filter::device_state::GpsPosition;

    fn at(lat: f64, lon: f64, heading: u16) -> PuzzleEvent {
        PuzzleEvent::DeviceStateChanged(DeviceState {
            heading,
            position: Some(GpsPosition {
                latitude: lat,
                longitude: lon,
                has_fix: true,
                ..Default::default()
            }),
            ..Default::default()
        })
    }

    #[test]
    fn distance_is_roughly_right() {
        // ~1 min of latitude ≈ 1852 m (one nautical mile).
        let d = distance_m(47.0, 8.0, 47.0 + 1.0 / 60.0, 8.0);
        assert!((d - 1852.0).abs() < 20.0, "got {d}");
    }

    #[test]
    fn bearing_cardinals() {
        // Due north, east, south, west from the same point.
        assert!((bearing_deg(47.0, 8.0, 48.0, 8.0) - 0.0).abs() < 1.0);
        assert!((bearing_deg(47.0, 8.0, 47.0, 9.0) - 90.0).abs() < 1.0);
        assert!((bearing_deg(47.0, 8.0, 46.0, 8.0) - 180.0).abs() < 1.0);
        assert!((bearing_deg(47.0, 8.0, 47.0, 7.0) - 270.0).abs() < 1.0);
    }

    #[test]
    fn no_fix_keeps_ring_off() {
        let mut p = WaypointsPuzzle::new(&TEST_WAYPOINTS);
        let mut st = PuzzleState::default();
        let ev = PuzzleEvent::DeviceStateChanged(DeviceState::default()); // position: None
        assert!(matches!(
            p.handle_event(ev, &mut st),
            Some(Action::SetLeds(LedPattern::Off))
        ));
    }

    #[test]
    fn advances_through_waypoints_then_completes() {
        let mut p = WaypointsPuzzle::new(&TEST_WAYPOINTS);
        let mut st = PuzzleState::default();

        // Far away → steer (Compass), not reached.
        assert!(matches!(
            p.handle_event(at(47.0, 8.0, 0), &mut st),
            Some(Action::SetLeds(LedPattern::Compass { .. }))
        ));
        assert_eq!(p.reached(), 0);

        // Sitting on waypoint 0 → reached, advance.
        let w0 = &TEST_WAYPOINTS[0];
        assert!(matches!(
            p.handle_event(at(w0.lat, w0.lon, 0), &mut st),
            Some(Action::SetLeds(LedPattern::Success))
        ));
        assert_eq!(p.reached(), 1);

        // Sitting on the final waypoint → complete.
        let w1 = &TEST_WAYPOINTS[1];
        assert!(matches!(
            p.handle_event(at(w1.lat, w1.lon, 0), &mut st),
            Some(Action::Complete)
        ));
        assert_eq!(p.reached(), 2);
    }
}
