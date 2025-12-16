// Waypoint navigation puzzle using GPS
// The user must bring the box to specific GPS coordinates

use crate::hal::gps::Position;
use libm::{asin, cos, sin, sqrt};

/// A GPS waypoint with target coordinates and acceptance radius
#[derive(Clone, Copy, Debug)]
pub struct Waypoint {
    /// Target latitude in degrees
    pub latitude: f64,
    /// Target longitude in degrees
    pub longitude: f64,
    /// Acceptance radius in meters
    pub radius_m: f32,
}

impl Waypoint {
    pub const fn new(latitude: f64, longitude: f64, radius_m: f32) -> Self {
        Self {
            latitude,
            longitude,
            radius_m,
        }
    }

    /// Calculate distance to a position using Haversine formula
    /// Returns distance in meters
    pub fn distance_to(&self, pos: &Position) -> f64 {
        const EARTH_RADIUS_M: f64 = 6_371_000.0;
        const DEG_TO_RAD: f64 = core::f64::consts::PI / 180.0;

        let lat1 = self.latitude * DEG_TO_RAD;
        let lat2 = pos.latitude * DEG_TO_RAD;
        let dlat = (pos.latitude - self.latitude) * DEG_TO_RAD;
        let dlon = (pos.longitude - self.longitude) * DEG_TO_RAD;

        let sin_dlat_2 = sin(dlat / 2.0);
        let sin_dlon_2 = sin(dlon / 2.0);
        let a = sin_dlat_2 * sin_dlat_2 + cos(lat1) * cos(lat2) * sin_dlon_2 * sin_dlon_2;

        let c = 2.0 * asin(sqrt(a));

        EARTH_RADIUS_M * c
    }

    /// Check if position is within the acceptance radius
    pub fn is_reached(&self, pos: &Position) -> bool {
        if !pos.valid {
            return false;
        }
        self.distance_to(pos) <= self.radius_m as f64
    }
}

/// Waypoint navigation puzzle
pub struct WaypointPuzzle {
    /// Target waypoint
    waypoint: Waypoint,
    /// Whether the waypoint has been reached
    reached: bool,
    /// Hold counter for confirmation (must stay in radius for some time)
    hold_counter: u8,
}

/// Required hold time in update cycles to confirm arrival
const REQUIRED_HOLD_CYCLES: u8 = 30; // 3 seconds at 100ms update

impl WaypointPuzzle {
    /// Create a new waypoint puzzle
    pub fn new(waypoint: Waypoint) -> Self {
        Self {
            waypoint,
            reached: false,
            hold_counter: 0,
        }
    }

    /// Update the puzzle with new GPS position
    /// Returns true if the waypoint is now confirmed reached
    pub fn update(&mut self, pos: &Position) -> bool {
        if self.reached {
            return true;
        }

        if self.waypoint.is_reached(pos) {
            self.hold_counter = self.hold_counter.saturating_add(1);

            if self.hold_counter >= REQUIRED_HOLD_CYCLES {
                self.reached = true;
                return true;
            }
        } else {
            // Reset hold counter if we leave the radius
            self.hold_counter = 0;
        }

        false
    }

    /// Check if the puzzle is solved
    pub fn is_solved(&self) -> bool {
        self.reached
    }

    /// Get distance to waypoint in meters (None if no valid GPS fix)
    pub fn distance(&self, pos: &Position) -> Option<f64> {
        if pos.valid {
            Some(self.waypoint.distance_to(pos))
        } else {
            None
        }
    }

    /// Get the target waypoint
    #[allow(dead_code)]
    pub fn target(&self) -> &Waypoint {
        &self.waypoint
    }

    /// Reset the puzzle
    #[allow(dead_code)]
    pub fn reset(&mut self) {
        self.reached = false;
        self.hold_counter = 0;
    }
}

// Example waypoints for the geocache (replace with actual coordinates)
#[allow(dead_code)]
pub mod waypoints {
    use super::Waypoint;

    /// First waypoint - example location
    pub const WAYPOINT_1: Waypoint = Waypoint::new(47.3769, 8.5417, 50.0); // Zurich example

    /// Second waypoint - example location
    pub const WAYPOINT_2: Waypoint = Waypoint::new(47.3770, 8.5420, 50.0);

    /// Final destination - example location
    pub const FINAL: Waypoint = Waypoint::new(47.3771, 8.5423, 30.0);
}
