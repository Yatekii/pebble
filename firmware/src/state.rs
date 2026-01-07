//! Global state and inter-task communication.
//!
//! This module re-exports types and channels from [`crate::filter::watches`]
//! for backwards compatibility. New code should import directly from
//! `filter::watches`.

pub use crate::filter::watches::*;
