//! Communications layer: protocol-agnostic messaging.
//!
//! This module provides abstractions for external communication.
//! Currently supports BLE, with planned support for MQTT over WiFi.
//!
//! # Architecture
//!
//! The communications layer uses a message-based abstraction that allows
//! different protocols to be used interchangeably:
//!
//! - [`messages`]: Common message types for all protocols
//! - [`ble`]: BLE GATT server implementation
//!
//! # Adding New Protocols
//!
//! To add a new protocol (e.g., MQTT):
//!
//! 1. Create a new module (e.g., `mqtt.rs`)
//! 2. Implement message serialization/deserialization
//! 3. Create a task that bridges the protocol to the watch channels

pub mod ble;
pub mod messages;
