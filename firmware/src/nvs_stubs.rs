//! NVS stub implementations for esp-radio on RISC-V chips
//!
//! The esp-radio crate only provides these stubs for Xtensa chips (`#[cfg(xtensa)]`),
//! so we need to provide them for RISC-V (ESP32-C6).
//!
//! See: https://github.com/esp-rs/esp-hal/blob/main/esp-radio/src/common_adapter.rs

#[unsafe(no_mangle)]
pub unsafe extern "C" fn __esp_radio_misc_nvs_deinit() {
    // No-op stub
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn __esp_radio_misc_nvs_init() -> i32 {
    0 // Success
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn __esp_radio_misc_nvs_restore() -> i32 {
    0 // Success
}
