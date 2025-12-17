//! WS2812B LED driver using esp-hal-smartled
//!
//! Drives 72 NeoPixel LEDs arranged in a circle via GPIO15.
//! Automatically broadcasts state changes via LED_STATE for BLE sync.

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::watch::Watch;
use esp_hal::Blocking;
use esp_hal::rmt::Rmt;
use esp_hal::time::Rate;
use esp_hal_smartled::SmartLedsAdapter;
use smart_leds::{RGB8, SmartLedsWrite, brightness, gamma};

/// Current LED state broadcast for BLE synchronization
/// Contains: (brightness, colors as 3 chunks of 72 bytes each)
#[derive(Clone, Copy)]
pub struct LedState {
    pub brightness: u8,
    pub chunk0: [u8; 72],
    pub chunk1: [u8; 72],
    pub chunk2: [u8; 72],
}

impl Default for LedState {
    fn default() -> Self {
        Self {
            brightness: 255,
            chunk0: [0u8; 72],
            chunk1: [0u8; 72],
            chunk2: [0u8; 72],
        }
    }
}

/// Global LED state watch - subscribe to receive updates when LEDs change
pub static LED_STATE: Watch<CriticalSectionRawMutex, LedState, 2> = Watch::new();

/// Number of LEDs in the ring
pub const NUM_LEDS: usize = 72;

/// RGB color
#[derive(Clone, Copy, Default)]
pub struct Color {
    pub r: u8,
    pub g: u8,
    pub b: u8,
}

impl Color {
    pub const fn new(r: u8, g: u8, b: u8) -> Self {
        Self { r, g, b }
    }

    pub const fn black() -> Self {
        Self::new(0, 0, 0)
    }

    pub const fn red() -> Self {
        Self::new(255, 0, 0)
    }

    pub const fn green() -> Self {
        Self::new(0, 255, 0)
    }

    pub const fn blue() -> Self {
        Self::new(0, 0, 255)
    }

    pub const fn white() -> Self {
        Self::new(255, 255, 255)
    }
}

impl From<Color> for RGB8 {
    fn from(c: Color) -> Self {
        RGB8::new(c.r, c.g, c.b)
    }
}

/// Buffer size for the LED strip
pub const LED_BUFFER_SIZE: usize = esp_hal_smartled::buffer_size(NUM_LEDS);

/// LED strip driver
pub struct LedStrip<'a> {
    adapter: SmartLedsAdapter<'a, LED_BUFFER_SIZE>,
    colors: [Color; NUM_LEDS],
    brightness_level: u8,
}

impl<'a> LedStrip<'a> {
    /// Set a single LED color
    pub fn set(&mut self, index: usize, color: Color) {
        if index < NUM_LEDS {
            self.colors[index] = color;
        }
    }

    /// Set all LEDs to the same color
    pub fn set_all(&mut self, color: Color) {
        self.colors.fill(color);
    }

    /// Clear all LEDs (set to black)
    pub fn clear(&mut self) {
        self.set_all(Color::black());
    }

    /// Set brightness level (0-255)
    pub fn set_brightness(&mut self, level: u8) {
        self.brightness_level = level;
    }

    /// Write the current colors to the LED strip and broadcast state for BLE sync
    pub fn show(&mut self) -> Result<(), esp_hal_smartled::LedAdapterError> {
        let iter = self.colors.iter().map(|c| RGB8::from(*c));

        // Disable interrupts during transmission to prevent timing glitches
        let result = critical_section::with(|_| {
            self.adapter
                .write(brightness(gamma(iter), self.brightness_level))
        });

        // Broadcast state for BLE synchronization
        let mut state = LedState {
            brightness: self.brightness_level,
            chunk0: [0u8; 72],
            chunk1: [0u8; 72],
            chunk2: [0u8; 72],
        };

        // Pack colors into chunks (24 LEDs × 3 bytes each = 72 bytes per chunk)
        for i in 0..24 {
            let c = self.colors[i];
            state.chunk0[i * 3] = c.r;
            state.chunk0[i * 3 + 1] = c.g;
            state.chunk0[i * 3 + 2] = c.b;
        }
        for i in 0..24 {
            let c = self.colors[24 + i];
            state.chunk1[i * 3] = c.r;
            state.chunk1[i * 3 + 1] = c.g;
            state.chunk1[i * 3 + 2] = c.b;
        }
        for i in 0..24 {
            let c = self.colors[48 + i];
            state.chunk2[i * 3] = c.r;
            state.chunk2[i * 3 + 1] = c.g;
            state.chunk2[i * 3 + 2] = c.b;
        }

        LED_STATE.sender().send(state);

        result
    }
}

/// Initialize the LED strip
pub fn init<'a>(
    rmt: esp_hal::peripherals::RMT<'a>,
    pin: esp_hal::peripherals::GPIO15<'a>,
    buffer: &'a mut [esp_hal::rmt::PulseCode; LED_BUFFER_SIZE],
) -> Result<LedStrip<'a>, esp_hal::rmt::Error> {
    let rmt = Rmt::<Blocking>::new(rmt, Rate::from_mhz(80))?;
    let adapter = SmartLedsAdapter::new(rmt.channel0, pin, buffer);

    Ok(LedStrip {
        adapter,
        colors: [Color::black(); NUM_LEDS],
        brightness_level: 255,
    })
}
