// WS2812B LED driver using RMT peripheral
// The WS2812B protocol uses specific timing:
// - 0 bit: 400ns high, 850ns low
// - 1 bit: 800ns high, 450ns low
// - Reset: >50µs low

use esp_hal::gpio::Level;
use esp_hal::peripherals::{GPIO15, RMT};
use esp_hal::rmt::{Channel, PulseCode, Rmt, Tx, TxChannelConfig, TxChannelCreator};
use esp_hal::time::Rate;
use esp_hal::Blocking;

/// Number of LEDs in the ring
pub const LED_COUNT: usize = 72;

/// RGB color
#[derive(Clone, Copy, Default)]
pub struct Rgb {
    pub r: u8,
    pub g: u8,
    pub b: u8,
}

impl Rgb {
    pub const fn new(r: u8, g: u8, b: u8) -> Self {
        Self { r, g, b }
    }

    pub const BLACK: Rgb = Rgb::new(0, 0, 0);
    pub const RED: Rgb = Rgb::new(255, 0, 0);
    pub const GREEN: Rgb = Rgb::new(0, 255, 0);
    pub const BLUE: Rgb = Rgb::new(0, 0, 255);
    pub const WHITE: Rgb = Rgb::new(255, 255, 255);
}

/// LED strip error
#[derive(Debug)]
pub enum LedError {
    Rmt(esp_hal::rmt::ConfigError),
    Config(esp_hal::rmt::ConfigError),
    Transmit(esp_hal::rmt::Error),
}

/// Initialize the LED strip with the RMT peripheral and GPIO15
pub fn init(rmt: RMT<'static>, pin: GPIO15<'static>) -> Result<LedStrip<'static>, LedError> {
    let rmt = Rmt::new(rmt, Rate::from_mhz(80)).map_err(LedError::Rmt)?;
    LedStrip::new(rmt.channel0, pin)
}

/// WS2812B LED strip driver using RMT
///
/// At 80MHz RMT clock with divider 1:
/// - 1 tick = 12.5ns
/// - T0H (400ns) = 32 ticks
/// - T0L (850ns) = 68 ticks
/// - T1H (800ns) = 64 ticks
/// - T1L (450ns) = 36 ticks
pub struct LedStrip<'ch> {
    channel: Channel<'ch, Blocking, Tx>,
    // Buffer for pulse codes: 24 bits per LED * LED_COUNT + 1 end marker
    buffer: [PulseCode; LED_COUNT * 24 + 1],
}

// Timing constants at 80MHz (12.5ns per tick)
const T0H: u16 = 32; // 400ns
const T0L: u16 = 68; // 850ns
const T1H: u16 = 64; // 800ns
const T1L: u16 = 36; // 450ns

impl<'ch> LedStrip<'ch> {
    /// Create a new LED strip driver from an RMT TX channel creator
    pub fn new<CH>(
        channel: CH,
        pin: impl esp_hal::gpio::interconnect::PeripheralOutput<'ch>,
    ) -> Result<Self, LedError>
    where
        CH: TxChannelCreator<'ch, Blocking>,
    {
        let config = TxChannelConfig::default()
            .with_clk_divider(1)
            .with_idle_output_level(Level::Low)
            .with_idle_output(true)
            .with_carrier_modulation(false);

        let channel = channel
            .configure_tx(&config)
            .map_err(LedError::Config)?
            .with_pin(pin);

        let mut buffer = [PulseCode::default(); LED_COUNT * 24 + 1];
        buffer[LED_COUNT * 24] = PulseCode::end_marker();

        Ok(Self { channel, buffer })
    }

    /// Convert a byte to 8 pulse codes for WS2812B protocol
    fn byte_to_pulses(byte: u8, pulses: &mut [PulseCode]) {
        for i in 0..8 {
            let bit = (byte >> (7 - i)) & 1;
            pulses[i] = if bit == 1 {
                PulseCode::new(Level::High, T1H, Level::Low, T1L)
            } else {
                PulseCode::new(Level::High, T0H, Level::Low, T0L)
            };
        }
    }

    /// Write colors to the LED strip
    ///
    /// WS2812B expects data in GRB order
    pub fn write(&mut self, colors: &[Rgb; LED_COUNT]) -> Result<(), LedError> {
        // Convert colors to pulse codes
        for (i, color) in colors.iter().enumerate() {
            let base = i * 24;
            // WS2812B uses GRB order
            Self::byte_to_pulses(color.g, &mut self.buffer[base..base + 8]);
            Self::byte_to_pulses(color.r, &mut self.buffer[base + 8..base + 16]);
            Self::byte_to_pulses(color.b, &mut self.buffer[base + 16..base + 24]);
        }

        // Transmit and wait
        let transaction = self
            .channel
            .reborrow()
            .transmit(&self.buffer)
            .map_err(|(e, _)| LedError::Transmit(e))?;
        transaction.wait().map_err(|(e, _)| LedError::Transmit(e))?;

        Ok(())
    }

    /// Set all LEDs to the same color
    pub fn fill(&mut self, color: Rgb) -> Result<(), LedError> {
        let colors = [color; LED_COUNT];
        self.write(&colors)
    }

    /// Turn off all LEDs
    pub fn clear(&mut self) -> Result<(), LedError> {
        self.fill(Rgb::BLACK)
    }
}
