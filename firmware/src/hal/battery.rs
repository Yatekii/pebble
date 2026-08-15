//! Battery voltage monitoring via ADC.
//!
//! VBAT is scaled down to the ADC input on GPIO6 (ADC1) by a 10M/1M resistor
//! divider (10M to VBAT, 1M to GND), so the pin sees `VBAT / 11`. The ADC read
//! is calibrated to millivolts and scaled back up to the real battery voltage.

use esp_hal::Blocking;
use esp_hal::analog::adc::{Adc, AdcCalCurve, AdcConfig, AdcPin, Attenuation};
use esp_hal::peripherals::{ADC1, GPIO6};

/// Resistor divider ratio: `VBAT = pin_voltage * (10M + 1M) / 1M = pin * 11`.
/// If the divider is ever populated the other way round, change this to match.
const DIVIDER_RATIO: u32 = 11;

/// Number of ADC samples averaged per reading to suppress noise. Averaging N
/// samples cuts random noise by ~sqrt(N); 64 turns ~±25mV jitter into ~±3mV.
const OVERSAMPLE: u32 = 64;

type BatteryPin<'a> = AdcPin<GPIO6<'a>, ADC1<'a>, AdcCalCurve<ADC1<'a>>>;

/// Battery voltage reader over ADC1.
pub struct Battery<'a> {
    adc: Adc<'a, ADC1<'a>, Blocking>,
    pin: BatteryPin<'a>,
}

impl<'a> Battery<'a> {
    /// Initialize the ADC on GPIO6 for battery voltage measurement.
    pub fn new(adc1: ADC1<'a>, pin: GPIO6<'a>) -> Self {
        let mut config = AdcConfig::new();
        // 11dB attenuation spans the full 0-3.3V ADC range; the divided VBAT
        // (~0.27-0.38V for a 1S Li-ion) sits comfortably inside it.
        let pin = config
            .enable_pin_with_cal::<GPIO6<'a>, AdcCalCurve<ADC1<'a>>>(pin, Attenuation::_11dB);
        let adc = Adc::new(adc1, config);
        Self { adc, pin }
    }

    /// Read the battery voltage in millivolts, averaged over [`OVERSAMPLE`]
    /// samples to suppress ADC noise.
    pub fn read_mv(&mut self) -> u16 {
        let mut sum: u32 = 0;
        for _ in 0..OVERSAMPLE {
            sum += self.read_pin_mv() as u32;
        }
        let pin_mv = sum / OVERSAMPLE;
        (pin_mv * DIVIDER_RATIO) as u16
    }

    /// Take a single calibrated ADC reading (millivolts at the pin).
    fn read_pin_mv(&mut self) -> u16 {
        // The calibrated oneshot read returns millivolts at the pin. The
        // conversion finishes within microseconds, so spin until it's ready.
        loop {
            if let Ok(mv) = self.adc.read_oneshot(&mut self.pin) {
                break mv;
            }
        }
    }
}
