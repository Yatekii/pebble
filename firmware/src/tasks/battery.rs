//! Battery task: periodically samples VBAT and broadcasts the voltage.

use defmt::info;
use embassy_time::{Duration, Timer};

use crate::hal::battery::Battery;
use crate::state::BATTERY_VOLTAGE;

/// How often to sample the battery voltage. Sampling faster than we report lets
/// the moving average smooth out low-frequency supply ripple (LEDs, boost
/// converter, servo) that per-read oversampling can't catch.
const SAMPLE_INTERVAL: Duration = Duration::from_millis(250);

/// EMA smoothing shift: `avg += (sample - avg) >> SHIFT`. A shift of 3 gives a
/// weight of 1/8 per sample (~2s settling at the 250ms sample rate).
const EMA_SHIFT: u32 = 3;

/// Run the battery monitoring task: sample VBAT periodically, low-pass filter
/// it with an EMA, and publish the smoothed voltage (mV) for BLE broadcast.
pub async fn run(battery: &mut Battery<'_>) -> ! {
    let sender = BATTERY_VOLTAGE.sender();

    // Seed the filter with the first reading so it converges immediately.
    let mut avg = battery.read_mv() as i32;

    loop {
        let sample = battery.read_mv() as i32;
        avg += (sample - avg) >> EMA_SHIFT;
        let mv = avg as u16;
        info!("Battery: {} mV", mv);
        sender.send(mv);
        Timer::after(SAMPLE_INTERVAL).await;
    }
}
