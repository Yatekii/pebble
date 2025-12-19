//! GPS task: polls UART for NMEA sentences and broadcasts position data.

use defmt::info;
use embassy_time::{Duration, Timer};

use crate::comms::ble::GpsBleData;
use crate::hal::gps::Gps;
use crate::state::GPS_DATA;

/// Run the GPS task.
///
/// Polls the GPS UART FIFO at 10Hz and broadcasts position data
/// when available.
pub async fn run(gps: &mut Gps<'_>) -> ! {
    info!("GPS task started");

    loop {
        if let Some(gps_data) = gps.poll() {
            if gps_data.position.has_fix() {
                info!(
                    "GPS: lat={} lon={} alt={}m sats={} fix={:?}",
                    gps_data.position.latitude,
                    gps_data.position.longitude,
                    gps_data.position.altitude,
                    gps_data.position.satellites,
                    gps_data.position.fix_quality
                );
            } else {
                info!(
                    "GPS: waiting for fix (sats={})",
                    gps_data.position.satellites
                );
            }

            GPS_DATA.sender().send(GpsBleData {
                latitude: gps_data.position.latitude,
                longitude: gps_data.position.longitude,
                altitude: gps_data.position.altitude,
                satellites: gps_data.position.satellites,
                fix_quality: gps_data.position.fix_quality as u8,
                has_fix: gps_data.position.has_fix(),
            });
        }

        // Poll every 100ms - FIFO is 128 bytes, at 9600 baud we get ~960 bytes/sec
        Timer::after(Duration::from_millis(100)).await;
    }
}
