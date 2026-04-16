//! GPS task: polls UART for NMEA sentences and broadcasts position data.

use defmt::info;
use embassy_time::{Duration, Timer};

use crate::comms::ble::{GpsBleData, SATS_PER_CHUNK, SatelliteBleData, satellites_to_chunk};
use crate::hal::gps::Gps;
use crate::state::{GPS_DATA, SATELLITES_0, SATELLITES_1};

/// Run the GPS task.
///
/// Polls the GPS UART FIFO at 10Hz and broadcasts position data
/// when available. If GPS is None, the task does nothing.
pub async fn run(gps: &mut Option<Gps<'_>>) -> ! {
    let Some(gps) = gps.as_mut() else {
        info!("GPS task disabled (no GPS hardware)");
        loop {
            Timer::after(Duration::from_secs(60)).await;
        }
    };

    info!("GPS task started");

    // Log every 100 polls = 10 seconds at 100ms poll rate
    let mut poll_count: u32 = 0;

    loop {
        if let Some(gps_data) = gps.poll() {
            poll_count += 1;
            if poll_count % 100 == 0 {
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
                        "GPS: waiting for fix (sats={}, in_view={})",
                        gps_data.position.satellites, gps_data.satellites_in_view
                    );
                }
            }

            // Send GPS position data
            GPS_DATA.sender().send(GpsBleData {
                latitude: gps_data.position.latitude,
                longitude: gps_data.position.longitude,
                altitude: gps_data.position.altitude,
                satellites: gps_data.position.satellites,
                fix_quality: gps_data.position.fix_quality as u8,
                has_fix: gps_data.position.has_fix(),
            });

            // Convert satellite info for BLE transmission
            let sat_count = gps_data.satellites_in_view as usize;
            let mut sats_ble: [SatelliteBleData; 24] = [SatelliteBleData::default(); 24];
            for (i, sat) in gps_data.satellites_info.iter().take(sat_count).enumerate() {
                sats_ble[i] = SatelliteBleData {
                    gnss_type: sat.gnss_type as u8,
                    prn: sat.prn,
                    elevation: sat.elevation,
                    azimuth: sat.azimuth,
                    snr: sat.snr,
                };
            }

            // Send satellite chunks
            let chunk0_count = sat_count.min(SATS_PER_CHUNK);
            let chunk0 = satellites_to_chunk(&sats_ble[..chunk0_count]);
            SATELLITES_0.sender().send(chunk0);

            if sat_count > SATS_PER_CHUNK {
                let chunk1_count = (sat_count - SATS_PER_CHUNK).min(SATS_PER_CHUNK);
                let chunk1 =
                    satellites_to_chunk(&sats_ble[SATS_PER_CHUNK..SATS_PER_CHUNK + chunk1_count]);
                SATELLITES_1.sender().send(chunk1);
            } else {
                // Send empty chunk if no satellites in this range
                SATELLITES_1.sender().send(satellites_to_chunk(&[]));
            }
        }

        // Poll every 100ms - FIFO is 128 bytes, at 9600 baud we get ~960 bytes/sec
        Timer::after(Duration::from_millis(100)).await;
    }
}
