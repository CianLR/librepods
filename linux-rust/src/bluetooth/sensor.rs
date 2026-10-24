// Sensor data streams (head tracking, heart rate) use AACP opcode 0x17. The payload after the
// opcode is: [descriptor, u32 LE] [length, u16 LE] [protobuf SensorDataWX]
// The messages are defined in proto/rtbuddy.proto, taken from the android/rewrite branch.

use prost::Message;
use std::time::Duration;

pub mod rtbuddy {
    include!(concat!(env!("OUT_DIR"), "/rtbuddy.rs"));
}

use rtbuddy::{SensorDataWx, SensorServiceSetting, SensorServiceType};

pub const SENSOR_DATA_WX_DESCRIPTOR: u32 = 0x0010_0000;
const REPORT_INTERVAL_SETTING: i32 = 2;

/// Builds the payload (after the opcode) asking the AirPods to report `service` every `interval`.
/// An interval of zero stops the stream.
pub fn report_interval_payload(
    sequence: i32,
    service: SensorServiceType,
    interval: Duration,
) -> Vec<u8> {
    let micros = u32::try_from(interval.as_micros()).unwrap_or(u32::MAX);
    let mut configuration = vec![0x01];
    configuration.extend_from_slice(&micros.to_le_bytes());

    let data = SensorDataWx {
        seq: sequence,
        service_settings: Some(SensorServiceSetting {
            service: service as i32,
            setting: REPORT_INTERVAL_SETTING,
            configuration,
        }),
        ..Default::default()
    }
    .encode_to_vec();

    let mut payload = Vec::with_capacity(6 + data.len());
    payload.extend_from_slice(&SENSOR_DATA_WX_DESCRIPTOR.to_le_bytes());
    payload.extend_from_slice(&(data.len() as u16).to_le_bytes());
    payload.extend_from_slice(&data);
    payload
}

/// Decodes the SensorDataWX message from a sensor data payload (after the opcode).
pub fn parse_sensor_data(payload: &[u8]) -> Option<SensorDataWx> {
    let descriptor = u32::from_le_bytes(payload.get(0..4)?.try_into().ok()?);
    let length = u16::from_le_bytes(payload.get(4..6)?.try_into().ok()?) as usize;
    if descriptor != SENSOR_DATA_WX_DESCRIPTOR {
        return None;
    }
    SensorDataWx::decode(payload.get(6..6 + length)?).ok()
}

/// Heart rate samples arrive in the `command` field with an 18-byte payload. Byte 1 is the BPM
/// and samples with a non-zero byte 17 are dropped (meaning unknown, probably no reliable reading
/// yet). The other bytes are not understood. Based on android/rewrite; untested on Linux.
pub fn heart_rate(data: &SensorDataWx) -> Option<u8> {
    let command = data.command.as_ref()?;
    let is_heart_rate = matches!(
        SensorServiceType::try_from(command.service),
        Ok(SensorServiceType::Heartrate | SensorServiceType::HeartratEv2)
    );
    if !is_heart_rate || command.payload.len() != 18 || command.payload[17] != 0 {
        return None;
    }
    Some(command.payload[1])
}

/// Like android/rewrite, firmware with a version3 starting with 9 or higher starts heart rate
/// monitoring through a different service.
pub fn heart_rate_service(version3: &str) -> SensorServiceType {
    match version3.chars().next().and_then(|c| c.to_digit(10)) {
        Some(major) if major >= 9 => SensorServiceType::HeartrateCommand,
        _ => SensorServiceType::Heartrate,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rtbuddy::SensorCommand;

    fn sample_payload(service: SensorServiceType, sample: Vec<u8>) -> Vec<u8> {
        let data = SensorDataWx {
            seq: 5,
            command: Some(SensorCommand {
                service: service as i32,
                payload: sample,
            }),
            ..Default::default()
        }
        .encode_to_vec();
        let mut payload = SENSOR_DATA_WX_DESCRIPTOR.to_le_bytes().to_vec();
        payload.extend_from_slice(&(data.len() as u16).to_le_bytes());
        payload.extend_from_slice(&data);
        payload
    }

    #[test]
    fn matches_captured_head_tracking_start_packet() {
        // From docs/AAP Definitions.md (after the 04 00 04 00 17 00 header): seq 289, ACTIVITY, 40 ms
        let captured = hex::decode("00001000100008a102420b080e10021a0501409c0000").unwrap();
        let payload = report_interval_payload(289, SensorServiceType::Activity, Duration::from_millis(40));
        assert_eq!(payload, captured);
    }

    #[test]
    fn decodes_heart_rate_sample() {
        let mut sample = vec![0; 18];
        sample[1] = 200;
        let data = parse_sensor_data(&sample_payload(SensorServiceType::HeartratEv2, sample)).unwrap();
        assert_eq!(heart_rate(&data), Some(200));
    }

    #[test]
    fn ignores_unusable_samples() {
        let mut unreliable = vec![0; 18];
        unreliable[17] = 1;
        for payload in [
            sample_payload(SensorServiceType::Heartrate, unreliable),
            sample_payload(SensorServiceType::Heartrate, vec![0; 10]),
            sample_payload(SensorServiceType::Activity, vec![0; 18]),
        ] {
            assert_eq!(heart_rate(&parse_sensor_data(&payload).unwrap()), None);
        }
    }

    #[test]
    fn rejects_malformed_payloads() {
        let mut truncated = sample_payload(SensorServiceType::Heartrate, vec![0; 18]);
        truncated.truncate(truncated.len() - 3);
        assert!(parse_sensor_data(&truncated).is_none());

        let mut other_descriptor = sample_payload(SensorServiceType::Heartrate, vec![0; 18]);
        other_descriptor[2] = 0x08;
        assert!(parse_sensor_data(&other_descriptor).is_none());
    }

    #[test]
    fn picks_service_from_version3() {
        assert_eq!(heart_rate_service("9442752"), SensorServiceType::HeartrateCommand);
        assert_eq!(heart_rate_service("6357536"), SensorServiceType::Heartrate);
        assert_eq!(heart_rate_service(""), SensorServiceType::Heartrate);
    }
}
