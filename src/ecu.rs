//! High-fidelity ECU simulator core with multi-profile support.

use std::fmt;
use std::str::FromStr;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EcuProfile {
    BoschMe75,
    VagKwp1281,
    GenericObd,
    MarelliIaw,
}

impl fmt::Display for EcuProfile {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::BoschMe75 => write!(f, "bosch-me75"),
            Self::VagKwp1281 => write!(f, "vag-kwp1281"),
            Self::GenericObd => write!(f, "generic-obd"),
            Self::MarelliIaw => write!(f, "marelli-iaw"),
        }
    }
}

impl FromStr for EcuProfile {
    type Err = String;
    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s.to_lowercase().replace('_', "-").as_str() {
            "bosch-me75" | "me7.5" | "me75" => Ok(Self::BoschMe75),
            "vag-kwp1281" | "kwp1281" => Ok(Self::VagKwp1281),
            "generic-obd" | "obd2" | "obd" => Ok(Self::GenericObd),
            "marelli-iaw" | "marelli" | "iaw" => Ok(Self::MarelliIaw),
            other => Err(format!("Unknown profile '{}'. Valid: bosch-me75, vag-kwp1281, generic-obd, marelli-iaw", other)),
        }
    }
}

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum EcuState {
    Idle,
    SyncSent,
    SessionActive,
}

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum DiagnosticProtocol {
    Iso14230,
    Iso9141,
    VagKwp1281,
}

#[derive(Debug, Clone)]
pub struct DiagnosticData {
    pub rpm: u16,
    pub coolant_temp_c: i16,
    pub speed_kmh: u8,
    pub battery_mv: u16,
    pub dtcs: Vec<[u8; 2]>, // e.g. [[0x03, 0x00], [0x01, 0x71]] for P0300, P0171
    pub vin: String,
}

impl Default for DiagnosticData {
    fn default() -> Self {
        Self {
            rpm: 850,
            coolant_temp_c: 90,
            speed_kmh: 0,
            battery_mv: 13800, // 13.8V
            dtcs: vec![[0x03, 0x00], [0x01, 0x71]], // P0300, P0171
            vin: "9BWCA05X12P123456".to_string(),
        }
    }
}

pub struct EcuSimulator {
    pub profile: EcuProfile,
    pub state: EcuState,
    pub protocol: DiagnosticProtocol,
    pub data: DiagnosticData,
    pub address: u8,
    pub kb1: u8,
    pub kb2: u8,
    ecu_addr: u8,
    tester_addr: u8,
}

impl Default for EcuSimulator {
    fn default() -> Self {
        Self::new(EcuProfile::BoschMe75)
    }
}

impl EcuSimulator {
    pub fn new(profile: EcuProfile) -> Self {
        let (address, kb1, kb2, ecu_addr, tester_addr) = match profile {
            EcuProfile::BoschMe75 => (0x01, 0x08, 0x08, 0x11, 0xF1), // Engine 0x01, KB 0x08 0x08, ECU 0x11
            EcuProfile::VagKwp1281 => (0x01, 0x01, 0x8A, 0x01, 0xF1), // VAG 0x01, KB 0x01 0x8A
            EcuProfile::GenericObd => (0x33, 0x08, 0x08, 0x11, 0xF1), // OBD-II 0x33, KB 0x08 0x08, ECU 0x11
            EcuProfile::MarelliIaw => (0x01, 0x8F, 0x6D, 0x10, 0xF1), // Fiat/Marelli 0x01, KB 0x8F 0x6D, ECU 0x10
        };

        Self {
            profile,
            state: EcuState::Idle,
            protocol: DiagnosticProtocol::Iso14230,
            data: DiagnosticData::default(),
            address,
            kb1,
            kb2,
            ecu_addr,
            tester_addr,
        }
    }

    /// Process a single byte (used during 5-baud handshake or byte-by-byte inspection).
    pub fn process_byte(&mut self, byte: u8) -> Vec<u8> {
        match self.state {
            EcuState::Idle => {
                if byte == self.address || byte == 0x33 || byte == 0x01 {
                    self.state = EcuState::SyncSent;
                    vec![0x55, self.kb1, self.kb2]
                } else {
                    Vec::new()
                }
            }
            EcuState::SyncSent => {
                let expected_inv_kb2 = !self.kb2;
                if byte == expected_inv_kb2 {
                    self.state = EcuState::SessionActive;
                    if self.profile == EcuProfile::VagKwp1281 {
                        self.protocol = DiagnosticProtocol::VagKwp1281;
                        Vec::new()
                    } else {
                        self.protocol = DiagnosticProtocol::Iso9141;
                        vec![!self.address]
                    }
                } else {
                    self.state = EcuState::Idle;
                    Vec::new()
                }
            }
            EcuState::SessionActive => Vec::new(),
        }
    }

    /// Process a complete ISO frame.
    pub fn process_frame(&mut self, frame: &[u8]) -> Option<Vec<u8>> {
        if frame.is_empty() {
            return None;
        }

        // 1. If in Idle state, check if this is an ISO 14230 Fast Init "StartCommunication" request
        if self.state == EcuState::Idle {
            if let Some(resp) = self.check_fast_init(frame) {
                self.state = EcuState::SessionActive;
                return Some(resp);
            }
            // Check if it's a 5-baud address byte packaged alone
            if frame.len() == 1 {
                let resp = self.process_byte(frame[0]);
                if !resp.is_empty() {
                    return Some(resp);
                }
            }
            return None;
        }

        // 2. If in SyncSent state and a single byte arrived (~KB2)
        if self.state == EcuState::SyncSent {
            if frame.len() == 1 {
                let resp = self.process_byte(frame[0]);
                if !resp.is_empty() {
                    return Some(resp);
                }
            }
            return None;
        }

        // 3. Session is Active: verify ISO checksum
        if frame.len() < 2 {
            return None;
        }

        let expected_csum = Self::calc_checksum(&frame[..frame.len() - 1]);
        if expected_csum != frame[frame.len() - 1] {
            return None; // Bad checksum, drop silently as physical ECU does
        }

        // Parse ISO header
        let (service, sub_payload) = self.parse_iso_service(frame)?;

        match service {
            // Mode 01: Show Current Data (Live Data)
            0x01 => {
                Some(self.handle_mode_01(sub_payload))
            }
            // Mode 02: Freeze Frame Data
            0x02 => {
                let pid = sub_payload.first().copied().unwrap_or(0x02);
                Some(self.handle_mode_02(pid))
            }
            // Mode 03: Show Stored DTCs
            0x03 => Some(self.handle_mode_03()),
            // Mode 04: Clear DTCs
            0x04 => {
                self.data.dtcs.clear();
                Some(self.wrap_iso_response(0x44, &[]))
            }
            // Mode 07: Show Pending DTCs
            0x07 => Some(self.wrap_iso_response(0x47, &[0x00])),
            // Mode 09: Vehicle Information (VIN, Calibration ID)
            0x09 => {
                let pid = sub_payload.first().copied().unwrap_or(0x02);
                Some(self.handle_mode_09(pid))
            }
            // TesterPresent ($3E)
            0x3E => {
                let sub = sub_payload.first().copied().unwrap_or(0x01);
                Some(self.wrap_iso_response(0x7E, &[sub]))
            }
            // StartCommunication ($81)
            0x81 => Some(self.wrap_iso_response(0xC1, &[self.kb1, self.kb2])),
            // StopCommunication ($82)
            0x82 => {
                self.state = EcuState::Idle;
                Some(self.wrap_iso_response(0xC2, &[]))
            }
            _ => {
                // Negative Response: ServiceNotSupported (0x11)
                Some(self.wrap_iso_response(0x7F, &[service, 0x11]))
            }
        }
    }

    fn check_fast_init(&mut self, frame: &[u8]) -> Option<Vec<u8>> {
        // Fast init StartCommunication: [Format, Target, Source, 0x81, Checksum]
        // E.g. ThinkDiag OBD2: [0xC1, 0x33, 0xF1, 0x81, 0x66]
        if frame.len() >= 4 && frame[frame.len() - 2] == 0x81 {
            let target = if frame.len() >= 5 { frame[1] } else { self.address };
            let source = if frame.len() >= 5 { frame[2] } else { self.tester_addr };
            if target == self.address || target == 0x33 || target == 0x11 || target == 0x01 || target == 0x10 {
                self.tester_addr = source;
                self.protocol = DiagnosticProtocol::Iso14230;
                // If targeted 0x33 (Functional OBD), Engine ECU physical address is 0x10
                if target == 0x33 {
                    self.ecu_addr = 0x10;
                }
                let mut resp = vec![0x83, self.tester_addr, self.ecu_addr, 0xC1, self.kb1, self.kb2];
                resp.push(Self::calc_checksum(&resp));
                return Some(resp);
            }
        }
        None
    }

    fn parse_iso_service<'a>(&mut self, frame: &'a [u8]) -> Option<(u8, &'a [u8])> {
        let payload = &frame[..frame.len() - 1]; // strip checksum
        if payload.is_empty() {
            return None;
        }

        let fmt = payload[0];

        // ISO 9141-2 / SAE J1979 request headers (0x68, 0x48)
        if fmt == 0x68 || fmt == 0x48 {
            if payload.len() >= 4 {
                self.tester_addr = payload[2];
                if payload[1] == 0x6A || payload[1] == 0x33 {
                    self.ecu_addr = 0x10;
                }
                let service = payload[3];
                let sub = &payload[4..];
                return Some((service, sub));
            }
            return None;
        }

        // ISO 14230 (KWP2000)
        if (fmt & 0xC0) != 0 {
            let len_in_fmt = (fmt & 0x3F) as usize;
            if len_in_fmt > 0 {
                // Header is [fmt, target, source] (3 bytes)
                if payload.len() >= 4 {
                    self.tester_addr = payload[2];
                    if payload[1] == 0x33 {
                        self.ecu_addr = 0x10;
                    }
                    let service = payload[3];
                    let sub = &payload[4..];
                    return Some((service, sub));
                }
            } else {
                // fmt & 0x3F == 0 (e.g. 0x80): Header is [fmt, target, source, len] (4 bytes)
                if payload.len() >= 5 {
                    self.tester_addr = payload[2];
                    if payload[1] == 0x33 {
                        self.ecu_addr = 0x10;
                    }
                    let service = payload[4];
                    let sub = &payload[5..];
                    return Some((service, sub));
                }
            }
            return None;
        }

        // Raw / single-byte or non-addressed
        let service = payload[0];
        let sub = &payload[1..];
        Some((service, sub))
    }

    fn handle_mode_01(&self, pids: &[u8]) -> Vec<u8> {
        let requested_pids = if pids.is_empty() { &[0x00][..] } else { pids };
        let mut payload = Vec::new();

        for &pid in requested_pids {
            match pid {
                // PID 00: Supported PIDs [01-20]
                // Supported: 01, 02, 03, 04, 05, 06, 07, 0B, 0C, 0D, 0E, 0F, 10, 11, 12, 13, 14, 15, 1C, 1F
                0x00 => {
                    payload.extend_from_slice(&[0x00, 0xFE, 0x3F, 0xF8, 0x12]);
                }
                // PID 01: Monitor Status Since DTCs Cleared
                0x01 => {
                    let mil_and_dtc = if self.data.dtcs.is_empty() { 0x00 } else { 0x80 | (self.data.dtcs.len() as u8) };
                    payload.extend_from_slice(&[0x01, mil_and_dtc, 0x07, 0xE5, 0x00]);
                }
                // PID 02: Freeze DTC
                0x02 => {
                    let dtc = self.data.dtcs.first().copied().unwrap_or([0x03, 0x00]);
                    payload.extend_from_slice(&[0x02, dtc[0], dtc[1]]);
                }
                // PID 03: Fuel System Status (0x02 = Closed loop, using O2 sensor)
                0x03 => {
                    payload.extend_from_slice(&[0x03, 0x02, 0x00]);
                }
                // PID 04: Calculated Engine Load (25% load = 64)
                0x04 => {
                    payload.extend_from_slice(&[0x04, 64]);
                }
                // PID 05: Engine Coolant Temperature: Temp(°C) = A - 40 => A = Temp + 40
                0x05 => {
                    let a = (self.data.coolant_temp_c + 40).clamp(0, 255) as u8;
                    payload.extend_from_slice(&[0x05, a]);
                }
                // PID 06: Short Term Fuel Trim Bank 1 (0% = 128)
                0x06 => {
                    payload.extend_from_slice(&[0x06, 128]);
                }
                // PID 07: Long Term Fuel Trim Bank 1 (+1.5% = 130)
                0x07 => {
                    payload.extend_from_slice(&[0x07, 130]);
                }
                // PID 0B: Intake Manifold Absolute Pressure (35 kPa manifold vacuum)
                0x0B => {
                    payload.extend_from_slice(&[0x0B, 35]);
                }
                // PID 0C: Engine RPM: RPM = ((A*256)+B)/4 => Raw = RPM * 4
                0x0C => {
                    let raw = self.data.rpm.saturating_mul(4);
                    let a = (raw >> 8) as u8;
                    let b = (raw & 0xFF) as u8;
                    payload.extend_from_slice(&[0x0C, a, b]);
                }
                // PID 0D: Vehicle Speed in km/h
                0x0D => {
                    payload.extend_from_slice(&[0x0D, self.data.speed_kmh]);
                }
                // PID 0E: Timing Advance (10 deg BTDC = (10 + 64) * 2 = 148)
                0x0E => {
                    payload.extend_from_slice(&[0x0E, 148]);
                }
                // PID 0F: Intake Air Temp: Temp(°C) = A - 40 => 35°C = 75
                0x0F => {
                    payload.extend_from_slice(&[0x0F, 75]);
                }
                // PID 10: MAF Air Flow Rate (3.50 g/s = 350 => 0x01, 0x5E)
                0x10 => {
                    payload.extend_from_slice(&[0x10, 0x01, 0x5E]);
                }
                // PID 11: Throttle Position: % = A / 2.55 => 15% = 38
                0x11 => {
                    payload.extend_from_slice(&[0x11, 38]);
                }
                // PID 12: Commanded Secondary Air Status (0x04 = Off)
                0x12 => {
                    payload.extend_from_slice(&[0x12, 0x04]);
                }
                // PID 13: Oxygen Sensors Present (Bank 1: Sensor 1 and Sensor 2 present: 0x03)
                0x13 => {
                    payload.extend_from_slice(&[0x13, 0x03]);
                }
                // PID 14: O2 Sensor 1 Voltage (0.75V = 150 = 0x96, STFT = 128 = 0%)
                0x14 => {
                    payload.extend_from_slice(&[0x14, 0x96, 128]);
                }
                // PID 15: O2 Sensor 2 Voltage (0.45V = 90 = 0x5A, STFT = 128 = 0%)
                0x15 => {
                    payload.extend_from_slice(&[0x15, 0x5A, 128]);
                }
                // PID 1C: OBD Standard: 0x01 = OBD-II (CARB)
                0x1C => {
                    payload.extend_from_slice(&[0x1C, 0x01]);
                }
                // PID 1F: Run Time Since Engine Start (450 seconds = 0x01, 0xC2)
                0x1F => {
                    payload.extend_from_slice(&[0x1F, 0x01, 0xC2]);
                }
                // PID 42: Control Module Voltage: V = ((A*256)+B)/1000
                0x42 => {
                    let a = (self.data.battery_mv >> 8) as u8;
                    let b = (self.data.battery_mv & 0xFF) as u8;
                    payload.extend_from_slice(&[0x42, a, b]);
                }
                _ => {}
            }
        }

        if payload.is_empty() {
            self.wrap_iso_response(0x7F, &[0x01, 0x12])
        } else {
            self.wrap_iso_response(0x41, &payload)
        }
    }

    fn handle_mode_02(&self, pid: u8) -> Vec<u8> {
        match pid {
            0x00 => self.wrap_iso_response(0x42, &[0x00, 0x00, 0xFE, 0x3F, 0x00, 0x00]),
            0x02 => {
                let dtc = self.data.dtcs.first().copied().unwrap_or([0x03, 0x00]);
                self.wrap_iso_response(0x42, &[0x02, 0x00, dtc[0], dtc[1]])
            }
            _ => self.handle_mode_01(&[pid]),
        }
    }

    fn handle_mode_09(&self, pid: u8) -> Vec<u8> {
        match pid {
            // PID 00: Supported Mode 09 PIDs [01-20] (PID 02 is bit 6 of Byte A: 0x40; PID 04 is bit 4: 0x10)
            0x00 => self.wrap_iso_response(0x49, &[0x00, 0x50, 0x00, 0x00, 0x00]),
            // PID 02: VIN
            0x02 => {
                let mut payload = vec![0x02, 0x01]; // PID 02, 1 message
                payload.extend_from_slice(self.data.vin.as_bytes());
                self.wrap_iso_response(0x49, &payload)
            }
            // PID 04: Calibration ID (e.g. 06A906032HP for VW ME 7.5)
            0x04 => {
                let mut payload = vec![0x04, 0x01]; // PID 04, 1 message
                payload.extend_from_slice(b"06A906032HP     ");
                self.wrap_iso_response(0x49, &payload)
            }
            _ => self.wrap_iso_response(0x7F, &[0x09, 0x12]),
        }
    }

    fn handle_mode_03(&self) -> Vec<u8> {
        let count = self.data.dtcs.len() as u8;
        let mut payload = vec![count];
        for dtc in &self.data.dtcs {
            payload.push(dtc[0]);
            payload.push(dtc[1]);
        }
        self.wrap_iso_response(0x43, &payload)
    }

    fn wrap_iso_response(&self, resp_service: u8, payload: &[u8]) -> Vec<u8> {
        let mut msg = match self.protocol {
            DiagnosticProtocol::Iso9141 => {
                let mut m = vec![0x48, 0x6B, self.ecu_addr, resp_service];
                m.extend_from_slice(payload);
                m
            }
            DiagnosticProtocol::Iso14230 | DiagnosticProtocol::VagKwp1281 => {
                let len = (payload.len() + 1) as u8;
                let fmt = 0x80 | (len & 0x3F);
                let mut m = vec![fmt, self.tester_addr, self.ecu_addr, resp_service];
                m.extend_from_slice(payload);
                m
            }
        };
        msg.push(Self::calc_checksum(&msg));
        msg
    }

    pub fn calc_checksum(bytes: &[u8]) -> u8 {
        bytes.iter().fold(0u8, |acc, &b| acc.wrapping_add(b))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_5baud_handshake_flow() {
        let mut sim = EcuSimulator::new(EcuProfile::BoschMe75);
        assert_eq!(sim.state, EcuState::Idle);

        let resp_sync = sim.process_byte(0x01);
        assert_eq!(resp_sync, vec![0x55, 0x08, 0x08]);
        assert_eq!(sim.state, EcuState::SyncSent);

        let resp_w4 = sim.process_byte(0xF7); // ~0x08 = 0xF7
        assert_eq!(resp_w4, vec![0xFE]); // ~0x01 = 0xFE
        assert_eq!(sim.state, EcuState::SessionActive);
    }

    #[test]
    fn test_live_data_rpm() {
        let mut sim = EcuSimulator::new(EcuProfile::BoschMe75);
        sim.state = EcuState::SessionActive;
        sim.data.rpm = 3000;

        let mut req = vec![0x68, 0x6A, 0xF1, 0x01, 0x0C];
        req.push(EcuSimulator::calc_checksum(&req));

        let resp = sim.process_frame(&req).expect("Expected RPM response");
        assert_eq!(resp[3], 0x41);
        assert_eq!(resp[4], 0x0C);
        let raw = ((resp[5] as u16) << 8) | (resp[6] as u16);
        assert_eq!(raw / 4, 3000);
    }

    #[test]
    fn test_read_and_clear_dtcs() {
        let mut sim = EcuSimulator::new(EcuProfile::BoschMe75);
        sim.state = EcuState::SessionActive;

        let mut req_dtc = vec![0x68, 0x6A, 0xF1, 0x03];
        req_dtc.push(EcuSimulator::calc_checksum(&req_dtc));
        let resp_dtc = sim.process_frame(&req_dtc).unwrap();
        assert_eq!(resp_dtc[3], 0x43);
        assert_eq!(resp_dtc[4], 2); // 2 DTCs

        let mut req_clear = vec![0x68, 0x6A, 0xF1, 0x04];
        req_clear.push(EcuSimulator::calc_checksum(&req_clear));
        let resp_clear = sim.process_frame(&req_clear).unwrap();
        assert_eq!(resp_clear[3], 0x44);

        let resp_after = sim.process_frame(&req_dtc).unwrap();
        assert_eq!(resp_after[4], 0); // 0 DTCs!
    }
}
