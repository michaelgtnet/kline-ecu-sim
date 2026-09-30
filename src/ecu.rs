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
            // AUTO-RECOVERY: If the scanner is already in an active session sending valid frames
            // (e.g. TesterPresent $3E, Mode 01, StartDiagnosticSession $10, etc.),
            // verify checksum and if addressed to this ECU, wake up into SessionActive!
            if frame.len() >= 2 {
                let expected_csum = Self::calc_checksum(&frame[..frame.len() - 1]);
                if expected_csum == frame[frame.len() - 1] {
                    if let Some((_service, _)) = self.parse_iso_service(frame) {
                        self.state = EcuState::SessionActive;
                        // Fall through to active session processing below!
                    } else {
                        return None;
                    }
                } else {
                    return None;
                }
            } else {
                return None;
            }
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
            0x01 => Some(self.handle_mode_01(sub_payload)),
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
            // Mode 0A: Show Permanent DTCs
            0x0A => Some(self.wrap_iso_response(0x4A, &[0x00])),
            // Mode 09: Vehicle Information (VIN, Calibration ID, CVN, ECU Name)
            0x09 => {
                let pid = sub_payload.first().copied().unwrap_or(0x02);
                Some(self.handle_mode_09(pid))
            }
            // Service 0x10: StartDiagnosticSession (KWP2000 standard/programming/development)
            0x10 => {
                let sub = sub_payload.first().copied().unwrap_or(0x81);
                Some(self.wrap_iso_response(0x50, &[sub]))
            }
            // Service 0x14: ClearDiagnosticInformation (KWP2000)
            0x14 => {
                self.data.dtcs.clear();
                Some(self.wrap_iso_response(0x54, &[]))
            }
            // Service 0x18: ReadDiagnosticTroubleCodesByStatus (KWP2000)
            0x18 => Some(self.handle_kwp_read_dtcs()),
            // Service 0x21: ReadDataByLocalIdentifier (VAG Measuring Blocks / Groups)
            0x21 => {
                let group = sub_payload.first().copied().unwrap_or(0x01);
                Some(self.handle_kwp_measuring_block(group))
            }
            // Service 0x22: ReadDataByIdentifier (KWP2000 / UDS)
            0x22 => Some(self.handle_kwp_read_by_id(sub_payload)),
            // Service 0x27: SecurityAccess (ME7.5 Flashing Backtest: Seed/Key)
            0x27 => {
                let sub = sub_payload.first().copied().unwrap_or(0x01);
                Some(self.handle_kwp_security_access(sub, sub_payload))
            }
            // Service 0x31: RoutineControl (Erase Flash Sector, Checksum Verify)
            0x31 => {
                let routine = sub_payload.first().copied().unwrap_or(0x01);
                Some(self.wrap_iso_response(0x71, &[routine]))
            }
            // Service 0x34: RequestDownload (Flash Download Request)
            0x34 => Some(self.wrap_iso_response(0x74, &[])),
            // Service 0x36: TransferData (Flash Data Block Transfer)
            0x36 => {
                let block_seq = sub_payload.first().copied().unwrap_or(0x01);
                Some(self.wrap_iso_response(0x76, &[block_seq]))
            }
            // Service 0x37: RequestTransferExit (Flash Transfer Complete)
            0x37 => Some(self.wrap_iso_response(0x77, &[])),
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
                // Supported: 01, 02, 03, 04, 05, 06, 07, 0B, 0C, 0D, 0E, 0F, 10, 11, 12, 13, 14, 15, 1C, 1F, 20
                0x00 => {
                    payload.extend_from_slice(&[0x00, 0xFE, 0x3F, 0xF8, 0x13]);
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
                // PID 20: Supported PIDs [21-40] (Supports 21, 2E, 2F, 30, 31, 33, 40)
                0x20 => {
                    payload.extend_from_slice(&[0x20, 0x80, 0x07, 0xA0, 0x01]);
                }
                // PID 21: Distance Traveled with MIL On (42 km)
                0x21 => {
                    payload.extend_from_slice(&[0x21, 0x00, 42]);
                }
                // PID 2E: Commanded Evaporative Purge (12%)
                0x2E => {
                    payload.extend_from_slice(&[0x2E, 31]);
                }
                // PID 2F: Fuel Tank Level Input (65%)
                0x2F => {
                    payload.extend_from_slice(&[0x2F, 166]);
                }
                // PID 30: Warm-ups Since Codes Cleared
                0x30 => {
                    payload.extend_from_slice(&[0x30, 15]);
                }
                // PID 31: Distance Traveled Since Codes Cleared (120 km)
                0x31 => {
                    payload.extend_from_slice(&[0x31, 0x00, 120]);
                }
                // PID 33: Absolute Barometric Pressure (101 kPa)
                0x33 => {
                    payload.extend_from_slice(&[0x33, 101]);
                }
                // PID 40: Supported PIDs [41-60] (Supports 42, 45, 46, 49, 4A, 4C)
                0x40 => {
                    payload.extend_from_slice(&[0x40, 0x74, 0x00, 0x00, 0x00]);
                }
                // PID 42: Control Module Voltage: V = ((A*256)+B)/1000
                0x42 => {
                    let a = (self.data.battery_mv >> 8) as u8;
                    let b = (self.data.battery_mv & 0xFF) as u8;
                    payload.extend_from_slice(&[0x42, a, b]);
                }
                // PID 45: Relative Throttle Position (12%)
                0x45 => {
                    payload.extend_from_slice(&[0x45, 31]);
                }
                // PID 46: Ambient Air Temperature (25°C = 65)
                0x46 => {
                    payload.extend_from_slice(&[0x46, 65]);
                }
                // PID 49: Accelerator Pedal Position D (15%)
                0x49 => {
                    payload.extend_from_slice(&[0x49, 38]);
                }
                // PID 4A: Accelerator Pedal Position E (15%)
                0x4A => {
                    payload.extend_from_slice(&[0x4A, 38]);
                }
                // PID 4C: Commanded Throttle Actuator (15%)
                0x4C => {
                    payload.extend_from_slice(&[0x4C, 38]);
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
            // PID 00: Supported Mode 09 PIDs [01-20] (PID 02, 04, 06, 0A)
            0x00 => self.wrap_iso_response(0x49, &[0x00, 0x54, 0x40, 0x00, 0x00]),
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
            // PID 06: CVN (Calibration Verification Number)
            0x06 => {
                let payload = vec![0x06, 0x01, 0xA1, 0xB2, 0xC3, 0xD4];
                self.wrap_iso_response(0x49, &payload)
            }
            // PID 0A: ECU Name
            0x0A => {
                let mut payload = vec![0x0A, 0x01];
                let mut name = b"BOSCH ME7.5 ECM     ".to_vec();
                name.truncate(20);
                payload.extend_from_slice(&name);
                self.wrap_iso_response(0x49, &payload)
            }
            _ => self.wrap_iso_response(0x7F, &[0x09, 0x12]),
        }
    }

    fn handle_kwp_read_dtcs(&self) -> Vec<u8> {
        let mut payload = vec![self.data.dtcs.len() as u8];
        for dtc in &self.data.dtcs {
            payload.push(dtc[0]);
            payload.push(dtc[1]);
            payload.push(0x21); // Status byte (Current / MIL illuminated)
        }
        self.wrap_iso_response(0x58, &payload)
    }

    fn handle_kwp_measuring_block(&self, group: u8) -> Vec<u8> {
        let mut payload = vec![group];
        match group {
            // Group 001: Basic Engine Data [RPM, Coolant Temp, Lambda Control, Basic Settings]
            0x01 => {
                let raw_rpm = self.data.rpm / 40;
                let raw_temp = (self.data.coolant_temp_c + 48).clamp(0, 255) as u8;
                payload.extend_from_slice(&[raw_rpm as u8, raw_temp, 0x80, 0x00]);
            }
            // Group 002: Load / MAF / Injection Time
            0x02 => {
                let raw_rpm = (self.data.rpm / 40) as u8;
                payload.extend_from_slice(&[raw_rpm, 0x28, 0x18, 0x22]);
            }
            // Group 003: RPM / MAF / Throttle Angle / Timing
            0x03 => {
                let raw_rpm = (self.data.rpm / 40) as u8;
                payload.extend_from_slice(&[raw_rpm, 0x20, 0x15, 0x90]);
            }
            _ => {
                payload.extend_from_slice(&[0x00, 0x00, 0x00, 0x00]);
            }
        }
        self.wrap_iso_response(0x61, &payload)
    }

    fn handle_kwp_read_by_id(&self, sub: &[u8]) -> Vec<u8> {
        let mut payload = Vec::new();
        payload.extend_from_slice(sub);
        payload.extend_from_slice(b"06A906032HP 1.8L R4/5VT     ");
        self.wrap_iso_response(0x62, &payload)
    }

    fn handle_kwp_security_access(&self, sub: u8, sub_payload: &[u8]) -> Vec<u8> {
        match sub {
            // 0x01: Request Seed (returns 4-byte seed)
            0x01 => {
                self.wrap_iso_response(0x67, &[0x01, 0x34, 0x78, 0x12, 0x56])
            }
            // 0x02: Send Key (grant access for backtesting)
            0x02 => {
                self.wrap_iso_response(0x67, &[0x02])
            }
            _ => {
                let p = if sub_payload.is_empty() { &[0x01][..] } else { sub_payload };
                self.wrap_iso_response(0x67, p)
            }
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

    #[test]
    fn test_auto_recovery_from_idle_on_tester_present() {
        let mut sim = EcuSimulator::new(EcuProfile::BoschMe75);
        assert_eq!(sim.state, EcuState::Idle);

        // Frame from ThinkDiag: [C2, 33, F1, 3E, 01, 25]
        let req = vec![0xC2, 0x33, 0xF1, 0x3E, 0x01, 0x25];
        let resp = sim.process_frame(&req).expect("Expected auto-recovery and response");

        assert_eq!(sim.state, EcuState::SessionActive);
        // Expect positive response to TesterPresent: service 0x7E, sub 0x01
        assert_eq!(resp[3], 0x7E);
        assert_eq!(resp[4], 0x01);
    }

    #[test]
    fn test_mode_01_extended_pids() {
        let mut sim = EcuSimulator::new(EcuProfile::BoschMe75);
        sim.state = EcuState::SessionActive;

        // Query PID 00 (Supported PIDs)
        let mut req_pid00 = vec![0x68, 0x6A, 0xF1, 0x01, 0x00];
        req_pid00.push(EcuSimulator::calc_checksum(&req_pid00));
        let resp00 = sim.process_frame(&req_pid00).unwrap();
        assert_eq!(resp00[3], 0x41);
        assert_eq!(resp00[4], 0x00);
        assert_eq!(resp00[8] & 0x01, 0x01); // PID 20 supported

        // Query PID 42 (Module Voltage)
        let mut req_pid42 = vec![0x68, 0x6A, 0xF1, 0x01, 0x42];
        req_pid42.push(EcuSimulator::calc_checksum(&req_pid42));
        let resp42 = sim.process_frame(&req_pid42).unwrap();
        assert_eq!(resp42[3], 0x41);
        assert_eq!(resp42[4], 0x42);
        let voltage_mv = ((resp42[5] as u16) << 8) | (resp42[6] as u16);
        assert_eq!(voltage_mv, 13800); // 13.8V
    }

    #[test]
    fn test_kwp2000_programming_and_security_access() {
        let mut sim = EcuSimulator::new(EcuProfile::BoschMe75);
        sim.state = EcuState::SessionActive;

        // 1. Start Diagnostic Session (Programming: 0x10 0x85)
        let mut req_prog = vec![0xC2, 0x33, 0xF1, 0x10, 0x85];
        req_prog.push(EcuSimulator::calc_checksum(&req_prog));
        let resp_prog = sim.process_frame(&req_prog).unwrap();
        assert_eq!(resp_prog[3], 0x50);
        assert_eq!(resp_prog[4], 0x85);

        // 2. Request Seed (0x27 0x01)
        let mut req_seed = vec![0xC2, 0x33, 0xF1, 0x27, 0x01];
        req_seed.push(EcuSimulator::calc_checksum(&req_seed));
        let resp_seed = sim.process_frame(&req_seed).unwrap();
        assert_eq!(resp_seed[3], 0x67);
        assert_eq!(resp_seed[4], 0x01);
        assert_eq!(resp_seed.len(), 10); // fmt, target, src, 67, 01, s1, s2, s3, s4, cs

        // 3. Send Key (0x27 0x02)
        let mut req_key = vec![0xC6, 0x33, 0xF1, 0x27, 0x02, 0xAA, 0xBB, 0xCC, 0xDD];
        req_key.push(EcuSimulator::calc_checksum(&req_key));
        let resp_key = sim.process_frame(&req_key).unwrap();
        assert_eq!(resp_key[3], 0x67);
        assert_eq!(resp_key[4], 0x02); // Granted!
    }
}
