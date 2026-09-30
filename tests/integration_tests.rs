//! Integration tests for kline-ecu-sim

use kline_ecu_sim::{EchoGuard, EcuProfile, EcuSimulator, EcuState};

#[test]
fn test_bosch_me75_5baud_and_obd_flow() {
    let mut ecu = EcuSimulator::new(EcuProfile::BoschMe75);
    assert_eq!(ecu.state, EcuState::Idle);

    // 1. Scanner sends 5-baud Address Byte 0x01
    let sync_resp = ecu.process_byte(0x01);
    assert_eq!(sync_resp, vec![0x55, 0x08, 0x08]); // Sync + KB1 + KB2
    assert_eq!(ecu.state, EcuState::SyncSent);

    // 2. Scanner sends ~KB2 (0xF7)
    let ack_resp = ecu.process_byte(0xF7);
    assert_eq!(ack_resp, vec![0xFE]); // ~0x01 = 0xFE
    assert_eq!(ecu.state, EcuState::SessionActive);

    // 3. Scanner requests RPM (Mode 01 PID 0C)
    ecu.data.rpm = 2400;
    let mut req_rpm = vec![0x68, 0x6A, 0xF1, 0x01, 0x0C];
    req_rpm.push(EcuSimulator::calc_checksum(&req_rpm));

    let resp_rpm = ecu.process_frame(&req_rpm).expect("Expected RPM response");
    assert_eq!(resp_rpm[3], 0x41); // Positive response 0x40 + 0x01
    assert_eq!(resp_rpm[4], 0x0C); // PID 0C
    let raw = ((resp_rpm[5] as u16) << 8) | (resp_rpm[6] as u16);
    assert_eq!(raw / 4, 2400);

    // 4. Scanner queries DTCs (Mode 03)
    let mut req_dtc = vec![0x68, 0x6A, 0xF1, 0x03];
    req_dtc.push(EcuSimulator::calc_checksum(&req_dtc));
    let resp_dtc = ecu.process_frame(&req_dtc).unwrap();
    assert_eq!(resp_dtc[3], 0x43);
    assert_eq!(resp_dtc[4], 2); // P0300, P0171

    // 5. Scanner clears DTCs (Mode 04)
    let mut req_clear = vec![0x68, 0x6A, 0xF1, 0x04];
    req_clear.push(EcuSimulator::calc_checksum(&req_clear));
    let resp_clear = ecu.process_frame(&req_clear).unwrap();
    assert_eq!(resp_clear[3], 0x44);

    // 6. Verify DTC count is now 0
    let resp_dtc_after = ecu.process_frame(&req_dtc).unwrap();
    assert_eq!(resp_dtc_after[4], 0);
}

#[test]
fn test_iso14230_fast_init_flow() {
    let mut ecu = EcuSimulator::new(EcuProfile::GenericObd);
    assert_eq!(ecu.state, EcuState::Idle);

    // Scanner sends StartCommunication ($81)
    let mut fast_req = vec![0x81, 0x33, 0xF1, 0x81];
    fast_req.push(EcuSimulator::calc_checksum(&fast_req));

    let resp = ecu.process_frame(&fast_req).expect("Expected Fast Init response");
    assert_eq!(resp[3], 0xC1); // Positive response to $81
    assert_eq!(resp[4], 0x08); // KB1
    assert_eq!(resp[5], 0x08); // KB2
    assert_eq!(ecu.state, EcuState::SessionActive);
}

#[test]
fn test_vag_kwp1281_handshake() {
    let mut ecu = EcuSimulator::new(EcuProfile::VagKwp1281);
    assert_eq!(ecu.state, EcuState::Idle);

    // 5-baud address 0x01
    let sync_resp = ecu.process_byte(0x01);
    assert_eq!(sync_resp, vec![0x55, 0x01, 0x8A]);

    // W4 complement ~0x8A = 0x75
    let ack_resp = ecu.process_byte(0x75);
    // KWP1281 skips ~Address
    assert!(ack_resp.is_empty());
    assert_eq!(ecu.state, EcuState::SessionActive);
}

#[test]
fn test_echoguard_filtering() {
    let mut echo = EchoGuard::new(10400);
    let tx_bytes = [0x55, 0x08, 0x08];
    echo.record_tx(&tx_bytes);

    // Incoming has reflected TX followed by real scanner bytes
    let incoming = [0x55, 0x08, 0x08, 0xF7];
    let filtered = echo.filter_rx(&incoming);
    assert_eq!(filtered, vec![0xF7]);
}

#[test]
fn test_thinkdiag_fast_init_end_to_end() {
    use kline_ecu_sim::SilencePacketizer;

    let mut packetizer = SilencePacketizer::new(10400, 15.0);
    let mut ecu = EcuSimulator::new(EcuProfile::BoschMe75);

    // 1. ThinkDiag sends 25ms break pulse (0x00)
    let f1 = packetizer.push(&[0x00]);
    assert_eq!(f1, None);

    // 2. ThinkDiag sends C1, 33, F1, 81, 66 (with 5-6ms inter-byte gaps, pushed chunk by chunk)
    assert_eq!(packetizer.push(&[0xC1]), None);
    assert_eq!(packetizer.push(&[0x33]), None);
    assert_eq!(packetizer.push(&[0xF1]), None);
    assert_eq!(packetizer.push(&[0x81]), None);
    let frame = packetizer.push(&[0x66]).expect("Expected complete Fast Init frame");

    assert_eq!(frame, vec![0xC1, 0x33, 0xF1, 0x81, 0x66]);

    // 3. Process frame with ECU simulator
    let resp = ecu.process_frame(&frame).expect("Expected Fast Init response");
    // Should be positive response: [0x83, 0xF1, 0x11, 0xC1, 0x08, 0x08, CS]
    assert_eq!(resp[0], 0x83);
    assert_eq!(resp[1], 0xF1); // Target = Tester
    assert_eq!(resp[2], 0x11); // Source = ECU (Bosch ME 7.5 engine)
    assert_eq!(resp[3], 0xC1); // Positive response to $81
    assert_eq!(resp[4], 0x08); // KB1
    assert_eq!(resp[5], 0x08); // KB2
    assert_eq!(resp[6], 0x56); // Checksum: 0x83+0xF1+0x11+0xC1+0x08+0x08 = 598 (0x256) & 0xFF = 0x56
    assert_eq!(ecu.state, EcuState::SessionActive);
}

