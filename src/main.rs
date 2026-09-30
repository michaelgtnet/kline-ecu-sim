//! kline-ecu-sim binary: High-performance automotive K-Line ECU simulator for bench testing.

use std::io::{Read, Write};
use std::sync::atomic::{AtomicU16, Ordering};
use std::sync::Arc;
use std::time::Duration;

use clap::Parser;
use tokio::sync::mpsc;
use tokio::time::sleep;
use tracing::{error, info, warn};
use tracing_subscriber::EnvFilter;

use kline_ecu_sim::{
    EchoGuard, EcuProfile, EcuSimulator, EcuState, NativeSerialPort, SilencePacketizer,
};

#[derive(Parser, Debug)]
#[command(
    name = "kline-ecu-sim",
    author = "Michael G. <michael@autolink.pro>",
    version = "0.1.0",
    about = "Automotive K-Line ECU Simulator for ISO 9141-2, ISO 14230 (KWP2000) and VW KWP1281"
)]
struct Args {
    /// Serial UART device port (e.g. /dev/ttyS3 on Orange Pi One)
    #[arg(short = 'd', long, default_value = "/dev/ttyS3")]
    device: String,

    /// Initial baud rate (standard K-Line is 10400 or 9600)
    #[arg(short = 'b', long, default_value_t = 10400)]
    baud: u32,

    /// ECU profile to simulate (bosch-me75, vag-kwp1281, generic-obd, marelli-iaw)
    #[arg(short = 'p', long, default_value = "bosch-me75")]
    profile: EcuProfile,

    /// Initial Engine RPM
    #[arg(long, default_value_t = 850)]
    rpm: u16,

    /// Dynamically sweep RPM (850 -> 3200 RPM) to verify live scanner gauges
    #[arg(long, default_value_t = false)]
    rpm_sweep: bool,

    /// Engine Coolant Temperature (°C)
    #[arg(long, default_value_t = 90)]
    coolant_temp: i16,

    /// Vehicle Speed (km/h)
    #[arg(long, default_value_t = 0)]
    speed: u8,

    /// Initial Stored DTCs (comma-separated, e.g. P0300,P0171)
    #[arg(long, default_value = "P0300,P0171")]
    dtcs: String,

    /// Vehicle Identification Number (VIN)
    #[arg(long, default_value = "9BWCA05X12P123456")]
    vin: String,
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    tracing_subscriber::fmt()
        .with_env_filter(
            EnvFilter::try_from_default_env().unwrap_or_else(|_| EnvFilter::new("info")),
        )
        .init();

    let args = Args::parse();

    println!("============================================================");
    println!(" 🚗⚡ kline-ecu-sim v0.1.0 — Automotive K-Line ECU Simulator");
    println!("============================================================");
    println!(" Profile      : {}", args.profile);
    println!(" Serial Port  : {} @ {} baud", args.device, args.baud);
    println!(" Initial RPM  : {} RPM (Sweep: {})", args.rpm, args.rpm_sweep);
    println!(" Coolant Temp : {} °C", args.coolant_temp);
    println!(" DTCs Loaded  : {}", args.dtcs);
    println!(" VIN          : {}", args.vin);
    println!("============================================================");
    println!(" Waiting for scanner to initiate connection...");
    println!("============================================================");

    // 1. Open Serial Port (termios2 BOTHER on Linux)
    let serial = NativeSerialPort::open(&args.device, args.baud)
        .map_err(|e| format!("Failed to open serial device {}: {}", args.device, e))?;

    let mut serial_writer = serial.try_clone()?;
    let mut serial_reader = serial;

    // 2. Setup Shared ECU State
    let mut ecu = EcuSimulator::new(args.profile);
    ecu.data.rpm = args.rpm;
    ecu.data.coolant_temp_c = args.coolant_temp;
    ecu.data.speed_kmh = args.speed;
    ecu.data.vin = args.vin.clone();

    // Parse DTCs (e.g. P0300 -> 0x03, 0x00)
    let mut parsed_dtcs = Vec::new();
    for code in args.dtcs.split(',') {
        let code = code.trim().to_uppercase();
        if code.starts_with('P') && code.len() == 5 {
            if let (Ok(hi), Ok(lo)) = (
                u8::from_str_radix(&code[1..3], 16),
                u8::from_str_radix(&code[3..5], 16),
            ) {
                parsed_dtcs.push([hi, lo]);
            }
        }
    }
    ecu.data.dtcs = parsed_dtcs;

    let shared_rpm = Arc::new(AtomicU16::new(args.rpm));
    if args.rpm_sweep {
        let rpm_clone = Arc::clone(&shared_rpm);
        tokio::spawn(async move {
            let mut rising = true;
            let mut current = 850u16;
            loop {
                sleep(Duration::from_millis(500)).await;
                if rising {
                    current = current.saturating_add(200);
                    if current >= 3200 {
                        rising = false;
                    }
                } else {
                    current = current.saturating_sub(200);
                    if current <= 850 {
                        rising = true;
                    }
                }
                rpm_clone.store(current, Ordering::Relaxed);
            }
        });
    }

    // 3. Spawn OS Thread for Non-Blocking UART Streaming
    let (tx, mut rx) = mpsc::channel::<Vec<u8>>(128);
    std::thread::spawn(move || {
        let mut buf = [0u8; 128];
        loop {
            match serial_reader.read(&mut buf) {
                Ok(0) => std::thread::sleep(Duration::from_millis(1)),
                Ok(n) => {
                    if tx.blocking_send(buf[..n].to_vec()).is_err() {
                        break;
                    }
                }
                Err(ref e) if e.kind() == std::io::ErrorKind::Interrupted => continue,
                Err(e) => {
                    error!("Serial read error: {}", e);
                    std::thread::sleep(Duration::from_millis(10));
                }
            }
        }
    });

    let mut packetizer = SilencePacketizer::new(args.baud, 1.8);
    let mut echo_guard = EchoGuard::new(args.baud);
    let mut last_session_state = ecu.state;

    // 4. Main Event Loop
    loop {
        let timeout_fut = async {
            if let Some(duration) = packetizer.time_until_silence() {
                sleep(duration).await;
                true
            } else {
                std::future::pending::<bool>().await
            }
        };

        tokio::select! {
            Some(raw_bytes) = rx.recv() => {
                let legitimate = echo_guard.filter_rx(&raw_bytes);
                if !legitimate.is_empty() {
                    info!("📥 [BUS RX] {} bytes: {:02X?}", legitimate.len(), legitimate);
                    if let Some(frame) = packetizer.push(&legitimate) {
                        handle_frame(&mut ecu, &mut serial_writer, &mut echo_guard, &frame, &shared_rpm);
                    }
                }
            }

            _ = timeout_fut => {
                if let Some(frame) = packetizer.check_timeout() {
                    handle_frame(&mut ecu, &mut serial_writer, &mut echo_guard, &frame, &shared_rpm);
                }
            }
        }

        if ecu.state != last_session_state {
            if ecu.state == EcuState::SessionActive {
                info!("✅ [SESSION ACTIVE] Diagnostic session fully opened with scanner!");
            } else if ecu.state == EcuState::Idle {
                warn!("⚠️ [SESSION RESET] Session reset to Idle.");
            }
            last_session_state = ecu.state;
        }
    }
}

fn handle_frame(
    ecu: &mut EcuSimulator,
    serial_writer: &mut NativeSerialPort,
    echo_guard: &mut EchoGuard,
    frame: &[u8],
    shared_rpm: &Arc<AtomicU16>,
) {
    if frame.is_empty() {
        return;
    }

    ecu.data.rpm = shared_rpm.load(Ordering::Relaxed);

    // 1. If in Idle or SyncSent state, check single-byte handshake
    if (ecu.state == EcuState::Idle || ecu.state == EcuState::SyncSent) && frame.len() == 1 {
        let incoming_byte = frame[0];
        // In Idle state, ignore 0x00 break pulse (it's the 25ms Fast Init wake pulse!)
        if ecu.state == EcuState::Idle && incoming_byte == 0x00 {
            return;
        }
        let resp = ecu.process_byte(incoming_byte);
        if !resp.is_empty() {
            info!("⚡ [HANDSHAKE] RX 0x{:02X} -> TX {:02X?}", incoming_byte, resp);
            echo_guard.record_tx(&resp);
            if let Err(e) = serial_writer.write_all(&resp) {
                error!("Serial write error: {}", e);
            }
            let _ = serial_writer.flush();
            return;
        }
    }

    // 2. Process ISO frame (Fast Init $81, Mode 01, Mode 03, Mode 04, Mode 09, etc.)
    if let Some(resp) = ecu.process_frame(frame) {
        log_diagnostic_exchange(frame, &resp, ecu.data.rpm);
        echo_guard.record_tx(&resp);
        if let Err(e) = serial_writer.write_all(&resp) {
            error!("Serial write error: {}", e);
        }
        let _ = serial_writer.flush();
    } else if frame != [0x00] {
        warn!("⚠️ [UNHANDLED/NO RESPONSE] {} bytes: {:02X?}", frame.len(), frame);
    }
}

fn log_diagnostic_exchange(req: &[u8], resp: &[u8], current_rpm: u16) {
    if req.is_empty() || resp.is_empty() {
        return;
    }

    let service = if req.len() >= 4 && (req[0] & 0xC0) != 0 {
        req[3]
    } else {
        req[0]
    };

    match service {
        0x01 => {
            let pid = if req.len() >= 5 { req[4] } else { 0x00 };
            match pid {
                0x0C => info!("📊 [REQ] Mode 01 PID 0C (Engine RPM) -> Responding {} RPM", current_rpm),
                0x05 => info!("🌡️ [REQ] Mode 01 PID 05 (Coolant Temp) -> Responding 90 °C"),
                0x0D => info!("🏎️ [REQ] Mode 01 PID 0D (Vehicle Speed)"),
                _ => info!("📊 [REQ] Mode 01 PID {:02X}", pid),
            }
        }
        0x03 => info!("🔍 [REQ] Mode 03 (Read Trouble Codes) -> Responding DTCs"),
        0x04 => info!("🧹 [REQ] Mode 04 (Clear Trouble Codes) -> DTCs Cleared!"),
        0x09 => info!("📋 [REQ] Mode 09 (Vehicle Information / VIN)"),
        0x81 => info!("⚡ [FAST INIT] StartCommunication ($81) -> Positive Response ($C1)"),
        0x3E => info!("💓 [REQ] TesterPresent ($3E)"),
        _ => info!("📩 [REQ] Service 0x{:02X} -> TX {:02X?}", service, resp),
    }
}
