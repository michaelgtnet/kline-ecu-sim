//! kline-ecu-sim binary: High-performance automotive K-Line ECU simulator for bench testing.

use std::io::{Read, Write};
use std::sync::atomic::{AtomicU16, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use clap::Parser;
use tokio::sync::mpsc;
use tokio::time::{sleep, sleep_until, Instant as TokioInstant};
use tracing::{error, info, warn};
use tracing_subscriber::EnvFilter;

use kline_ecu_sim::{
    CarLed, EchoGuard, EcuProfile, EcuSimulator, EcuState, NativeSerialPort, SilencePacketizer,
    SlowInitDetector,
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

    /// GPIO line for Car-side TX activity LED (default: 10 on gpiochip0 / PA10)
    #[arg(long, default_value_t = 10)]
    car_led_line: u32,
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

    let mut packetizer = SilencePacketizer::new(args.baud, 15.0);
    let mut echo_guard = EchoGuard::new(args.baud);
    let mut car_led = CarLed::open(Some(args.car_led_line));
    let mut slow_init = SlowInitDetector::new();
    let mut wake_deadline: Option<TokioInstant> = None;
    let mut pending_init_addr: Option<u8> = None;
    let mut kb2_deadline: Option<TokioInstant> = None;
    let mut last_session_state = ecu.state;

    // 4. Main Event Loop
    loop {
        car_led.service();

        let timeout_fut = async {
            if let Some(duration) = packetizer.time_until_silence() {
                sleep(duration).await;
                true
            } else {
                std::future::pending::<bool>().await
            }
        };

        let wd = wake_deadline;
        let wake_fut = async move {
            match wd {
                Some(deadline) => sleep_until(deadline).await,
                None => std::future::pending::<()>().await,
            }
        };

        let kd = kb2_deadline;
        let kb2_fut = async move {
            match kd {
                Some(deadline) => sleep_until(deadline).await,
                None => std::future::pending::<()>().await,
            }
        };

        tokio::select! {
            Some(raw_bytes) = rx.recv() => {
                let legitimate = echo_guard.filter_rx(&raw_bytes);
                if !legitimate.is_empty() {
                    info!("📥 [BUS RX] {} bytes: {:02X?}", legitimate.len(), legitimate);

                    // 1. If in Idle state and incoming bytes are break pulses (0x00), track 5-baud wake
                    if ecu.state == EcuState::Idle && legitimate.iter().all(|&b| b == 0x00) {
                        if let Some((target_time, addr)) = slow_init.push(Instant::now()) {
                            let delay = target_time.saturating_duration_since(Instant::now());
                            wake_deadline = Some(TokioInstant::now() + delay);
                            pending_init_addr = Some(addr);
                            info!(
                                "⚡ [HANDSHAKE] Wake 5-baud pattern recognized (bursts de 0x00 para addr 0x{:02X}); scheduling 55 KB1 KB2 in {} ms",
                                addr,
                                delay.as_millis()
                            );
                        }
                    }

                    // 2. ALWAYS pass incoming bytes to packetizer (handles Fast Init [C1, 33, F1, 81, 66] and active session commands)
                    if let Some(frame) = packetizer.push(&legitimate) {
                        // When a complete frame arrives, cancel any pending slow-init wake
                        wake_deadline = None;
                        pending_init_addr = None;
                        slow_init.reset();
                        handle_frame(&mut ecu, &mut serial_writer, &mut echo_guard, &mut car_led, &frame, &shared_rpm);
                        if ecu.state == EcuState::SessionActive {
                            kb2_deadline = None;
                        }
                    }
                }
            }

            _ = wake_fut => {
                wake_deadline = None;
                if ecu.state == EcuState::Idle {
                    if let Some(addr) = pending_init_addr.take() {
                        ecu.address = addr;
                    }
                    let resp = vec![0x55, ecu.kb1, ecu.kb2];
                    info!("⚡ [HANDSHAKE] Wake 5-baud (addr 0x{:02X}) -> TX {:02X?}", ecu.address, resp);
                    echo_guard.record_tx(&resp);
                    car_led.pulse();
                    if let Err(e) = serial_writer.write_all(&resp) {
                        error!("Serial write error: {}", e);
                    }
                    let _ = serial_writer.flush();
                    ecu.state = EcuState::SyncSent;
                    kb2_deadline = Some(TokioInstant::now() + Duration::from_millis(800));
                }
            }

            _ = kb2_fut => {
                kb2_deadline = None;
                if ecu.state == EcuState::SyncSent {
                    warn!("⚠️ [HANDSHAKE] Scanner did not respond with ~KB2 in 800ms; returning to Idle");
                    ecu.state = EcuState::Idle;
                    slow_init.reset();
                }
            }

            _ = timeout_fut => {
                if let Some(frame) = packetizer.check_timeout() {
                    handle_frame(&mut ecu, &mut serial_writer, &mut echo_guard, &mut car_led, &frame, &shared_rpm);
                    if ecu.state == EcuState::SessionActive {
                        kb2_deadline = None;
                    }
                }
            }
        }

        if ecu.state != last_session_state {
            if ecu.state == EcuState::SessionActive {
                info!("✅ [SESSION ACTIVE] Diagnostic session fully opened with scanner!");
                wake_deadline = None;
                kb2_deadline = None;
                slow_init.reset();
            } else if ecu.state == EcuState::Idle {
                warn!("⚠️ [SESSION RESET] Session reset to Idle.");
                wake_deadline = None;
                kb2_deadline = None;
                slow_init.reset();
            }
            last_session_state = ecu.state;
        }
    }
}

fn handle_frame(
    ecu: &mut EcuSimulator,
    serial_writer: &mut NativeSerialPort,
    echo_guard: &mut EchoGuard,
    car_led: &mut CarLed,
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
            car_led.pulse();
            std::thread::sleep(Duration::from_millis(30)); // ISO W4 window: 25-50ms
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
        car_led.pulse();
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
        0x10 => info!("🔧 [REQ] StartDiagnosticSession ($10) -> Mode {:?}", req.get(4)),
        0x21 => info!("📈 [REQ] ReadDataByLocalIdentifier ($21) -> Group {:?}", req.get(4)),
        0x22 => info!("🏷️ [REQ] ReadDataByIdentifier ($22)"),
        0x27 => info!("🔑 [REQ] SecurityAccess ($27) -> Seed/Key"),
        0x31 => info!("⚙️ [REQ] RoutineControl ($31)"),
        0x34 => info!("💾 [REQ] RequestDownload ($34) -> Flash Address"),
        0x36 => info!("📦 [REQ] TransferData ($36) -> Flash Data Block"),
        0x37 => info!("🏁 [REQ] RequestTransferExit ($37) -> Flash Complete"),
        0x81 => info!("⚡ [FAST INIT] StartCommunication ($81) -> Positive Response ($C1)"),
        0x3E => info!("💓 [REQ] TesterPresent ($3E)"),
        _ => info!("📩 [REQ] Service 0x{:02X} -> TX {:02X?}", service, resp),
    }
}
