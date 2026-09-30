//! kline-ecu-sim core library

pub mod ecu;
pub mod echo;
pub mod led;
pub mod packetizer;
pub mod serial;
pub mod slowinit;

pub use ecu::{DiagnosticData, EcuProfile, EcuSimulator, EcuState};
pub use echo::EchoGuard;
pub use led::CarLed;
pub use packetizer::SilencePacketizer;
pub use serial::{NativeSerialPort, PhysicalSerialPort};
pub use slowinit::SlowInitDetector;
