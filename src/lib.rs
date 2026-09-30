//! kline-ecu-sim core library

pub mod ecu;
pub mod echo;
pub mod packetizer;
pub mod serial;

pub use ecu::{DiagnosticData, EcuProfile, EcuSimulator, EcuState};
pub use echo::EchoGuard;
pub use packetizer::SilencePacketizer;
pub use serial::{NativeSerialPort, PhysicalSerialPort};
