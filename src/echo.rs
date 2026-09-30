//! Temporal EchoGuard for single-wire half-duplex physical lines (L9613 / L9637D / MC33660).

use std::time::{Duration, Instant};

#[derive(Debug)]
pub struct EchoGuard {
    baud_rate: u32,
    tx_in_flight: Vec<u8>,
    window_deadline: Option<Instant>,
}

impl EchoGuard {
    pub fn new(baud_rate: u32) -> Self {
        Self {
            baud_rate,
            tx_in_flight: Vec::new(),
            window_deadline: None,
        }
    }

    pub fn record_tx(&mut self, data: &[u8]) {
        self.tx_in_flight.extend_from_slice(data);
        let byte_us = 10_000_000u64.div_ceil(self.baud_rate as u64);
        let total_us = (data.len() as u64) * byte_us + 30_000; // tx duration + 30ms margin for OS thread jitter
        self.window_deadline = Some(Instant::now() + Duration::from_micros(total_us));
    }

    pub fn filter_rx(&mut self, incoming: &[u8]) -> Vec<u8> {
        if self.tx_in_flight.is_empty() {
            return incoming.to_vec();
        }

        if let Some(deadline) = self.window_deadline {
            if Instant::now() > deadline {
                self.tx_in_flight.clear();
                self.window_deadline = None;
                return incoming.to_vec();
            }
        }

        let mut filtered = Vec::with_capacity(incoming.len());
        for &byte in incoming {
            if let Some(expected) = self.tx_in_flight.first() {
                if byte == *expected {
                    self.tx_in_flight.remove(0);
                    continue;
                }
            }
            filtered.push(byte);
        }

        if self.tx_in_flight.is_empty() {
            self.window_deadline = None;
        }

        filtered
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_echo_suppression_matching() {
        let mut guard = EchoGuard::new(10400);
        guard.record_tx(&[0x68, 0x6A, 0xF1]);
        let incoming = [0x68, 0x6A, 0xF1, 0x48, 0x6B];
        let legitimate = guard.filter_rx(&incoming);
        assert_eq!(legitimate, vec![0x48, 0x6B]);
    }
}
