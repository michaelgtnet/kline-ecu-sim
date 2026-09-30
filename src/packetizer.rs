//! Dynamic silence-based packetizer for ISO 9141 and ISO 14230 frames.

use std::time::{Duration, Instant};

#[derive(Debug)]
pub struct SilencePacketizer {
    buffer: Vec<u8>,
    silence_threshold: Duration,
    max_chunk_size: usize,
    last_rx_time: Option<Instant>,
}

impl SilencePacketizer {
    pub fn new(baud: u32, base_silence_ms: f32) -> Self {
        let byte_time_us = 10_000_000u64.div_ceil(baud as u64);
        let min_silence_us = (byte_time_us as f64 * 1.8) as u64;
        let config_silence_us = (base_silence_ms * 1000.0) as u64;
        let chosen_silence_us = config_silence_us.max(min_silence_us);

        Self {
            buffer: Vec::with_capacity(1024),
            silence_threshold: Duration::from_micros(chosen_silence_us),
            max_chunk_size: 256,
            last_rx_time: None,
        }
    }

    pub fn push(&mut self, bytes: &[u8]) -> Option<Vec<u8>> {
        if bytes.is_empty() {
            return None;
        }

        self.last_rx_time = Some(Instant::now());
        self.buffer.extend_from_slice(bytes);

        if self.buffer.len() >= self.max_chunk_size {
            Some(self.flush())
        } else {
            None
        }
    }

    pub fn check_timeout(&mut self) -> Option<Vec<u8>> {
        if let Some(last_rx) = self.last_rx_time {
            if Instant::now().duration_since(last_rx) >= self.silence_threshold && !self.buffer.is_empty() {
                return Some(self.flush());
            }
        }
        None
    }

    pub fn time_until_silence(&self) -> Option<Duration> {
        let last_rx = self.last_rx_time?;
        if self.buffer.is_empty() {
            return None;
        }

        let elapsed = Instant::now().saturating_duration_since(last_rx);
        if elapsed >= self.silence_threshold {
            Some(Duration::ZERO)
        } else {
            Some(self.silence_threshold - elapsed)
        }
    }

    pub fn flush(&mut self) -> Vec<u8> {
        self.last_rx_time = None;
        std::mem::take(&mut self.buffer)
    }

    pub fn is_empty(&self) -> bool {
        self.buffer.is_empty()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_packetizer_flush() {
        let mut p = SilencePacketizer::new(10400, 1.8);
        p.push(&[0x01, 0x02, 0x03]);
        assert_eq!(p.flush(), vec![0x01, 0x02, 0x03]);
        assert!(p.is_empty());
    }
}
