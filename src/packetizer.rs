//! Dynamic silence and length-aware packetizer for ISO 9141 and ISO 14230 frames.

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
        let min_silence_us = (byte_time_us as f64 * 2.0) as u64;
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

        // Strip leading 0x00 (fast-init break pulses) if followed by real frame bytes
        while self.buffer.len() > 1 && self.buffer[0] == 0x00 {
            self.buffer.remove(0);
        }

        // Check if buffer contains a complete, verified ISO frame right now
        if let Some(frame_len) = Self::detect_complete_frame(&self.buffer) {
            self.last_rx_time = None;
            let frame: Vec<u8> = self.buffer.drain(..frame_len).collect();
            return Some(frame);
        }

        if self.buffer.len() >= self.max_chunk_size {
            Some(self.flush())
        } else {
            None
        }
    }

    pub fn is_partial_known_frame(&self) -> bool {
        if self.buffer.is_empty() {
            return false;
        }
        let fmt = self.buffer[0];
        // Valid KWP2000 format bytes sent by testers: 0x80 or 0xC1..0xCE
        if fmt == 0x80 || (fmt >= 0xC1 && fmt <= 0xCE) {
            let len_in_fmt = (fmt & 0x3F) as usize;
            let total_len = if len_in_fmt > 0 {
                3 + len_in_fmt + 1
            } else if self.buffer.len() >= 4 {
                4 + (self.buffer[3] as usize) + 1
            } else {
                5
            };
            return self.buffer.len() < total_len;
        }
        // ISO 9141-2 request header
        if (fmt == 0x68 || fmt == 0x48) && self.buffer.len() < 5 {
            return true;
        }
        false
    }

    pub fn current_threshold(&self) -> Duration {
        if self.is_partial_known_frame() {
            Duration::from_millis(60) // Allow up to 60ms for scanner to transmit rest of frame
        } else {
            self.silence_threshold
        }
    }

    pub fn check_timeout(&mut self) -> Option<Vec<u8>> {
        if let Some(last_rx) = self.last_rx_time {
            let threshold = self.current_threshold();
            if Instant::now().duration_since(last_rx) >= threshold && !self.buffer.is_empty() {
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

        let threshold = self.current_threshold();
        let elapsed = Instant::now().saturating_duration_since(last_rx);
        if elapsed >= threshold {
            Some(Duration::ZERO)
        } else {
            Some(threshold - elapsed)
        }
    }

    pub fn flush(&mut self) -> Vec<u8> {
        self.last_rx_time = None;
        std::mem::take(&mut self.buffer)
    }

    pub fn is_empty(&self) -> bool {
        self.buffer.is_empty()
    }

    /// Inspect buffer to see if a complete, valid ISO frame is already present.
    fn detect_complete_frame(buf: &[u8]) -> Option<usize> {
        if buf.is_empty() {
            return None;
        }

        let fmt = buf[0];

        // 1. ISO 14230 (KWP2000) with addressing (bits 7..6 != 0)
        if (fmt & 0xC0) != 0 {
            let len_in_fmt = (fmt & 0x3F) as usize;
            if len_in_fmt > 0 {
                // Header: [fmt, target, source] (3 bytes) + data (len_in_fmt) + checksum (1 byte)
                let total_len = 3 + len_in_fmt + 1;
                if buf.len() >= total_len {
                    let expected_csum = buf[..total_len - 1]
                        .iter()
                        .fold(0u8, |acc, &b| acc.wrapping_add(b));
                    if expected_csum == buf[total_len - 1] {
                        return Some(total_len);
                    }
                }
            } else {
                // fmt & 0x3F == 0 (e.g. 0x80):
                // Header: [fmt, target, source, len_byte] (4 bytes) + data (len_byte) + checksum (1 byte)
                if buf.len() >= 4 {
                    let len_byte = buf[3] as usize;
                    let total_len = 4 + len_byte + 1;
                    if buf.len() >= total_len {
                        let expected_csum = buf[..total_len - 1]
                            .iter()
                            .fold(0u8, |acc, &b| acc.wrapping_add(b));
                        if expected_csum == buf[total_len - 1] {
                            return Some(total_len);
                        }
                    }
                }
            }
        }

        // 2. ISO 9141-2 / SAE J1979 request headers (0x68, 0x48)
        if (fmt == 0x68 || fmt == 0x48) && buf.len() >= 5 {
            let expected_csum = buf[..buf.len() - 1]
                .iter()
                .fold(0u8, |acc, &b| acc.wrapping_add(b));
            if expected_csum == buf[buf.len() - 1] {
                return Some(buf.len());
            }
        }

        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_packetizer_flush() {
        let mut p = SilencePacketizer::new(10400, 15.0);
        p.push(&[0x01, 0x02, 0x03]);
        assert_eq!(p.flush(), vec![0x01, 0x02, 0x03]);
        assert!(p.is_empty());
    }

    #[test]
    fn test_fast_init_byte_by_byte_assembly() {
        let mut p = SilencePacketizer::new(10400, 15.0);
        // Fast init [C1, 33, F1, 81, 66] pushed one byte at a time
        assert_eq!(p.push(&[0xC1]), None);
        assert_eq!(p.push(&[0x33]), None);
        assert_eq!(p.push(&[0xF1]), None);
        assert_eq!(p.push(&[0x81]), None);
        assert_eq!(p.push(&[0x66]), Some(vec![0xC1, 0x33, 0xF1, 0x81, 0x66]));
        assert!(p.is_empty());
    }

    #[test]
    fn test_fast_init_with_leading_break() {
        let mut p = SilencePacketizer::new(10400, 15.0);
        // Break pulse 0x00 arrives first, then fast init bytes
        assert_eq!(p.push(&[0x00]), None);
        assert_eq!(p.push(&[0xC1, 0x33]), None);
        assert_eq!(p.push(&[0xF1, 0x81, 0x66]), Some(vec![0xC1, 0x33, 0xF1, 0x81, 0x66]));
        assert!(p.is_empty());
    }
}
