//! 5-baud Slow Init detector for physical K-Line (ISO 9141-2 / ISO 14230-4 / KWP1281).
//!
//! On physical half-duplex transceivers (L9613 / L9637D), a 5-baud address byte
//! (200ms per bit) received by a 10400-baud UART produces break/framing errors
//! read as `0x00` bytes during LOW periods.
//!
//! This detector identifies the burst sequence (2 or 3 pulses spaced by 200-800ms)
//! and schedules the `[0x55, KB1, KB2]` synchronization response after the 5-baud
//! transmission ends.

use std::time::{Duration, Instant};

pub const MIN_GAP: Duration = Duration::from_millis(150);
pub const MAX_GAP: Duration = Duration::from_millis(2500);
pub const RESPONSE_DELAY: Duration = Duration::from_millis(150);

#[derive(Debug, Default)]
pub struct SlowInitDetector {
    pulses: Vec<Instant>,
}

impl SlowInitDetector {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn reset(&mut self) {
        self.pulses.clear();
    }

    /// Record a received 0x00 break pulse in Idle state.
    /// Returns the target Instant and target address byte (e.g. 0x33 or 0x01)
    /// to transmit `[0x55, KB1, KB2]` once detected.
    pub fn push(&mut self, now: Instant) -> Option<(Instant, u8)> {
        if let Some(&last) = self.pulses.last() {
            let gap = now.saturating_duration_since(last);
            if gap < MIN_GAP {
                return None; // Same burst, fragmented into multiple reads
            }
            if gap > MAX_GAP {
                self.pulses.clear(); // Previous attempt timed out; start fresh
            }
        }
        self.pulses.push(now);
        self.pulses.retain(|t| now.saturating_duration_since(*t) <= MAX_GAP);

        // When 3 pulses arrive (standard 5-baud 0x33 sequence: Start, D2-D3, D6-D7)
        if self.pulses.len() >= 3 {
            self.pulses.clear();
            return Some((now + RESPONSE_DELAY, 0x33));
        }

        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_x100_wake_pattern_detection() {
        let mut det = SlowInitDetector::new();
        let t0 = Instant::now();
        assert_eq!(det.push(t0), None);
        assert_eq!(det.push(t0 + Duration::from_millis(800)), None);
        let fire = det.push(t0 + Duration::from_millis(1600));
        assert_eq!(fire, Some((t0 + Duration::from_millis(1600) + RESPONSE_DELAY, 0x33)));
    }

    #[test]
    fn test_fragmented_burst_debouncing() {
        let mut det = SlowInitDetector::new();
        let t0 = Instant::now();
        assert_eq!(det.push(t0), None);
        assert_eq!(det.push(t0 + Duration::from_millis(5)), None); // debounced
        assert_eq!(det.push(t0 + Duration::from_millis(800)), None);
        assert_eq!(det.push(t0 + Duration::from_millis(805)), None); // debounced
        assert!(det.push(t0 + Duration::from_millis(1600)).is_some());
    }

    #[test]
    fn test_stale_pulses_reset() {
        let mut det = SlowInitDetector::new();
        let t0 = Instant::now();
        assert_eq!(det.push(t0), None);
        assert_eq!(det.push(t0 + Duration::from_secs(5)), None); // stale reset
        assert_eq!(det.push(t0 + Duration::from_millis(5800)), None);
        assert!(det.push(t0 + Duration::from_millis(6600)).is_some());
    }

    #[test]
    fn test_manual_reset() {
        let mut det = SlowInitDetector::new();
        let t0 = Instant::now();
        det.push(t0);
        det.push(t0 + Duration::from_millis(800));
        det.reset();
        assert_eq!(det.push(t0 + Duration::from_millis(1600)), None);
    }
}
