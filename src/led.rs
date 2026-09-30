//! Hardware LED indicator for ECU responses (Car LED on bench).
//! Uses Linux GPIO Character Device (`/dev/gpiochip0`) via `gpio-cdev`.
//! Gracefully degrades to a no-op if the line is busy or unavailable.

use std::time::{Duration, Instant};
use tracing::{info, warn};

#[cfg(target_os = "linux")]
use gpio_cdev::{Chip, LineHandle, LineRequestFlags};

pub enum LedBackend {
    #[cfg(target_os = "linux")]
    Cdev(LineHandle),
    Virtual { state: bool },
    Disabled,
}

pub struct CarLed {
    backend: LedBackend,
    deadline: Option<Instant>,
    stretch: Duration,
    pin: Option<u32>,
}

impl CarLed {
    /// Opens the hardware GPIO line on /dev/gpiochip0 via chardev.
    /// If gpio_pin is None or claim fails (e.g. EBUSY), logs a warning and
    /// degrades gracefully to an inactive no-op instance.
    pub fn open(gpio_pin: Option<u32>) -> Self {
        let stretch = Duration::from_millis(60);
        let Some(pin) = gpio_pin else {
            return Self {
                backend: LedBackend::Disabled,
                deadline: None,
                stretch,
                pin: None,
            };
        };

        #[cfg(target_os = "linux")]
        {
            match Chip::new("/dev/gpiochip0") {
                Ok(mut chip) => match chip.get_line(pin) {
                    Ok(line) => match line.request(LineRequestFlags::OUTPUT, 0, "autolink-dir-led") {
                        Ok(handle) => {
                            info!("CarLed: claimed GPIO line {} on /dev/gpiochip0 (chardev)", pin);
                            Self {
                                backend: LedBackend::Cdev(handle),
                                deadline: None,
                                stretch,
                                pin: Some(pin),
                            }
                        }
                        Err(e) => {
                            warn!("CarLed: could not request GPIO line {} on /dev/gpiochip0: {}. Continuing without LED indicator.", pin, e);
                            Self {
                                backend: LedBackend::Disabled,
                                deadline: None,
                                stretch,
                                pin: Some(pin),
                            }
                        }
                    },
                    Err(e) => {
                        warn!("CarLed: could not get GPIO line {} on /dev/gpiochip0: {}. Continuing without LED indicator.", pin, e);
                        Self {
                            backend: LedBackend::Disabled,
                            deadline: None,
                            stretch,
                            pin: Some(pin),
                        }
                    }
                },
                Err(e) => {
                    warn!("CarLed: could not open /dev/gpiochip0: {}. Continuing without LED indicator.", e);
                    Self {
                        backend: LedBackend::Disabled,
                        deadline: None,
                        stretch,
                        pin: Some(pin),
                    }
                }
            }
        }

        #[cfg(not(target_os = "linux"))]
        {
            Self {
                backend: LedBackend::Virtual { state: false },
                deadline: None,
                stretch,
                pin: Some(pin),
            }
        }
    }

    /// Creates a virtual LED instance for unit testing.
    /// If `simulate_busy` is true, simulates a line claim failure (EBUSY).
    pub fn new_virtual(pin: u32, simulate_busy: bool) -> Self {
        let stretch = Duration::from_millis(60);
        if simulate_busy {
            warn!("CarLed: simulated line {} busy", pin);
            Self {
                backend: LedBackend::Disabled,
                deadline: None,
                stretch,
                pin: Some(pin),
            }
        } else {
            Self {
                backend: LedBackend::Virtual { state: false },
                deadline: None,
                stretch,
                pin: Some(pin),
            }
        }
    }

    pub fn pin(&self) -> Option<u32> {
        self.pin
    }

    pub fn is_available(&self) -> bool {
        match self.backend {
            #[cfg(target_os = "linux")]
            LedBackend::Cdev(_) => true,
            LedBackend::Virtual { .. } => true,
            LedBackend::Disabled => false,
        }
    }

    pub fn is_active(&self) -> bool {
        match &self.backend {
            #[cfg(target_os = "linux")]
            LedBackend::Cdev(handle) => handle.get_value().map(|v| v != 0).unwrap_or(false),
            LedBackend::Virtual { state } => *state,
            LedBackend::Disabled => false,
        }
    }

    pub fn pulse(&mut self) {
        match &mut self.backend {
            #[cfg(target_os = "linux")]
            LedBackend::Cdev(handle) => {
                let _ = handle.set_value(1);
                self.deadline = Some(Instant::now() + self.stretch);
            }
            LedBackend::Virtual { state } => {
                *state = true;
                self.deadline = Some(Instant::now() + self.stretch);
            }
            LedBackend::Disabled => {}
        }
    }

    pub fn time_until_service(&self) -> Option<Duration> {
        self.deadline.map(|dl| {
            let now = Instant::now();
            if now >= dl {
                Duration::ZERO
            } else {
                dl - now
            }
        })
    }

    pub fn service(&mut self) {
        if let Some(dl) = self.deadline {
            if Instant::now() >= dl {
                match &mut self.backend {
                    #[cfg(target_os = "linux")]
                    LedBackend::Cdev(handle) => {
                        let _ = handle.set_value(0);
                    }
                    LedBackend::Virtual { state } => {
                        *state = false;
                    }
                    LedBackend::Disabled => {}
                }
                self.deadline = None;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::thread::sleep;

    #[test]
    fn test_car_led_none_is_noop() {
        let mut led = CarLed::open(None);
        assert!(!led.is_available());
        assert!(!led.is_active());
        led.pulse();
        assert!(!led.is_active());
        led.service();
        assert!(!led.is_active());
    }

    #[test]
    fn test_car_led_pulse_and_service() {
        let mut led = CarLed::new_virtual(0, false);
        assert!(led.is_available());
        assert!(!led.is_active());

        // Pulse -> line becomes active
        led.pulse();
        assert!(led.is_active());

        // Before 60ms deadline -> remains active
        sleep(Duration::from_millis(10));
        led.service();
        assert!(led.is_active());

        // After 60ms deadline -> turns inactive
        sleep(Duration::from_millis(55));
        led.service();
        assert!(!led.is_active());
    }

    #[test]
    fn test_car_led_busy_fails_gracefully() {
        // Simulates line claim failure (busy or permission error)
        let mut led = CarLed::new_virtual(0, true);
        assert!(!led.is_available());
        assert!(!led.is_active());

        // Pulse and service must be safe no-ops
        led.pulse();
        assert!(!led.is_active());
        led.service();
        assert!(!led.is_active());
    }
}
