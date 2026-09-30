//! Hardware LED indicator for ECU responses (Car LED on bench).

use std::time::{Duration, Instant};

pub struct CarLed {
    val_path: Option<String>,
    deadline: Option<Instant>,
    stretch: Duration,
}

impl CarLed {
    pub fn open(gpio_pin: Option<u32>) -> Self {
        let Some(pin) = gpio_pin else {
            return Self {
                val_path: None,
                deadline: None,
                stretch: Duration::from_millis(60),
            };
        };

        // Export GPIO if needed
        let _ = std::fs::write("/sys/class/gpio/export", pin.to_string());
        let dir_path = format!("/sys/class/gpio/gpio{}/direction", pin);
        let _ = std::fs::write(&dir_path, "out");

        let val_path = format!("/sys/class/gpio/gpio{}/value", pin);
        let _ = std::fs::write(&val_path, "0");

        Self {
            val_path: Some(val_path),
            deadline: None,
            stretch: Duration::from_millis(60),
        }
    }

    pub fn pulse(&mut self) {
        if let Some(ref path) = self.val_path {
            let _ = std::fs::write(path, "1");
            self.deadline = Some(Instant::now() + self.stretch);
        }
    }

    pub fn service(&mut self) {
        if let Some(dl) = self.deadline {
            if Instant::now() >= dl {
                if let Some(ref path) = self.val_path {
                    let _ = std::fs::write(path, "0");
                }
                self.deadline = None;
            }
        }
    }
}
