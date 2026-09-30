//! Hardware LED indicator for ECU responses (Car LED on bench).

use std::fs::{File, OpenOptions};
use std::io::Write;
use std::time::{Duration, Instant};

pub struct CarLed {
    value_file: Option<File>,
    deadline: Option<Instant>,
    stretch: Duration,
}

impl CarLed {
    pub fn open(gpio_pin: Option<u32>) -> Self {
        let Some(pin) = gpio_pin else {
            return Self {
                value_file: None,
                deadline: None,
                stretch: Duration::from_millis(60),
            };
        };

        // Export GPIO if needed
        let _ = std::fs::write("/sys/class/gpio/export", pin.to_string());
        let dir_path = format!("/sys/class/gpio/gpio{}/direction", pin);
        let _ = std::fs::write(&dir_path, "out");

        let val_path = format!("/sys/class/gpio/gpio{}/value", pin);
        let file = OpenOptions::new().write(true).open(&val_path).ok();

        Self {
            value_file: file,
            deadline: None,
            stretch: Duration::from_millis(60),
        }
    }

    pub fn pulse(&mut self) {
        if let Some(ref mut f) = self.value_file {
            let _ = f.write_all(b"1\n");
            let _ = f.flush();
            self.deadline = Some(Instant::now() + self.stretch);
        }
    }

    pub fn service(&mut self) {
        if let Some(dl) = self.deadline {
            if Instant::now() >= dl {
                if let Some(ref mut f) = self.value_file {
                    let _ = f.write_all(b"0\n");
                    let _ = f.flush();
                }
                self.deadline = None;
            }
        }
    }
}
