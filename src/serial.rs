//! Hardware UART abstraction with Linux termios2 (BOTHER) custom baud support (10400 baud).

use std::io::{self, Read, Write};

#[cfg(target_os = "linux")]
use std::os::unix::io::AsRawFd;

pub trait PhysicalSerialPort: Read + Write + Send + 'static {
    fn set_line_break(&self, active: bool) -> io::Result<()>;
}

#[cfg(target_os = "linux")]
mod linux {
    use super::*;
    use std::fs::{File, OpenOptions};
    use std::os::raw::{c_int, c_ulong};

    #[repr(C)]
    pub struct Termios2 {
        pub c_iflag: u32,
        pub c_oflag: u32,
        pub c_cflag: u32,
        pub c_lflag: u32,
        pub c_line: u8,
        pub c_cc: [u8; 19],
        pub c_ispeed: u32,
        pub c_ospeed: u32,
    }

    const TCGETS2: c_ulong = 0x802C542A;
    const TCSETS2: c_ulong = 0x402C542B;
    const BOTHER: u32 = 0x00001000;
    const CBAUD: u32 = 0x0000100F;
    const CS8: u32 = 0x00000030;
    const CREAD: u32 = 0x00000080;
    const CLOCAL: u32 = 0x00000800;

    const TIOCSBRK: c_ulong = 0x5427;
    const TIOCCBRK: c_ulong = 0x5428;

    extern "C" {
        fn ioctl(fd: c_int, request: c_ulong, ...) -> c_int;
    }

    pub struct LinuxSerialPort {
        file: File,
        fd: c_int,
    }

    impl LinuxSerialPort {
        pub fn open(device: &str, baud: u32) -> io::Result<Self> {
            let file = OpenOptions::new()
                .read(true)
                .write(true)
                .open(device)?;
            let fd = file.as_raw_fd();

            unsafe {
                let mut tio: Termios2 = std::mem::zeroed();
                if ioctl(fd, TCGETS2, &mut tio) < 0 {
                    return Err(io::Error::last_os_error());
                }

                tio.c_iflag = 0;
                tio.c_oflag = 0;
                tio.c_lflag = 0;
                tio.c_cflag &= !CBAUD;
                tio.c_cflag |= BOTHER | CS8 | CREAD | CLOCAL;

                tio.c_ispeed = baud;
                tio.c_ospeed = baud;

                if ioctl(fd, TCSETS2, &tio) < 0 {
                    return Err(io::Error::last_os_error());
                }
            }

            Ok(Self { file, fd })
        }

        pub fn try_clone(&self) -> io::Result<Self> {
            let file = self.file.try_clone()?;
            let fd = file.as_raw_fd();
            Ok(Self { file, fd })
        }
    }

    impl Read for LinuxSerialPort {
        fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
            self.file.read(buf)
        }
    }

    impl Write for LinuxSerialPort {
        fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
            self.file.write(buf)
        }

        fn flush(&mut self) -> io::Result<()> {
            self.file.flush()
        }
    }

    impl PhysicalSerialPort for LinuxSerialPort {
        fn set_line_break(&self, active: bool) -> io::Result<()> {
            unsafe {
                let req = if active { TIOCSBRK } else { TIOCCBRK };
                if ioctl(self.fd, req) < 0 {
                    return Err(io::Error::last_os_error());
                }
            }
            Ok(())
        }
    }
}

#[cfg(target_os = "linux")]
pub use linux::LinuxSerialPort as NativeSerialPort;

#[cfg(not(target_os = "linux"))]
mod host_mock {
    use super::*;

    pub struct MockSerialPort {
        pub buffer: Vec<u8>,
        pub break_state: bool,
    }

    impl MockSerialPort {
        pub fn open(_device: &str, _baud: u32) -> io::Result<Self> {
            Ok(Self {
                buffer: Vec::new(),
                break_state: false,
            })
        }

        pub fn try_clone(&self) -> io::Result<Self> {
            Ok(Self {
                buffer: self.buffer.clone(),
                break_state: self.break_state,
            })
        }
    }

    impl Read for MockSerialPort {
        fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
            if self.buffer.is_empty() {
                // Non-blocking simulation
                return Ok(0);
            }
            let to_read = buf.len().min(self.buffer.len());
            buf[..to_read].copy_from_slice(&self.buffer[..to_read]);
            self.buffer.drain(..to_read);
            Ok(to_read)
        }
    }

    impl Write for MockSerialPort {
        fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
            self.buffer.extend_from_slice(buf);
            Ok(buf.len())
        }

        fn flush(&mut self) -> io::Result<()> {
            Ok(())
        }
    }

    impl PhysicalSerialPort for MockSerialPort {
        fn set_line_break(&self, _active: bool) -> io::Result<()> {
            Ok(())
        }
    }
}

#[cfg(not(target_os = "linux"))]
pub use host_mock::MockSerialPort as NativeSerialPort;
