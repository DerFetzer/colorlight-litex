use litex_pac::UART;

pub struct Uart {
    pub registers: Option<UART>,
}

impl Uart {
    pub fn putc(&self, c: u8) {
        match self.registers.as_ref() {
            Some(reg) => unsafe {
                // Wait until TXFULL is `0`
                while reg.txfull.read().bits() != 0 {
                    ()
                }
                reg.rxtx.write(|w| w.rxtx().bits(c));
            },
            None => (),
        }
    }
}

use core::fmt::{Error, Write};

impl Write for Uart {
    fn write_str(&mut self, s: &str) -> Result<(), Error> {
        for c in s.bytes() {
            self.putc(c);
        }
        Ok(())
    }
}

#[macro_use]
#[cfg(not(test))]
pub mod print_hardware {
    use crate::print::*;
    pub static mut SUPERVISOR_UART: Uart = Uart { registers: None };

    pub fn set_hardware(uart: UART) {
        unsafe {
            SUPERVISOR_UART.registers = Some(uart);
        }
    }

    #[macro_export]
    macro_rules! print
    {
        ($($args:tt)+) => ({
                use core::fmt::Write;
                unsafe {
                    let _ = write!(crate::print::print_hardware::SUPERVISOR_UART, $($args)+);
                }
        });
    }
}

#[macro_export]
macro_rules! println
{
    () => ({
        print!("\r\n")
    });
    ($fmt:expr) => ({
        print!(concat!($fmt, "\r\n"))
    });
    ($fmt:expr, $($args:tt)+) => ({
        print!(concat!($fmt, "\r\n"), $($args)+)
    });
}

use log::{Level, LevelFilter, Metadata, Record, SetLoggerError};

pub struct UartLogger;

static LOGGER: UartLogger = UartLogger;

impl UartLogger {
    pub fn init() -> Result<(), SetLoggerError> {
        unsafe { log::set_logger_racy(&LOGGER).map(|()| log::set_max_level(LevelFilter::Debug)) }
    }
}

impl log::Log for UartLogger {
    fn enabled(&self, metadata: &Metadata) -> bool {
        metadata.level() <= Level::Info
    }

    fn log(&self, record: &Record) {
        if self.enabled(record.metadata()) {
            println!("{} - {}", record.level(), record.args());
        }
    }

    fn flush(&self) {}
}
