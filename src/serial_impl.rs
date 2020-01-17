use std;
use serialport;

use std::ops::DerefMut;

use {
    crate::CommunicationError,
    crate::Interface,
    crate::BaudRate,
};

impl From<serialport::Error> for CommunicationError {
    fn from(e: serialport::Error) -> CommunicationError {
        match e {
            serialport::Error{kind: serialport::ErrorKind::Io(std::io::ErrorKind::TimedOut), ..} => CommunicationError::TimedOut,
            _ => CommunicationError::Other,
        }
    }
}

impl Interface for std::boxed::Box<dyn serialport::SerialPort> {
    fn set_baud_rate(&mut self, b: BaudRate) -> Result<(), CommunicationError> {
        match serialport::SerialPort::set_baud_rate(self.deref_mut(), u32::from(b)) {
            Ok(_) => Ok(()),
            Err(_) => Err(CommunicationError::UnsupportedBaud(b)),
        }
    }

    fn flush(&mut self) {
        let mut buf = Vec::new();
        let _res = self.read_to_end(&mut buf);
    }

    fn read(&mut self, data: &mut [u8]) -> Result<(), CommunicationError> {
        self.set_timeout(std::time::Duration::new(0, 100000000))?;
        Ok(std::io::Read::read_exact(self, data)?)
    }

    fn write(&mut self, data: &[u8]) -> Result<(), CommunicationError> {
        self.set_timeout(std::time::Duration::new(0, 100000000))?;
        Ok(std::io::Write::write_all(self, data)?)
    }
}
