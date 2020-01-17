//! Dynamixel Protocol 1
//!
//! Documentation can be found in: http://support.robotis.com/en/product/actuator/dynamixel/dxl_communication.html

use crate::BaudRate;
use crate::CommunicationError;
use log::warn;

use bit_field::BitField;

#[macro_use]
mod control_table;
pub(crate) mod instruction;
mod checksum;

/// Enumerate all protocol 1 servos connected to the interface
/// This functions returns a Vec and thus requires the `std` feature.
#[cfg(feature="std")]
pub fn enumerate<I: crate::Interface>(interface: &mut I) -> Result<Vec<ServoInfo>, CommunicationError> {
    let mut servos = Vec::new();

    for b in vec![&BaudRate::Baud1000000] {

        if let Err(_) = interface.set_baud_rate(*b) {
            warn!(target: "protocol1", "not able to enumerate devices on baudrate: {}", u32::from(*b));
        }

        interface.flush();
        let ping = crate::protocol1::instruction::Ping::new(PacketID::Broadcast);
        interface.write(&crate::protocol1::Instruction::serialize(&ping))?;

        loop {
            let mut received_data = [0u8; 6];

            match interface.read(&mut received_data) {
                Ok(_) => (),
                Err(CommunicationError::TimedOut) => break,
                Err(e) => return Err(e),
            };

            match <crate::protocol1::instruction::Pong as crate::protocol1::Status>::deserialize(&received_data) {
                Ok(pong) => {
                    interface.flush();
                    let read_model = crate::protocol1::instruction::ReadData::<GenericModelNumber>::new(crate::protocol1::PacketID::from(pong.id));
                    interface.write(&crate::protocol1::Instruction::serialize(&read_model))?;
                    let mut received_data_model = [0u8; 8];
                    interface.read(&mut received_data_model)?;

                    let model_number = match <crate::protocol1::instruction::ReadDataResponse<GenericModelNumber> as crate::protocol1::Status>::deserialize(&received_data_model) {
                        Ok(response) => response.data.value(),
                        Err(e) => {
                            warn!(target: "protocol1", "Found servo with baud: {} and id: {}. Could not resolve model number due to error: {:?}", u32::from(*b), u8::from(pong.id), e);
                            continue;
                        },
                    };
                                            
                    servos.push(
                        ServoInfo{
                            baud_rate: *b,
                            model_number: model_number,
                            id: pong.id,
                        });
                },
                Err(e) => {
                    warn!(target: "protocol1", "received error: {:?} when waiting for enumeration on baud: {}", e, u32::from(*b));
                    continue;
                },
            };
        }   
    }
    Ok(servos)
}

/// Connect genericly to a protocol 2 servo
///
/// Only offers basic functionality. If you need more functionality use the connect method of the correct servo type instead.
/// This functions returns a Boxed trait and this requires the `std` feature.
#[cfg(feature="std")]
pub(crate) fn connect<I: crate::Interface + 'static>(_interface: &mut I, info: ServoInfo) -> Result<Box<dyn (crate::Servo<I>)>, CommunicationError>{
    match info.model_number {
        crate::dynamixel::mx28::MX28::<I>::MODEL_NUMBER => Ok(Box::new(crate::dynamixel::mx28::MX28::<I>::new(info.id, info.baud_rate))),
        crate::dynamixel::ax12::AX12::<I>::MODEL_NUMBER => Ok(Box::new(crate::dynamixel::ax12::AX12::<I>::new(info.id, info.baud_rate))),
        _ => unimplemented!(),
    }
}


macro_rules! protocol1_servo {
    ($name:ident, $write:path, $read:path, $model_number:expr) => {
        pub struct $name<I: crate::Interface> {
            id: crate::protocol1::ServoID,
            baudrate: crate::BaudRate,
            interface: crate::lib::marker::PhantomData<I>,
        }
        
        impl<I: crate::Interface> $name<I> {
            pub const MODEL_NUMBER: u16 = $model_number;

            /// Creates a new servo without `ping`ing or taking any other measure to make sure it exists.
            pub fn new(id: crate::protocol1::ServoID, baudrate: crate::BaudRate) -> Self {
                $name{
                    id: id,
                    baudrate: baudrate,
                    interface: crate::lib::marker::PhantomData{},
                }
            }
            
            fn read_response(&mut self, interface: &mut I, data: &mut [u8]) -> Result<usize, crate::CommunicationError> {
                // first read header
                interface.read(&mut data[..4])?;

                // then read rest of message depending on header length
                let length = data[3] as usize;
                interface.read(&mut data[4..4+length])?;
                Ok(4+length)
            }

            /// Ping the servo, returning `Ok(())` if it exists.
            pub fn ping(&mut self, interface: &mut I) -> Result<(), crate::protocol1::Error> {
                interface.set_baud_rate(self.baudrate)?;
                interface.flush();

                let ping = crate::protocol1::instruction::Ping::new(crate::protocol1::PacketID::from(self.id));
                interface.write(&crate::protocol1::Instruction::serialize(&ping))?;
                let mut received_data = [0u8; 6];
                self.read_response(interface, &mut received_data)?;
                <crate::protocol1::instruction::Pong as crate::protocol1::Status>::deserialize(&received_data)?;
                Ok(())
            }
            
            /// Write the given data `register` to the servo.
            pub fn write_data<W: $write>(&mut self, interface: &mut I, register: W) -> Result<(), crate::protocol1::Error> {
                interface.set_baud_rate(self.baudrate)?;
                interface.flush();
                let write = crate::protocol1::instruction::WriteData::new(crate::protocol1::PacketID::from(self.id), register);
                interface.write(&crate::protocol1::Instruction::serialize(&write)[0..<crate::protocol1::instruction::WriteData<W> as crate::protocol1::Instruction>::LENGTH as usize + 4])?;
                let mut received_data = [0u8; 11];
                let length = self.read_response(interface, &mut received_data)?;
                match <crate::protocol1::instruction::WriteDataResponse as crate::protocol1::Status>::deserialize(&received_data[0..length]) {
                    Ok(crate::protocol1::instruction::WriteDataResponse{id: _}) => Ok(()),
                    Err(e) => Err(e),
                }
            }
            
            pub fn read_data<R: $read>(&mut self, interface: &mut I) -> Result<R, crate::protocol1::Error> {
                interface.set_baud_rate(self.baudrate)?;
                interface.flush();

                let read = crate::protocol1::instruction::ReadData::<R>::new(crate::protocol1::PacketID::from(self.id));
                interface.write(&crate::protocol1::Instruction::serialize(&read))?;
                let mut received_data = [0u8; 20];
                let length = self.read_response(interface, &mut received_data)?;
                match <crate::protocol1::instruction::ReadDataResponse<R> as crate::protocol1::Status>::deserialize(&received_data[0..length]) {
                    Ok(r) => Ok(r.data),
                    Err(e) => Err(e),
                }
            }
        }
    };
}


pub trait Register {
    const SIZE: u8;
    const ADDRESS: u8;
}
    
pub trait ReadRegister: Register {
    fn deserialize(data: &[u8]) -> Self;
}

pub trait WriteRegister: Register {
    // TODO: change 4 to Self::SIZE when const generics land
    fn serialize(&self) -> [u8; 4];
}

pub(crate) trait Instruction {
    // The array type is no longer needed when const generics land
    // replace with [u8; Self::LENGTH]
    type Array;
    const LENGTH: u8;
    const INSTRUCTION_VALUE: u8;

    // Serialize can be implemented generically once const generics land
    fn serialize(&self) -> Self::Array;
}

pub(crate) trait Status {
    const LENGTH: u8;

    fn deserialize_parameters(id: ServoID, parameters: &[u8]) -> Self;
    
    fn deserialize(data: &[u8]) -> Result<Self, Error>
        where Self: Sized {
        // check for formating error stuff
        
        // check for processing errors
        if let Some(error) = ProcessingError::decode(data[4]).map_err(|()| Error::Format(FormatError::InvalidError))? {
            return Err(Error::Processing(error));
        }
        
        let length = data[3];
        if length != Self::LENGTH {
            return Err(Error::Format(FormatError::Length));
        }

        let id = ServoID::new(data[2]);
        
        let parameters_range = 5..(5 + Self::LENGTH as usize - 2);
        Ok( Self::deserialize_parameters(id, &data[parameters_range]) )
    }
}

/// All information needed to connect to a protocol 1 servo
#[derive(Debug, Clone)]
pub struct ServoInfo {
    baud_rate: crate::BaudRate,
    model_number: u16,
    id: ServoID,
}

   
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum Error {
    Communication(crate::CommunicationError),
    Format(FormatError),
    Processing(ProcessingError),
}

impl From<crate::protocol1::Error> for crate::Error {
    fn from(e: crate::protocol1::Error) -> crate::Error {
        match e {
            crate::protocol1::Error::Communication(ce) => crate::Error::Communication(ce),
            crate::protocol1::Error::Format(_) => crate::Error::Format,
            crate::protocol1::Error::Processing(_) => crate::Error::Processing,
        }
    }
}

impl From<crate::CommunicationError> for Error {
    fn from(e: crate::CommunicationError) -> Error {
        Error::Communication(e)
    }
}

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum FormatError {
    ID,
    Header,
    CRC,
    Length,
    InvalidError,
}

#[derive(PartialEq, Eq, Clone, Copy)]
pub struct ProcessingError(u8);

impl From<ProcessingError> for u8 {
    fn from(e: ProcessingError) -> u8 {
        e.0
    }
}

impl ProcessingError {
    pub fn decode(v: u8) -> Result<Option<Self>, ()> {
        if v == 0 {
            Ok(None)
        } else if v.get_bit(7) {
            Err(())
        } else {
            Ok(Some(ProcessingError(v)))
        }
    }
    
    pub fn instruction_error(&self) -> bool {
        self.0.get_bit(6)
    }

    pub fn overload_error(&self) -> bool {
        self.0.get_bit(5)
    }

    pub fn checksum_error(&self) -> bool {
        self.0.get_bit(4)
    }

    pub fn range_error(&self) -> bool {
        self.0.get_bit(3)
    }

    pub fn overheating_error(&self) -> bool {
        self.0.get_bit(3)
    }

    pub fn angle_limit_error(&self) -> bool {
        self.0.get_bit(1)
    }

    pub fn input_voltage_error(&self) -> bool {
        self.0.get_bit(0)
    }
}

impl crate::lib::fmt::Debug for ProcessingError {
    fn fmt(&self, f: &mut crate::lib::fmt::Formatter) -> crate::lib::fmt::Result {
        write!(f, "The current ProcessingError, {:?}, decodes to the following errors: [", self.0)?;
        if self.instruction_error() {write!(f, "instruction_error")?;}
        if self.overload_error() {write!(f, "overload_error")?;}
        if self.checksum_error() {write!(f, "checksum_error")?;}
        if self.range_error() {write!(f, "range_error")?;}
        if self.overheating_error() {write!(f, "overheating_error")?;}
        if self.angle_limit_error() {write!(f, "angle_limit_error")?;}
        if self.input_voltage_error() {write!(f, "input_voltage_error")?;}
        write!(f, "]")
    }
}

#[derive(Debug, PartialEq, Eq, Copy, Clone)]
pub struct ServoID(u8);

impl ServoID {
    pub fn new(id: u8) -> ServoID {
        assert!(id <= 253);
        ServoID(id)
    }
}

#[derive(Debug, PartialEq, Eq, Copy, Clone)]
pub(crate) enum PacketID {
    Unicast(ServoID),
    Broadcast,
}

impl PacketID {
    pub fn unicast(id: u8) -> PacketID {
        assert!(id <= 253);
        PacketID::Unicast(ServoID::new(id))
    }

    pub fn broadcast() -> PacketID {
        PacketID::Broadcast
    }
}

impl From<ServoID> for PacketID {
    fn from(id: ServoID) -> PacketID {
        PacketID::Unicast(id)
    }
}

impl From<PacketID> for u8 {
    fn from(id: PacketID) -> u8 {
        match id {
            PacketID::Unicast(x) => u8::from(x),
            PacketID::Broadcast => 254,
        }
    }
}

impl From<ServoID> for u8 {
    fn from(id: ServoID) -> u8 {
        id.0
    }
}

/// This is the model number register for all protocol 1 servos.
struct GenericModelNumber(u16);

impl GenericModelNumber {
    fn value(&self) -> u16 {
        self.0
    }
}
                      
impl Register for GenericModelNumber {
    const SIZE: u8 =  2;
    const ADDRESS: u8 = 0x00;
}
    
impl ReadRegister for GenericModelNumber {
    fn deserialize(bytes: &[u8]) -> Self {
        assert_eq!(bytes.len(), 2);
        GenericModelNumber(bytes[0] as u16 | (bytes[1] as u16) << 8)        
    }
}
