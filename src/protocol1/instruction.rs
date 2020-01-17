use crate::protocol1::*;
use crate::protocol1::ServoID;

pub(crate) struct Ping {
    pub id: PacketID,
}

impl Ping {
    pub fn new(id: PacketID) -> Self {
        Ping{id: id}
    }
}

impl Instruction for Ping {
    type Array = [u8; 6];
    const LENGTH: u8 = 2;
    const INSTRUCTION_VALUE: u8 = 0x01;

    fn serialize(&self) -> [u8; 6] {
        let mut array = [0xff, 0xff, u8::from(self.id), Self::LENGTH, Self::INSTRUCTION_VALUE, 0x00];
        array[5] = u8::from(checksum::Checksum::calc(&array[2..5]));
        array
    }
}

#[derive(Debug, PartialEq, Eq, Clone)]
pub(crate) struct Pong {
    pub id: ServoID,
}

impl Status for Pong {
    const LENGTH: u8 = 2;

    fn deserialize_parameters(id: ServoID, parameters: &[u8]) -> Self {
        assert_eq!(parameters.len(), 0);
        Pong {id: id}
    }
}


pub(crate) struct WriteData<T: WriteRegister> {
    pub id: PacketID,
    pub data: T,
}

impl<T: WriteRegister> WriteData<T> {
    pub fn new(id: PacketID, data: T) -> Self {
        WriteData{id: id, data: data}
    }
}

impl<T: WriteRegister> Instruction for WriteData<T>{
    // Use max size (4) untill const generics land
    type Array = [u8; 11];
    const LENGTH: u8 = 3 + T::SIZE;
    const INSTRUCTION_VALUE: u8 = 0x03;

    fn serialize(&self) -> [u8; 11] {
        let mut array = [0xff, 0xff, u8::from(self.id), Self::LENGTH, Self::INSTRUCTION_VALUE, T::ADDRESS, 0x00, 0x00, 0x00, 0x00, 0x00];
        let data = self.data.serialize();
        for i in 0..T::SIZE as usize {
            array[6+i] = data[i];
        }
        array[6+T::SIZE as usize] = u8::from(checksum::Checksum::calc(&array[2..(6+T::SIZE) as usize]));
        array
    }
}

pub(crate) struct WriteDataResponse {
    pub id: ServoID,
}

impl Status for WriteDataResponse {
    const LENGTH: u8 = 2;
    
    fn deserialize_parameters(id: ServoID, parameters: &[u8]) -> Self {
        assert_eq!(parameters.len(), 0);
        WriteDataResponse {id: id}
    }
}

pub(crate) struct ReadData<T: ReadRegister> {
    pub id: PacketID,
    reg: crate::lib::marker::PhantomData<T>,
}

impl<T: ReadRegister> ReadData<T> {
    pub(crate) fn new(id: PacketID) -> Self {
        ReadData {
            id: id,
            reg: crate::lib::marker::PhantomData{},
        }
    }
}

impl<T: ReadRegister> Instruction for ReadData<T>{
    type Array = [u8; 8];
    const LENGTH: u8 = 4;
    const INSTRUCTION_VALUE: u8 = 0x02;

    fn serialize(&self) -> [u8; 8] {
        let mut array = [0xff, 0xff, u8::from(self.id), Self::LENGTH, Self::INSTRUCTION_VALUE, T::ADDRESS, T::SIZE, 0x00];
        array[7] = u8::from(checksum::Checksum::calc(&array[2..7]));
        array
    }
}


pub(crate) struct ReadDataResponse<T: ReadRegister> {
    pub id: ServoID,
    pub data: T,
}

impl<T: ReadRegister> Status for ReadDataResponse<T> {
    // Use max size (4) untill const generics land
    const LENGTH: u8 = 2 + T::SIZE;
    
    fn deserialize_parameters(id: ServoID, parameters: &[u8]) -> Self {
        assert_eq!(parameters.len(), T::SIZE as usize);
        ReadDataResponse {id: id, data: T::deserialize(parameters)}
    }
}





#[cfg(test)]
mod tests {
    // Using the same test case that can be found at:
    // http://support.robotis.com/en/product/actuator/dynamixel/communication/dxl_instruction.htm
    
    use crate::protocol1::*;
    use crate::protocol1::instruction::*;

    #[test]
    fn test_ping() {
        assert_eq!(Ping::new(PacketID::unicast(1)).serialize(), [0xff, 0xff, 0x01, 0x02, 0x01, 0xfb]);
    }
    
    #[test]
    fn test_pong() {
        assert_eq!(Pong::deserialize(&[0xff, 0xff, 0x01, 0x02, 0x00, 0x00]), //todo: fix checksum
                   Ok(Pong{id: ServoID::new(1)})
        );
    }
    
    #[test]
    fn test_write() {
        assert_eq!(WriteData::new(PacketID::unicast(1), crate::dynamixel::ax12::control_table::GoalPosition::new(0x123)).serialize(), [0xff, 0xff, 0x01, 0x05, 0x03, 30, 0x23, 0x01, 180, 0x00, 0x00]);
        assert_eq!(WriteData::new(PacketID::broadcast(), crate::dynamixel::ax12::control_table::GoalPosition::new(0x123)).serialize(), [0xff, 0xff, 0xfe, 0x05, 0x03, 30, 0x23, 0x01, 183, 0x00, 0x00]);
    }

    #[test]
    fn test_read() {
        assert_eq!(ReadData::<crate::dynamixel::ax12::control_table::PresentPosition>::new(PacketID::unicast(1)).serialize(), [0xff, 0xff, 0x01, 0x04, 0x02, 36, 0x2, 210]);
        assert_eq!(ReadData::<crate::dynamixel::ax12::control_table::PresentPosition>::new(PacketID::broadcast()).serialize(), [0xff, 0xff, 0xfe, 0x04, 0x02, 36, 0x2, 213]);
    }
}
