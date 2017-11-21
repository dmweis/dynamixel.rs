use protocol2::{
    self,
    Register,
    PacketID,
    ReadRegister,
    WriteRegister,
    Instruction,
};

pub struct Ping {
    id: PacketID,
}

impl Ping {
    pub fn new(id: PacketID) -> Self {
        Ping{id: id}
    }
}

impl Instruction for Ping {
    type Array = [u8; 10];
    const LENGTH: u16 = 10;
    const INSTRUCTION_VALUE: u8 = 0x01;

    fn serialize(&self) -> [u8; 10] {
        let mut array = [0xff, 0xff, 0xfd, 0x00, u8::from(self.id), 0x03, 0x00, Self::INSTRUCTION_VALUE, 0x00, 0x00];
        let crc = u16::from(protocol2::crc::CRC::calc(&array[0..Self::LENGTH as usize - 2]));
        array[Self::LENGTH as usize - 2] = crc as u8;
        array[Self::LENGTH as usize - 1] = (crc >> 8) as u8;
        array
    }
}

pub struct Read<T: ReadRegister> {
    id: PacketID,
    data: T,
}

impl<T: ReadRegister> Instruction for Read<T> {
    type Array = [u8; 14];
    const LENGTH: u16 = 14;
    const INSTRUCTION_VALUE: u8 = 0x02;
}

pub struct Write<T: WriteRegister> {
    id: PacketID,
    data: T,
}

impl<T: WriteRegister> Instruction for Write<T>{
    // Use max size (4) untill const generics land
    type Array = [u8; 16];
    const LENGTH: u16 = 12 + T::SIZE;
    const INSTRUCTION_VALUE: u8 = 0x03;
}

pub struct FactoryReset {
    id: PacketID,
}

impl Instruction for FactoryReset {
    type Array = [u8; 11];
    const LENGTH: u16 = 11;
    const INSTRUCTION_VALUE: u8 = 0x06;
}

pub struct Reboot {
    id: PacketID,
}

impl Instruction for Reboot {
    type Array = [u8; 10];
    const LENGTH: u16 = 10;
    const INSTRUCTION_VALUE: u8 = 0x08;
}

#[cfg(test)]
mod tests {
    // Using the same test case that can be found at:
    // http://support.robotis.com/en/product/actuator/dynamixel_pro/communication/instruction_status_packet.htm
    
    use protocol2::*;
    use protocol2::instruction::*;

    #[test]
    fn test_ping() {
        assert_eq!(Ping::new(PacketID::unicast(1)).serialize(), [0xff, 0xff, 0xfd, 0x00, 0x01, 0x03, 0x00, 0x01, 0x19, 0x4e]);
        assert_eq!(Ping::new(PacketID::broadcast()).serialize(), [0xff, 0xff, 0xfd, 0x00, 0xfe, 0x03, 0x00, 0x01, 0x31, 0x42]);
    }
}
