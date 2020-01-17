use crate::protocol2::*;
// use crate::dynamixel::mx28::control_table;

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub(crate) struct Ping {
    id: PacketID,
}

impl Ping {
    pub fn new(id: PacketID) -> Self {
        Ping{id: id}
    }
}

impl Instruction for Ping {
    const PARAMETERS: u16 = 0;
    const INSTRUCTION_VALUE: u8 = 0x01;

    fn id(&self) -> PacketID {
        self.id
    }

    fn parameter(&self, _index: usize) -> u8 {
        panic!("No parameters exists for Ping");
    }
}

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub(crate) struct Pong {
    pub id: ServoID,
    pub model_number: u16,
    pub fw_version: u8,
}

impl Status for Pong {
    const PARAMETERS: u16 = 3;
    
    fn deserialize(id: ServoID, parameters: &[u8]) -> Self {
        assert_eq!(parameters.len(), 3);
        Pong {
            id: id,
            model_number: (parameters[0] as u16) | (parameters[1] as u16) << 8,
            fw_version: parameters[2],
        }
    }
}

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub(crate) struct Read<T: ReadRegister> {
    id: PacketID,
    phantom: crate::lib::marker::PhantomData<T>,
}

impl<T: ReadRegister> Read<T> {
    pub fn new(id: PacketID) -> Self {
        Read{id: id, phantom: crate::lib::marker::PhantomData}
    }
}

impl<T: ReadRegister> Instruction for Read<T> {
    const PARAMETERS: u16 = 4;
    const INSTRUCTION_VALUE: u8 = 0x02;

    fn id(&self) -> PacketID {
        self.id
    }

    fn parameter(&self, index: usize) -> u8 {
        match index {
            0 => T::ADDRESS as u8,
            1 => (T::ADDRESS >> 8) as u8,
            2 => T::SIZE as u8,
            3 => (T::SIZE >> 8) as u8,
            x => panic!("Read instruction parameter indexed with {}, only 4 parameters exists", x),
        }
    }
}

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub(crate) struct ReadResponse<T: ReadRegister> {
    pub id: ServoID,
    pub value: T,
}

impl<T: ReadRegister> Status for ReadResponse<T> {
    const PARAMETERS: u16 = T::SIZE;

    fn deserialize(id: ServoID, parameters: &[u8]) -> Self{
        ReadResponse{
            id: id,
            value: T::deserialize(parameters)
        }
    }
}


#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub(crate) struct Write<T: WriteRegister> {
    id: PacketID,
    data: T,
}

impl<T: WriteRegister> Write<T> {
    pub fn new(id: PacketID, data: T) -> Self {
        Write{id: id, data: data}
    }
}

impl<T: WriteRegister> Instruction for Write<T>{
    const PARAMETERS: u16 = 2 + T::SIZE;
    const INSTRUCTION_VALUE: u8 = 0x03;

    fn id(&self) -> PacketID {
        self.id
    }
    
    fn parameter(&self, index: usize) -> u8 {
        match index {
            0 => T::ADDRESS as u8,
            1 => (T::ADDRESS >> 8) as u8,
            2 => self.data.serialize()[0],
            3 => self.data.serialize()[1],
            4 => self.data.serialize()[2],
            5 => self.data.serialize()[3],
            x => panic!("Read instruction parameter indexed with {}, only 6 parameters exists", x),
        }
    }
}

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub(crate) struct WriteResponse {
    pub id: ServoID,
}

impl Status for WriteResponse {
    const PARAMETERS: u16 = 0;
    
    fn deserialize(id: ServoID, parameters: &[u8]) -> Self {
        assert_eq!(parameters.len(), 0);
        WriteResponse {id: id}
    }
}

#[cfg(test)]
mod tests {
    // Using the same test case that can be found at:
    // http://support.robotis.com/en/product/actuator/dynamixel_pro/communication/instruction_status_packet.htm
    
    use crate::protocol2::*;
    use crate::protocol2::instruction::*;
    use crate::pro;

    #[test]
    fn test_ping() {
        let ping = Ping::new(PacketID::unicast(1));
        let mut array = [0u8; 10];
        for (i, b) in ping.serialize().enumerate() {
            array[i] = b;
        }
        assert_eq!(array, [0xff, 0xff, 0xfd, 0x00, 0x01, 0x03, 0x00, 0x01, 0x19, 0x4e]);
        
        let ping = Ping::new(PacketID::broadcast());
        let mut array = [0u8; 10];
        for (i, b) in ping.serialize().enumerate() {
            array[i] = b;
        }
        assert_eq!(array, [0xff, 0xff, 0xfd, 0x00, 0xfe, 0x03, 0x00, 0x01, 0x31, 0x42]);
    }
    #[test]
    fn test_pong() {
        let mut deserializer = Deserializer::<Pong>::new()
            .deserialize_header([0xff, 0xff, 0xfd, 0x00, 0x01, 0x07, 0x00, 0x55, 0x00])
            .unwrap();

        assert_eq!(deserializer.remaining_bytes(), 5);
        assert_eq!(deserializer.deserialize(&[0x06, 0x04, 0x026, 0x65, 0x5d]), Ok(DeserializationStatus::Finished));
        
        assert_eq!(deserializer.build(),
                   Ok(Pong{
                       id: ServoID::new(0x01),
                       model_number: 0x0406,
                       fw_version: 0x26,
                   })
        );
    }

    #[test]
    fn test_pong_mixed() {
        let mut deserializer = Deserializer::<Pong>::new()
            .deserialize_header([0xff, 0xff, 0xfd, 0x00, 0x01, 0x07, 0x00, 0x55, 0x00])
            .unwrap();
    
        assert_eq!(deserializer.deserialize(&[0x18]), Ok(DeserializationStatus::Ok));
        assert_eq!(deserializer.deserialize(&[0xa9]), Ok(DeserializationStatus::Ok));
        assert_eq!(deserializer.deserialize(&[0x19]), Ok(DeserializationStatus::Ok));
        assert_eq!(deserializer.deserialize(&[0x76]), Ok(DeserializationStatus::Ok));
        
        assert!(!deserializer.is_finished());
        assert_eq!(deserializer.deserialize(&[0x32]), Ok(DeserializationStatus::Finished));        
        assert!(deserializer.is_finished());
        
        assert_eq!(deserializer.build(),
                   Ok(Pong{
                       id: ServoID::new(0x01),
                       model_number: 0xa918,
                       fw_version: 0x19,
                   })
        );

    }

    
    #[test]
    fn test_write() {
        let mut array = [0u8; 16];
        let write = Write::new(PacketID::unicast(1), crate::pro::control_table::GoalPosition::new(0xabcd));
        for (i, b) in write.serialize().enumerate() {
            array[i] = b;
        }
        assert_eq!(
            array,
            [0xff, 0xff, 0xfd, 0x00, 0x01, 0x09, 0x00, 0x03, 0x54, 0x02, 0xcd, 0xab, 0x00, 0x00, 0x0d, 0xe5]
        );

        // Test write that needs stuffing
        let mut array = [0u8; 17];
        let write = Write::new(PacketID::unicast(1), crate::pro::control_table::GoalPosition::new(0xfdffff));
        for (i, b) in write.serialize().enumerate() {
            array[i] = b;
        }
        assert_eq!(
            array,
            [0xff, 0xff, 0xfd, 0x00, 0x01, 0x0a, 0x00, 0x03, 0x54, 0x02, 0xff, 0xff, 0xfd, 0xfd, 0x00, 33, 53]
        );

    }

    #[test]
    fn test_write_response_byte() {
        let mut deserializer = Deserializer::<WriteResponse>::new()
            .deserialize_header([0xff, 0xff, 0xfd, 0x00, 0x01, 0x04, 0x00, 0x55, 0x00])
            .unwrap();
        
        assert_eq!(deserializer.deserialize(&[0xa1]), Ok(DeserializationStatus::Ok));
        assert_eq!(deserializer.deserialize(&[0x0c]), Ok(DeserializationStatus::Finished));     

        assert!(deserializer.is_finished());
        
        assert_eq!(deserializer.build(),
                   Ok(WriteResponse{id: ServoID::new(0x01)})
        );

    }

    #[test]
    fn test_write_response_mixed() {
        let mut deserializer = Deserializer::<WriteResponse>::new()
            .deserialize_header([0xff, 0xff, 0xfd, 0x00, 0x01, 0x04, 0x00, 0x55, 0x00])
            .unwrap();
        
        assert_eq!(deserializer.remaining_bytes(), 2);
        assert_eq!(deserializer.deserialize(&[0xa1]), Ok(DeserializationStatus::Ok));
        
        assert!(!deserializer.is_finished());
        assert_eq!(deserializer.deserialize(&[0x0c]), Ok(DeserializationStatus::Finished));        
        assert!(deserializer.is_finished());
        
        assert_eq!(deserializer.build(),
                   Ok(WriteResponse{id: ServoID::new(0x01)})
        );

    }


    #[test]
    fn test_read() {
        let mut array = [0u8; 14];
        let read = Read::<crate::pro::control_table::PresentPosition>::new(PacketID::unicast(1));
        for (i, b) in read.serialize().enumerate() {
            array[i] = b;
        }
        assert_eq!(
            array,
            [0xff, 0xff, 0xfd, 0x00, 0x01, 0x07, 0x00, 0x02, 611u16 as u8, (611u16 >> 8) as u8, 0x04, 0x00, 27, 249]
        );
    }

    #[test]
    fn test_read_response_slice() {
        let mut deserializer = Deserializer::<ReadResponse<crate::pro::control_table::GoalPosition>>::new()
            .deserialize_header([0xff, 0xff, 0xfd, 0x00, 0x01, 0x08, 0x00, 0x55, 0x00])
            .unwrap();

        assert_eq!(deserializer.deserialize(&[0xa6, 0x00, 0x00, 0x00, 0x8c, 0xc0]), Ok(DeserializationStatus::Finished));
        
        assert_eq!(deserializer.build(),
                   Ok(ReadResponse{
                       value: crate::pro::control_table::GoalPosition::new(0x000000a6),
                       id: ServoID::new(0x01),
                   })
        );

    }

    #[test]
    fn test_read_response_byte() {
        let mut deserializer = Deserializer::<ReadResponse<crate::pro::control_table::GoalPosition>>::new()
            .deserialize_header([0xff, 0xff, 0xfd, 0x00, 0x01, 0x08, 0x00, 0x55, 0x00])
            .unwrap();

        for b in [0xa6, 0x00, 0x00, 0x00, 0x8c, 0xc0].iter() {
            deserializer.deserialize(&[*b]).unwrap();
        }

        assert!(deserializer.is_finished());
        
        assert_eq!(deserializer.build(),
                   Ok(ReadResponse{
                       value: crate::pro::control_table::GoalPosition::new(0x000000a6),
                       id: ServoID::new(0x01),
                   })
        );

    }
}
