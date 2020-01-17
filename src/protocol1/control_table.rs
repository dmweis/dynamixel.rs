macro_rules! rw_reg1{
    ($name:ident, $type:ident, $address:expr) => {
        register_impl1!($name, $type, $address);

        impl $name {
            pub fn new(v: $type) -> Self {
                $name(v)
            }
        }
        
        impl From<$name> for $type {
            fn from(v: $name) -> $type {
                v.0
            }
        }

        read_register_impl1!($name, $type);
        write_register_impl1!($name, $type);
    };
}

macro_rules! r_reg1{
    ($name:ident, $type:ident, $address:expr) => {
        register_impl1!($name, $type, $address);

        impl From<$name> for $type {
            fn from(v: $name) -> $type {
                v.0
            }
        }
        
        read_register_impl1!($name, $type);
    };
}

macro_rules! register_impl1{
    ($name:ident, bool, $address:expr) => {
        #[derive(Debug, PartialEq, Eq, Clone, Copy)]
        pub struct $name(bool);

        impl crate::protocol1::Register for $name {
            const SIZE: u8 = 1;
            const ADDRESS: u8 = $address;
        }
    };
    ($name:ident, u8, $address:expr) => {
        #[derive(Debug, PartialEq, Eq, Clone, Copy)]
        pub struct $name(u8);
        
        impl crate::protocol1::Register for $name {
            const SIZE: u8 = 1;
            const ADDRESS: u8 = $address;
        }
    };
    ($name:ident, i16, $address:expr) => {
        #[derive(Debug, PartialEq, Eq, Clone, Copy)]
        pub struct $name(i16);
        
        impl crate::protocol1::Register for $name {
            const SIZE: u8 = 2;
            const ADDRESS: u8 = $address;
        }
    };
    ($name:ident, u16, $address:expr) => {
        #[derive(Debug, PartialEq, Eq, Clone, Copy)]
        pub struct $name(u16);
        
        impl crate::protocol1::Register for $name {
            const SIZE: u8 = 2;
            const ADDRESS: u8 = $address;
        }
    };
    ($name:ident, i32, $address:expr) => {
        #[derive(Debug, PartialEq, Eq, Clone, Copy)]
        pub struct $name(i32);
        
        impl crate::protocol1::Register for $name {
            const SIZE: u8 = 4;
            const ADDRESS: u8 = $address;
        }
    };

}

macro_rules! read_register_impl1{
    ($name:ident, bool) => {
        impl ReadRegister for $name {}
        
        impl crate::protocol1::ReadRegister for $name {
            fn deserialize(data: &[u8]) -> Self {
                assert_eq!(data.len(), 1);
                $name(data[0]&1 == 1)
            }
        }
    };
    ($name:ident, u8) => {
        impl ReadRegister for $name {}
        
        impl crate::protocol1::ReadRegister for $name {
            fn deserialize(data: &[u8]) -> Self {
                assert_eq!(data.len(), 1);
                $name(data[0])
            }
        }
    };
    ($name:ident, i8) => {
        impl ReadRegister for $name {}
        
        impl crate::protocol1::ReadRegister for $name {
            fn deserialize(data: &[u8]) -> Self {
                assert_eq!(data.len(), 1);
                $name(data[0] as i8)
            }
        }
    };
    ($name:ident, i16) => {
        impl ReadRegister for $name {}
        
        impl crate::protocol1::ReadRegister for $name {
            fn deserialize(data: &[u8]) -> Self {
                assert_eq!(data.len(), 2);
                $name(data[0] as i16 | ((data[1] as u16) << 8) as i16)
            }
        }
    };
    ($name:ident, u16) => {
        impl ReadRegister for $name {}
        
        impl crate::protocol1::ReadRegister for $name {
            fn deserialize(data: &[u8]) -> Self {
                assert_eq!(data.len(), 2);
                $name(data[0] as u16 | ((data[1] as u16) << 8))
            }
        }
    };
    ($name:ident, u32) => {
        impl ReadRegister for $name {}

        impl crate::protocol1::ReadRegister for $name {
            fn deserialize(data: &[u8]) -> Self {
                assert_eq!(data.len(), 4);
                $name((data[0] as u32 | (data[1] as u32) << 8 | (data[2] as u32) << 16 | (data[3] as u32) << 24))
            }
        }
    };
    ($name:ident, i32) => {
        impl ReadRegister for $name {}
        
        impl crate::protocol1::ReadRegister for $name {
            fn deserialize(data: &[u8]) -> Self {
                assert_eq!(data.len(), 4);
                $name((data[0] as u32 | (data[1] as u32) << 8 | (data[2] as u32) << 16 | (data[3] as u32) << 24) as i32)
            }
        }
    };
}

macro_rules! write_register_impl1{
    ($name:ident, bool) => {
        impl WriteRegister for $name {}
        
        impl crate::protocol1::WriteRegister for $name {
            fn serialize(&self) -> [u8; 4] {
                [self.0 as u8, 0, 0, 0]
            }    
        }
    };
    ($name:ident, u8) => {
        impl WriteRegister for $name {}
        
        impl crate::protocol1::WriteRegister for $name {
            fn serialize(&self) -> [u8; 4] {
                [self.0, 0, 0, 0]
            }    
        }
    };
    ($name:ident, i8) => {
        impl WriteRegister for $name {}
        
        impl crate::protocol1::WriteRegister for $name {
            fn serialize(&self) -> [u8; 4] {
                [self.0 as u8, 0, 0, 0]
            }    
        }
    };
    ($name:ident, i16) => {
        impl WriteRegister for $name {}
        
        impl crate::protocol1::WriteRegister for $name {
            fn serialize(&self) -> [u8; 4] {
                [self.0 as u8, (self.0 >> 8) as u8, 0, 0]
            }    
        }
    };
    ($name:ident, u16) => {
        impl WriteRegister for $name {}
        
        impl crate::protocol1::WriteRegister for $name {
            fn serialize(&self) -> [u8; 4] {
                [self.0 as u8, (self.0 >> 8) as u8, 0, 0]
            }    
        }
    };
    ($name:ident, u32) => {
        impl WriteRegister for $name {}
        
        impl crate::protocol1::WriteRegister for $name {
            fn serialize(&self) -> [u8; 4] {
                [self.0 as u8, (self.0 >> 8) as u8, (self.0 >> 16) as u8, (self.0 >> 24) as u8]
            }    
        }
    };
    ($name:ident, i32) => {
        impl WriteRegister for $name {}
        
        impl crate::protocol1::WriteRegister for $name {
            fn serialize(&self) -> [u8; 4] {
                [self.0 as u8, (self.0 >> 8) as u8, (self.0 >> 16) as u8, (self.0 >> 24) as u8]
            }    
        }
    };
}
