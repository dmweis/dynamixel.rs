pub trait Register: crate::protocol1::Register {}
pub trait ReadRegister: crate::protocol1::ReadRegister {}
pub trait WriteRegister: crate::protocol1::WriteRegister {}

rw_reg1!(TorqueEnable, bool, 24);
rw_reg1!(Led, bool, 25);
rw_reg1!(GoalPosition, u16, 30);
r_reg1!(PresentPosition, u16, 36);
