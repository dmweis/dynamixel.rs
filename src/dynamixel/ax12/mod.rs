pub mod control_table;

use crate::Interface;
use crate::Servo;

protocol1_servo!(AX12, crate::dynamixel::ax12::control_table::WriteRegister, crate::dynamixel::ax12::control_table::ReadRegister, 12);

fn constrain(value: u16, min: u16, max: u16) -> u16 {
    if value < min {
        min
    } else if value > max {
        max
    } else {
        value
    }
}

impl<I: Interface> Servo<I> for AX12<I> {
    fn set_enable_torque(&mut self, interface: &mut I, enable_torque: bool) -> Result<(), crate::Error> {
        Ok(self.write_data(interface, control_table::TorqueEnable::new(enable_torque))?)
    }
    
    fn set_position(&mut self, interface: &mut I, angle: f32) -> Result<(), crate::Error> {
        let goal_position = ((angle*3.41) as i32) as u16;
        let goal_limited = constrain(goal_position, 0, 1023);
        let a = self.write_data(interface, control_table::GoalPosition::new(goal_limited))?;
        Ok(a)
    }
    
    fn get_position(&mut self, interface: &mut I) -> Result<f32, crate::Error> {
        let pos_fixed = f32::from(u16::from(self.read_data::<crate::dynamixel::ax12::control_table::PresentPosition>(interface)?));
        let pos_rad = pos_fixed / 3.41;
        Ok(pos_rad)
    }
}
