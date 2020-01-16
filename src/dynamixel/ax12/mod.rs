pub mod control_table;

use Interface;
use Servo;
use std::cmp::*;

protocol1_servo!(AX12, ::dynamixel::ax12::control_table::WriteRegister, ::dynamixel::ax12::control_table::ReadRegister, 12);

impl<I: Interface> Servo<I> for AX12<I> {
    fn set_enable_torque(&mut self, interface: &mut I, enable_torque: bool) -> Result<(), ::Error> {
        Ok(self.write_data(interface, control_table::TorqueEnable::new(enable_torque))?)
    }
    
    fn set_position(&mut self, interface: &mut I, angle: f32) -> Result<(), ::Error> {
        let goal_position = ((angle*3.41) as i32) as u16;
        let goal_limited = max(0, min(1023, goal_position));
        let a = self.write_data(interface, control_table::GoalPosition::new(goal_limited))?;
        Ok(a)
    }
    
    fn get_position(&mut self, interface: &mut I) -> Result<f32, ::Error> {
        let pos_fixed = f32::from(u16::from(self.read_data::<::dynamixel::ax12::control_table::PresentPosition>(interface)?));
        let pos_rad = pos_fixed / 3.41;
        Ok(pos_rad)
    }
}
