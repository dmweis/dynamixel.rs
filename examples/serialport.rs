extern crate dynamixel;
extern crate serialport;
extern crate badlog;

use std::io::{
    self
};
use std::str::FromStr;
use dynamixel::BaudRate;
use dynamixel::Interface;

fn main() {
    badlog::init_from_env("LOG_LEVEL");

    let ports = serialport::available_ports().unwrap();

    println!("Choose a serial device (0 - {})", ports.len() - 1);
    for (i, port) in ports.iter().enumerate() {
        println!("({}) {:?}", i, port);
    }

    let reader = io::stdin();
    let mut buffer = String::new();
    reader.read_line(&mut buffer).ok().unwrap();
    let input = buffer.trim();

    let index = usize::from_str(input).unwrap();

    let mut serial = serialport::open(&ports[index].port_name).unwrap();
    serial.set_baud_rate(BaudRate::Baud1000000).unwrap();

    let interfaces = dynamixel::enumerate(&mut serial).unwrap();
    println!("Found following servos:");
    for (i, port) in interfaces.iter().enumerate() {
        println!("({}) {:?}", i, port);
    }

    let mut buffer = String::new();
    reader.read_line(&mut buffer).ok().unwrap();
    buffer.pop().unwrap(); // remove new-line
    let input = buffer.trim();

    let index = usize::from_str(input).unwrap();

    let mut servo = dynamixel::connect(&mut serial, interfaces[index].clone()).unwrap();

    let pos = servo.get_position(&mut serial).unwrap();
    println!("current position: {}", pos);

    println!("moving to center");
    servo.set_enable_torque(&mut serial, true).unwrap();
    servo.set_position(&mut serial, 0.0).unwrap();
}
