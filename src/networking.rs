use crate::physics::{*};
use std::net::{UdpSocket,SocketAddr};
use log::{debug, error, trace, info};
use std::time::Duration;
use crate::conf::UDP_PORT;
use prost::Message;
use crate::tank_proto::*;
const REQUEST_STRING : &'static str = "Give me command";

const BUFFER_SIZE : usize = 2048;
pub struct ConnectedRobot {
    socket : UdpSocket,
}
pub struct RobotServer {
    connected_robots : Vec<ConnectedRobot>
}



impl RobotServer {
    pub fn new() -> RobotServer{
        RobotServer {
            connected_robots : Vec::new()
        }
    }
    pub fn wait_connections(&mut self,expected_connections:u32,p_engine: &mut PhysicsEngine) -> Vec<String> {
        let mut dedicated_connection_port = UDP_PORT;
        info!("Waiting connections on port {}",UDP_PORT);
        let socket = UdpSocket::bind(("127.0.0.1", UDP_PORT)).expect("Not able to open socket: {}");
        let mut buffer : [u8;BUFFER_SIZE] = [0;BUFFER_SIZE];
        let mut names = Vec::<String>::new();
        for _ in 0..expected_connections {
            let (amt, src) = socket.recv_from(&mut buffer).expect("Not received data");
            debug!("Received a packet from {}",src);
            let tank_id =
            match RegisterTank::decode(&buffer[..amt]) {
                Ok(tank_id) => tank_id,
                Err(error) => {error!("Error receiving registration{:?}",error);continue},
            };
            println!("{}",tank_id.name);
            dedicated_connection_port +=1;
            let dedicated_socket = UdpSocket::bind(("127.0.0.1", dedicated_connection_port)).expect("Not able to open socket");
            println!("tanksname{}",tank_id.name);
            debug!("Assigned port{:?}",dedicated_connection_port);
            dedicated_socket.connect(src).unwrap();
            dedicated_socket.set_nonblocking(true).unwrap();
            // Send answer asap
            let mut answer = CommandResult::default();
            answer.tick = p_engine.tick();
            answer.success = true;
            let mut transmit_buff = Vec::new();
            answer.encode(&mut transmit_buff).expect("Failed to encode CommandResult");
            dedicated_socket.send(&transmit_buff).unwrap();
            self.connected_robots.push(ConnectedRobot{socket : dedicated_socket});
            p_engine.add_tank();
            names.push(tank_id.name);
            
        } 
        names
    }

    fn get_status(p_engine : &PhysicsEngine, tank_index :usize) -> TankStatus {
        let mut tank_status = TankStatus::default();
        tank_status.tick = p_engine.tick();
        tank_status.velocity = p_engine.tank_velocity(tank_index);
        tank_status.angle = p_engine.get_tank_position(tank_index).rotation.angle();
        tank_status.energy = p_engine.tank_energy(tank_index);
        tank_status.damage = p_engine.tank_damage(tank_index);
        tank_status.cannon_angle = p_engine.tank_cannon_angle(tank_index);
        tank_status
    }

    fn set_engine_power(p_engine : &mut PhysicsEngine, tank_index :usize,power_percentage : f32) -> CommandResult {
        let mut command_result = CommandResult::default();
        p_engine.set_tank_engine_power(power_percentage,tank_index);
        command_result.tick = p_engine.tick();
        command_result.success=true;
        command_result
    }

    fn set_turning_speed(p_engine : &mut PhysicsEngine, tank_index :usize,turning_speed_fraction : f32) -> CommandResult {
        let mut command_result = CommandResult::default();
        p_engine.set_tank_angle_speed(turning_speed_fraction,tank_index);
        command_result.tick = p_engine.tick();
        command_result.success=true;
        command_result
    }



    fn process_command(buffer:&[u8],index:usize,p_engine:&mut PhysicsEngine) -> Result<Vec<u8>,prost::DecodeError> {
        let rec_command = Command::decode(buffer)?; 
        //let message : Box<dyn Message>=
        let mut transmit_buff = Vec::new();
        match rec_command.command() {
            command::CommandId::GetStatus => Self::get_status(p_engine,index).encode(&mut transmit_buff).unwrap(),
            command::CommandId::SetEnginePower => Self::set_engine_power(p_engine,index,rec_command.argument1).encode(&mut transmit_buff).unwrap(),
            command::CommandId::SetTurningSpeed => Self::set_turning_speed(p_engine,index,rec_command.argument1).encode(&mut transmit_buff).unwrap(),
            _ => panic!("Unsupported command")
        }
        Ok(transmit_buff)
        
    }
    
    pub fn process_request(&self,p_engine: &mut PhysicsEngine) {
        let mut buffer : [u8;BUFFER_SIZE] = [0;BUFFER_SIZE];
        for index in 0..self.connected_robots.len() {

            let ConnectedRobot{socket} = &self.connected_robots[index];

            let num_rec_bytes= match socket.recv(&mut buffer) {
                Ok(num) => num,
                Err(_) => {continue},

            };
            let buf = &buffer[..num_rec_bytes];
            let transmit_buffer = Self::process_command(&buffer[..num_rec_bytes],index,p_engine);
            match transmit_buffer {
                Ok(transmit_data) => {socket.send(&transmit_data).expect("Failed to send");},
                Err(_) => continue
            }
            
        }
    }
}