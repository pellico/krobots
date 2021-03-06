/*
krobots
Copyright (C) 2021  Oreste Bernardi

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
use crate::physics::{PhysicsEngine,Vector2,Real,Isometry2,Rotation2};
use crate::tank_proto::*;
use log::{debug, error, info};
use prost::Message;
use std::time::Duration;
use std::{io};
use super::GameStateSender;
mod udp;
mod tcp;


const BUFFER_SIZE: usize = 2048;
pub struct ConnectedRobot {
    connection: Box<dyn ClientConnection>,
    name: String,
}
pub struct RobotServer {
    connected_robots: Vec<ConnectedRobot>,
    debug_mode: bool,
    tcp_connection:bool
}

trait ClientConnection {
    fn recv(&mut self,buffer :&mut [u8]) -> io::Result<usize>;
    fn send(&mut self,buffer :&[u8]) -> io::Result<usize>;
}

trait NetInterface {
    fn wait_new_tank(&mut self,buffer :&mut [u8]) -> (Box<dyn ClientConnection>,usize);
}

impl RobotServer {
    pub fn new(debug_mode:bool,tcp_connection:bool) -> RobotServer {
        RobotServer {
            connected_robots: Vec::new(),
            debug_mode : debug_mode,
            tcp_connection : tcp_connection
        }
    }
    pub fn wait_connections(
        &mut self,
        p_engine: &mut PhysicsEngine,
        udp_port: u16,
        state_sender : & mut Box<dyn GameStateSender>
        
    ) {
        info!("Waiting connections on port {}", udp_port);
        let mut client_interface : Box<dyn NetInterface> = if self.tcp_connection {
            Box::new(tcp::new(udp_port,self.debug_mode))
        } else {
            Box::new(udp::new(udp_port,self.debug_mode))
        };
    
        let mut buffer: [u8; BUFFER_SIZE] = [0; BUFFER_SIZE];
        let mut names = Vec::<String>::new();
        //Position of first tank. Other tank positions are computed by rotating it.
        let position_vector = Vector2::new(p_engine.conf.start_distance, 0.0);
        for tank_index in 0..p_engine.max_num_tanks {
            let (mut client_connection,amt)=client_interface.wait_new_tank(&mut buffer);
            let tank_id = match RegisterTank::decode(&buffer[..amt]) {
                Ok(tank_id) => tank_id,
                Err(error) => {
                    error!("Error receiving registration{:?}", error);
                    continue;
                }
            };

            //info!("IP:{} is registering tank {} server port {}",src,tank_id.name,self.connection_port_counter);
            //Compute position of new tank
            let tank_pos_angle = (2.0 * std::f32::consts::PI / p_engine.max_num_tanks as f32)
                * (tank_index + 1) as f32;
            let tank_vector_position = Isometry2::rotation(tank_pos_angle) * position_vector;
            //Angle to compute starting position of tank
            let tank_position = Isometry2::new(tank_vector_position, tank_pos_angle);
            p_engine.add_tank(tank_position,tank_id.name.clone());
            names.push(tank_id.name.clone());
      
            //Send answer
            let answer = self.get_register_tank_answer(p_engine);
            let mut transmit_buff = Vec::new();
            answer
                .encode(&mut transmit_buff)
                .expect("Failed to encode CommandResult");
            client_connection.send(&transmit_buff).unwrap();
            self.connected_robots.push(ConnectedRobot {
                connection: client_connection,
                name: tank_id.name
            });
            state_sender.send(p_engine).expect("Error sending info to UI");

        }
        //After registration of all tank send start packet
        for tank in &mut self.connected_robots {
            let answer = Self::get_status(p_engine,p_engine.tanks.len()-1);
            let mut transmit_buff = Vec::new();
            answer
                .encode(&mut transmit_buff)
                .expect("Failed to encode CommandResult");
            tank.connection
                .send(&transmit_buff)
                .expect("not able to send start packet");
            debug!("Sent start packet to {}",tank.name);
        }
        
    }

    fn get_register_tank_answer(&self,p_engine:&PhysicsEngine) -> SimulationConfig {
        let mut answer = SimulationConfig::default();
        answer.tank_energy_max = p_engine.conf.tank_energy_max;
        answer.damage_max =  p_engine.conf.damage_max;
        answer.bullet_max_range =  p_engine.conf.bullet_max_range;
        answer.zero_power_limit = p_engine.conf.zero_power_limit;
        answer.radar_angle_increment_max =  p_engine.conf.radar_angle_increment_max;
        answer.radar_width_max = p_engine.conf.radar_width_max;
        answer.radar_max_detection_range =  p_engine.conf.radar_max_detection_distance;
        answer.bullet_speed =  p_engine.conf.bullet_speed;
        answer.max_forward_power =  p_engine.conf.tank_engine_power_max;
        answer.max_turning_power =  p_engine.conf.turning_power_max;
        answer.debug_mode = self.debug_mode;
        answer
    }

    fn get_status(p_engine: &PhysicsEngine, tank_index: usize) -> TankStatus {
        let mut tank_status = TankStatus::default();
        tank_status.tick = p_engine.tick();
        let (vel,angvel) = p_engine.tank_velocity(tank_index);
        tank_status.velocity = Some(PolarVector {
            r: vel.norm(),
            p: vel.y.atan2(vel.x),
        });
        let tank_position = p_engine.get_tank_position(tank_index);
        tank_status.angle = tank_position.rotation.angle();
        tank_status.angvel = angvel;
        tank_status.energy = p_engine.tank_energy(tank_index);
        tank_status.damage = p_engine.tank_damage(tank_index);
        tank_status.cannon_angle = p_engine.tank_cannon_angle(tank_index);
        let tank_position = tank_position.translation.vector;
        tank_status.power_source = Some(PolarVector {
            r : tank_position.norm(),
            p : Rotation2::rotation_between(&Vector2::<Real>::x(), &(-tank_position)).angle(),
        });
        tank_status.success = true;
        tank_status.cannon_temp = p_engine.cannon_temperature(tank_index);
        tank_status
    }

    fn set_engine_power(
        p_engine: &mut PhysicsEngine,
        tank_index: usize,
        power_fraction: f32,
        turning_power_fraction : f32
    ) -> TankStatus {
        let command_result = Self::get_status(p_engine, tank_index);
        p_engine.set_tank_engine_power(power_fraction, tank_index);
        p_engine.set_tank_turning_power(turning_power_fraction, tank_index);
        command_result
    }

    fn get_radar_result(
        p_engine: &mut PhysicsEngine,
        tank_index: usize,
        radar_increment: f32,
        radar_width : f32
    ) -> RadarResult {
        let mut command_result = RadarResult::default();
        command_result.tick = p_engine.tick();
        let (angle,detected_tanks) = p_engine.get_radar_result(tank_index,radar_increment,radar_width);
        command_result.angle = angle;
        let mut tanks_radar = Vec::new();
        for (tank,distance) in detected_tanks {
            let t_radar = TankRadar {
                damage : tank.damage,
                distance : distance,
            };
            tanks_radar.push(t_radar);
        }
        tanks_radar.sort_by(|a, b| a.distance.partial_cmp(&b.distance).unwrap());
        tanks_radar.truncate(10);
        command_result.tanks = tanks_radar;
        command_result
    }

    fn set_cannon_position(
        p_engine: &mut PhysicsEngine,
        tank_index: usize,
        angle: f32,
    ) -> TankStatus {
        let command_result = Self::get_status(p_engine, tank_index);
        p_engine.set_cannon_position(tank_index,angle);
        command_result
    }

    #[inline]
    fn fire_cannon(
        p_engine: &mut PhysicsEngine,
        tank_index: usize,
    ) -> TankStatus {
        let mut command_result = Self::get_status(p_engine, tank_index);
        command_result.success = p_engine.fire_cannon(tank_index);
        command_result
    }

    fn process_command(
        buffer: &[u8],
        index: usize,
        p_engine: &mut PhysicsEngine,
    ) -> Result<Vec<u8>, prost::DecodeError> {
        let rec_command = Command::decode(buffer)?;
        //let message : Box<dyn Message>=
        let mut transmit_buff = Vec::new();
        match rec_command.command() {
            command::CommandId::GetStatus => Self::get_status(p_engine, index)
                .encode(&mut transmit_buff)
                .unwrap(),
            command::CommandId::SetEnginePower => {
                Self::set_engine_power(p_engine, index, rec_command.argument1,rec_command.argument2)
                    .encode(&mut transmit_buff)
                    .unwrap()
            }
            command::CommandId::GetRadarResult => {
                Self::get_radar_result(p_engine, index, rec_command.argument1,rec_command.argument2)
                    .encode(&mut transmit_buff)
                    .unwrap()
            }
            command::CommandId::SetCannonPosition => {
                Self::set_cannon_position(p_engine, index, rec_command.argument1)
                    .encode(&mut transmit_buff)
                    .unwrap()
            }
            command::CommandId::FireCannon => {
                Self::fire_cannon(p_engine, index)
                    .encode(&mut transmit_buff)
                    .unwrap()
            }
            #[allow(unreachable_patterns)]
            _ => panic!("Received unsupported command"),
        }
        Ok(transmit_buff)
    }

    pub fn process_request(&mut self, p_engine: &mut PhysicsEngine) {
        let mut buffer: [u8; BUFFER_SIZE] = [0; BUFFER_SIZE];
        for index in 0..self.connected_robots.len() {
            let ConnectedRobot {connection,name } = &mut self.connected_robots[index];
            let num_rec_bytes =  if self.debug_mode {
                // If in debug mode, wait for command of client 
                // but it shall be possible to interrupt the process with e.g. Ctrl-C
                loop {
                    match connection.recv(&mut buffer) {
                        Ok(num) => break num,
                        Err(_) => continue,
                    };
                } 

            } else {
                // In regular mode don't wait for tank command and go to next tank    
                match connection.recv(&mut buffer) {
                    Ok(num) => num,
                    Err(_) => continue,
                }
            };
            let transmit_buffer = Self::process_command(&buffer[..num_rec_bytes], index, p_engine);
            match transmit_buffer {
                Ok(transmit_data) => {
                    connection.send(&transmit_data).expect("Failed to send");
                }
                Err(_) => {
                    error!("Unable to decode command of tank {}", name );
                    continue
                },
            }
        }
    }
}
