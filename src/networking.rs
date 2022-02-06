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
use crate::conf;
use crate::physics::{PhysicsEngine,Vector2,Real,Isometry2,Rotation2};
use crate::tank_proto::*;
use log::{debug, error, info};
use prost::Message;
use std::net::{UdpSocket};
use std::time::Duration;


const BUFFER_SIZE: usize = 2048;
pub struct ConnectedRobot {
    socket: UdpSocket,
    name: String,
}
pub struct RobotServer {
    connected_robots: Vec<ConnectedRobot>,
    debug_mode: bool,
}

impl RobotServer {
    pub fn new(debug_mode:bool) -> RobotServer {
        RobotServer {
            connected_robots: Vec::new(),
            debug_mode : debug_mode
        }
    }
    pub fn wait_connections(
        &mut self,
        expected_connections: u8,
        p_engine: &mut PhysicsEngine,
        udp_port: u16
    ) -> Vec<String> {
        let mut dedicated_connection_port = udp_port;
        info!("Waiting connections on port {}", udp_port);
        let socket = UdpSocket::bind(("127.0.0.1", udp_port)).expect("Not able to open socket: {}");
        let mut buffer: [u8; BUFFER_SIZE] = [0; BUFFER_SIZE];
        let mut names = Vec::<String>::new();
        //Position of first tank. Other tank positions are computed by rotating it.
        let position_vector = Vector2::new(conf::START_DISTANCE, 0.0);
        for tank_index in 0..expected_connections {
            let (amt, src) = socket.recv_from(&mut buffer).expect("Not received data");
            debug!("Received a packet from {}", src);
            let tank_id = match RegisterTank::decode(&buffer[..amt]) {
                Ok(tank_id) => tank_id,
                Err(error) => {
                    error!("Error receiving registration{:?}", error);
                    continue;
                }
            };

            info!("IP:{} is registering tank {} server port {}",src,tank_id.name,dedicated_connection_port);
            //Compute position of new tank
            let tank_pos_angle = (2.0 * std::f32::consts::PI / expected_connections as f32)
                * (tank_index + 1) as f32;
            let tank_vector_position = Isometry2::rotation(tank_pos_angle) * position_vector;
            //Angle to compute starting position of tank
            let tank_position = Isometry2::new(tank_vector_position, tank_pos_angle);
            p_engine.add_tank(tank_position,tank_id.name.clone());
            names.push(tank_id.name.clone());

            //Create connection and store connection data
            dedicated_connection_port += 1;
            let dedicated_socket = UdpSocket::bind(("127.0.0.1", dedicated_connection_port))
                .expect("Not able to open socket");
            dedicated_socket.connect(src).unwrap();           
            
            if self.debug_mode {
                // When in debug mode we want to wait for tank client Command
                // but we would like to be able to kill process as well.
                dedicated_socket.set_nonblocking(false).unwrap();
                dedicated_socket.set_read_timeout(Some(Duration::from_secs(3))).unwrap();
            } else {
                dedicated_socket.set_nonblocking(true).unwrap();
            }
            //Send answer
            let answer = Self::get_register_tank_answer();
            let mut transmit_buff = Vec::new();
            answer
                .encode(&mut transmit_buff)
                .expect("Failed to encode CommandResult");
            dedicated_socket.send(&transmit_buff).unwrap();
            self.connected_robots.push(ConnectedRobot {
                socket: dedicated_socket,
                name: tank_id.name
            });

        }
        //After registration of all tank send start packet
        for tank in &self.connected_robots {
            let answer = Self::get_status(p_engine,p_engine.tanks.len()-1);
            let mut transmit_buff = Vec::new();
            answer
                .encode(&mut transmit_buff)
                .expect("Failed to encode CommandResult");
            tank.socket
                .send(&transmit_buff)
                .expect("not able to send start packet");
            debug!("Sent start packet to {}",tank.name);
        }
        names
    }

    fn get_register_tank_answer() -> SimulationConfig {
        let mut answer = SimulationConfig::default();
        answer.tank_energy_max = conf::TANK_ENERGY_MAX;
        answer.damage_max = conf::DAMAGE_MAX;
        answer.bullet_max_range = conf::BULLET_MAX_RANGE;
        answer.zero_power_limit = conf::ZERO_POWER_LIMIT;
        answer.radar_angle_increment_max = conf::RADAR_ANGLE_INCREMENT_MAX;
        answer.radar_width_max = conf::RADAR_WIDTH_MAX;
        answer.radar_max_detection_range = conf::RADAR_MAX_DETECTION_DISTANCE;
        answer.bullet_speed = conf::BULLET_SPEED;
        answer.max_forward_power = conf::TANK_ENGINE_POWER_MAX;
        answer.max_turning_power = conf::TURNING_POWER_MAX;
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

    pub fn process_request(&self, p_engine: &mut PhysicsEngine) {
        let mut buffer: [u8; BUFFER_SIZE] = [0; BUFFER_SIZE];
        for index in 0..self.connected_robots.len() {
            let ConnectedRobot { socket,name } = &self.connected_robots[index];
            let num_rec_bytes =  if self.debug_mode {
                // If in debug mode, wait for command of client 
                // but it shall be possible to interrupt the process with e.g. Ctrl-C
                loop {
                    match socket.recv(&mut buffer) {
                        Ok(num) => break num,
                        Err(_) => continue,
                    };
                } 

            } else {
                // In regular mode don't wait for tank command and go to next tank    
                match socket.recv(&mut buffer) {
                    Ok(num) => num,
                    Err(_) => continue,
                }
            };

            let transmit_buffer = Self::process_command(&buffer[..num_rec_bytes], index, p_engine);
            match transmit_buffer {
                Ok(transmit_data) => {
                    socket.send(&transmit_data).expect("Failed to send");
                }
                Err(_) => {
                    error!("Unable to decode command of tank {}", name );
                    continue
                },
            }
        }
    }
}
