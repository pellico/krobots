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
mod networking;
mod report;
mod tank;
mod ui_interface;
mod util;
pub use self::tank::{Bullet, Tank};
pub use self::ui_interface::*;
use self::util::*;
use crate::conf::*;
use crate::{is_exit_application, signal_exit, Opts};
use log::{debug, error, info, warn};
use networking::RobotServer;
pub use rapier2d::na::{vector, Isometry2, Rotation2};
pub use rapier2d::na::{Point2, Vector2};
pub use rapier2d::prelude::Real;
use rapier2d::prelude::*;
use serde::{Deserialize, Serialize};
use std::f32::consts::PI;
use std::thread::{spawn, JoinHandle};
use std::time;

const TANK_GROUP: InteractionGroups = InteractionGroups::new(0b001, 0b101);
const TURRET_GROUP: InteractionGroups = InteractionGroups::new(0b010, 0b110);
const BULLET_GROUP: InteractionGroups = InteractionGroups::new(0b100, 0b011);

struct MyPhysicsHooks;

impl PhysicsHooks<RigidBodySet, ColliderSet> for MyPhysicsHooks {
    fn filter_contact_pair(
        &self,
        context: &PairFilterContext<RigidBodySet, ColliderSet>,
    ) -> Option<SolverFlags> {
        // This is a silly example of contact pair filter that:
        // - Enables contact and force computation if both colliders have same user-data.
        // - Disables contact computation otherwise.
        let user_data1 = context.colliders[context.collider1].user_data;
        let user_data2 = context.colliders[context.collider2].user_data;
        if user_data1 != user_data2 {
            debug!("Detect hit");
            Some(SolverFlags::COMPUTE_IMPULSES)
        } else {
            debug!("skip hit");
            None
        }
    }

    fn filter_intersection_pair(&self, _: &PairFilterContext<RigidBodySet, ColliderSet>) -> bool {
        error!("Not here");
        true //This function is not used
    }
}

#[derive(Clone, Copy, Serialize, Deserialize, Debug)]
pub enum SimulationState {
    WaitingConnection,
    Running,
}
impl Default for SimulationState {
    fn default() -> Self {
        SimulationState::WaitingConnection
    }
}

pub struct PhysicsEngine {
    max_num_tanks: usize,
    max_ticks: u32,
    tanks_alive: u32,
    tanks: Vec<Tank>,
    bullets: Vec<Bullet>,
    tick: u32,
    rigid_body_set: RigidBodySet,
    collider_set: ColliderSet,
    integration_parameters: IntegrationParameters,
    physics_pipeline: PhysicsPipeline,
    island_manager: IslandManager,
    broad_phase: BroadPhase,
    narrow_phase: NarrowPhase,
    joint_set: JointSet,
    ccd_solver: CCDSolver,
    physics_hooks: MyPhysicsHooks,
    event_handler: (),
    gravity_vector: Vector2<Real>,
    debug_mode: bool,
    state: SimulationState,
    conf: Conf,
}

impl PhysicsEngine {
    pub fn new(
        conf: Conf,
        opts: &Opts,
        mut state_sender: Box<dyn GameStateSender>,
        command_receiver: Box<dyn UICommandReceiver>,
    ) -> JoinHandle<()> {
        let udp_port = opts.port;
        let simulation_rate = opts.sim_step_rate;
        let use_tcp_tank_client = opts.tank_client_protocol == "tcp";
        let mut p_engine = PhysicsEngine {
            max_num_tanks: opts.num_tanks,
            max_ticks: opts.max_steps,
            tanks_alive: 0,
            tanks: vec![],
            bullets: vec![],
            tick: 0,
            rigid_body_set: RigidBodySet::new(),
            collider_set: ColliderSet::new(),
            integration_parameters: IntegrationParameters::default(),
            physics_pipeline: PhysicsPipeline::new(),
            island_manager: IslandManager::new(),
            broad_phase: BroadPhase::new(),
            narrow_phase: NarrowPhase::new(),
            joint_set: JointSet::new(),
            ccd_solver: CCDSolver::new(),
            physics_hooks: MyPhysicsHooks {},
            event_handler: (),
            gravity_vector: vector![0.0, 0.0], //No gravity
            debug_mode: opts.debug_mode,
            state: SimulationState::WaitingConnection,
            conf: conf,
        };

        let now = time::Instant::now();
        // show some fps measurements every 5 seconds
        let mut fps_counter = ticktock::Timer::apply(|delta_t, prev_tick| (delta_t, *prev_tick), 0)
            .every(time::Duration::from_secs(5))
            .start(now);
        //Create thread that perform physics simulation
        let join_handle = spawn(move || {
            info!("Start waiting connections");
            let mut server = RobotServer::new(p_engine.debug_mode, use_tcp_tank_client);
            server.wait_connections(&mut p_engine, udp_port, &mut state_sender);
            info!("Starting simulation");
            p_engine.state = SimulationState::Running;
            for (tick, now) in ticktock::Clock::framerate(simulation_rate).iter() {
                {
                    //Check if received command to exit
                    match command_receiver.receive() {
                        Some(UICommand::QUIT) => p_engine.exit_simulation(),
                        None => (),
                    };
                    // Process all request coming from client
                    server.process_request(&mut p_engine);
                    p_engine.step();
                    
                    // Try to lazly send game state to UI 
                    // If failing teh send ignore it.
                    match state_sender.send(&p_engine) {
                        Ok(_) => (),
                        Err(_) => debug!("Failed sending state to UI ignore it"),

                    }

                    if let Some((delta_t, prev_tick)) = fps_counter.update(now) {
                        fps_counter.set_value(tick);
                        let fps = (tick - prev_tick) as f64 / delta_t.as_secs_f64();
                        debug!("FPS: {}", fps);
                        if fps < (simulation_rate - 1.0) {
                            warn!(
                                "Simulation framerate is low {} expected {}",
                                fps, simulation_rate
                            )
                        }
                    }
                    // The position of exit is very important for --no-gui option.
                    // When exiting the application we allows to send a message to UI sender thread  (state_sender) so it can wakeup and exit. 
                    if is_exit_application() {
                        debug!("Exiting physical simulation thread");
                        break;
                    }
                }
            }
        });

        return join_handle;
    }
    pub fn add_tank(&mut self, tank_position: Isometry2<Real>, name: String) {
        //This tank index is used to set userdata of all collider to skip detection.
        let tank_index = self.tanks.len();
        let tank = Tank::new(self, tank_position, tank_index, name);
        self.tanks.push(tank);
    }

    pub fn step(&mut self) {
        //Execute all command
        for (tank_index, tank) in self.tanks.iter_mut().enumerate() {
            tank.turret.update_cannon_temp();

            if !tank.update_energy(&self.conf) {
                continue;
            }

            if tank.is_dead() {
                continue;
            }

            let tank_rigid_body = &mut self.rigid_body_set[tank.phy_body_handle];
            // Power = F . v. Here we consider the speed along the direction of tank
            Self::apply_engine_power(tank_rigid_body, tank);
            tank_rigid_body.apply_torque_impulse(
                tank.turning_power / (tank.angular_velocity.abs() + 1.0),
                true,
            );
            tank.set_cannon_position(&mut self.joint_set, &self.conf);
            let turret = &mut tank.turret;
            if turret.fire {
                let (bullet_body, collider) = Self::create_bullet(
                    &self.conf,
                    &self.rigid_body_set[turret.phy_body_handle],
                    tank_index,
                );
                let bullet_position = *bullet_body.position();
                let collider_polyline = Self::get_collider_polyline_cuboid(&collider);
                let rigid_body_handle = self.rigid_body_set.insert(bullet_body);
                let collider_handle = self.collider_set.insert_with_parent(
                    collider,
                    rigid_body_handle,
                    &mut self.rigid_body_set,
                );
                let bullet = Bullet {
                    collider_handle: collider_handle,
                    phy_body_handle: rigid_body_handle,
                    tick_counter: std::cmp::max(
                        1,
                        (self.conf.bullet_max_range / self.conf.bullet_speed * 60.0) as u32,
                    ), //remember that step is 1/60 simulation sec.
                    shape_polyline: collider_polyline,
                    position: bullet_position,
                };
                self.bullets.push(bullet);
                turret.fire = false;
                turret.cannon_temperature += self.conf.cannon_fire_temp_increase;
            }
        }
        self.physics_pipeline.step(
            &self.gravity_vector,
            &self.integration_parameters,
            &mut self.island_manager,
            &mut self.broad_phase,
            &mut self.narrow_phase,
            &mut self.rigid_body_set,
            &mut self.collider_set,
            &mut self.joint_set,
            &mut self.ccd_solver,
            &self.physics_hooks,
            &self.event_handler,
        );
        self.tick += 1;
        //Read back present status
        for tank in &mut self.tanks {
            //Tank body
            let tank_rigid_body = &self.rigid_body_set[tank.phy_body_handle];
            tank.position = *tank_rigid_body.position();
            tank.linvel = *tank_rigid_body.linvel();
            tank.angular_velocity = tank_rigid_body.angvel();
            let collider = &self.collider_set[tank.collider_handle];
            tank.shape_polyline = Self::get_collider_polyline_cuboid(collider);

            //Update turrets
            let turret = &mut tank.turret;
            let turret_rigid_body = &self.rigid_body_set[turret.phy_body_handle];
            turret.angle = turret_rigid_body.position().rotation.angle();
            let collider_turret = &self.collider_set[turret.collider_handle];
            turret.shape_polyline = Self::get_collider_polyline_cuboid(collider_turret);
        }
        for bullet in &mut self.bullets {
            for contact_pair in self.narrow_phase.contacts_with(bullet.collider_handle) {
                /*Skip if no contact. This should be false for bullet in contact
                with tank that has fired the same bullet. See physics hook.
                */
                if !contact_pair.has_any_active_contact {
                    continue;
                }
                let other_collider = if contact_pair.collider1 == bullet.collider_handle {
                    contact_pair.collider2
                } else {
                    contact_pair.collider1
                };
                // :TODO: Consider if bullet hit turret
                match self
                    .tanks
                    .iter()
                    .position(|x| x.collider_handle == other_collider)
                {
                    Some(target_tank_index) => {
                        debug!("Tank {} has been hit", target_tank_index);
                        self.tanks[target_tank_index].bullet_damage()
                    }
                    None => (),
                }
                // Process the contact pair in a way similar to what we did in
                // the previous example.
                bullet.tick_counter = 1;
            }
            bullet.tick_counter -= 1;
            //Update polyline for drawing
            bullet.shape_polyline =
                Self::get_collider_polyline_cuboid(&self.collider_set[bullet.collider_handle]);
            bullet.position = *self.rigid_body_set[bullet.phy_body_handle].position();
            if bullet.tick_counter == 0 {
                //If expired remove from physics engine
                self.rigid_body_set.remove(
                    bullet.phy_body_handle,
                    &mut self.island_manager,
                    &mut self.collider_set,
                    &mut self.joint_set,
                );
            }
        }
        //Remove expired bullet from bullets vector.
        self.bullets.retain(|bullet| {
            if bullet.tick_counter == 0 {
                debug!("Bullet destroyed");
                false
            } else {
                true
            }
        });

        //Counts how many tanks are alive
        self.tanks_alive = 0;
        for tank in &self.tanks {
            if tank.is_dead() {
                continue;
            } else {
                self.tanks_alive += 1;
            }
        }

        //If all tanks are dead except one exit from simulation
        if self.tanks_alive <= 1 {
            if self.tanks_alive == 0 {
                info!("All tanks destroyed ... exiting");
            } else {
                info!("All tanks except one are destroyed ... exiting");
            }
            self.exit_simulation();
        }

        // If reached max number of simulation exit from simulation
        if self.max_ticks != 0 && self.tick >= self.max_ticks {
            info!("Reached max number of ticks .. exiting");
            self.exit_simulation();
        }
    }

    pub fn create_bullet(
        conf: &Conf,
        cannon_body: &RigidBody,
        tank_index: usize,
    ) -> (RigidBody, Collider) {
        let cannon_position = cannon_body.position();
        let velocity_cannon_edge =
            get_velocity_at_point(conf.turret_width_m / 2.0, 0.0, cannon_body);
        let angle = cannon_position.rotation.angle();
        debug!("Created bullet angle:{}", angle * 180.0 / PI);
        //Compute bullet speed and sum cannon edge speed (world speed)
        let velocity = (cannon_position * vector![conf.bullet_speed, 0.0]) + velocity_cannon_edge;
        //bullet shall be created in front of cannon and outside of the tank
        let bullet_position = cannon_position * Point2::new(1.8, 0.0);
        let bullet_body = RigidBodyBuilder::new_dynamic()
            .translation(bullet_position.coords)
            .linvel(velocity)
            .rotation(angle)
            .ccd_enabled(true)
            .linear_damping(0.0)
            .angular_damping(0.0)
            .build();

        let bullet_collider = ColliderBuilder::cuboid(0.05, 0.2)
            .density(0.1)
            .collision_groups(BULLET_GROUP)
            .active_hooks(ActiveHooks::FILTER_CONTACT_PAIRS)
            .user_data(tank_index as u128) //Will be used by physics hook to avoid collision with tank that fired bullet
            .build();
        (bullet_body, bullet_collider)
    }

    #[inline]
    pub fn tick(&self) -> u32 {
        self.tick
    }

    #[inline]
    pub fn tank_engine_power_percentage(&self, tank_id: usize) -> f32 {
        let tank = &self.tanks[tank_id];
        tank.engine_power / tank.max_engine_power
    }
    pub fn set_tank_engine_power(&mut self, power_fraction: f32, tank_id: usize) {
        let tank = &mut self.tanks[tank_id];
        let energy = if power_fraction > 1.0 {
            1.0
        } else if power_fraction < -1.0 {
            -1.0
        } else {
            power_fraction
        };
        tank.engine_power = energy * tank.max_engine_power;
    }

    pub fn get_position(&self, tank: &Tank) -> &Isometry<Real> {
        let p = &self.rigid_body_set[tank.phy_body_handle];
        return p.position();
    }
    pub fn get_tank_position(&self, tank_id: usize) -> &Isometry<Real> {
        self.get_position(&self.tanks[tank_id])
    }

    pub fn tank_velocity(&self, tank_id: usize) -> (Vector<Real>, Real) {
        let tank = &self.tanks[tank_id];
        let rigid_body = &self.rigid_body_set[tank.phy_body_handle];
        (*rigid_body.linvel(), rigid_body.angvel())
    }

    #[inline]
    pub fn tank_energy(&self, tank_id: usize) -> f32 {
        self.tanks[tank_id].energy
    }

    #[inline]
    pub fn tank_damage(&self, tank_id: usize) -> f32 {
        self.tanks[tank_id].damage
    }

    pub fn tank_cannon_angle(&self, tank_id: usize) -> f32 {
        let rigid_body = &self.rigid_body_set[self.tanks[tank_id].turret.phy_body_handle];
        rigid_body.position().rotation.angle()
    }

    #[inline]
    fn apply_engine_power(tank_rigid_body: &mut RigidBody, tank: &Tank) {
        //We don't have infinite force at 0 speed.
        let force = tank.engine_power / (tank.forward_velocity().abs() + 0.5);
        let force_forward_vector = tank_rigid_body.position() * vector![force, 0.0];
        tank_rigid_body.apply_force(force_forward_vector, true);
    }
    /// Get turning power.
    #[inline]
    pub fn tank_turning_power(&self, tank_id: usize) -> f32 {
        let tank = &self.tanks[tank_id];
        tank.turning_power / tank.turning_power_max
    }

    pub fn set_tank_turning_power(&mut self, power_fraction: f32, tank_id: usize) {
        let tank = &mut self.tanks[tank_id];
        let power_fraction_wrapped = if power_fraction > 1.0 {
            1.0
        } else if power_fraction < -1.0 {
            -1.0
        } else {
            power_fraction
        };
        tank.turning_power = tank.turning_power_max * power_fraction_wrapped;
    }

    fn get_collider_polyline_cuboid(collider: &Collider) -> Vec<Point2<Real>> {
        let cuboid = collider.shape().as_cuboid().unwrap();
        let mut vertexs = cuboid.to_polyline();
        let position = collider.position();
        for v in &mut vertexs {
            *v = position * *v;
        }
        return vertexs;
    }

    /// Move radar and return detected tanks
    pub fn get_radar_result(
        &mut self,
        tank_id: usize,
        radar_increment: f32,
        radar_width: f32,
    ) -> (f32, Vec<(&Tank, f32)>) {
        let tank = &mut self.tanks[tank_id];
        // Update radar position
        let enough_energy = tank.update_radar_attribute(radar_increment, radar_width);
        // If not enough energy for operation return present position and empty list of detected tank
        if !enough_energy {
            return (tank.radar_position, Vec::new());
        }
        // Detect tank in radar detection area.
        let mut result = Vec::new();
        let tank = &self.tanks[tank_id];
        //Search detected tank
        for index in 0..self.tanks.len() {
            //Tank don't detect itself
            if index == tank_id {
                continue;
            }
            let target_tank = &self.tanks[index];
            // This is the vector from this tank to target tank
            let relative_vector =
                target_tank.position.translation.vector - tank.position.translation.vector;
            let distance = relative_vector.norm();

            if distance < target_tank.radar_max_detection_distance {
                let radar_vector =
                    Isometry2::rotation(tank.radar_position + tank.position.rotation.angle())
                        * Vector2::<Real>::x();
                //angle from radar vector to target_tank vector
                let angle = Rotation2::rotation_between(&radar_vector, &relative_vector).angle();
                if angle.abs() < tank.radar_width / 2.0 {
                    result.push((target_tank, distance));
                }
            }
        }
        (tank.radar_position, result)
    }

    pub fn set_cannon_position(&mut self, tank_id: usize, angle: f32) {
        self.tanks[tank_id].turret.new_angle = Some(angle);
    }

    pub fn fire_cannon(&mut self, tank_id: usize) -> bool {
        let tank = &mut self.tanks[tank_id];
        if tank.ready_to_fire() {
            tank.turret.fire = true;
            true
        } else {
            false
        }
    }

    pub fn cannon_temperature(&self, tank_id: usize) -> f32 {
        self.tanks[tank_id].turret.cannon_temperature
    }

    pub fn exit_simulation(&self) {
        let path = "simulation_output.csv";
        info!("Exiting simulation and saving result to {}", &path);
        report::save_tank_report(path, &self.tanks).unwrap();
        signal_exit();
    }
}
