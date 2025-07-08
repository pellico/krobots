/*
krobots
Copyright (C) 2021  Oreste Bernardi

This program is& free software: you can redistribute it and/or modify
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

mod report;
mod tank;
mod tank_wasm;
mod ui_interface;
mod util;
pub use self::tank::{Bullet, ObjUID, Tank};
pub use self::ui_interface::*;
use self::util::*;
use crate::conf::*;
use crate::{is_exit_application, signal_exit, Opts};
use log::{debug, error, info, warn};
pub use rapier2d::na::{vector, Isometry2, Rotation2};
pub use rapier2d::na::{Point2, Vector2};
use rapier2d::prelude::*;
pub use rapier2d::prelude::{Real, RigidBodyHandle};
use serde::{Deserialize, Serialize};
use std::f32::consts::PI;
use std::path::Path;
use std::sync::{Arc, Mutex};
use std::thread::{spawn, JoinHandle};
use std::time;
use tank_wasm::WasmTanks;

/**
Tank body collision group used in colliders.
*/
const TANK_GROUP: InteractionGroups =
    InteractionGroups::new(Group::GROUP_1, Group::GROUP_1.union(Group::GROUP_3));
const TURRET_GROUP: InteractionGroups =
    InteractionGroups::new(Group::GROUP_2, Group::GROUP_2.union(Group::GROUP_3));
const BULLET_GROUP: InteractionGroups =
    InteractionGroups::new(Group::GROUP_3, Group::GROUP_1.union(Group::GROUP_2));

struct MyPhysicsHooks;

impl PhysicsHooks for MyPhysicsHooks {
    fn filter_contact_pair(&self, context: &PairFilterContext) -> Option<SolverFlags> {
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

    fn filter_intersection_pair(&self, _: &PairFilterContext) -> bool {
        error!("Not here");
        true //This function is not used
    }
}

#[derive(Clone, Copy, Serialize, Deserialize, Debug)]
pub enum SimulationState {
    /// Waiting connection from all tanks
    WaitingConnection,
    /// Simulation running
    Running,
}
impl Default for SimulationState {
    fn default() -> Self {
        SimulationState::WaitingConnection
    }
}

pub struct PhysicsEngine {
    /// Maximum numbers of tick allowed. If `max_ticks` == 0 simulation
    /// is stopped only when only one tank is not disabled/dead.
    max_ticks: u32,
    /// How many tanks are still alive
    tanks_alive: u32,
    /// All tanks in the game
    tanks: Vec<Tank>,
    /// All bullets in the simulation
    bullets: Vec<Bullet>,
    /// Present number of ticks
    tick: u32,
    /// If true simulation wait for commands from tanks
    debug_mode: bool,
    /// Simulation state
    state: SimulationState,
    /// Simulation configuration
    conf: Conf,
    // Parameters required bu Rapier2D
    rigid_body_set: RigidBodySet,
    collider_set: ColliderSet,
    integration_parameters: IntegrationParameters,
    physics_pipeline: PhysicsPipeline,
    island_manager: IslandManager,
    broad_phase: BroadPhase,
    narrow_phase: NarrowPhase,
    joint_set: ImpulseJointSet,
    multibody_joints: MultibodyJointSet,
    ccd_solver: CCDSolver,
    physics_hooks: (),
    event_handler: (),
    gravity_vector: Vector2<Real>,
}

/// Create Point2
/// Workaround of rust analyzer
/// https://github.com/rust-analyzer/rust-analyzer/issues/8654

fn new_point2(x: f32, y: f32) -> Point<f32> {
    [x, y].into()
}

impl Default for PhysicsEngine {
    fn default() -> Self {
          PhysicsEngine {
            max_ticks: 0,
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
            joint_set: ImpulseJointSet::new(),
            multibody_joints: MultibodyJointSet::new(),
            ccd_solver: CCDSolver::new(),
            physics_hooks: (),
            event_handler: (),
            gravity_vector: vector![0.0, 0.0], //No gravity
            debug_mode: true,
            state: SimulationState::WaitingConnection,
            conf:Conf::default()
        }
        
    }
}

impl PhysicsEngine {
    fn new(conf: Conf, opts: &Opts) -> PhysicsEngine {
        PhysicsEngine {
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
            joint_set: ImpulseJointSet::new(),
            multibody_joints: MultibodyJointSet::new(),
            ccd_solver: CCDSolver::new(),
            physics_hooks: (),
            event_handler: (),
            gravity_vector: vector![0.0, 0.0], //No gravity
            debug_mode: opts.debug_mode,
            state: SimulationState::WaitingConnection,
            conf,
        }
    }
    /**
    Create a simulation engine and thread that execute the simulation
    # Arguments
    * `conf` - Game configuration
    * `opts` - Options collected from command line
    * `state_sender` - Proxy used to send simulation state to ui client
    * `command_receiver` - Proxy used to check if there is a command from ui client
    */
    pub fn new_simulation_thread(
        conf: Conf,
        opts: &Opts,
        mut state_sender: Box<dyn GameStateSender>,
        command_receiver: Box<dyn UICommandReceiver>,
    ) -> JoinHandle<()> {
        let simulation_rate = opts.sim_step_rate;
        let mut p_engine = Self::new(conf, opts);
        let tank_folder = opts.tank_folder.clone();
        let now = time::Instant::now();
        // show some fps measurements every 5 seconds
        let mut fps_counter = ticktock::Timer::apply(|delta_t, prev_tick| (delta_t, *prev_tick), 0)
            .every(time::Duration::from_secs(5))
            .start(now);
        //Create thread that perform physics simulation
        spawn(move || {
            info!("Load tanks");
            let mut wasm_tanks = WasmTanks::new(tank_folder, &mut p_engine);
            info!("Starting simulation");

            for (tick, now) in ticktock::Clock::framerate(simulation_rate).iter() {
                {
                    p_engine.state = SimulationState::Running;
                    //Check if received command to exit
                    match command_receiver.receive() {
                        Some(UICommand::QUIT) => p_engine.exit_simulation(),
                        None => (),
                    };
                    // Process all request coming from client
                    if let Err(val) = wasm_tanks.next_step(&mut p_engine) {
                        error!("Error in some tanks{:?}", val);
                        p_engine.exit_simulation();
                    };
                    p_engine.step();

                    // Try to lazily send game state to UI
                    // If failing teh send ignore it.
                    match state_sender.send(&p_engine) {
                        Ok(_) => (),
                        Err(_) => debug!("Failed sending state to UI ignore it"),
                    }

                    // Compute fps and show message if it is too low.
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
        })
    }

    /**
    Add tank to simulation.
    This can be used only before calling  `step` function.
    # Arguments
    * `tank_position` - Initial position of tank
    * `name` - Tank name

    # Return
    * Tank index
    */
    fn add_tank(&mut self, tank_position: Isometry2<Real>, name: String)->usize {
        //This tank index is used to set userdata of all collider to skip detection.
        let tank_index = self.tanks.len();
        let tank = Tank::new(self, tank_position, tank_index, name);
        self.tanks.push(tank);
        tank_index
    }

    /// Execute one simulation step
    fn step(&mut self) {
        //Execute all command
        for (tank_index, tank) in self.tanks.iter_mut().enumerate() {
            let tank_rigid_body = &mut self.rigid_body_set[tank.phy_body_handle];
            /* In new version of Rapier forces are not reset after simulation step.
            So I have to reset them
             */
            tank_rigid_body.reset_forces(false);
            tank_rigid_body.reset_torques(false);

            tank.turret.update_cannon_temp();

            if !tank.update_energy(&self.conf) {
                continue;
            }

            if tank.is_dead() {
                continue;
            }

            // Power = F . v. Here we consider the speed along the direction of tank
            Self::apply_engine_power(tank_rigid_body, tank);
            tank_rigid_body.apply_torque_impulse(
                tank.turning_power / (tank.angular_velocity.abs() + 1.0),
                true,
            );
            tank.set_cannon_position_physics(&mut self.joint_set, &self.conf);
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
                    collider_handle,
                    phy_body_handle: rigid_body_handle,
                    tick_counter: std::cmp::max(
                        1,
                        (self.conf.bullet_max_range / self.conf.bullet_speed * 60.0).ceil() as u32
                            + 1, //+1 because later all bullet will be evaluated and tick will be decreased.
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
            &mut self.multibody_joints,
            &mut self.ccd_solver,
            None,
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
            for contact_pair in self.narrow_phase.contact_pairs_with(bullet.collider_handle) {
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
                if let Some(target_tank_index) = self
                    .tanks
                    .iter()
                    .position(|x| x.collider_handle == other_collider)
                {
                    debug!("Tank {} has been hit", target_tank_index);
                    self.tanks[target_tank_index].bullet_damage()
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
                    &mut self.multibody_joints,
                    true,
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

    /**
    Create one bullet. When bullet is created the speed of cannon edge
    is added to the speed of bullet

    # Arguments
    * `conf` - Simulation configuration
    * `cannon_body` - Cannon body required to get edge speed
    * `tank_index` - index in the [`PhysicsEngine.tanks`]
     */
    fn create_bullet(
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
        let bullet_position = cannon_position * new_point2(1.8, 0.0);
        let bullet_body = RigidBodyBuilder::dynamic()
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

    /**
     * Add tank in a circle at distance as specified in the conf store in p_engine
     *
     * # Arguments
     * `name` - Tank name
     * `max_num_tanks` - Maximum number of expected tanks
     */
    fn add_tank_in_circle(&mut self, name: String,max_num_tanks:usize)->usize {
        let position_vector = Vector2::new(self.conf.start_distance, 0.0);
        //Compute position of new tank
        let tank_pos_angle = (2.0 * PI / max_num_tanks as f32) * (self.tanks.len() + 1) as f32;
        let tank_vector_position = Isometry2::rotation(tank_pos_angle) * position_vector;
        //Angle to compute starting position of tank
        let tank_position = Isometry2::new(tank_vector_position, tank_pos_angle);
        self.add_tank(tank_position, name)
    }

    /// Get how many simulation steps are executed
    #[inline]
    pub fn tick(&self) -> u32 {
        self.tick
    }

    #[inline]
    pub fn tank(&self, tank_id: usize) -> &Tank {
        &self.tanks[tank_id]
    }

    #[inline]
    pub fn tank_mut(&mut self, tank_id: usize) -> &mut Tank {
        &mut self.tanks[tank_id]
    }

    #[inline]
    fn apply_engine_power(tank_rigid_body: &mut RigidBody, tank: &Tank) {
        //We don't have infinite force at 0 speed.
        let force = tank.engine_power / (tank.forward_velocity().abs() + 0.5);
        let force_forward_vector = tank_rigid_body.position() * vector![force, 0.0];
        tank_rigid_body.add_force(force_forward_vector, true);
    }

    fn get_collider_polyline_cuboid(collider: &Collider) -> Vec<Point2<Real>> {
        let cuboid = collider.shape().as_cuboid().unwrap();
        let mut vertexs = cuboid.to_polyline();
        let position = collider.position();
        for v in &mut vertexs {
            *v = position * *v;
        }
        vertexs
    }


    // Set radar position if not enough energy or tank is dead no update and return false
    pub fn set_radar_position(&mut self, tank_id: usize,radar_increment: f32,radar_width: f32)->bool {
         let tank = &mut self.tanks[tank_id];
        // Update radar position
        tank.update_radar_attribute(radar_increment, radar_width)

    }

    /// Move radar and return detected tanks
    pub fn get_radar_result(
        &self,
        tank_id: usize
    ) -> (f32, Vec<(&Tank, f32)>) {
        let tank = &self.tanks[tank_id];
        if tank.is_dead() {
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
                if angle.abs() < tank.radar_width() / 2.0 {
                    result.push((target_tank, distance));
                }
            }
        }
        (tank.radar_position, result)
    }

    pub fn exit_simulation(&self) {
        let path = "simulation_output.csv";
        info!("Exiting simulation and saving result to {}", &path);
        report::save_tank_report(path, &self.tanks).unwrap();
        signal_exit();
    }
}

#[cfg(test)]
mod tests {
    use super::util::*;
    use super::PhysicsEngine;
    use crate::conf::Conf;
    use clap::Parser;
    use float_eq::assert_float_eq;
    use nalgebra::vector;
    pub use rapier2d::na::Vector2;
    use std::f32::consts::PI;

    fn setup_engine(num: u32, distance: Option<f32>) -> PhysicsEngine {
        let mut conf = Conf::default();
        match distance {
            Some(dist) => {
                conf.start_distance = dist;
            }
            None => (),
        };
        let opts = crate::Opts::try_parse_from(["Application", &num.to_string()])
            .expect("Failed parse string");
        let mut engine = PhysicsEngine::new(conf, &opts);
        for x in 0..num {
            engine.add_tank_in_circle(format!("tank{}", x),num as usize);
        }
        engine
    }

    /*
    Test initialization values.
     */
    #[test]
    fn test_setup_initialization_values() {
        let engine = setup_engine(2, None);
        let tank0 = &engine.tanks[0];
        let tank1 = &engine.tanks[1];
        // First tank at -180 degrees
        assert_eq!(tank0.position().rotation.angle(), -3.1415925);
        // First tank at almost 0 degrees
        assert_eq!(tank1.position().rotation.angle(), 0.00000017484555);
        // Check distance from center
        assert_eq!(tank0.position().translation.x, -500.0);
        assert_eq!(tank1.position().translation.x, 500.0);
        // Check velocity
        let vel_lin = tank0.linvel();
        let vel_ang = tank0.angular_velocity();
        assert_eq!(vel_lin, Vector2::new(0.0, 0.0));
        assert_eq!(vel_ang, 0.0);
    }
    #[test]
    fn test_angular_speed() {
        let mut engine = setup_engine(2, None);
        let tank0 = engine.tank_mut(0);
        // Set maximum counterclock and check velocity
        tank0.set_turning_power(1.0);
        assert_eq!(tank0.turning_power_fraction(), 1.0);
        assert_eq!(tank0.angular_velocity(), 0.0);
        for _ in 0..500 {
            engine.step()
        }

        let tank0 = engine.tank_mut(0);
        assert_eq!(tank0.angular_velocity(), 1.0365888);
        // Set to 0 and verify it is stopping
        tank0.set_turning_power(0.0);
        assert_eq!(tank0.turning_power_fraction(), 0.0);
        for _ in 0..500 {
            engine.step()
        }

        let tank0 = engine.tank_mut(0);
        assert_eq!(tank0.angular_velocity(), 0.0);
        let tank0 = engine.tank_mut(0);
        // Set clockwise and check velocity
        tank0.set_turning_power(-1.0);
        assert_eq!(tank0.turning_power_fraction(), -1.0);
        assert_eq!(tank0.angular_velocity(), 0.0);
        for _ in 0..500 {
            engine.step()
        }

        let tank0 = engine.tank_mut(0);
        assert_eq!(tank0.angular_velocity(), -1.0365888);
        // check wrapping set of angular speed
        tank0.set_turning_power(-2.0);
        assert_eq!(tank0.turning_power_fraction(), -1.0);
        let tank0 = engine.tank_mut(0);
        tank0.set_turning_power(2.0);
        assert_eq!(tank0.turning_power_fraction(), 1.0);
    }

    #[test]
    fn test_linear_speed() {
        // 3 to avoid collision
        let mut engine = setup_engine(3, None);
        let tank0 = engine.tank_mut(0);
        // Set maximum forward and check that value is wrapped
        tank0.set_engine_power(0.1);
        assert_eq!(tank0.engine_power_fraction(), 0.1);
        assert_eq!(tank0.linvel().norm(), 0.0);
        assert_eq!(engine.tick, 0, "Wrong tick number");
        for _ in 0..400 {
            engine.step()
        }
        assert_eq!(engine.tick, 400, "Wrong tick number");
        let tank0 = engine.tank_mut(0);
        {
            // Tank angle and velocity angle shall be almost the same
            let velocity_vector = tank0.linvel();
            assert_eq!(velocity_vector.norm(), 8.001642, "Wrong speed");
            let velocity_angle = velocity_vector.y.atan2(velocity_vector.x);
            assert_eq!(velocity_angle, 2.0795524);
            let tank_angle = tank0.position().rotation.angle();
            assert_eq!(tank_angle, 2.079504);
            assert!((velocity_angle - tank_angle).abs() < 0.001);
        }

        // Check position api by computing the distance traveled by tank in 60 min.
        let pos1 = tank0.position().translation.vector;
        for _ in 0..60 {
            engine.step()
        }
        let tank0 = engine.tank_mut(0);
        let pos2 = tank0.position().translation.vector;
        let distance = (pos1 - pos2).norm();
        assert_eq!(distance, 8.002805, "Wrong distance");

        // Stop tank
        tank0.set_engine_power(0.0);
        for _ in 0..400 {
            engine.step()
        }

        let tank0 = engine.tank_mut(0);
        assert_eq!(tank0.linvel().norm(), 0.0, "Wrong speed");
        //Invert direction
        tank0.set_engine_power(-0.1);
        for _ in 0..400 {
            engine.step()
        }

        let tank0 = engine.tank_mut(0);
        let velocity_vector = tank0.linvel();
        assert_eq!(velocity_vector.norm(), 8.001644, "Wrong speed");
        let velocity_angle = velocity_vector.y.atan2(velocity_vector.x);
        assert_eq!(velocity_angle, -1.0620875);
    }

    #[test]
    fn test_turret_move() {
        let mut engine = setup_engine(2, None);
        engine.step();
        let tank1 = engine.tank_mut(1);
        tank1.turret_mut().set_cannon_position(-1.5);
        tank1.set_turning_power(0.5);

        let tank0 = engine.tank_mut(0);
        tank0.turret_mut().set_cannon_position(PI);
        for _ in 0..600 {
            //engine.tank_mut(1).turret_mut().set_cannon_position(-1.5);
            engine.step();
        }

        let tank1 = engine.tank_mut(1);
        assert_float_eq!(
            angle_wrapping(tank1.turret().angle() - tank1.position().rotation.angle()),
            -1.5,
            abs <= 0.06
        );

        let tank0 = engine.tank_mut(0);
        assert_float_eq!(
            angle_wrapping(tank0.turret().angle() - tank0.position().rotation.angle()),
            PI,
            abs <= 0.05
        );

        for _ in 0..600 {
            //engine.tank_mut(1).turret_mut().set_cannon_position(-1.5);
            engine.step();
        }

        // Check that position is stable.
        let tank1 = engine.tank(1);
        assert_float_eq!(
            angle_wrapping(tank1.turret().angle() - tank1.position().rotation.angle()),
            -1.5,
            abs <= 0.06
        );

        let tank0 = engine.tank(0);
        assert_float_eq!(
            angle_wrapping(tank0.turret().angle() - tank0.position().rotation.angle()),
            PI,
            abs <= 0.05
        );
    }

    #[test]
    fn test_bullet() {
        let mut engine = setup_engine(2, Some(50.0));
        for _ in 0..1 {
            //engine.tank_mut(1).turret_mut().set_cannon_position(-1.5);
            engine.step();
        }
        let tank1 = engine.tank_mut(1);
        let turret1 = tank1.turret_mut();
        turret1.fire();
        let turret_angle_at_file = turret1.angle();
        let tank_position_at_fire = tank1.position().translation.vector;
        for _ in 0..1 {
            //engine.tank_mut(1).turret_mut().set_cannon_position(-1.5);
            engine.step();
        }
        // Compare angle of bullet with angle of cannon
        let bullet1 = &engine.bullets[0];
        assert_float_eq!(
            bullet1.position.rotation.angle(),
            turret_angle_at_file,
            abs <= 0.00001
        );

        // Compute velocity vector and angle
        let bullet_position0 = bullet1.position().translation.vector;
        engine.step();
        let bullet_position1 = (&engine.bullets[0]).position().translation.vector;
        let velocity_vector = (bullet_position1 - bullet_position0) * 60.0;
        let velocity_abs = velocity_vector.norm();
        let angle = velocity_vector.angle(&vector![1.0, 0.0]);
        // Speed as defined in conf
        assert_float_eq!(velocity_abs, &engine.conf.bullet_speed, abs <= 0.001);
        // angle the same as the turret when fired.
        assert_float_eq!(angle, turret_angle_at_file, abs <= 0.001);

        // Check that hit and damage other tank
        let mut last_position = bullet_position1;
        while !engine.bullets.is_empty() {
            last_position = (&engine.bullets[0]).position().translation.vector;
            engine.step();
        }
        assert!(
            (last_position - tank_position_at_fire).norm()
                <= engine.conf.bullet_max_range + engine.conf.bullet_speed / 60.0
        );
        println!("{}", last_position - tank_position_at_fire);
    }

    #[test]
    fn test_get_tick() {
        let mut engine = setup_engine(2, None);
        assert!(engine.tick() == 0);
        engine.step();
        assert!(engine.tick() == 1);
    }
}
