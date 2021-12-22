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
use crate::conf::*;
use log::{debug, error, info, trace};
pub use nalgebra::{vector, Isometry2, Rotation2};
pub use nalgebra::{Point2, Vector2};
pub use rapier2d::prelude::Real;
use rapier2d::prelude::*;
use std::f32::consts::PI;
mod report;

const TANK_GROUP: InteractionGroups = InteractionGroups::new(0b001, 0b101);
const TURRET_GROUP: InteractionGroups = InteractionGroups::new(0b010, 0b110);
const BULLET_GROUP: InteractionGroups = InteractionGroups::new(0b100, 0b011);

#[derive(Clone, Debug)]
pub struct Turret {
    phy_body_handle: RigidBodyHandle,
    collider_handle: ColliderHandle,
    pub angle: f32, //Updated during step
    pub shape_polyline: Vec<Point2<Real>>,
    pub fire: bool,
    pub new_angle: Option<f32>, // New position None if no command change position
}

#[derive(Clone, Debug)]
pub struct Bullet {
    phy_body_handle: RigidBodyHandle,
    collider_handle: ColliderHandle,
    tick_counter: u32, //tick count down when zero the bullet will be destroyed
    pub shape_polyline: Vec<Point2<Real>>,
    pub position: Isometry2<Real>,
}

#[derive(Clone, Debug)]
pub struct Tank {
    phy_body_handle: RigidBodyHandle,
    collider_handle: ColliderHandle,
    cannon_joint_handle: JointHandle,
    pub name: String,
    pub turret: Turret,
    pub damage: f32,
    energy: f32,
    pub engine_power: f32, 
    pub max_engine_power: f32,
    pub max_turning_power: f32,
    pub turning_power: f32, 
    pub shape_polyline: Vec<Point2<Real>>,
    pub position: Isometry<Real>,
    pub linvel: Vector<Real>,
    pub angular_velocity: Real, //Present angular velocity
    pub radar_position: f32,
    pub radar_width: f32,
    pub detected_tank: Vec<Tank>,
    pub cannon_heat: u32,
}

fn wrap_value<T: PartialOrd + Copy>(value: T, lower: T, upper: T) -> T {
    if value > upper {
        upper
    } else if value < lower {
        lower
    } else {
        value
    }
}

/*
Velocity of a point of rigidbody
# Arguments

* `x` - x coordinates relative to rigid body
* `y` - y coordinates relative to rigid body
* `body` - rigidbody
*/
fn get_velocity_at_point(x: f32, y: f32, rigid_body: &RigidBody) -> Vector<Real> {
    let point_relative = Point::new(x, y);
    let point_world = rigid_body.position() * point_relative;
    rigid_body.velocity_at_point(&point_world)
}

/// Wrap angle in range ]-PI,PI]
///
/// ```
/// let result = angle_wrapping(PI);
/// assert_eq!(result, -PI);
/// ```
fn angle_wrapping(angle: f32) -> f32 {
    let mut angle_res = angle;
    loop {
        if angle_res > PI {
            angle_res = angle_res - 2.0 * PI
        } else if angle_res <= -PI {
            angle_res = angle_res + 2.0 * PI
        } else {
            break;
        }
    }
    angle_res
}

impl Tank {
    #[inline]
    pub fn linear_velocity(&self) -> Real {
        self.linvel.norm()
    }

    #[inline]
    pub fn forward_velocity(&self) -> Real {
        let unit_vector = Vector2::<f32>::identity();
        let direction_vector = self.position * unit_vector;
        direction_vector.dot(&self.linvel)
    }

    #[inline]
    pub fn energy(&self) -> f32 {
        self.energy
    }

    pub fn set_cannon_position(&mut self, joint_set: &mut JointSet) {
        match self.turret.new_angle {
            Some(angle) => {
                let joint = joint_set.get_mut(self.cannon_joint_handle).unwrap();
                match &mut joint.params {
                    JointParams::BallJoint(ball_joint) => ball_joint.configure_motor_position(
                        Rotation::new(angle),
                        TURRET_STIFFNESS,
                        TURRET_DAMPING,
                    ),
                    _ => (),
                };
                self.turret.new_angle = None;
            }
            None => (),
        };
    }

    /*
    Add delta to energy. Energy will not go lower than 0 and higher TANK_ENERGY_MAX
    */
    #[inline]
    fn delta_energy(&mut self, delta: f32) {
        let new_energy = self.energy + delta;
        self.energy = if new_energy < 0.0 {
            0.0
        } else if new_energy > TANK_ENERGY_MAX {
            TANK_ENERGY_MAX
        } else {
            new_energy
        }
    }

    /*
    Update tank energy
    return true if there is enough energy for the operation
    :TODO: Bad design split in two function ?
    */
    pub fn update_energy(&mut self) -> bool {
        // Decrease if a bullet is fired
        let bullet_energy = if self.turret.fire { BULLET_ENERGY } else { 0.0 };
        // Energy transmission decrease linearly and go to zero at distance ZERO_POWER_LIMIT
        let distance_from_center = self.position.translation.vector.norm();
        // Beyond ZERO_POWER_LIMIT energy will be drained from the tank.
        let charged_energy =
            POWER_ENERGY_SOURCE_STEP * (1.0 - distance_from_center / ZERO_POWER_LIMIT);
        let delta_energy =
            -self.engine_power.abs()/60.0 - self.turning_power.abs()/60.0 - bullet_energy + charged_energy;
        let new_energy = self.energy + delta_energy;
        if new_energy < 0.0 {
            self.delta_energy(charged_energy);
            false
        } else {
            self.delta_energy(delta_energy);
            true
        }
    }

    pub fn update_timers(&mut self) {
        if self.cannon_heat > 0 {
            self.cannon_heat -= 1;
        }
    }

    pub fn ready_to_fire(&self) -> bool {
        self.cannon_heat <= CANNON_TEMP_LIMIT
    }

    /*
    Get min max angle in world coordinates
    */
    pub fn min_max_radar_angle(&self) -> (f32, f32) {
        let world_angle = self.radar_position + self.position.rotation.angle();
        let max_angle = world_angle + self.radar_width / 2.0;
        let min_angle = world_angle - self.radar_width / 2.0;
        (min_angle, max_angle)
    }

    /*
    Compute and apply bullet damage.
    If tanks is dead stop any motion.
    */
    fn bullet_damage(&mut self) {
        self.damage += BULLET_DAMAGE;
        if self.is_dead() {
            self.engine_power = 0.0;
            self.turning_power = 0.0;
        }
    }

    #[inline]
    pub fn is_dead(&self) -> bool {
        self.damage > DAMAGE_MAX
    }
}

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

pub struct PhysicsEngine {
    max_ticks: u32,
    tanks_alive: u16,
    pub tanks: Vec<Tank>,
    pub bullets: Vec<Bullet>,
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
}

impl PhysicsEngine {
    pub fn add_tank(&mut self, tank_position: Isometry2<Real>, name: String) {
        //This tank index is used to set userdata of all collider to skip detection.
        let tank_index = self.tanks.len();
        let body = RigidBodyBuilder::new_dynamic()
            .position(tank_position)
            .linear_damping(LINEAR_DAMPING)
            .angular_damping(ANGULAR_DAMPING)
            .build();

        let rigid_body_handle = self.rigid_body_set.insert(body);

        let collider = ColliderBuilder::cuboid(TANK_WIDTH_M / 2.0, TANK_DEPTH_M / 2.0)
            .restitution(0.7)
            .density(TANK_COLLIDER_DENSITY)
            .collision_groups(TANK_GROUP)
            .active_hooks(ActiveHooks::FILTER_CONTACT_PAIRS)
            .user_data(tank_index as u128)
            .build();
        let shape_polyline_tank = Self::get_collider_polyline_cuboid(&collider);
        let collider_handle = self.collider_set.insert_with_parent(
            collider,
            rigid_body_handle,
            &mut self.rigid_body_set,
        );

        /*
        Setup turret
        */
        let turret_body = RigidBodyBuilder::new_dynamic()
            .translation(tank_position.translation.vector)
            .rotation(0.0)
            .build();
        let rigid_body_turret_handle = self.rigid_body_set.insert(turret_body);

        let turret_collider = ColliderBuilder::cuboid(TURRET_WIDTH_M / 2.0, TURRET_DEPTH_M / 2.0)
            .density(TURRET_COLLIDER_DENSITY)
            .collision_groups(TURRET_GROUP)
            .active_hooks(ActiveHooks::FILTER_CONTACT_PAIRS)
            .user_data(tank_index as u128)
            .build();

        let shape_polyline_turret = Self::get_collider_polyline_cuboid(&turret_collider);

        let collider_turret_handle = self.collider_set.insert_with_parent(
            turret_collider,
            rigid_body_turret_handle,
            &mut self.rigid_body_set,
        );
        // Create joint to move turret toghether with tank.
        let mut joint = BallJoint::new(point![0.0, 0.0], point![-TURRET_WIDTH_M / 2.0, 0.0]);
        joint.configure_motor_model(SpringModel::VelocityBased);
        joint.configure_motor_position(Rotation::new(0.0), TURRET_STIFFNESS, TURRET_DAMPING);
        let cannon_joint_handle =
            self.joint_set
                .insert(rigid_body_handle, rigid_body_turret_handle, joint);

        let tank = Tank {
            name: name,
            phy_body_handle: rigid_body_handle,
            collider_handle: collider_handle,
            cannon_joint_handle: cannon_joint_handle,
            energy: TANK_ENERGY_MAX,
            damage: 0.0,
            turret: Turret {
                phy_body_handle: rigid_body_turret_handle,
                collider_handle: collider_turret_handle,
                angle: 0.0,
                shape_polyline: shape_polyline_turret,
                fire: false,
                new_angle: None,
            },
            engine_power: 0.0,
            max_engine_power: TANK_ENGINE_POWER_MAX,
            turning_power: 0.0,
            max_turning_power: TURNING_POWER_MAX,
            shape_polyline: shape_polyline_tank,
            position: Isometry2::identity(),
            linvel: Vector2::identity(),
            angular_velocity: 0.0,
            radar_position: 0.0,
            radar_width: RADAR_WIDTH_MAX,
            detected_tank: Vec::new(),
            cannon_heat: 0,
        };
        self.tanks.push(tank);
    }

    pub fn step(&mut self) {
        //Execute all command
        for (index, tank) in self.tanks.iter_mut().enumerate() {
            tank.update_timers();

            if !tank.update_energy() {
                continue;
            }

            if tank.is_dead() {
                continue;
            }

            let tank_rigid_body = &mut self.rigid_body_set[tank.phy_body_handle];
            Self::apply_engine_power(tank_rigid_body, tank.engine_power);
            tank_rigid_body
                .apply_torque_impulse(tank.turning_power / (tank.angular_velocity.abs() + 1.0), true);
            tank.set_cannon_position(&mut self.joint_set);
            let turret = &mut tank.turret;
            if turret.fire {
                let (bullet_body, collider) =
                    Self::create_bullet(&self.rigid_body_set[turret.phy_body_handle], index);
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
                    tick_counter: std::cmp::max(1, (BULLET_MAX_RANGE / BULLET_SPEED * 60.0) as u32), //remember that step is 1/60 simulation sec.
                    shape_polyline: collider_polyline,
                    position: bullet_position,
                };
                self.bullets.push(bullet);
                turret.fire = false;
                tank.cannon_heat += CANNON_HEAT_FOR_FIRE;
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

    pub fn create_bullet(cannon_body: &RigidBody, tank_index: usize) -> (RigidBody, Collider) {
        let cannon_position = cannon_body.position();
        let velocity_cannon_edge = get_velocity_at_point(TURRET_WIDTH_M / 2.0, 0.0, cannon_body);
        let angle = cannon_position.rotation.angle();
        debug!("Created bullet angle:{}", angle * 180.0 / PI);
        //Compute bullet speed and sum cannon edge speed (world speed)
        let velocity = (cannon_position * vector![BULLET_SPEED, 0.0]) + velocity_cannon_edge;
        //bullet shall be created in front of cannon and outside of the tank
        let bullet_position = cannon_position * Point2::new(1.8, 0.0);
        let bullet_body = RigidBodyBuilder::new_dynamic()
            .translation(bullet_position.coords)
            .linvel(velocity)
            .rotation(angle)
            .ccd_enabled(true)
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
        self.tanks[tank_id].engine_power / TANK_ENGINE_POWER_MAX
    }
    pub fn set_tank_engine_power(&mut self, energy: f32, tank_id: usize) {
        let tank = &mut self.tanks[tank_id];
        let energy = if energy > 1.0 {
            1.0
        } else if energy < -1.0 {
            -1.0
        } else {
            energy
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
    fn apply_engine_power(tank_rigid_body: &mut RigidBody, power: f32) {
        if power != 0.0 {
            //force is liner anly when speed is greater than 1 . We don't have infinite force at 0 speed.
            let tank_speed = tank_rigid_body.linvel().norm();
            let force = power / (tank_speed.abs() + 1.0);
            let force_forward_vector = tank_rigid_body.position() * vector![force, 0.0];
            tank_rigid_body.apply_force(force_forward_vector, true);
        }
    }

    pub fn tank_turning_power(&self, tank_id: usize) -> f32 {
        self.tanks[tank_id].turning_power / TURNING_POWER_MAX
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
        tank.turning_power = tank.max_turning_power * power_fraction_wrapped;
    }

    fn get_forward_speed(position: &Isometry2<Real>, speed: &Vector2<f32>) -> Vector2<f32> {
        let unit_vector = Vector2::<f32>::identity();
        let direction_vector = position * unit_vector;
        speed - direction_vector.dot(speed) * direction_vector
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

    pub fn update_radar_attribute(
        &mut self,
        tank_id: usize,
        radar_increment: f32,
        radar_width: f32,
    ) {
        let radar_incr = wrap_value(
            radar_increment,
            -RADAR_ANGLE_INCREMENT_MAX,
            RADAR_ANGLE_INCREMENT_MAX,
        );
        let radar_w = wrap_value(radar_width, 0.0, RADAR_WIDTH_MAX);
        let tank = &mut self.tanks[tank_id];
        tank.radar_position += radar_incr;
        //Keep in expected range
        tank.radar_position = angle_wrapping(tank.radar_position);
        tank.radar_width = radar_w;
    }

    /// Move radar and return detected tanks
    pub fn get_radar_result(&self, tank_id: usize) -> (f32, Vec<(&Tank, f32)>) {
        let mut result = Vec::new();
        let tank = &self.tanks[tank_id];
        // I shall use present value not the one of step ago stored in the Tank struct
        let tank_position = self.rigid_body_set[tank.phy_body_handle].position();
        //Search detected tank
        for target_tank in &self.tanks {
            //Tank don't detect itself
            if std::ptr::eq(target_tank, tank) {
                continue;
            }
            let target_tank_position = self.rigid_body_set[target_tank.phy_body_handle].position();
            // This is the vector from this tank to target tank
            let relative_vector =
                target_tank_position.translation.vector - tank_position.translation.vector;
            let distance = relative_vector.norm();

            if distance < RADAR_MAX_DETECTION_DISTANCE {
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

    pub fn exit_simulation(&self) -> ! {
        info!("Exiting simulation");
        report::save_tank_report("simulation.output.csv", &self.tanks).unwrap();
        std::process::exit(0);
    }
}

pub fn create_physics_engine(max_steps: u32) -> PhysicsEngine {
    let engine = PhysicsEngine {
        max_ticks: max_steps,
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
    };
    return engine;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_add() {
        let mut result = angle_wrapping(PI);
        assert_eq!(result, PI);
        result = angle_wrapping(-PI);
        assert_eq!(result, PI);
        result = angle_wrapping(-2.0 * PI);
        assert_eq!(result, 0.0);
        result = angle_wrapping(-4.0 * PI);
        assert_eq!(result, 0.0);
        result = angle_wrapping(4.0 * PI);
        assert_eq!(result, 0.0);
    }
}
