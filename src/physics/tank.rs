use super::util::*;
use super::PhysicsEngine;
use crate::conf::*;
use rapier2d::na::Isometry2;
use rapier2d::na::{Point2, Vector2};
use rapier2d::prelude::*;
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Turret {
    pub(super) phy_body_handle: RigidBodyHandle,
    pub(super) collider_handle: ColliderHandle,
    pub angle: f32, //Updated during step
    pub shape_polyline: Vec<Point2<Real>>,
    pub fire: bool,
    pub new_angle: Option<f32>, // New position None if no command change
    pub(super) cannon_temperature: f32,
    cannon_max_temp: f32,
    cannon_min_temp: f32,
    cannon_temp_decrease_step: f32,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Bullet {
    pub(super) phy_body_handle: RigidBodyHandle,
    pub(super) collider_handle: ColliderHandle,
    pub(super) tick_counter: u32, //tick count down when zero the bullet will be destroyed
    pub shape_polyline: Vec<Point2<Real>>,
    pub position: Isometry2<Real>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Tank {
    pub(super) phy_body_handle: RigidBodyHandle,
    pub(super) collider_handle: ColliderHandle,
    pub(super) cannon_joint_handle: ImpulseJointHandle,
    pub name: String,
    pub turret: Turret,
    pub damage: f32,
    pub(super) energy: f32,
    pub engine_power: f32,
    pub max_engine_power: f32,
    pub turning_power_max: f32,
    pub turning_power: f32,
    pub shape_polyline: Vec<Point2<Real>>,
    pub position: Isometry<Real>,
    pub linvel: Vector<Real>,
    pub angular_velocity: Real, //Present angular velocity
    pub radar_position: f32,
    pub radar_width: f32,
    pub detected_tank: Vec<Tank>,
    tank_energy_max: f32,
    damage_max: f32,
    radar_angle_increment_max: f32,
    radar_operation_energy : f32,
    radar_width_max : f32,
    pub(super) radar_max_detection_distance : f32,
    bullet_damage : f32,

}

impl Tank {
    pub fn new(p_engine:&mut PhysicsEngine,tank_position:Isometry<Real>,tank_index:usize,name:String) -> Tank {
        let body = RigidBodyBuilder::dynamic()
            .position(tank_position)
            .linear_damping(p_engine.conf.linear_damping)
            .angular_damping(p_engine.conf.angular_damping)
            .build();

        let rigid_body_handle = p_engine.rigid_body_set.insert(body);

        let collider =
            ColliderBuilder::cuboid(p_engine.conf.tank_width_m / 2.0, p_engine.conf.tank_depth_m / 2.0)
                .restitution(0.7)
                .density(p_engine.conf.tank_collider_density)
                .collision_groups(super::TANK_GROUP)
                .active_hooks(ActiveHooks::FILTER_CONTACT_PAIRS)
                .user_data(tank_index as u128)
                .build();
        let shape_polyline_tank = PhysicsEngine::get_collider_polyline_cuboid(&collider);
        let collider_handle = p_engine.collider_set.insert_with_parent(
            collider,
            rigid_body_handle,
            &mut p_engine.rigid_body_set,
        );

        /*
        Setup turret
        */
        let turret_body = RigidBodyBuilder::dynamic()
            .translation(tank_position.translation.vector)
            .rotation(0.0)
            .build();
        let rigid_body_turret_handle = p_engine.rigid_body_set.insert(turret_body);

        let turret_collider = ColliderBuilder::cuboid(
            p_engine.conf.turret_width_m / 2.0,
            p_engine.conf.turret_depth_m / 2.0,
        )
        .density(p_engine.conf.turret_collider_density)
        .collision_groups(super::TURRET_GROUP)
        .active_hooks(ActiveHooks::FILTER_CONTACT_PAIRS)
        .user_data(tank_index as u128)
        .build();

        let shape_polyline_turret = PhysicsEngine::get_collider_polyline_cuboid(&turret_collider);

        let collider_turret_handle = p_engine.collider_set.insert_with_parent(
            turret_collider,
            rigid_body_turret_handle,
            &mut p_engine.rigid_body_set,
        );
        // Create joint to move turret together with tank.
        let joint = RevoluteJointBuilder::new()
        .local_anchor1(point![0.0, 0.0])
        .local_anchor2(point![-p_engine.conf.turret_width_m / 2.0, 0.0])
        .motor_model(MotorModel::AccelerationBased)
        .motor_position(
            0.0,
            p_engine.conf.turret_stiffness,
            p_engine.conf.turret_damping,
        ).build();
        let cannon_joint_handle =
            p_engine.joint_set
                .insert(rigid_body_handle, rigid_body_turret_handle, joint,true);

        Tank {
            name,
            phy_body_handle: rigid_body_handle,
            collider_handle,
            cannon_joint_handle,
            energy: p_engine.conf.tank_energy_max,
            damage: 0.0,
            turret: Turret {
                phy_body_handle: rigid_body_turret_handle,
                collider_handle: collider_turret_handle,
                angle: 0.0,
                shape_polyline: shape_polyline_turret,
                fire: false,
                new_angle: None,
                cannon_temperature: p_engine.conf.cannon_min_temp,
                cannon_max_temp : p_engine.conf.cannon_max_temp,
                cannon_min_temp : p_engine.conf.cannon_min_temp,
                cannon_temp_decrease_step : p_engine.conf.cannon_temp_decrease_step,
            },
            engine_power: 0.0,
            max_engine_power: p_engine.conf.tank_engine_power_max,
            turning_power: 0.0,
            turning_power_max: p_engine.conf.turning_power_max,
            shape_polyline: shape_polyline_tank,
            position: Isometry2::identity(),
            linvel: Vector2::identity(),
            angular_velocity: 0.0,
            radar_position: 0.0,
            radar_width: p_engine.conf.radar_width_max,
            detected_tank: Vec::new(),
            tank_energy_max : p_engine.conf.tank_energy_max,
            damage_max : p_engine.conf.damage_max,
            radar_angle_increment_max : p_engine.conf.radar_angle_increment_max,
            radar_operation_energy : p_engine.conf.radar_operation_energy,
            radar_width_max : p_engine.conf.radar_width_max,
            radar_max_detection_distance : p_engine.conf.radar_max_detection_distance,
            bullet_damage : p_engine.conf.bullet_damage
        }
    }

    #[inline]
    pub fn linear_velocity(&self) -> Real {
        self.linvel.norm()
    }

    #[inline]
    pub fn radar_range(&self) -> f32 {
        self.radar_max_detection_distance
    }

    #[inline]
    /// Get the velocity along the tank direction
    pub fn forward_velocity(&self) -> Real {
        let unit_vector = Vector2::<f32>::identity();
        let direction_vector = self.position * unit_vector;
        direction_vector.dot(&self.linvel)
    }

    #[inline]
    pub fn energy(&self) -> f32 {
        self.energy
    }

    /**
     * Set cannon position
     */
    pub (super) fn set_cannon_position(&mut self, joint_set: &mut ImpulseJointSet,conf:&Conf) {
        match self.turret.new_angle {
            Some(angle) => {
                let joint = joint_set.get_mut(self.cannon_joint_handle).unwrap();
                if let Some(ball_joint) = joint.data.as_revolute_mut() {
                    ball_joint.set_motor_position(
                        angle,
                        conf.turret_stiffness,
                        conf.turret_damping,
                    );
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
        } else if new_energy > self.tank_energy_max {
            self.tank_energy_max
        } else {
            new_energy
        }
    }

    /*
    Update tank energy
    return true if there is enough energy for the operation
    :TODO: Bad design split in two function ?
    */
    pub fn update_energy(&mut self,conf:&Conf) -> bool {
        // Decrease if a bullet is fired
        let bullet_energy = if self.turret.fire { conf.bullet_energy } else { 0.0 };
        // Energy transmission decrease linearly and go to zero at distance ZERO_POWER_LIMIT
        let distance_from_center = self.position.translation.vector.norm();
        // Beyond ZERO_POWER_LIMIT energy will be drained from the tank.
        let charged_energy =
            conf.power_energy_source_step * (1.0 - distance_from_center / conf.zero_power_limit);
        let delta_energy =
            -self.engine_power.abs() / 60.0 - self.turning_power.abs() / 60.0 - bullet_energy
                + charged_energy;
        let new_energy = self.energy + delta_energy;
        if new_energy < 0.0 {
            self.delta_energy(charged_energy);
            false
        } else {
            self.delta_energy(delta_energy);
            true
        }
    }

    pub fn ready_to_fire(&self) -> bool {
        self.turret.cannon_temperature <= self.turret.cannon_max_temp
    }

    /*
    Get min max angle in world coordinates
    */
    pub fn min_max_radar_angle(&self) -> (f32, f32) {
        let world_angle = self.radar_position + self.position.rotation.angle();
        let max_angle = angle_wrapping(world_angle + self.radar_width / 2.0);
        let min_angle = angle_wrapping(world_angle - self.radar_width / 2.0);
        (min_angle, max_angle)
    }

    /*
    Compute and apply bullet damage.
    If tanks is dead stop any motion.
    */
    pub(super) fn bullet_damage(&mut self) {
        self.damage += self.bullet_damage;
        if self.is_dead() {
            self.engine_power = 0.0;
            self.turning_power = 0.0;
        }
    }

    #[inline]
    pub fn is_dead(&self) -> bool {
        self.damage > self.damage_max
    }

    /// Update radar position and decrease energy if there is enough energy available
    /// Return false if there is not enough energy.
    pub(super) fn update_radar_attribute(
        &mut self,
        radar_increment: f32,
        radar_width: f32,
    ) -> bool {
        // If tank is dead do nothing and return
        if self.is_dead() || self.energy < self.radar_operation_energy {
            return false;
        }
        self.energy -= self.radar_operation_energy;
        let radar_incr = wrap_value(
            radar_increment,
            -self.radar_angle_increment_max,
            self.radar_angle_increment_max,
        );
        let radar_w = wrap_value(radar_width, 0.0, self.radar_width_max);
        self.radar_position += radar_incr;
        //Keep in expected range
        self.radar_position = angle_wrapping(self.radar_position);
        self.radar_width = radar_w;
        true
    }
}

impl Turret {
    /// Update the temperature cannon.
    /// executed at every simulation step
    #[inline]
    pub fn update_cannon_temp(&mut self) {
        let new_temp = self.cannon_temperature - self.cannon_temp_decrease_step;
        self.cannon_temperature = self.cannon_min_temp.max(new_temp);
    }
}
