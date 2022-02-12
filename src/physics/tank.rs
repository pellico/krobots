use crate::conf::*;
use serde::{Serialize, Deserialize};
use rapier2d::prelude::*;
use rapier2d::na::{Isometry2};
use rapier2d::na::{Point2, Vector2};
use super::util::*;

#[derive(Clone, Debug,Serialize,Deserialize)]
pub struct Turret {
    pub (super) phy_body_handle: RigidBodyHandle,
    pub (super) collider_handle: ColliderHandle,
    pub angle: f32, //Updated during step
    pub shape_polyline: Vec<Point2<Real>>,
    pub fire: bool,
    pub new_angle: Option<f32>, // New position None if no command change 
    pub (super) cannon_temperature: f32,
}

#[derive(Clone, Debug,Serialize,Deserialize)]
pub struct Bullet {
    pub (super) phy_body_handle: RigidBodyHandle,
    pub (super) collider_handle: ColliderHandle,
    pub (super) tick_counter: u32, //tick count down when zero the bullet will be destroyed
    pub shape_polyline: Vec<Point2<Real>>,
    pub position: Isometry2<Real>,
}

#[derive(Clone, Debug,Serialize,Deserialize)]
pub struct Tank {
    pub (super) phy_body_handle: RigidBodyHandle,
    pub (super) collider_handle: ColliderHandle,
    pub (super) cannon_joint_handle: JointHandle,
    pub name: String,
    pub turret: Turret,
    pub damage: f32,
    pub (super) energy: f32,
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
}

impl Tank {
    #[inline]
    pub fn linear_velocity(&self) -> Real {
        self.linvel.norm()
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

    pub fn ready_to_fire(&self) -> bool {
        self.turret.cannon_temperature <= CANNON_MAX_TEMP
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
    pub (super) fn bullet_damage(&mut self) {
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

    /// Update radar position and decrease energy if there is enough energy available
    /// Return false if there is not enough energy.
    pub (super) fn update_radar_attribute (
        &mut self,
        radar_increment: f32,
        radar_width: f32,
    ) -> bool {
        // If tank is dead do nothing and return
        if self.is_dead() || self.energy < RADAR_OPERATION_ENERGY {
                return false;
        }
        self.energy -= RADAR_OPERATION_ENERGY;
        let radar_incr = wrap_value(
            radar_increment,
            -RADAR_ANGLE_INCREMENT_MAX,
            RADAR_ANGLE_INCREMENT_MAX,
        );
        let radar_w = wrap_value(radar_width, 0.0, RADAR_WIDTH_MAX);
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
        let new_temp = self.cannon_temperature - CANNON_TEMP_DECREASE_STEP;
        self.cannon_temperature = CANNON_MIN_TEMP.max(new_temp);
    }
}