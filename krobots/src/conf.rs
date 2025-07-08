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

use confy;
use log::debug;
use serde::{Deserialize, Serialize};
use std::path::Path;

#[derive(Serialize, Deserialize)]
pub struct Conf {
    pub tank_width_m: f32,
    pub tank_depth_m: f32,
    pub turret_width_m: f32,
    pub turret_depth_m: f32,
    pub turret_stiffness: f32,
    pub turret_damping: f32,
    pub turret_collider_density: f32,
    pub tank_collider_density: f32,
    pub tank_energy_max: f32,   //Maximum energy of battery
    pub turning_power_max: f32, //Maximum impulse for turning
    pub damage_max: f32,        //Maximum damage before destruction.
    pub linear_damping: f32,
    pub angular_damping: f32,
    pub tank_engine_power_max: f32, //Max power
    pub start_distance: f32, //700.0, //Distance from power source used deploy tank at game start
    pub radar_angle_increment_max: f32,
    pub radar_width_max: f32,
    pub radar_max_detection_distance: f32,
    pub radar_operation_energy: f32, // Energy consumed to operate radar.
    pub bullet_max_range: f32,
    pub bullet_speed: f32, //  m/sec
    pub bullet_damage: f32,
    pub bullet_energy: f32, //This energy is subtracted from tank energy
    /// Power energy source for each step needed to charge a tank at 0,0 position in 10 seconds simulation time
    pub power_energy_source_step: f32,
    /// When tank distance from center is >= ZERO_POWER_LIMIT no charging is possible.
    /// When tank distance from center is < ZERO_POWER_LIMIT charging power is POWER_ENERGY_SOURCE_STEP *(1-distance/ZERO_POWER_LIMIT)
    pub zero_power_limit: f32,
    /// Minimum temperature of cannon.
    pub cannon_min_temp: f32,
    /// Max temperature of cannon. When cannon temperature is higher than this value
    /// no fire is possible
    pub cannon_max_temp: f32,
    /// Temperature increase for each fire
    pub cannon_fire_temp_increase: f32,
    /// Temperature decrease for each simulation step
    /// When high temp. tank can fire 1 bullet each second  
    pub cannon_temp_decrease_step: f32,
}

impl Conf {
    pub fn load_configuration(path: &str) -> Result<Conf, confy::ConfyError> {
        let path_full = Path::new(path);
        debug!(
            "Writing or reading configuration from path {}",
            path_full.display()
        );
        let conf: Conf = confy::load_path(path_full)?;
        Ok(conf)
    }
}

impl Default for Conf {
    fn default() -> Self {
        const TANK_ENERGY_MAX: f32 = 1.0E7; //Maximum energy of battery
        /// Temperature increase for each fire
        const CANNON_FIRE_TEMP_INCREASE: f32 = 120.0;
        Conf {
            tank_width_m: 7.0,
            tank_depth_m: 3.0,
            turret_width_m: 2.5,
            turret_depth_m: 0.1,
            turret_stiffness: 100.0,
            turret_damping: 40.0,
            turret_collider_density: 5.0,
            tank_collider_density: 7.0,
            tank_energy_max: TANK_ENERGY_MAX, //Maximum energy of battery
            turning_power_max: 25.0,          //Maximum impulse for turning
            damage_max: 100.0,                //Maximum damage before destruction.
            linear_damping: 1.0,
            angular_damping: 1.0,
            tank_engine_power_max: 100000.0, //Max power
            start_distance: 500.0, //700.0, //Distance from power source used deploy tank at game start
            radar_angle_increment_max: std::f32::consts::PI / 180.0 * 20.0,
            radar_width_max: std::f32::consts::PI / 180.0 * 10.0,
            radar_max_detection_distance: 300.0,
            radar_operation_energy: 1000.0, // Energy consumed to operate radar.
            bullet_max_range: 200.0,
            bullet_speed: 800.0, //  m/sec
            bullet_damage: 10.0,
            bullet_energy: 100000.0, //This energy is subtracted from tank energy
            // Power energy source for each step needed to charge a tank at 0,0 position in 10 seconds simulation time
            power_energy_source_step: TANK_ENERGY_MAX / 60.0 / 60.0,
            // When tank distance from center is >= ZERO_POWER_LIMIT no charging is possible.
            // When tank distance from center is < ZERO_POWER_LIMIT charging power is POWER_ENERGY_SOURCE_STEP *(1-distance/ZERO_POWER_LIMIT)
            zero_power_limit: 300.0,
            // Minimum temperature of cannon.
            cannon_min_temp: 20.0,
            // Max temperature of cannon. When cannon temperature is higher than this value
            // no fire is possible
            cannon_max_temp: 320.0,
            // Temperature increase for each fire
            cannon_fire_temp_increase: CANNON_FIRE_TEMP_INCREASE,
            // Temperature decrease for each simulation step
            // When high temp. tank can fire 1 bullet each second  
            cannon_temp_decrease_step: CANNON_FIRE_TEMP_INCREASE / 60.0,
        }
    }
}
