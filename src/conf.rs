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

pub const TANK_WIDTH_M:f32 = 7.0;
pub const TANK_DEPTH_M:f32 = 3.0;
pub const TURRET_WIDTH_M:f32 = 2.5;
pub const TURRET_DEPTH_M:f32 = 0.1;
pub const TURRET_STIFFNESS : f32 = 0.1;
pub const TURRET_DAMPING : f32 = 0.8;
pub const TURRET_COLLIDER_DENSITY :f32 = 5.0;
pub const TANK_COLLIDER_DENSITY :f32 = 7.0;
pub const TANK_ENERGY_MAX: f32 = 1.0E7; //Maximum energy of battery
pub const TURNING_POWER_MAX : f32 = 25.0; //Maximum impulse for turning
pub const DAMAGE_MAX:f32=100.0; //Maximum damage before destruction.
pub const LINEAR_DAMPING :f32 = 1.0; 
pub const ANGULAR_DAMPING : f32 = 1.0;
pub const TANK_ENGINE_POWER_MAX : f32 = 100000.0; //Max power
pub const START_DISTANCE : f32 = 300.0;//700.0; //Distance from power source used deploy tank at game start
pub const RADAR_ANGLE_INCREMENT_MAX : f32 = std::f32::consts::PI / 180.0 * 20.0;
pub const RADAR_WIDTH_MAX :f32 = std::f32::consts::PI / 180.0 * 10.0;
pub const RADAR_MAX_DETECTION_DISTANCE : f32 = 400.0;
pub const RADAR_OPERATION_ENERGY : f32 = 1000.0; // Energy consumed to operate radar.
pub const BULLET_MAX_RANGE : f32 = 300.0;
pub const BULLET_SPEED : f32 = 800.0; //  m/sec
pub const BULLET_DAMAGE : f32 =10.0;
pub const BULLET_ENERGY : f32 = 100000.0; //This energy is subtracted from tank energy
/// Power energy source for each step needed to charge a tank at 0,0 position in 10 seconds simulation time
pub const POWER_ENERGY_SOURCE_STEP : f32 = TANK_ENERGY_MAX / 60.0 / 60.0; 
/// When tank distance from center is >= ZERO_POWER_LIMIT no charging is possible.
/// When tank distance from center is < ZERO_POWER_LIMIT charging power is POWER_ENERGY_SOURCE_STEP *(1-distance/ZERO_POWER_LIMIT)
pub const ZERO_POWER_LIMIT : f32 = 400.0;
/// Minimum temperature of cannon.
pub const CANNON_MIN_TEMP : f32 = 20.0;
/// Max temperature of cannon. When cannon temperature is higher than this value
/// no fire is possible
pub const CANNON_MAX_TEMP : f32 = 320.0;
/// Temperature increase for each fire 
pub const CANNON_FIRE_TEMP_INCREASE :f32 = 120.0;
/// Temperature decrease for each simulation step
/// When high temp. tank can fire 1 bullet each second  
pub const CANNON_TEMP_DECREASE_STEP :f32 = CANNON_FIRE_TEMP_INCREASE / 60.0;

