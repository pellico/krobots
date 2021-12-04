use macroquad::color::{*};

pub const TANK_WIDTH_M:f32 = 5.0;
pub const TANK_DEPTH_M:f32 = 3.0;
pub const TURRET_WIDTH_M:f32 = 2.5;
pub const TURRET_DEPTH_M:f32 = 0.1;
pub const TURRET_STEP_TO_RELOAD : u32 = 60; //How many simulation steps before reloading
pub const TURRET_STIFFNESS : f32 = 0.1;
pub const TURRET_DAMPING : f32 = 0.8;
pub const TURRET_COLLIDER_DENSITY :f32 = 5.0;
pub const TANK_COLLIDER_DENSITY :f32 = 10.0;
pub const TANK_ENERGY_MAX: f32 = 1.0E8; //Maximum energy of battery
pub const TURNING_IMPULSE_MAX : f32 = 100.0; //Maximum impulse for turning
pub const DAMAGE_MAX:f32=100.0; //Maximum damage before destruction.
pub const LINEAR_DAMPING :f32 = 0.9; 
pub const ANGULAR_DAMPING : f32 = 0.9;
pub const TANK_ENGINE_POWER_MAX : f32 = 10000.0*60.0;
pub const TANK_ENGINE_POWER_MAX_STEP : f32 = TANK_ENGINE_POWER_MAX / 60.0; //Max energy / step 
pub const START_DISTANCE : f32 = 800.0; //Distance from power source used deploy tank at game start
pub const RADAR_ANGLE_INCREMENT_MAX : f32 = std::f32::consts::PI / 180.0 * 10.0;
pub const RADAR_WIDTH_MAX :f32 = std::f32::consts::PI / 180.0 * 10.0;
pub const RADAR_MAX_DETECTION_DISTANCE : f32 = 500.0;
pub const BULLET_MAX_RANGE : f32 = 400.0;
pub const BULLET_SPEED : f32 = 200.0; //  m/sec
pub const BULLET_DAMAGE : f32 =10.0;
pub const BULLET_ENERGY : f32 = 1000.0; //This energy is substracted from tank energy
//Power energy source for each step needed to charge a tank at 0,0 postion in 5 seconds simulation time
pub const POWER_ENERGY_SOURCE_STEP : f32 = TANK_ENERGY_MAX / 5.0 / 60.0; 
// When tank distance from center is >= ZERO_POWER_LIMIT no charging is possible.
// When tank distance from center is < ZERO_POWER_LIMIT charging power is POWER_ENERGY_SOURCE_STEP *(1-distance/ZERO_POWER_LIMIT)
pub const ZERO_POWER_LIMIT : f32 = 500.0;
pub const TANK_COLORS : [Color;11] =[BLUE,GREEN,YELLOW,MAGENTA,VIOLET,PURPLE,LIME,BROWN,ORANGE,DARKBLUE,DARKGREEN];



