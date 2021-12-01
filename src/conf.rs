pub const TANK_WIDTH_M:f32 = 5.0;
pub const TANK_DEPTH_M:f32 = 3.0;
pub const TURRET_WIDTH_M:f32 = 2.5;
pub const TURRET_DEPTH_M:f32 = 0.1;
pub const TURRET_STIFFNESS : f32 = 0.1;
pub const TURRET_DAMPING : f32 = 0.8;
pub const TURRET_COLLIDER_DENSITY :f32 = 5.0;
pub const TANK_COLLIDER_DENSITY :f32 = 10.0;
pub const TANK_ENERGY_MAX: f32 = 100.0; //Maximum energy of battery
pub const TURNING_IMPULSE_MAX : f32 = 100.0; //Maximum impulse for turning
pub const DAMAGE_MAX:f32=100.0; //Maximum damage before destruction.
pub const LINEAR_DAMPING :f32 = 0.9; 
pub const ANGULAR_DAMPING : f32 = 0.9;
pub const TANK_ENGINE_POWER_MAX : f32 = 10000.0;
pub const START_DISTANCE : f32 = 10.0; //Distance from power source used deploy tank at game start
pub const RADAR_ANGLE_INCREMENT_MAX : f32 = std::f32::consts::PI / 180.0 * 10.0;
pub const RADAR_WIDTH_MAX :f32 = std::f32::consts::PI / 180.0 * 10.0;
pub const RADAR_MAX_DETECTION_DISTANCE : f32 = 500.0;
pub const BULLET_MAX_RANGE : f32 = 5000.0;
pub const BULLET_SPEED : f32 = 1000.0;


