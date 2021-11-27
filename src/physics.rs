use crate::conf::*;
use nalgebra::{vector, Isometry2,Rotation2};
pub use nalgebra::{Point2, Vector2};
pub use rapier2d::prelude::Real;
use rapier2d::prelude::*;
use std::f32::consts::PI;

const TANK_GROUP: InteractionGroups = InteractionGroups::new(0b01, 0b01);
const TURRET_GROUP: InteractionGroups = InteractionGroups::new(0b10, 0b10);

#[derive(Clone)]
pub struct Turret {
    phy_body_handle: RigidBodyHandle,
    collider_handle: ColliderHandle,
    pub angle : f32, //Updated during step
    pub shape_polyline : Vec<Point2<Real>>,
}

#[derive(Clone)]
pub struct Tank {
    phy_body_handle: RigidBodyHandle,
    collider_handle: ColliderHandle,
    pub turret: Turret,
    pub damage: f32,
    pub energy: f32,
    pub engine_power : f32, // [-1.0,1.0]
    pub max_engine_power : f32,
    pub max_turning_impulse : f32,
    pub turning_impulse : f32, // [-1.0,1.0]
    pub shape_polyline:Vec<Point2<Real>>,
    pub position: Isometry<Real>,
    pub linvel: Vector<Real>,
    pub angular_velocity: Real,
    pub radar_position : f32,
    pub radar_width : f32,
    pub detected_tank : Vec<Tank>
}

fn wrap_value<T:PartialOrd + Copy>(value:T,lower:T,upper:T)->T {
    if value > upper {
        upper
    } else if value < lower {
        lower
    } else {
        value
    }

}

impl Tank {

    #[inline]
    pub fn linear_velocity(&self) -> Real {

        self.linvel.norm()
    }

    #[inline]
    pub fn forward_velocity(&self) ->  Real {
        let unit_vector = Vector2::<f32>::identity();
        let direction_vector = self.position * unit_vector;
        direction_vector.dot(&self.linvel)
    }

    /*
    Get min max angle in world coordinates
    */
    pub fn min_max_radar_angle(&self) -> (f32,f32) {
        let world_angle = self.radar_position + self.position.rotation.angle();
        let max_angle = world_angle + self.radar_width/2.0;
        let min_angle = world_angle - self.radar_width/2.0;
        (min_angle,max_angle)
    }
}

pub struct PhysicsEngine {
    pub tanks: Vec<Tank>,
    tick:u32,
    rigid_body_set: RigidBodySet,
    collider_set: ColliderSet,
    integration_parameters: IntegrationParameters,
    physics_pipeline: PhysicsPipeline,
    island_manager: IslandManager,
    broad_phase: BroadPhase,
    narrow_phase: NarrowPhase,
    joint_set: JointSet,
    ccd_solver: CCDSolver,
    physics_hooks: (),
    event_handler: (),
}

impl PhysicsEngine {
    pub fn init_physics(&mut self) {
        
    }

    pub fn add_tank(&mut self,tank_position:Isometry2<Real>) {
        
        let body = RigidBodyBuilder::new_dynamic()
            .position(tank_position)
            .linear_damping(LINEAR_DAMPING)
            .angular_damping(ANGULAR_DAMPING)
            .build();

        let rigid_body_handle = self.rigid_body_set.insert(body);

        let collider = ColliderBuilder::cuboid(TANK_WIDTH_M / 2.0, TANK_DEPTH_M / 2.0)
            .restitution(0.7)
            .density(COLLIDER_DENSITY)
            .collision_groups(TANK_GROUP)
            .build();
        let shape_polyline_tank = Self::get_collider_polyline_cuboid(&collider);
        
        let collider_handle = self.collider_set.insert_with_parent(
            collider,
            rigid_body_handle,
            &mut self.rigid_body_set,
        );

  
        let turret_body = RigidBodyBuilder::new_dynamic()
            .position(tank_position)
            .build();
        let rigid_body_turret_handle = self.rigid_body_set.insert(turret_body);

        let turret_collider = ColliderBuilder::cuboid(TURRET_WIDTH_M / 2.0, TURRET_DEPTH_M / 2.0)
        .density(COLLIDER_DENSITY)
        .collision_groups(TURRET_GROUP)
        .build(); 

        let shape_polyline_turret = Self::get_collider_polyline_cuboid(&turret_collider);

        let collider_turret_handle = self.collider_set.insert_with_parent(
            turret_collider,
            rigid_body_turret_handle,
            &mut self.rigid_body_set,
        );
        let mut joint = BallJoint::new(point![0.0, 0.0], point![0.0, TURRET_DEPTH_M / 2.0]);
        let target_angle =Rotation::<Real>::new(0.0);
        //joint.configure_motor_model(SpringModel::Disabled);
        //joint.configure_motor_position(target_angle, 1.0, 0.1);
        joint.configure_motor_velocity(0.8, 0.2);
        self.joint_set.insert(rigid_body_handle, rigid_body_turret_handle, joint);

        let tank = Tank {
            phy_body_handle: rigid_body_handle,
            collider_handle: collider_handle,
            energy: TANK_ENERGY_MAX,
            damage: 0.0,
            turret: Turret {
                phy_body_handle : rigid_body_turret_handle,
                collider_handle : collider_turret_handle,
                angle : 0.0,
                shape_polyline :shape_polyline_turret,
            },
            engine_power : 0.0,
            max_engine_power : TANK_ENGINE_POWER_MAX,
            turning_impulse : 0.0,
            max_turning_impulse : TURNING_IMPULSE_MAX,
            shape_polyline : shape_polyline_tank,
            position : Isometry2::identity(),
            linvel : Vector2::identity(),
            angular_velocity : 0.0,
            radar_position : 0.0,
            radar_width : RADAR_WIDTH_MAX,
            detected_tank : Vec::new()
        };
        self.tanks.push(tank);
    }

    pub fn step(&mut self) {
        let p = vector![0.0, 0.0]; //No gravity
        for tank in &self.tanks {
            let tank_rigid_body = &mut self.rigid_body_set[tank.phy_body_handle];
            Self::apply_engine_energy(tank_rigid_body,tank.engine_power);
            tank_rigid_body.apply_torque_impulse(tank.turning_impulse, true);
        }
        self.physics_pipeline.step(
            &p,
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
        self.tick +=1;
        //Read back present status
        for tank in &mut self.tanks {
            let tank_rigid_body = & self.rigid_body_set[tank.phy_body_handle];
            tank.position = *tank_rigid_body.position();
            tank.linvel = *tank_rigid_body.linvel();
            tank.angular_velocity = tank_rigid_body.angvel();

            let turret_rigid_body = &self.rigid_body_set[tank.turret.phy_body_handle];
            tank.turret.angle = turret_rigid_body.position().rotation.angle();

            let collider = &self.collider_set[tank.collider_handle];
            tank.shape_polyline = Self::get_collider_polyline_cuboid(collider);

            let collider_turret = &self.collider_set[tank.turret.collider_handle];
            tank.turret.shape_polyline = Self::get_collider_polyline_cuboid(collider_turret);

        }

    }

    #[inline]
    pub fn tick(&self) -> u32 {
        self.tick
    }

    #[inline]
    pub fn tank_engine_power_percentage(&self,tank_id:usize) -> f32 {
        self.tanks[tank_id].engine_power/TANK_ENGINE_POWER_MAX
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
        tank.engine_power = energy*tank.max_engine_power;
    }

    pub fn get_position(&self, tank: &Tank) -> &Isometry<Real> {
        let p = &self.rigid_body_set[tank.phy_body_handle];
        return p.position();
    }
    pub fn get_tank_position(&self, tank_id: usize) -> &Isometry<Real> {
        self.get_position(&self.tanks[tank_id])
    }

    pub fn tank_velocity(&self,tank_id: usize) -> Vector<Real> {
        let tank = &self.tanks[tank_id];
        let rigid_body = &self.rigid_body_set[tank.phy_body_handle];
        *rigid_body.linvel()
        
    }

    #[inline]
    pub fn tank_energy(&self,tank_id:usize) -> f32 {
        self.tanks[tank_id].energy
    }

    #[inline]
    pub fn tank_damage(&self,tank_id:usize) -> f32 {
        self.tanks[tank_id].damage
    }

    pub fn tank_cannon_angle(&self,tank_id:usize) -> f32 {
        let rigid_body = &self.rigid_body_set[self.tanks[tank_id].turret.phy_body_handle];
        rigid_body.position().rotation.angle()
    }

    pub fn set_torque_speed(&mut self, speed: f32, tank_id: usize) {
        let tank = &self.tanks[tank_id];
        let tank_rigid_body = &mut self.rigid_body_set[tank.phy_body_handle];
        let mass = tank_rigid_body.mass();
        tank_rigid_body.apply_torque_impulse(speed * mass, true);
    }

    #[inline]
    fn apply_engine_energy(tank_rigid_body:&mut RigidBody, energy: f32) {
        if energy != 0.0 {
            let force = energy/tank_rigid_body.linvel().norm();
            let force_forward_vector = tank_rigid_body.position() * vector![force,0.0];
            tank_rigid_body.apply_force(force_forward_vector, true);
        }
    }


    pub fn set_tank_angle_impulse(&mut self, impulse_fraction: f32, tank_id: usize) {
        let tank = &mut self.tanks[tank_id];
        let speed_fraction = if impulse_fraction > 1.0 {
            1.0 
        } else if impulse_fraction < -1.0 {
            -1.0
        } else {
            impulse_fraction
        };
        tank.turning_impulse = tank.max_turning_impulse * speed_fraction;
    }

    fn get_forward_speed(position: &Isometry2<Real>, speed: &Vector2<f32>) -> Vector2<f32> {
        let unit_vector = Vector2::<f32>::identity();
        let direction_vector = position * unit_vector;
        speed - direction_vector.dot(speed) * direction_vector
    }

    fn get_collider_polyline_cuboid(collider:&Collider)-> Vec<Point2<Real>> {
        let cuboid = collider.shape().as_cuboid().unwrap();
        let mut vertexs = cuboid.to_polyline();
        let position = collider.position();
        for v in &mut vertexs {
            *v = position * *v;
        }
        return vertexs;

    } 

    pub fn update_radar_attribute(&mut self,tank_id:usize,radar_increment:f32,radar_width:f32) {
        let radar_incr = wrap_value(radar_increment,-RADAR_ANGLE_INCREMENT_MAX,RADAR_ANGLE_INCREMENT_MAX);
        let radar_w = wrap_value(radar_width,0.0,RADAR_WIDTH_MAX);
        let tank = &mut self.tanks[tank_id];
        tank.radar_position += radar_incr;
        //Keep in expected range
        tank.radar_position =  if tank.radar_position > 2.0*PI {
            tank.radar_position - 2.0*PI
        } else if tank.radar_position < -2.0*PI {
            tank.radar_position + 2.0*PI
        } else {
            tank.radar_position
        };
        tank.radar_width = radar_w;

    }

    pub fn get_radar_result(&self,tank_id:usize) -> (f32,Vec<(&Tank,f32)>) {
        let mut result = Vec::new();
        let tank = &self.tanks[tank_id];
        let tank_position = tank.position.translation.vector;
        //Search detected tank
        for target_tank in &self.tanks {
            //Tank don't detect itself
            if std::ptr::eq(target_tank,tank) {
                continue
            }
            let relative_vector = target_tank.position.translation.vector - tank_position;
            let distance = relative_vector.norm();
        
            if  distance < RADAR_MAX_DETECTION_DISTANCE {
                let radar_vector = Isometry2::rotation(tank.radar_position+tank.position.rotation.angle()) * Vector2::<Real>::x();
                let angle = Rotation2::rotation_between(&radar_vector, &relative_vector).angle();
                if angle.abs() < tank.radar_width/2.0 {
                    result.push((target_tank,distance));
                }
            }
                
        }
    
        (tank.radar_position,result)
    }
        
    
}

pub fn create_physics_engine() -> PhysicsEngine {
    let mut engine = PhysicsEngine {
        tanks: vec![],
        tick : 0,
        rigid_body_set: RigidBodySet::new(),
        collider_set: ColliderSet::new(),
        integration_parameters: IntegrationParameters::default(),
        physics_pipeline: PhysicsPipeline::new(),
        island_manager: IslandManager::new(),
        broad_phase: BroadPhase::new(),
        narrow_phase: NarrowPhase::new(),
        joint_set: JointSet::new(),
        ccd_solver: CCDSolver::new(),
        physics_hooks: (),
        event_handler: (),
    };

    engine.init_physics();
    return engine;
}
