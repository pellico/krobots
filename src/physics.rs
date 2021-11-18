use crate::conf::*;
use nalgebra::{vector, Isometry2, SVector,Rotation2};
pub use nalgebra::{Point2, Vector2};
pub use rapier2d::prelude::Real;
use rapier2d::prelude::*;

const TANK_GROUP: InteractionGroups = InteractionGroups::new(0b01, 0b01);
const TURRET_GROUP: InteractionGroups = InteractionGroups::new(0b10, 0b10);

pub struct Turret {
    phy_body_handle: RigidBodyHandle,
    collider_handle: ColliderHandle,
}

pub struct Tank {
    phy_body_handle: RigidBodyHandle,
    collider_handle: ColliderHandle,
    turret: Turret,
    damage: f32,
    energy: f32,
    name: String,
    engine_power : f32, // [-1.0,1.0]
    max_engine_power : f32,
    turning_impulse : f32
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

    pub fn add_tank(&mut self,name :String) {
        let index_tank = self.tanks.len() + 1;
        let tank_position = Isometry::new(
            vector![0.0, (TANK_DEPTH_M * 2.0) * (index_tank as f32)],
            0.0,
        );
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
            energy: ENERGY_MAX,
            damage: 0.0,
            turret: Turret {
                phy_body_handle : rigid_body_turret_handle,
                collider_handle : collider_turret_handle
            },
            name : name,
            engine_power : 0.0,
            max_engine_power : MAX_ENGINE_POWER,
            turning_impulse : 0.0
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
    }

    #[inline]
    pub fn tick(&self) -> u32 {
        self.tick
    }

    #[inline]
    pub fn tank_engine_power_percentage(&self,tank_id:usize) -> f32 {
        self.tanks[tank_id].engine_power/MAX_ENGINE_POWER
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

    #[inline]
    pub fn tank_name(&self,tank_id:usize) -> &str {
        &self.tanks[tank_id].name[..]
    }

    pub fn get_position(&self, tank: &Tank) -> &Isometry<Real> {
        let p = &self.rigid_body_set[tank.phy_body_handle];
        return p.position();
    }
    pub fn get_tank_position(&self, tank_id: usize) -> &Isometry<Real> {
        self.get_position(&self.tanks[tank_id])
    }


    pub fn tank_velocity(&self,tank_id: usize) -> f32 {
        let tank = &self.tanks[tank_id];
        let rigig_body = &self.rigid_body_set[tank.phy_body_handle];
        let velocity = rigig_body.linvel().norm();
        velocity
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
            let force_forward_vector = tank_rigid_body.position() * vector![0.0, force];
            tank_rigid_body.apply_force(force_forward_vector, true);
        }
    }


    pub fn set_tank_angle_speed(&mut self, speed_fraction: f32, tank_id: usize) {
        let tank = &mut self.tanks[tank_id];
        let speed_fraction = if speed_fraction > 1.0 {
            1.0 
        } else if speed_fraction < -1.0 {
            -1.0
        } else {
            speed_fraction
        };
        tank.turning_impulse = TANK_ANGULAR_IMPULSE_MAX * speed_fraction;
    }

    fn get_forward_speed(position: &Isometry2<Real>, speed: &Vector2<f32>) -> Vector2<f32> {
        let unit_vector = Vector2::<f32>::identity();
        let direction_vector = position * unit_vector;
        speed - direction_vector.dot(speed) * direction_vector
    }

    fn get_collider_polyline_cuboid(&self,collider_handle:ColliderHandle)-> Vec<Point2<Real>> {
        let collider = &self.collider_set[collider_handle];
        let cuboid = collider.shape().as_cuboid().unwrap();
        let mut vertexs = cuboid.to_polyline();
        let position = collider.position();
        for v in &mut vertexs {
            *v = position * *v;
        }
        return vertexs;

    } 

    pub fn get_collider_polyline(&self, tank_id: usize) -> [Vec<Point2<Real>>;2] {
        let tank = &self.tanks[tank_id];
        let body_polyline = self.get_collider_polyline_cuboid(tank.collider_handle);
        let turret_polyline =  self.get_collider_polyline_cuboid(tank.turret.collider_handle);
        [body_polyline,turret_polyline]

        
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
