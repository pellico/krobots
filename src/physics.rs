use crate::conf;
use rapier2d::prelude::*;
pub use rapier2d::prelude::{Real};
pub use nalgebra::{Point2,Vector2};
use nalgebra::{Isometry2,SVector, vector};

pub struct Tank {
    phy_body_handle: RigidBodyHandle,
    collider_handle: ColliderHandle,
}


pub struct PhysicsEngine {
    pub tanks : Vec<Tank>,
    rigid_body_set:RigidBodySet,
    collider_set : ColliderSet,
    integration_parameters : IntegrationParameters,
    physics_pipeline : PhysicsPipeline,
    island_manager : IslandManager,
    broad_phase : BroadPhase,
    narrow_phase : NarrowPhase,
    joint_set : JointSet,
    ccd_solver : CCDSolver,
    physics_hooks : (),
    event_handler : (),
}



impl PhysicsEngine {
    pub fn init_physics(& mut self) {
        
        for _ in 0..conf::NUM_TANKS {
            self.add_tank();
    }
}

    pub fn add_tank(&mut self) {
        let index_tank = self.tanks.len() +1 ;
        let body = RigidBodyBuilder::new_dynamic()
        .position(Isometry::new(vector![100.0, (2.0 + conf::TANK_DEPTH_M*2.0)*(index_tank as f32)], 0.0))
        .linear_damping(0.5)
        .angular_damping(10.0)
        .build();
  
        let rigid_body_handle = self.rigid_body_set.insert(body);

        let collider = ColliderBuilder::cuboid(conf::TANK_WIDTH_M / 2.0,conf::TANK_DEPTH_M / 2.0)
        .restitution(0.7)
        .density(10.0)
        .build();
        let collider_handle = self.collider_set.insert_with_parent(collider, rigid_body_handle, &mut self.rigid_body_set);
        let tank = Tank{
            phy_body_handle: rigid_body_handle,
            collider_handle: collider_handle,
        };
        self.tanks.push(tank);

    }

    pub fn step(&mut self) {
        let p = vector![0.0,0.0]; //No gravity
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
    }
    pub fn get_position(&self,tank : &Tank) -> &Isometry<Real> {
        let p =  &self.rigid_body_set[tank.phy_body_handle];
        return p.position();
    }
    pub fn get_tank_position(&self,tank_id:usize) -> &Isometry<Real> {
        self.get_position(&self.tanks[tank_id])
    }

    pub fn set_torque_speed(& mut self,speed:f32,tank_id:usize) {
        let tank = &self.tanks[tank_id];
        let tank_rigid_body = &mut self.rigid_body_set[tank.phy_body_handle];
        let mass = tank_rigid_body.mass();
        tank_rigid_body.apply_torque_impulse(speed * mass,true);
    }

    pub fn set_acceleration(& mut self,acceleration:f32,tank_id:usize) {
        let tank = &self.tanks[tank_id];
        let tank_rigid_body = &mut self.rigid_body_set[tank.phy_body_handle];
        let mass = tank_rigid_body.mass();
        let force_forward_vector = tank_rigid_body.position() * vector![0.0,acceleration * mass];
        tank_rigid_body.apply_force(force_forward_vector,true);
    }

    fn get_forward_speed(position : &Isometry2<Real>,speed: &Vector2<f32>) -> Vector2<f32> {
        let unit_vector = Vector2::<f32>::identity();
        let direction_vector = position * unit_vector;
        speed - direction_vector.dot(speed)*direction_vector

    }

    pub fn get_collider_polyline(&self,tank_id:usize) -> Vec<Point2<Real>> {
        let tank = &self.tanks[tank_id];
        let collider = &self.collider_set[tank.collider_handle];
        let cuboid = collider.shape().as_cuboid().unwrap();
        let mut vertexs = cuboid.to_polyline();
        let position = collider.position();
        for v in &mut vertexs {
            *v = position * *v;
        }
        return vertexs;

    }

}

pub fn create_physics_engine()-> PhysicsEngine {
    
    let mut engine = PhysicsEngine {
        tanks: vec![],
        rigid_body_set: RigidBodySet::new(),
        collider_set: ColliderSet::new(),
        integration_parameters : IntegrationParameters::default(),
        physics_pipeline : PhysicsPipeline::new(),
        island_manager : IslandManager::new(),
        broad_phase : BroadPhase::new(),
        narrow_phase : NarrowPhase::new(),
        joint_set : JointSet::new(),
        ccd_solver : CCDSolver::new(),
        physics_hooks : (),
        event_handler : (),
    };

    engine.init_physics();
    return engine;

}





