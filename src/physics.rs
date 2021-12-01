use crate::conf::*;
use nalgebra::{vector, Isometry2,Rotation2};
pub use nalgebra::{Point2, Vector2};
pub use rapier2d::prelude::Real;
use rapier2d::prelude::*;
use std::f32::consts::PI;
use log::{debug, error, info, trace};

const TANK_GROUP: InteractionGroups = InteractionGroups::  new(0b001, 0b101);
const TURRET_GROUP: InteractionGroups = InteractionGroups::new(0b010, 0b110);
const BULLET_GROUP: InteractionGroups = InteractionGroups::new(0b100, 0b011);

#[derive(Clone)]
pub struct Turret {
    phy_body_handle: RigidBodyHandle,
    collider_handle: ColliderHandle,
    pub angle : f32, //Updated during step
    pub shape_polyline : Vec<Point2<Real>>,
    pub fire : bool
}

#[derive(Clone)]
pub struct Bullet {
    phy_body_handle: RigidBodyHandle,
    collider_handle: ColliderHandle,
    tick_counter :u32, //tick count down when zero the bullet will be destroyed
    pub shape_polyline :  Vec<Point2<Real>>,

}

#[derive(Clone)]
pub struct Tank {
    phy_body_handle: RigidBodyHandle,
    collider_handle: ColliderHandle,
    cannon_joint_handle : JointHandle,
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

    fn bullet_damage(&mut self) {
        self.damage += BULLET_DAMAGE;
        if self.is_dead() {
            self.engine_power = 0.0;
            self.turning_impulse = 0.0;
        }
    }

    #[inline]
    fn is_dead(&self) -> bool {
        self.damage > DAMAGE_MAX
    }
}

pub struct PhysicsEngine {
    pub tanks: Vec<Tank>,
    pub bullets : Vec<Bullet>,
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
    gravity_vector : Vector2<Real>
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
            .density(TANK_COLLIDER_DENSITY)
            .collision_groups(TANK_GROUP)
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
        .build(); 

        let shape_polyline_turret = Self::get_collider_polyline_cuboid(&turret_collider);

        let collider_turret_handle = self.collider_set.insert_with_parent(
            turret_collider,
            rigid_body_turret_handle,
            &mut self.rigid_body_set,
        );
        //let mut joint = BallJoint::new(point![0.0, 0.0], point![TURRET_WIDTH_M / 2.0, 0.0]);
        let mut joint = BallJoint::new(point![0.0, 0.0], point![-TURRET_WIDTH_M / 2.0, 0.0]);
        joint.configure_motor_model(SpringModel::VelocityBased);
        joint.configure_motor_position(Rotation::new(0.0),TURRET_STIFFNESS , TURRET_DAMPING);
        let cannon_joint_handle = self.joint_set.insert(rigid_body_handle, rigid_body_turret_handle, joint);

        let tank = Tank {
            phy_body_handle: rigid_body_handle,
            collider_handle: collider_handle,
            cannon_joint_handle : cannon_joint_handle,
            energy: TANK_ENERGY_MAX,
            damage: 0.0,
            turret: Turret {
                phy_body_handle : rigid_body_turret_handle,
                collider_handle : collider_turret_handle,
                angle : 0.0,
                shape_polyline :shape_polyline_turret,
                fire : false,
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
        for tank in &mut self.tanks {
            if tank.is_dead() {
                continue
            }
            let tank_rigid_body = &mut self.rigid_body_set[tank.phy_body_handle];
            Self::apply_engine_energy(tank_rigid_body,tank.engine_power);
            tank_rigid_body.apply_torque_impulse(tank.turning_impulse, true);
            let turret = &mut tank.turret;
            if turret.fire {
                let cannon_position = self.rigid_body_set[turret.phy_body_handle].position();
                let (bullet_body,collider)=Self::create_bullet(cannon_position);
                let collider_polyline = Self::get_collider_polyline_cuboid(&collider);
                let rigid_body_handle = self.rigid_body_set.insert(bullet_body);
                let collider_handle = self.collider_set.insert_with_parent(collider, rigid_body_handle, &mut self.rigid_body_set);
                let bullet = Bullet {
                    collider_handle : collider_handle,
                    phy_body_handle : rigid_body_handle,
                    tick_counter : std::cmp::max(1,(BULLET_MAX_RANGE / BULLET_SPEED * 60.0) as u32), //remember that step is 1/60 simualtion sec.
                    shape_polyline : collider_polyline
                };
                self.bullets.push(bullet);
                turret.fire=false;
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
        self.tick +=1;
        //Read back present status
        for tank in &mut self.tanks {
            //Tank body
            let tank_rigid_body = & self.rigid_body_set[tank.phy_body_handle];
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
                let other_collider = if contact_pair.collider1 == bullet.collider_handle {
                    contact_pair.collider2
                } else {
                    contact_pair.collider1
                };
                let target_tank_index = self.tanks.iter().position(|x| x.collider_handle == other_collider).unwrap();
                self.tanks[target_tank_index].bullet_damage();
                // Process the contact pair in a way similar to what we did in
                // the previous example.
                bullet.tick_counter =1;
            }
            bullet.tick_counter -=1;
            //Update polyline for drawing
            bullet.shape_polyline=Self::get_collider_polyline_cuboid(&self.collider_set[bullet.collider_handle]);
            if bullet.tick_counter == 0 {
                //If expired remove from physics engine
                self.rigid_body_set.remove(bullet.phy_body_handle,&mut self.island_manager,&mut self.collider_set,& mut self.joint_set);
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
        })

    }


    pub fn create_bullet(cannon_position:&Isometry2<Real>) -> (RigidBody,Collider) {
        let angle = cannon_position.rotation.angle();
        debug!("Created bullet angle:{}",angle* 360.0/PI);
        let velocity = cannon_position * vector![BULLET_SPEED,0.0];
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
    .build();
    (bullet_body,bullet_collider)
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

    /// Move radar and return detected tanks
    pub fn get_radar_result(&self,tank_id:usize) -> (f32,Vec<(&Tank,f32)>) {
        let mut result = Vec::new();
        let tank = &self.tanks[tank_id];
        // I shall use present value not the one of step ago stored in the Tank struct
        let tank_position = self.rigid_body_set[tank.phy_body_handle].position();
        //Search detected tank
        for target_tank in &self.tanks {
            //Tank don't detect itself
            if std::ptr::eq(target_tank,tank) {
                continue
            }
            let target_tank_position =  self.rigid_body_set[target_tank.phy_body_handle].position();
            // This is the vector from this tank to target tank
            let relative_vector = target_tank_position.translation.vector - tank_position.translation.vector;
            let distance = relative_vector.norm();
        
            if  distance < RADAR_MAX_DETECTION_DISTANCE {
                let radar_vector = Isometry2::rotation(tank.radar_position+tank.position.rotation.angle()) * Vector2::<Real>::x();
                //angle from radar vector to target_tank vector
                let angle = Rotation2::rotation_between(&radar_vector, &relative_vector).angle();
                if angle.abs() < tank.radar_width/2.0 {
                    result.push((target_tank,distance));
                }
            }
                
        }
        (tank.radar_position,result)
    }
     
    pub fn set_cannon_position(&mut self,tank_id:usize,angle:f32) {
        let tank = &self.tanks[tank_id];
        let joint  = self.joint_set.get_mut(tank.cannon_joint_handle).unwrap();
        match &mut joint.params {
            JointParams::BallJoint(ball_joint) => ball_joint.configure_motor_position(Rotation::new(angle),TURRET_STIFFNESS, TURRET_DAMPING),
            _ => ()

        }
    }

    pub fn fire_cannon(&mut self,tank_id:usize) {
        self.tanks[tank_id].turret.fire=true;
    }
    
}

pub fn create_physics_engine() -> PhysicsEngine {
    let mut engine = PhysicsEngine {
        tanks: vec![],
        bullets : vec![],
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
        gravity_vector : vector![0.0, 0.0], //No gravity
    };

    engine.init_physics();
    return engine;
}
