use crate::conf;
use crate::conf::*;
use macroquad::prelude::*;
use crate::physics::{*};
use macroquad::telemetry::{begin_zone,end_zone};
use macroquad_profiler;
struct Tank {
    p_index :usize,
    texture_body : Texture2D

}

fn draw_polyline(polyline:&Vec<Point2<Real>>,scaling_factor:f32) {
    let polyline_size = polyline.len();
    for index in 1..polyline_size {
        let point1 = &polyline[index-1];
        let point2 = &polyline[index];
        draw_line(point1.x * scaling_factor, point1.y * scaling_factor, point2.x * scaling_factor, point2.y * scaling_factor,2.0,RED);
    }
    let last_point=&polyline[polyline_size-1];
    let first_point=&polyline[0];
    //Close shape
    draw_line(last_point.x * scaling_factor, last_point.y * scaling_factor, first_point.x * scaling_factor, first_point.y * scaling_factor,2.0,RED);
    
}

impl Tank {
    fn draw(&self,p_engine : &PhysicsEngine,scaling_factor:f32) {
        let p_tank_position = p_engine.get_tank_position(self.p_index);
        let g_x:f32 = p_tank_position.translation.x * scaling_factor- self.texture_body.width() / 2.;
        let g_y:f32 = p_tank_position.translation.y * scaling_factor- self.texture_body.height() / 2.;
        let angle : f32 = p_tank_position.rotation.angle();
        draw_texture_ex(self.texture_body, g_x, g_y, BLUE,DrawTextureParams{
            dest_size:None,
            source : None,
            rotation:angle,
            ..Default::default()
        });

    }
    
    fn draw_collider(&self,p_engine : &PhysicsEngine,scaling_factor:f32) {
        let multi_polyline = p_engine.get_collider_polyline(self.p_index);
        for polyline in &multi_polyline {
            draw_polyline(polyline,scaling_factor);
        }
    
    }
}


fn draw_tanks <'a,'b>(tanks:&'a Vec<Tank>,p_engine : &'b PhysicsEngine,scaling_factor:f32) {
    for tank in tanks {
        tank.draw(p_engine,scaling_factor);
        tank.draw_collider(p_engine, scaling_factor);
    }
}



fn exit_application() -> ! {
    println!("Application properly exit");
    std::process::exit(0);
}

fn input_tanks (selected_tank:& mut usize,p_engine : &mut PhysicsEngine) {
    if is_key_down(KeyCode::Left) {
        p_engine.set_torque_speed(-conf::TANK_ANGULAR_SPEED, *selected_tank);
    }

    if is_key_down(KeyCode::Right) {
        p_engine.set_torque_speed(conf::TANK_ANGULAR_SPEED, *selected_tank);
    }

    if is_key_down(KeyCode::Up) {
        p_engine.set_acceleration(-conf::TANK_ACCELERATION, *selected_tank)
    }

    if is_key_down(KeyCode::Down) {
        p_engine.set_acceleration(conf::TANK_ACCELERATION, *selected_tank)
    }

    if is_key_down(KeyCode::F1) {
        *selected_tank=0;
    }

    if is_key_down(KeyCode::F2) {
        *selected_tank=1;
    }

    if is_key_down(KeyCode::Escape) {
        exit_application();
    }



} 

fn scaling_factor()->f32{
    10.0
}


pub async fn main() { 
    
    let mut p_engine = create_physics_engine();
    let mut tanks : Vec<Tank> = Vec::with_capacity(conf::NUM_TANKS);
    let a = &p_engine;
    let mut selected_tank:usize = 0;
    for index in 0..a.tanks.len() {
        let texture_body : Texture2D = load_texture("body.png").await.unwrap();
        tanks.push(Tank{
            p_index : index,
            texture_body : texture_body,
        });
    }; 
    
    println!("Hellp worlds");
    loop {
        macroquad_profiler::profiler(Default::default());
        let scaling_factor:f32 = scaling_factor();
        input_tanks(& mut selected_tank,& mut p_engine);
        begin_zone("Physics engine");
        p_engine.step();
        end_zone();
        draw_tanks(&tanks,&p_engine,scaling_factor);
        next_frame().await;
       
       
    }
}