use crate::conf;
use crate::conf::*;
use macroquad::prelude::*;
use crate::physics::{*};
use macroquad::telemetry::{begin_zone,end_zone};
use macroquad_profiler;
struct Tank {
    texture_body : Texture2D

}


fn draw_tanks <'a,'b>(tanks:&'a Vec<Tank>,p_engine : &'b PhysicsEngine,scaling_factor:&f32) {
    for index in 0..conf::NUM_TANKS {
        let p_tank_position = p_engine.get_tank_position(index);
        let g_x:f32 = p_tank_position.translation.x* scaling_factor- tanks[index].texture_body.width() / 2.;
        let g_y:f32 = p_tank_position.translation.y* scaling_factor- tanks[index].texture_body.height() / 2.;
        let angle : f32 = p_tank_position.rotation.angle();
        draw_texture_ex(tanks[index].texture_body, g_x, g_y, BLUE,DrawTextureParams{
            dest_size:None,
            source : None,
            rotation:angle,
            ..Default::default()
        });
        draw_circle(g_x, g_y, 5.0, RED);
    }


}

fn draw_tanks_collider <'a,'b>(tanks:&'a Vec<Tank>,p_engine : &'b PhysicsEngine,scaling_factor:&f32) {
    for index in 0..conf::NUM_TANKS {
        let polyline = p_engine.get_collider_polyline(index);
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
    1.0
}


pub async fn main() { 
    
    let mut p_engine = create_physics_engine();
    let mut tanks : Vec<Tank> = Vec::with_capacity(conf::NUM_TANKS);
    let a = &p_engine;
    let mut selected_tank:usize = 0;
    for _ in &a.tanks {
        let texture_body : Texture2D = load_texture("body.png").await.unwrap();
        tanks.push(Tank{
            texture_body : texture_body,
        });
    }; 
    
    println!("Hellp worlds");
    loop {
        macroquad_profiler::profiler(Default::default());
        let scaling_factor:f32 = scaling_factor();
        //input_tanks(& mut selected_tank,& mut p_engine);
        begin_zone("Physics engine");
        //p_engine.step();
        end_zone();
        //draw_tanks(&tanks,&p_engine,&scaling_factor);
        draw_tanks_collider(&tanks,&p_engine,&scaling_factor);
        next_frame().await;
       
       
    }
}