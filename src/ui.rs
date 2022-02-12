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
use crate::conf::*;
use crate::physics::*;
use log::{debug};
use macroquad::prelude::*;
use macroquad::ui::{
    hash, root_ui,
    widgets::{self},
};
use macroquad_particles::{AtlasConfig, BlendMode, Emitter, EmitterConfig};
use macroquad_profiler;
use nalgebra;
use std::sync::{mpsc};
use std::{path};



struct GTank {
    texture_body: Texture2D,
    body_texture_size: Vec2,
    texture_turret: Texture2D,
    turret_texture_size: Vec2,
    texture_radar: Texture2D,
    radar_texture_size: Vec2,
    color: Color,
    last_damage: f32,
    hit_emitter: Emitter,
}

struct GameUI {
    tanks: Vec<GTank>,
    ui_visible: bool,
    zoom: f32,
    camera: Camera2D,
    bullet_texture: Texture2D,
    hit_texture: Texture2D,
    show_stats: bool,
    selected_tank: usize,
}

async fn texture_load(path :&str) -> Texture2D {
    let mut executable_path = std::env::current_exe().expect("Unable to get executable path");
    executable_path.pop();
    executable_path.push(path::PathBuf::from(path));
    let string_path = executable_path.to_str().expect("Unable to convert texture path to string");
    load_texture(string_path).await.expect(&format!("Unable to load texture: {}",path))

}

impl GameUI {
    async fn initialize(&mut self, p_tanks: &Vec<Tank>) {
        for index in 0..p_tanks.len() {
            let texture_body: Texture2D = texture_load("body.png").await;
            let texture_turret: Texture2D = texture_load("turret.png").await;
            let texture_radar: Texture2D = texture_load("radar.png").await;
            self.tanks.push(GTank {
                texture_body: texture_body,
                texture_turret: texture_turret,
                texture_radar: texture_radar,
                body_texture_size: Vec2::new(
                    texture_body.width() * 5.0,
                    texture_body.height() * 5.0,
                ),
                turret_texture_size: Vec2::new(
                    texture_turret.width() * 5.0,
                    texture_turret.height() * 5.0,
                ),
                radar_texture_size: Vec2::new(
                    texture_radar.width() * 5.0,
                    texture_radar.height() * 5.0,
                ),
                color: TANK_COLORS[index % TANK_COLORS.len()],
                last_damage: 0.0,
                hit_emitter: Emitter::new(EmitterConfig {
                    texture: Some(self.hit_texture),
                    one_shot: true,
                    emitting: false,
                    lifetime: 0.5,
                    lifetime_randomness: 0.7,
                    explosiveness: 0.95,
                    amount: 30,
                    initial_direction_spread: 2.0 * std::f32::consts::PI,
                    initial_velocity: 200.0,
                    size: 30.0,
                    gravity: vec2(0.0, -1000.0),
                    atlas: Some(AtlasConfig::new(4, 4, 8..)),
                    blend_mode: BlendMode::Additive,
                    ..Default::default()
                }),
            });
        }
    }

    fn draw_tanks<'b>(&mut self, p_tanks: &'b Vec<Tank>, scaling_factor: f32) {
        for index in 0..self.tanks.len() {
            let g_tank = &mut self.tanks[index];
            let p_tank = &p_tanks[index];
            g_tank.draw(p_tank, scaling_factor);
            g_tank.draw_collider(p_tank, scaling_factor);
            g_tank.draw_radar_range(p_tank, scaling_factor);
        }
    }

    fn draw_bullets<'a>(&self, p_bullets: &'a Vec<Bullet>, scaling_factor: f32) {
        for p_bullet in p_bullets {
            let bullet_position = p_bullet.position;
            let g_x: f32 =
                bullet_position.translation.x * scaling_factor - self.bullet_texture.width() / 2.;
            let g_y: f32 =
                bullet_position.translation.y * scaling_factor - self.bullet_texture.height() / 2.;
            let bullet_angle: f32 = bullet_position.rotation.angle() + std::f32::consts::FRAC_PI_2;

            draw_texture_ex(
                self.bullet_texture,
                g_x,
                g_y,
                WHITE,
                DrawTextureParams {
                    dest_size: None,
                    source: None,
                    rotation: bullet_angle,
                    ..Default::default()
                },
            );
            draw_polyline(&p_bullet.shape_polyline, scaling_factor);
        }
    }

    fn robot_data_ui(&mut self, p_tanks: &Vec<Tank>) {
        if is_key_released(KeyCode::Q) {
            self.ui_visible ^= true;
        }
        if self.ui_visible == false {
            return;
        }
        //This to show/hide stats
        if is_key_released(KeyCode::F1) {
            self.show_stats ^= true;
        }
        widgets::Window::new(hash!(), vec2(0., 0.), vec2(250., 300.))
            .label("Robots")
            .titlebar(true)
            .ui(&mut *root_ui(), |ui| {
                for index in 0..self.tanks.len() {
                    let p_tank = &p_tanks[index];
                    let uppercase_label;
                    let label = if index == self.selected_tank {
                        uppercase_label = p_tank.name.to_uppercase();
                        &uppercase_label
                    } else {
                        &p_tank.name
                    };
                    let dead_label;
                    let final_label: &str = if p_tank.is_dead() {
                        dead_label = "DEAD ".to_owned() + label;
                        &dead_label
                    } else {
                        label
                    };

                    ui.tree_node(hash!(&p_tank.name), final_label, |ui| {
                        ui.label(None, &format!("Speed abs {:.3}", p_tank.linear_velocity()));
                        ui.label(
                            None,
                            &format!(
                                "Speed vector (r:{:.5},p:{:.5})",
                                p_tank.linvel.norm(),
                                p_tank.linvel.y.atan2(p_tank.linvel.x),
                            ),
                        );
                        ui.label(
                            None,
                            &format!(
                                "Speed vector (x:{:.5} y:{:.5})",
                                p_tank.linvel.x,
                                p_tank.linvel.y
                            ),
                        );
                        ui.label(None, &format!("Engine power {:.3}", p_tank.engine_power));
                        ui.label(
                            None,
                            &format!("Angle {:.3}", p_tank.position.rotation.angle()),
                        );
                        ui.label(
                            None,
                            &format!("Angular velocity {:.3}", p_tank.angular_velocity),
                        );
                        ui.label(None, &format!("Turning_power {:.3}", p_tank.turning_power));
                        ui.label(None, &format!("Radar angle {:.3}", p_tank.radar_position));
                        ui.label(None, &format!("Turret angle {:.3}", p_tank.turret.angle));
                        ui.label(None, &format!("Energy {:.3}", p_tank.energy()));
                        ui.label(None, &format!("Damage {:.3}", p_tank.damage));
                    });
                    ui.separator();
                }
            });
    }

    fn process_keyboard_input(&mut self, p_tanks: &Vec<Tank>, scaling_factor: f32,tx_ui_command : &CommandLocalSender) {
        let zoom = &mut self.zoom;
        let camera = &mut self.camera;
        if is_key_down(KeyCode::KpAdd) {
            *zoom *= 1.1f32.powf(1.0);
            debug!("zoom {}", zoom);
        }
        if is_key_down(KeyCode::KpSubtract) {
            *zoom *= 1.1f32.powf(-1.0);
            debug!("zoom {}", zoom);
        }
        camera.zoom = vec2(*zoom, *zoom * screen_width() / screen_height());
        if is_key_down(KeyCode::Kp4) {
            camera.target.x -= 0.05 / *zoom;
        }
        if is_key_down(KeyCode::Kp6) {
            camera.target.x += 0.05 / *zoom;
        }
        if is_key_down(KeyCode::Kp8) {
            camera.target.y -= 0.05 / *zoom;
        }
        if is_key_down(KeyCode::Kp2) {
            camera.target.y += 0.05 / *zoom;
        }
        //Reset camera to home and default zoom
        if is_key_down(KeyCode::Kp5) {
            camera.target.x = 0.0;
            camera.target.y = 0.0;
            *zoom = DEFAULT_CAMERA_ZOOM;
        }
        //Reset camera to selected tank
        if is_key_down(KeyCode::Kp0) {
            let tank_screen_position =
                p_tanks[self.selected_tank].position.translation.vector * scaling_factor;
            camera.target = tank_screen_position.into();
        }
        camera.zoom = vec2(*zoom, *zoom * screen_width() / screen_height());
        set_camera(camera);

        if is_key_released(KeyCode::PageUp) {
            if self.selected_tank > 0 {
                self.selected_tank -= 1;
            }
        }
    
        if is_key_released(KeyCode::PageDown) {
            if self.selected_tank < self.tanks.len() - 1 {
                self.selected_tank += 1;
            };
        }

        if is_key_down(KeyCode::Q) && is_key_down(KeyCode::LeftControl) {
            tx_ui_command.send(UICommand::QUIT).expect("Failed to send quit command");
        } 
    }


}

fn draw_polyline(polyline: &Vec<Point2<Real>>, scaling_factor: f32) {
    let polyline_size = polyline.len();
    for index in 1..polyline_size {
        let point1 = &polyline[index - 1];
        let point2 = &polyline[index];
        draw_line(
            point1.x * scaling_factor,
            point1.y * scaling_factor,
            point2.x * scaling_factor,
            point2.y * scaling_factor,
            2.0,
            RED,
        );
    }
    let last_point = &polyline[polyline_size - 1];
    let first_point = &polyline[0];
    //Close shape
    draw_line(
        last_point.x * scaling_factor,
        last_point.y * scaling_factor,
        first_point.x * scaling_factor,
        first_point.y * scaling_factor,
        2.0,
        RED,
    );
}

impl GTank {
    fn draw(&mut self, p_tank: &Tank, scaling_factor: f32) {
        let p_tank_position = p_tank.position;
        let t_x = p_tank_position.translation.x * scaling_factor;
        let t_y = p_tank_position.translation.y * scaling_factor;
        let g_x: f32 = t_x - self.body_texture_size.x / 2.;
        let g_y: f32 = t_y - self.body_texture_size.y / 2.;
        let tank_angle: f32 = p_tank_position.rotation.angle() + std::f32::consts::FRAC_PI_2;
        let turret_angle: f32 = p_tank.turret.angle;
        let radar_angle: f32 = p_tank.radar_position + p_tank_position.rotation.angle();
        let color = if p_tank.is_dead() { GRAY } else { self.color };
        draw_texture_ex(
            self.texture_body,
            g_x,
            g_y,
            color,
            DrawTextureParams {
                dest_size: Some(self.body_texture_size),
                source: None,
                rotation: tank_angle,
                ..Default::default()
            },
        );
        let turret_x: f32 = t_x - self.turret_texture_size.x / 2.;
        let turret_y: f32 = t_y - self.turret_texture_size.y / 2.;
        draw_texture_ex(
            self.texture_turret,
            turret_x,
            turret_y,
            color,
            DrawTextureParams {
                dest_size: Some(self.turret_texture_size),
                source: None,
                rotation: turret_angle + std::f32::consts::FRAC_PI_2,
                ..Default::default()
            },
        );
        let radar_x: f32 = t_x - self.radar_texture_size.x / 2.;
        let radar_y: f32 = t_y - self.radar_texture_size.y / 2.;
        draw_texture_ex(
            self.texture_radar,
            radar_x,
            radar_y,
            WHITE,
            DrawTextureParams {
                dest_size: Some(self.radar_texture_size),
                source: None,
                rotation: radar_angle + std::f32::consts::FRAC_PI_2,
                ..Default::default()
            },
        );
        //Draw explosion if tank is hit
        if self.last_damage < p_tank.damage {
            self.hit_emitter.config.emitting = true;
        }
        self.hit_emitter.draw(vec2(t_x, t_y));
        self.last_damage = p_tank.damage;
    }

    fn draw_collider(&self, p_tank: &Tank, scaling_factor: f32) {
        draw_polyline(&p_tank.shape_polyline, scaling_factor);
        draw_polyline(&p_tank.turret.shape_polyline, scaling_factor);
    }

    fn draw_radar_range(&self, p_tank: &Tank, scaling_factor: f32) {
        //:TODO: optimize speed . use glam
        let v1 = p_tank.position.translation.vector * scaling_factor;
        let (min_angle, max_angle) = p_tank.min_max_radar_angle();
        let scaled_distance = RADAR_MAX_DETECTION_DISTANCE * scaling_factor;
        let v2 = (nalgebra::Isometry2::rotation(min_angle)
            * nalgebra::vector![scaled_distance, 0.0])
            + v1;
        let v3 = (nalgebra::Isometry2::rotation(max_angle)
            * nalgebra::vector![scaled_distance, 0.0])
            + v1;

        draw_triangle_lines(v1.into(), v2.into(), v3.into(), 2.0, GREEN);
    }
}

fn scaling_factor() -> f32 {
    10.0
}

/*
Print text on screen centered
*/
fn print_centered(text: &str, x: f32, y: f32, font_size: f32, color: Color) {
    let text_dimension = measure_text(text, None, font_size as u16, 1.0);
    draw_text(
        text,
        x - text_dimension.width / 2.0,
        y - text_dimension.height / 2.0,
        font_size,
        color,
    );
}

struct UILocalSender {
    tx_data : mpsc::Sender<(Vec<Tank>, Vec<Bullet>)>

}

impl GameStateSender for UILocalSender {
    #[inline]
    fn send(&self,state : (&Vec<Tank>, &Vec<Bullet>)) -> Result<(),ErrorUIComm> {
        match self.tx_data
        .send((
            state.0.to_owned(),
            state.1.to_owned(),
        )) {
            Ok(_) => Ok(()),
            Err(_) => Err(ErrorUIComm)

        }
    }
}

struct UILocalReceiver {
    rx_data : mpsc::Receiver<(Vec<Tank>, Vec<Bullet>)>,

}

impl GameStateReceiver for UILocalReceiver {
    #[inline]
    fn receiver(&self) -> Option<(Vec<Tank>, Vec<Bullet>)>{
        // Keep just last data in queue
        self.rx_data.try_iter().last()
    }
}

struct CommandLocalSender {
    tx_data : mpsc::Sender<UICommand>

}

impl UICommandSender for CommandLocalSender {
    #[inline]
    fn send(&self,command : UICommand) -> Result<(),ErrorUIComm> {
        match self.tx_data.send(command) {
            Ok(_) => Ok(()),
            Err(_) => Err(ErrorUIComm)
        }
    }
}

struct CommandLocalReceiver {
    rx_data : mpsc::Receiver<UICommand>,

}
impl UICommandReceiver for CommandLocalReceiver {
    #[inline]
    fn receive(&self) -> Option<UICommand> {
        match self.rx_data.try_recv() {
            Ok(a) => Some(a),
            Err(_) => None
        }
    }
}


fn create_state_channels() -> (UILocalSender,UILocalReceiver) {
    let (tx_data, rx_data) = mpsc::channel::<(Vec<Tank>, Vec<Bullet>)>();
    let sender = UILocalSender {tx_data : tx_data};
    let receiver = UILocalReceiver {rx_data : rx_data};
    (sender,receiver)
}


fn create_command_channels() -> (CommandLocalSender,CommandLocalReceiver) {
    let (tx_data, rx_data) = mpsc::channel::<UICommand>();
    let sender = CommandLocalSender {tx_data : tx_data};
    let receiver = CommandLocalReceiver {rx_data : rx_data};
    (sender,receiver)
}


pub fn start_gui(opts: crate::Opts) {
    let num_tanks = opts.num_tanks;
    let (tx_state, rx_state) = create_state_channels();
    let (tx_ui_command,rx_ui_command) =  create_command_channels();
    PhysicsEngine::new(&opts,Box::new(tx_state),Box::new(rx_ui_command));
    // Bypass the macro. Not supported by macroquad
    // see macroquad macro main source code.
    let conf =  Conf {
        window_title: "ETank".to_owned(),
        window_width: 1024,
        window_height: 768,
        //fullscreen: true,
        ..Default::default()
    };
    macroquad::Window::from_config(conf, ui_main(rx_state,num_tanks,tx_ui_command,opts.debug_mode));
    
}


async fn ui_main(rx_data:UILocalReceiver,num_tanks:u8,tx_ui_command : CommandLocalSender,debug_mode:bool) {
    let mut p_tanks;
    let mut p_bullets;
    let message = if debug_mode {
        format!("Debug Mode: Waiting for all {} tanks\n", num_tanks)
    } else {
        format!("Waiting for all {} tanks\n", num_tanks)
    };
    loop {
        match rx_data.receiver() {
            Some((tanks, bullets)) => {
                p_tanks = tanks;
                p_bullets = bullets;
                break;
            }
            None => (),
    }

        print_centered(
            &message,
            screen_width() / 2.0,
            screen_height() / 2.0,
            40.0,
            GREEN,
        );
        next_frame().await;
    }
    let mut game_ui = GameUI {
        tanks: Vec::<GTank>::with_capacity(p_tanks.len()),
        ui_visible: true,
        camera: Camera2D {
            zoom: vec2(
                DEFAULT_CAMERA_ZOOM,
                DEFAULT_CAMERA_ZOOM * screen_width() / screen_height(),
            ),
            target: Vec2::new(0.0, 0.0),
            ..Default::default()
        },
        zoom: DEFAULT_CAMERA_ZOOM,
        bullet_texture: texture_load("bullet.png").await,
        hit_texture: texture_load("smoke_fire.png").await,
        show_stats: false,
        selected_tank : 0,
    };
    game_ui.initialize(&p_tanks).await;

    /*
    Used to track when received an update in order to avoid too many message
    from physics engine.
    Assumption that physics engine is faster than graphical engine.
    */
    loop {
        if game_ui.show_stats {
            macroquad_profiler::profiler(Default::default());
        };

        match rx_data.receiver() {
            Some((tanks,bullets)) => {p_tanks=tanks; p_bullets = bullets},
            None => ()
        };

        let scaling_factor: f32 = scaling_factor();
        game_ui.process_keyboard_input(&p_tanks, scaling_factor,&tx_ui_command);
        //Draw background
        draw_circle_lines(0.0, 0.0, ZERO_POWER_LIMIT * scaling_factor, 3.0, RED);
        game_ui.draw_tanks(&p_tanks, scaling_factor);
        game_ui.draw_bullets(&p_bullets, scaling_factor);
        game_ui.robot_data_ui(&p_tanks);
        next_frame().await;
    }
    //handle.join().unwrap();
}
