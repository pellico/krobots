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
use crate::physics::*;
use crate::{is_exit_application,signal_exit};
use macroquad::prelude::*;
use macroquad::ui::{
    hash, root_ui,Skin,
    widgets::{self},
};
use miniquad::conf::Icon;
use macroquad_particles::{AtlasConfig, BlendMode, Emitter, EmitterConfig};
use macroquad_profiler;
use nalgebra;
use log::{info};

pub const TANK_COLORS : [Color;11] =[BLUE,GREEN,YELLOW,MAGENTA,VIOLET,PURPLE,LIME,BROWN,ORANGE,DARKBLUE,DARKGREEN];
//pub const DEFAULT_CAMERA_ZOOM : f32 = 0.00007848368;

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
    camera: Camera2D,
    /// Precomputed camera to draw text. For some reason the zoom y shall be negative 
    /// in order to have correct draw text
    camera_text : Camera2D,
    bullet_texture: Texture2D,
    hit_texture: Texture2D,
    show_stats: bool,
    selected_tank: usize,
    font_name : Font,
    /// Multiplication factor for physical simulation entity e.g. collider box, position etc...
    physical_scaling_factor : f32,
    default_zoom : f32,
    zoom : f32,
}

const BULLET_IMAGE : &[u8] = include_bytes!("icons\\flower_64x64.data");
const TANK_BODY_IMAGE : &[u8] = include_bytes!("icons\\body_36x38.data");
const TURRET_IMAGE : &[u8] = include_bytes!("icons\\turret_20x54.data");
const RADAR_IMAGE : &[u8] = include_bytes!("icons\\radar_22x16.data");
const SMOKE_FIRE_IMAGE : &[u8] = include_bytes!("icons\\smoke_fire_64x64.data");
const ARIAL_TTF : &[u8] = include_bytes!("icons\\arial.ttf");
/* No longer use. Keep here for future use
async fn texture_load(path :&str) -> Texture2D {
    let mut executable_path = std::env::current_exe().expect("Unable to get executable path");
    executable_path.pop();
    executable_path.push(path::PathBuf::from(path));
    let string_path = executable_path.to_str().expect("Unable to convert texture path to string");
    load_texture(string_path).await.expect(&format!("Unable to load texture: {}",path))

}
*/

impl GameUI {
    #[inline]
    fn compute_camera_text(camera:&Camera2D) -> Camera2D {
        // Workaround to draw correct text
        // We need some special settings
        let mut camera_text = camera.clone();
        camera_text.zoom[1] = -camera.zoom[1];
        camera_text.rotation = -camera.rotation;
        camera_text.target[1] = -camera.target[1];
        camera_text

    }
    /// Get camera zoom such that world height fit in camera space
    fn get_default_camera(zoom_y : f32) -> Camera2D {        
        Camera2D {
            zoom: vec2(
                zoom_y * screen_height()/ screen_width(),
                zoom_y
            ),
            //target: Vec2::new(0.0, 0.0),
            ..Default::default()
        }   
    }

    fn update_camera_zoom(&mut self) {
        self.camera.zoom = vec2(
            self.zoom * screen_height()/ screen_width(),
            self.zoom
        );
    } 

    fn reset_camera(&mut self) {
        self.zoom = self.default_zoom;
        self.camera = Self::get_default_camera(self.zoom);
        self.camera_text =  Self::compute_camera_text(&self.camera);
    }
 
    async fn new(game_state: &UIGameState,physical_scaling_factor:f32) -> GameUI {
        let default_zoom = 1.0/(game_state.zero_power_limit * physical_scaling_factor);
        let camera_default = Self::get_default_camera(default_zoom);
        let label_style = root_ui().style_builder().font(ARIAL_TTF.clone()).unwrap()
        .text_color(Color::from_rgba(0, 0, 0, 255))
        .color_selected(Color::from_rgba(255, 0, 0, 255))
        .font_size(15)
        .build();
        let default_skin = Skin {
            tabbar_style : label_style.clone(),
            label_style,
            ..root_ui().default_skin()
        };
        root_ui().push_skin(&default_skin);
        let mut game_ui = GameUI {
            tanks: Vec::<GTank>::with_capacity(game_state.tanks.len()),
            ui_visible: true,
            camera_text : Self::compute_camera_text(&camera_default),
            camera: camera_default.clone(),
            default_zoom,
            zoom : default_zoom,
            bullet_texture: Texture2D::from_rgba8(64, 64, BULLET_IMAGE),
            hit_texture: Texture2D::from_rgba8(64, 64, SMOKE_FIRE_IMAGE),
            show_stats: false,
            selected_tank : 0,
            font_name : load_ttf_font_from_bytes(ARIAL_TTF).unwrap(),
            physical_scaling_factor,
    
        };
       
        let p_tanks = &game_state.tanks;
        for index in 0..p_tanks.len() {
            let texture_body: Texture2D = Texture2D::from_rgba8(36, 38,TANK_BODY_IMAGE);
            let texture_turret: Texture2D = Texture2D::from_rgba8(20, 54,TURRET_IMAGE);
            let texture_radar: Texture2D = Texture2D::from_rgba8(22, 16,RADAR_IMAGE);
            game_ui.tanks.push(GTank {
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
                    texture: Some(game_ui.hit_texture),
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
        game_ui
    }


    fn draw_tanks<'b>(&mut self, p_tanks: &'b Vec<Tank>) {
        for index in 0..self.tanks.len() {
            let g_tank = &mut self.tanks[index];
            let p_tank = &p_tanks[index];
            g_tank.draw(p_tank, self.physical_scaling_factor,self.font_name,&self.camera_text);
            g_tank.draw_collider(p_tank, self.physical_scaling_factor);
            g_tank.draw_radar_range(p_tank, self.physical_scaling_factor);
        }
    }

    fn draw_bullets<'a>(&self, p_bullets: &'a Vec<Bullet>) {
        for p_bullet in p_bullets {
            let bullet_position = p_bullet.position;
            let g_x: f32 =
                bullet_position.translation.x * self.physical_scaling_factor - self.bullet_texture.width() / 2.;
            let g_y: f32 =
                bullet_position.translation.y * self.physical_scaling_factor - self.bullet_texture.height() / 2.;
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
            draw_polyline(&p_bullet.shape_polyline, self.physical_scaling_factor);
        }
    }

    fn robot_data_ui(&mut self, p_tanks: &Vec<Tank>) {
        if self.ui_visible == false {
            return;
        }
        widgets::Window::new(hash!(), vec2(0., 0.), vec2(300., 400.))
            .label("Tanks")
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
                        dead_label = "DISABLED ".to_owned() + label;
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

    fn process_keyboard_input(&mut self, p_tanks: &Vec<Tank>,tx_ui_command : &Box<dyn UICommandSender>) {
        if is_key_released(KeyCode::Q) && !is_key_down(KeyCode::LeftControl) {
            self.ui_visible ^= true;
        }
        //This to show/hide stats
        if is_key_released(KeyCode::F1) {
            self.show_stats ^= true;
        }

        if is_key_down(KeyCode::KpAdd) {
            self.zoom *= 1.1f32.powf(1.0);
        }
        if is_key_down(KeyCode::KpSubtract) {
            self.zoom *= 1.1f32.powf(-1.0);
        }
        self.update_camera_zoom();

        if is_key_down(KeyCode::Kp4) {
            self.camera.target.x -= 0.05 / self.camera.zoom[0];
        }
        if is_key_down(KeyCode::Kp6) {
            self.camera.target.x += 0.05 / self.camera.zoom[0];
        }
        if is_key_down(KeyCode::Kp8) {
            self.camera.target.y -= 0.05 / self.camera.zoom[1];
        }
        if is_key_down(KeyCode::Kp2) {
            self.camera.target.y += 0.05 / self.camera.zoom[1];
        }
        //Reset camera to home and default zoom
        if is_key_down(KeyCode::Kp5) {
            self.reset_camera();

        }
        //Reset camera to selected tank
        if is_key_down(KeyCode::Kp0) {
            let tank_screen_position =
                p_tanks[self.selected_tank].position.translation.vector * self.physical_scaling_factor;
            self.camera.target = tank_screen_position.into();
        }

         // Compute camera to be used for text drawing
         self.camera_text = Self::compute_camera_text(&self.camera);
        

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

        if (is_key_released(KeyCode::Q) && is_key_down(KeyCode::LeftControl)) || is_quit_requested()  {
            tx_ui_command.send(UICommand::QUIT).expect("Failed to send quit command");
            signal_exit();
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
    fn draw(&mut self, p_tank: &Tank, scaling_factor: f32,font_name:Font,camera_text:&Camera2D) {
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
        // Draw tank name
        // Need special workaround. Draw text has some issue in present macroquad version.
        push_camera_state();
        set_camera(camera_text);
        draw_text_ex(&p_tank.name, g_x, -g_y + 20.0, TextParams{
            font_size : 40,
            color: WHITE,
            font : font_name,
            ..Default::default()
        });
        pop_camera_state();

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
        let scaled_distance = p_tank.radar_range() * scaling_factor;
        let v2 = (nalgebra::Isometry2::rotation(min_angle)
            * nalgebra::vector![scaled_distance, 0.0])
            + v1;
        let v3 = (nalgebra::Isometry2::rotation(max_angle)
            * nalgebra::vector![scaled_distance, 0.0])
            + v1;

        draw_triangle_lines(v1.into(), v2.into(), v3.into(), 2.0, GREEN);
    }
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

const ICON : Icon = Icon {
    small : *std::include_bytes!("icons\\tank_icox16.data"),
    medium : *std::include_bytes!("icons\\tank_icox32.data"),
    big : *std::include_bytes!("icons\\tank_icox64.data")

};
pub fn start_gui (rx_state : Box<dyn GameStateReceiver> ,tx_ui_command : Box<dyn UICommandSender>,physical_scaling_factor:f32) {    
    // Bypass the macro. Not supported by macroquad
    // see macroquad macro main source code.
    let conf =  Conf {
        window_title: "KTanks".to_owned(),
        window_width: 1024,
        window_height: 768,
        icon : Some(ICON),
        //fullscreen: true,
        ..Default::default()
    };
    macroquad::Window::from_config(conf, ui_main(rx_state,tx_ui_command,physical_scaling_factor));
    
}

async fn ui_main(mut rx_data:Box<dyn GameStateReceiver>,tx_ui_command : Box<dyn UICommandSender>,physical_scaling_factor:f32) {
    prevent_quit();
    let mut game_state : UIGameState= UIGameState::default();
    loop {
        if is_exit_application() {
            break;
        }
        // Wait for first message
        match rx_data.receiver() {
            Some(state) => {game_state = state},
            None => (),
        }
        if matches!(game_state.state,SimulationState::Running) {
            break;
        }
        let message = if game_state.debug_mode {
            format!("Debug Mode: Connected {} tanks of {}\n", game_state.tanks.len(),game_state.max_num_tanks)
        } else {
            format!("Connected {} tanks of {}\n", game_state.tanks.len(),game_state.max_num_tanks)
        };

        print_centered(
            &message,
            screen_width() / 2.0,
            screen_height() / 2.0,
            40.0,
            GREEN,
        );

        next_frame().await;
    }
    if is_exit_application() {
        return;
    }
    info!("Registered num tanks {}",game_state.tanks.len());
    let mut game_ui = GameUI::new(&game_state,physical_scaling_factor).await;

    /*
    Used to track when received an update in order to avoid too many message
    from physics engine.
    Assumption that physics engine is faster than graphical engine.
    */
    loop {
        if is_exit_application() {
            break;
        }
        clear_background(BLACK);
        set_camera(&game_ui.camera);
        if game_ui.show_stats {
            macroquad_profiler::profiler(Default::default());
        };

        game_ui.process_keyboard_input(&game_state.tanks,&tx_ui_command);
        match rx_data.receiver() {
            Some(state) => {game_state=state},
            None => ()
        };
        //Draw background
        draw_circle_lines(0.0, 0.0, game_state.zero_power_limit * game_ui.physical_scaling_factor, 3.0, RED);
        game_ui.draw_tanks(&game_state.tanks);
        game_ui.draw_bullets(&game_state.bullets);
        game_ui.robot_data_ui(&game_state.tanks);
        next_frame().await;
    }
}
