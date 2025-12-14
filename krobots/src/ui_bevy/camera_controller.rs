use crate::physics::UICommand;
use crate::ui_bevy::SimulatorTx;

use super::{PhysicsState, UiState};
use bevy::input::mouse::MouseMotion;
use bevy::prelude::*;
use bevy::window::{CursorGrabMode, CursorOptions};
use std::fmt;

#[derive(Component)]
pub struct CameraController {
    pub sensitivity: f32,
    pub key_zoom_out: KeyCode,
    pub key_zoom_in: KeyCode,
    pub key_left: KeyCode,
    pub key_right: KeyCode,
    pub key_up: KeyCode,
    pub key_down: KeyCode,
    pub key_run: KeyCode,
    pub key_toggle_tank_track: KeyCode,
    pub mouse_key_enable_mouse: MouseButton,
    pub walk_speed: f32,
    pub run_speed: f32,
    pub friction: f32,
    pub velocity: Vec3,
    pub track_tank_enabled: bool,
}

impl Default for CameraController {
    fn default() -> Self {
        Self {
            sensitivity: 1.0,
            key_zoom_out: KeyCode::KeyQ,
            key_zoom_in: KeyCode::KeyE,
            key_left: KeyCode::KeyA,
            key_right: KeyCode::KeyD,
            key_up: KeyCode::KeyS,
            key_down: KeyCode::KeyW,
            key_run: KeyCode::ShiftLeft,
            key_toggle_tank_track: KeyCode::KeyT,
            mouse_key_enable_mouse: MouseButton::Left,
            walk_speed: 50.0,
            run_speed: 100.0,
            friction: 0.5,
            velocity: Vec3::ZERO,
            track_tank_enabled: false,
        }
    }
}

impl fmt::Display for CameraController {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "
Freecam Controls:
    MOUSE\t- Move camera orientation
    {:?}\t- Enable mouse movement
    {:?}/{:?}\t- forward/backward
    {:?}/{:?}\t- strafe left/right
    {:?}\t- 'run'
    {:?}\t- up
    {:?}\t- down
    {:?}\t- toogle tank track
    ",
            self.mouse_key_enable_mouse,
            self.key_zoom_out,
            self.key_zoom_in,
            self.key_left,
            self.key_right,
            self.key_run,
            self.key_up,
            self.key_down,
            self.key_toggle_tank_track,
        )
    }
}

pub fn camera_controller(
    time: Res<Time>,
    key_input: Res<ButtonInput<KeyCode>>,
    simulator_tx: Res<SimulatorTx>,
    mut query: Query<(&mut Transform, &mut CameraController, &mut Projection), With<Camera>>,
    (ui_state, physics_state): (Res<UiState>, Res<PhysicsState>),
) {
    let dt = time.delta_secs();

    if let Ok((mut transform, mut options, projection_enum)) = query.single_mut() {
        // Handle key input
        let projection = match projection_enum.into_inner() {
            Projection::Orthographic(a) => a,
            _ => return,
        };
        let mut axis_input = Vec3::ZERO;
        if key_input.pressed(options.key_zoom_out) {
            projection.scale *= 1.01f32.powf(1.0);
        }
        if key_input.pressed(options.key_zoom_in) {
            projection.scale *= 1.01f32.powf(-1.0);
        }

        // always ensure you end up with sane values
        // (pick an upper and lower bound for your application)
        projection.scale = projection.scale.clamp(0.05, 10.0);

        // Compute direction
        if key_input.pressed(options.key_right) {
            axis_input.x += 1.0;
        }
        if key_input.pressed(options.key_left) {
            axis_input.x -= 1.0;
        }
        if key_input.pressed(options.key_up) {
            axis_input.y += 1.0;
        }
        if key_input.pressed(options.key_down) {
            axis_input.y -= 1.0;
        }

        if key_input.just_pressed(options.key_toggle_tank_track) {
            options.track_tank_enabled = !options.track_tank_enabled;
        }

        if key_input.pressed(KeyCode::F1) {
            simulator_tx
                .tx_ui_command
                .lock()
                .unwrap()
                .send(UICommand::EnterDebugMode)
                .expect("Failed to send quit command to simulator");
        }
        if key_input.pressed(KeyCode::F2) {
            simulator_tx
                .tx_ui_command
                .lock()
                .unwrap()
                .send(UICommand::ExitDebugMode)
                .expect("Failed to send quit command to simulator");
        }

        if key_input.just_released(KeyCode::F3) {
            simulator_tx
                .tx_ui_command
                .lock()
                .unwrap()
                .send(UICommand::NextStep)
                .expect("Failed to send quit command to simulator");
        }

        // Apply movement update
        if axis_input != Vec3::ZERO {
            let max_speed = if key_input.pressed(options.key_run) {
                options.run_speed * projection.scale
            } else {
                options.walk_speed * projection.scale
            };

            // Compute velocity vector
            options.velocity = axis_input.normalize() * max_speed;
        } else {
            let friction = options.friction.clamp(0.0, 1.0);
            options.velocity *= 1.0 - friction;
            if options.velocity.length_squared() < 1e-6 {
                options.velocity = Vec3::ZERO;
            }
        }

        let right: Vec3 = transform.right().into();
        transform.translation +=
            options.velocity.x * dt * right + options.velocity.y * dt * Vec3::Y;

        if options.track_tank_enabled {
            match ui_state.selected_tank_id {
                // disable tracking if no tank is selected
                None => options.track_tank_enabled = false,
                Some(ref tank_uid) => match physics_state.tanks.get(tank_uid) {
                    // disable tracking if I cannot find the tank
                    None => options.track_tank_enabled = false,
                    Some(tank) => {
                        transform.translation.x = tank.position().translation.x;
                        transform.translation.y = tank.position().translation.y;
                    }
                },
            }
        }
    }
}

pub fn camera_controller_mouse(
    mut windows_options: Query<(&mut Window, &mut CursorOptions)>,
    mut mouse_events: MessageReader<MouseMotion>,
    mouse_button_input: Res<ButtonInput<MouseButton>>,
    mut query: Query<(&mut Transform, &mut CameraController), With<Camera>>,
) {
    if let Ok((mut transform, options)) = query.single_mut() {
        // Handle key input

        // Handle mouse input
        let mut mouse_delta = Vec2::ZERO;
        if mouse_button_input.pressed(options.mouse_key_enable_mouse) {
            for (window, mut cursor_options) in &mut windows_options {
                if !window.focused {
                    cursor_options.grab_mode = CursorGrabMode::None;
                    cursor_options.visible = true;
                    continue;
                }

                cursor_options.grab_mode = CursorGrabMode::Locked;
                cursor_options.visible = false;
            }

            for mouse_event in mouse_events.read() {
                mouse_delta += mouse_event.delta;
            }
        }
        if mouse_button_input.just_released(options.mouse_key_enable_mouse) {
            for (_, mut cursor_options) in &mut windows_options {
                cursor_options.grab_mode = CursorGrabMode::None;
                cursor_options.visible = true;
            }
        }

        if mouse_delta != Vec2::ZERO {
            let right: Vec3 = transform.right().into();

            transform.translation += -options.sensitivity * mouse_delta.x * right
                + options.sensitivity * mouse_delta.y * Vec3::Y;
        }
    }
}
