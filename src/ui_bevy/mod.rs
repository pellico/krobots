use crate::physics::{EntityId, GameStateReceiver, Point2, Real, SimulationState, UICommandSender};
use bevy::{math::Vec3Swizzles, prelude::*, window::PrimaryWindow};
use bevy_egui::{egui, EguiContexts, EguiPlugin, EguiSettings};
use std::collections::{HashMap, HashSet};
use std::sync::{Arc, Mutex};
mod camera_controller_plugin;
use bevy_embedded_assets::EmbeddedAssetPlugin;
use camera_controller_plugin::{CameraController, CameraControllerPlugin};

const TIME_STEP: f32 = 1.0 / 60.0;
const BOUNDS: Vec2 = Vec2::new(1200.0, 640.0);
pub fn start_gui(
    rx_data: Box<dyn GameStateReceiver>,
    tx_ui_command: Box<dyn UICommandSender>,
    physical_scaling_factor: f32,
) {
    App::new()
        .add_plugins(
            DefaultPlugins
                .build()
                .add_before::<bevy::asset::AssetPlugin, _>(EmbeddedAssetPlugin),
        )
        .add_plugins((CameraControllerPlugin, EguiPlugin))
        .insert_resource(ClearColor(Color::rgb(0.0, 0.0, 0.0)))
        .insert_resource(Msaa::Sample4)
        .init_resource::<UiState>()
        .insert_resource(FixedTime::new_from_secs(TIME_STEP))
        .insert_resource(CommunicationChannels {
            rx_data: Mutex::new(rx_data),
            tx_ui_command: Mutex::new(tx_ui_command),
        })
        .insert_resource(PhysicsState {
            tanks: HashMap::new(),
            bullets: HashMap::new(),
            max_num_tanks: 0,
            tick: 0,
            max_ticks: 0,
            debug_mode: false,
            state: SimulationState::WaitingConnection,
            zero_power_limit: 0.0,
            physical_scaling_factor,
        })
        .add_systems(Startup, setup)
        .add_systems(Update, get_physical_state)
        .add_systems(Update, gizmos.after(get_physical_state))
        .add_systems(Update, spawn_spawn_tanks.after(get_physical_state))
        .add_systems(Update, bevy::window::close_on_esc)
        .add_systems(Startup, configure_visuals_system)
        .add_systems(Startup, configure_ui_state_system)
        .add_systems(Update, update_ui_scale_factor_system)
        .add_systems(Update, ui_example_system)
        .run();
}

#[derive(Component)]
struct TankBody {}

#[derive(Component)]
struct TankTurret {}

#[derive(Component)]
struct TankRadar {}

#[derive(Component)]
struct Bullet {}

#[derive(Component)]
struct EnergyCircle {}

#[derive(Component)]
struct RigidBodyID {
    // Used as unique ID
    phy_id: EntityId,
}

#[derive(Resource)]
struct CommunicationChannels {
    rx_data: Mutex<Box<dyn GameStateReceiver>>,
    tx_ui_command: Mutex<Box<dyn UICommandSender>>,
}

#[derive(Resource)]
struct PhysicsState {
    tanks: HashMap<EntityId, super::physics::Tank>,
    bullets: HashMap<EntityId, super::physics::Bullet>,
    max_num_tanks: usize,
    tick: u32,
    max_ticks: u32,
    debug_mode: bool,
    state: SimulationState,
    zero_power_limit: f32,
    physical_scaling_factor: f32,
}

#[derive(Resource)]
struct Sprites {
    tank_body_sprite: Handle<Image>,
    tank_turret_sprite: Handle<Image>,
    tank_radar_sprite: Handle<Image>,
}

#[derive(Default, Resource)]
struct UiState {
    label: String,
    value: f32,
    inverted: bool,
    egui_texture_handle: Option<egui::TextureHandle>,
    is_window_open: bool,
}

fn get_physical_state(
    // these will panic if the resources don't exist
    mut comm_channels: ResMut<CommunicationChannels>,
    mut physics_state: ResMut<PhysicsState>,
) {
    let rx_data = comm_channels.rx_data.get_mut().unwrap();
    // Wait for first message
    if let Some(state) = rx_data.receiver() {
        let bullets_hashmap = state.bullets.into_iter().map(|x| (x.get_id(), x)).collect();
        physics_state.bullets = bullets_hashmap;

        let tank_hashmap: HashMap<_, _> =
            state.tanks.into_iter().map(|x| (x.get_id(), x)).collect();
        physics_state.tanks = tank_hashmap;
        physics_state.max_num_tanks = state.max_num_tanks;
        physics_state.tick = state.tick;
        physics_state.max_ticks = state.max_ticks;
        physics_state.debug_mode = state.debug_mode;
        physics_state.state = state.state;
        physics_state.zero_power_limit = state.zero_power_limit;
    }
}

fn setup(mut commands: Commands, asset_server: Res<AssetServer>) {
    let tank_body_sprite = asset_server.load("body.png");
    let tank_turret_sprite = asset_server.load("turret.png");
    let tank_radar_sprite = asset_server.load("radar.png");
    commands.insert_resource(Sprites {
        tank_body_sprite,
        tank_turret_sprite,
        tank_radar_sprite,
    });

    // 2D orthographic camera
    let camera_controller = CameraController::default();
    info!("{}", camera_controller);
    commands.spawn((Camera2dBundle::default(), camera_controller));

    let horizontal_margin = BOUNDS.x / 4.0;
    let vertical_margin = BOUNDS.y / 4.0;
    // spawn the parent and get its Entity id
}

fn spawn_spawn_tanks(
    mut commands: Commands,
    mut query: Query<
        (&mut Transform, &RigidBodyID, Entity, &Children),
        (With<TankBody>, Without<TankTurret>, Without<TankRadar>),
    >,
    mut turrets: Query<&mut Transform, (With<TankTurret>, Without<TankBody>, Without<TankRadar>)>,
    mut radar: Query<&mut Transform, (With<TankRadar>, Without<TankBody>, Without<TankTurret>)>,
    physics_state: Res<PhysicsState>,
    sprites: Res<Sprites>,
) {
    let mut tank_id_in_ui: HashSet<EntityId> = HashSet::new();
    for (mut tank_transform, id_tank, entity, children) in query.iter_mut() {
        tank_id_in_ui.insert(id_tank.phy_id);

        match physics_state.tanks.get(&id_tank.phy_id) {
            None => commands.entity(entity).despawn_recursive(),
            Some(phy_tank) => {
                tank_transform.translation.x = phy_tank.position().translation.x;
                tank_transform.translation.y = phy_tank.position().translation.y;
                tank_transform.rotation= Quat::from_rotation_z(phy_tank.position().rotation.angle());
                for child in children.iter() {
                    if let Ok(mut trans_radar) = radar.get_component_mut::<Transform>(*child) {
                        trans_radar.rotation=Quat::from_rotation_z(phy_tank.radar_position());
                    }
                    if let Ok(mut trans_turret) = turrets.get_component_mut::<Transform>(*child) {
                        trans_turret.rotation=Quat::from_rotation_z(
                            phy_tank.turret().angle() - phy_tank.position().rotation.angle(),
                        )
                    }
                }
            }
        }
    }
    let phy_tanks = &physics_state.tanks;

    for (&phy_id, tank) in phy_tanks.iter() {
        if !tank_id_in_ui.contains(&phy_id) {
            let tank_body = commands
                .spawn((
                    SpriteBundle {
                        texture: sprites.tank_body_sprite.clone(),
                        transform: Transform::IDENTITY.with_translation(Vec3 { x: 0.0, y: 0.0, z: 1.0 }),
                        ..default()
                    },
                    TankBody {},
                    RigidBodyID { phy_id },
                ))
                .id();

            let turret = commands
                .spawn((
                    SpriteBundle {
                        texture: sprites.tank_turret_sprite.clone(),
                        transform: Transform::IDENTITY.with_translation(Vec3 { x: 0.0, y: 0.0, z: 2.0 }),
                        ..default()
                    },
                    TankTurret {},
                ))
                .id();
            let radar = commands
                .spawn((
                    SpriteBundle {
                        texture: sprites.tank_radar_sprite.clone(),
                        transform: Transform::IDENTITY.with_translation(Vec3 { x: 0.0, y: 0.0, z: 3.0 }),
                        ..default()
                    },
                    TankRadar {},
                ))
                .id();
            // add the child to the parent
            commands.entity(tank_body).push_children(&[turret, radar]);
        }
    }
}

fn draw_polyline(gizmos: &mut Gizmos, polyline: &[Point2<Real>], scaling_factor: f32) {
    let polyline_size = polyline.len();
    let poly_vec: Vec<Vec2> = polyline
        .iter()
        .map(|&x| <Point2<Real> as Into<Vec2>>::into(x) * scaling_factor)
        .collect();
    gizmos.linestrip_2d(poly_vec, Color::RED);
    //Close shape
    gizmos.line_2d(
        <Point2<Real> as Into<Vec2>>::into(polyline[polyline_size - 1]) * scaling_factor,
        <Point2<Real> as Into<Vec2>>::into(polyline[0]) * scaling_factor,
        Color::RED,
    );
}

fn gizmos(mut gizmos: Gizmos, physics_state: Res<PhysicsState>) {
    let physical_scaling_factor = 1.0;
    // Draw tank and turret
    for tank in physics_state.tanks.values() {
        draw_polyline(&mut gizmos, tank.shape_polyline(), physical_scaling_factor);
        draw_polyline(
            &mut gizmos,
            tank.turret().shape_polyline(),
            physical_scaling_factor,
        );
    }

    // Draw bullets
    for tank in physics_state.bullets.values() {
        let a = tank.shape_polyline();
        draw_polyline(&mut gizmos, a, physical_scaling_factor);
    }

    // Draw zero power limit
    gizmos.circle_2d(
        Vec2::ZERO,
        physics_state.zero_power_limit * physical_scaling_factor,
        Color::GREEN,
    );
}

fn configure_visuals_system(mut contexts: EguiContexts) {
    contexts.ctx_mut().set_visuals(egui::Visuals {
        window_rounding: 0.0.into(),
        ..Default::default()
    });
}

fn configure_ui_state_system(mut ui_state: ResMut<UiState>) {
    ui_state.is_window_open = true;
}

fn update_ui_scale_factor_system(
    keyboard_input: Res<Input<KeyCode>>,
    mut toggle_scale_factor: Local<Option<bool>>,
    mut egui_settings: ResMut<EguiSettings>,
    windows: Query<&Window, With<PrimaryWindow>>,
) {
    if keyboard_input.just_pressed(KeyCode::Slash) || toggle_scale_factor.is_none() {
        *toggle_scale_factor = Some(!toggle_scale_factor.unwrap_or(true));

        if let Ok(window) = windows.get_single() {
            let scale_factor = if toggle_scale_factor.unwrap() {
                1.0
            } else {
                1.0 / window.scale_factor()
            };
            egui_settings.scale_factor = scale_factor;
        }
    }
}

fn ui_example_system(
    mut ui_state: ResMut<UiState>,
    mut is_initialized: Local<bool>,
    mut contexts: EguiContexts,
) {
    let egui_texture_handle = ui_state
        .egui_texture_handle
        .get_or_insert_with(|| {
            contexts.ctx_mut().load_texture(
                "example-image",
                egui::ColorImage::example(),
                Default::default(),
            )
        })
        .clone();

    let mut load = false;
    let mut remove = false;
    let mut invert = false;

    if !*is_initialized {
        *is_initialized = true;
    }

    let ctx = contexts.ctx_mut();

    egui::SidePanel::left("side_panel")
        .default_width(200.0)
        .show(ctx, |ui| {
            ui.heading("Side Panel");

            ui.horizontal(|ui| {
                ui.label("Write something: ");
                ui.text_edit_singleline(&mut ui_state.label);
            });

            ui.add(egui::widgets::Image::new(
                egui_texture_handle.id(),
                egui_texture_handle.size_vec2(),
            ));

            ui.add(egui::Slider::new(&mut ui_state.value, 0.0..=10.0).text("value"));
            if ui.button("Increment").clicked() {
                ui_state.value += 1.0;
            }

            ui.allocate_space(egui::Vec2::new(1.0, 100.0));
            ui.horizontal(|ui| {
                load = ui.button("Load").clicked();
                invert = ui.button("Invert").clicked();
                remove = ui.button("Remove").clicked();
            });

            ui.allocate_space(egui::Vec2::new(1.0, 10.0));
            ui.checkbox(&mut ui_state.is_window_open, "Window Is Open");

            ui.with_layout(egui::Layout::bottom_up(egui::Align::Center), |ui| {
                ui.add(egui::Hyperlink::from_label_and_url(
                    "powered by egui",
                    "https://github.com/emilk/egui/",
                ));
            });
        });

    egui::TopBottomPanel::top("top_panel").show(ctx, |ui| {
        // The top panel is often a good place for a menu bar:
        egui::menu::bar(ui, |ui| {
            egui::menu::menu_button(ui, "File", |ui| {
                if ui.button("Quit").clicked() {
                    std::process::exit(0);
                }
            });
        });
    });
}
