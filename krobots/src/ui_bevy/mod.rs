use crate::physics::{
    GameStateReceiver, ObjUID, Point2, Real, SimulationState, UICommand, UICommandSender,
};
use bevy::app::AppExit;
use bevy::color::palettes::css::GOLD;
use bevy::{prelude::*, window::PrimaryWindow};
use bevy_egui::EguiPlugin;
use std::collections::{HashMap, HashSet};
use std::f32::consts::PI;
use std::sync::Mutex;
mod camera_controller_plugin;
use bevy_embedded_assets::EmbeddedAssetPlugin;
use camera_controller_plugin::{CameraController, CameraControllerPlugin};
mod gizmos;
use gizmos::gizmos;
mod ui;
use ui::*;

const TIME_STEP: f64 = 1.0 / 60.0;
const BOUNDS: Vec2 = Vec2::new(1200.0, 640.0);
const TANK_BODY_Z: f32 = 1.0;
const TANK_TURRET_Z: f32 = 2.0;
const TANK_RADAR_Z: f32 = 3.0;
const BULLET_Z: f32 = 4.0;
const TANK_TEXT_Z: f32 = 5.0;
// Offset (x,y) of tank name
const TANK_TEXT_OFFSET: Vec2 = Vec2::from_array([0.0, 3.0]);

pub fn start_gui(
    rx_data: Box<dyn GameStateReceiver>,
    tx_ui_command: Box<dyn UICommandSender>,
    physical_scaling_factor: f32,
) {
    App::new()
        .add_plugins(
            DefaultPlugins
                .build()
                .add_before::<bevy::asset::AssetPlugin, _>(EmbeddedAssetPlugin::default()),
        )
        .add_plugins((CameraControllerPlugin, EguiPlugin))
        .insert_resource(ClearColor(Color::rgb(0.0, 0.0, 0.0)))
        .insert_resource(Msaa::Sample4)
        .init_resource::<UiState>()
        .insert_resource(Time::<Fixed>::from_seconds(TIME_STEP))
        .insert_resource(SimulatorRx {
            rx_data: Mutex::new(rx_data),
        })
        .insert_resource(SimulatorTx {
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
        })
        .insert_resource(TankUISpaceState {
            tank_scaling_factor: physical_scaling_factor,
        })
        .add_systems(Startup, setup)
        .add_systems(Update, get_physical_state)
        .add_systems(Update, gizmos.after(get_physical_state))
        .add_systems(Update, bullet_spawn_update.after(get_physical_state))
        .add_systems(Update, tank_spawn_update.after(get_physical_state))
        .add_systems(Startup, configure_visuals_system)
        .add_systems(Startup, configure_ui_state_system)
        .add_systems(Update, ui_update)
        .add_systems(Update, tank_label)
        .add_systems(Update, exit_system)
        .add_systems(Update, check_exit_button)
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
struct PhysicalObjUID {
    // Used as unique ID
    phy_id: ObjUID,
}

#[derive(Bundle)]
struct BulletBundle {
    sprite_bundle: SpriteBundle,
    phy_id: PhysicalObjUID,
    marker: Bullet,
}

#[derive(Bundle)]
struct TankBodyBundle {
    sprite_bundle: SpriteBundle,
    phy_id: PhysicalObjUID,
    marker: TankBody,
}

#[derive(Bundle)]
struct TankTextBundle {
    text_bundle: Text2dBundle,
    phy_id: PhysicalObjUID,
}

#[derive(Resource)]
struct SimulatorRx {
    rx_data: Mutex<Box<dyn GameStateReceiver>>,
}

#[derive(Resource)]
struct SimulatorTx {
    tx_ui_command: Mutex<Box<dyn UICommandSender>>,
}

#[derive(Resource)]
struct PhysicsState {
    tanks: HashMap<ObjUID, super::physics::Tank>,
    bullets: HashMap<ObjUID, super::physics::Bullet>,
    max_num_tanks: usize,
    tick: u32,
    max_ticks: u32,
    debug_mode: bool,
    state: SimulationState,
    zero_power_limit: f32,
}

#[derive(Resource)]
/// Settings of tank space rendering
struct TankUISpaceState {
    tank_scaling_factor: f32,
}

#[derive(Resource)]
struct TankAssets {
    tank_body_sprite: Handle<Image>,
    tank_turret_sprite: Handle<Image>,
    tank_radar_sprite: Handle<Image>,
    bullet_sprite: Handle<Image>,
    tank_font: Handle<Font>,
}

fn exit_system(mut exit: EventWriter<AppExit>) {
    if crate::is_exit_application() {
        exit.send(AppExit::Success);
    }
}

fn check_exit_button(key_input: Res<ButtonInput<KeyCode>>, simulator_tx: Res<SimulatorTx>) {
    if key_input.just_pressed(KeyCode::Escape) {
        simulator_tx
            .tx_ui_command
            .lock()
            .unwrap()
            .send(UICommand::QUIT)
            .unwrap();
    }
}

fn get_physical_state(
    // these will panic if the resources don't exist
    mut comm_channels: ResMut<SimulatorRx>,
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
    let bullet_sprite = asset_server.load("bullet.png");
    let tank_font = asset_server.load("arial.ttf");
    commands.insert_resource(TankAssets {
        tank_body_sprite,
        tank_turret_sprite,
        tank_radar_sprite,
        bullet_sprite,
        tank_font,
    });

    // 2D orthographic camera
    let camera_controller = CameraController::default();
    info!("{}", camera_controller);
    commands.spawn((Camera2dBundle::default(), camera_controller));

    let horizontal_margin = BOUNDS.x / 4.0;
    let vertical_margin = BOUNDS.y / 4.0;
    // spawn the parent and get its Entity id
}

fn bullet_spawn_update(
    mut commands: Commands,
    mut query: Query<(&mut Transform, &PhysicalObjUID, Entity), With<Bullet>>,
    physics_state: Res<PhysicsState>,
    sprites: Res<TankAssets>,
) {
    let mut bullets_id_in_ui: HashSet<ObjUID> = HashSet::new(); // Bullets in UI

    // Update existing position of bullets in UI and remove UI bullets no longer present in physical model.
    for (mut bullet_transform, bullet_obj_id, entity_id) in query.iter_mut() {
        bullets_id_in_ui.insert(bullet_obj_id.phy_id);
        match physics_state.bullets.get(&bullet_obj_id.phy_id) {
            None => commands.entity(entity_id).despawn(),
            Some(phy_bullet) => {
                bullet_transform.translation.x = phy_bullet.position().translation.x;
                bullet_transform.translation.y = phy_bullet.position().translation.y;
                bullet_transform.rotation =
                    Quat::from_rotation_z(phy_bullet.position().rotation.angle());
            }
        }
    }

    // Spawn bullets in physical simulation but not in UI
    for (phy_obj_id, phy_bullet) in physics_state.bullets.iter() {
        // For physical bulletes that are not in the UI spawn related entity
        if !bullets_id_in_ui.contains(phy_obj_id) {
            commands.spawn(BulletBundle {
                sprite_bundle: SpriteBundle {
                    texture: sprites.bullet_sprite.clone(),
                    transform: Transform {
                        translation: Vec3 {
                            x: phy_bullet.position().translation.x,
                            y: phy_bullet.position().translation.y,
                            z: BULLET_Z,
                        },
                        rotation: Quat::from_rotation_z(phy_bullet.position().rotation.angle()),
                        ..Default::default()
                    },
                    ..Default::default()
                },
                phy_id: PhysicalObjUID {
                    phy_id: *phy_obj_id,
                },
                marker: Bullet {},
            });
        }
    }
}

fn tank_spawn_update(
    mut commands: Commands,
    mut query: Query<
        (&mut Transform, &PhysicalObjUID, Entity, &Children),
        (With<TankBody>, Without<TankTurret>, Without<TankRadar>),
    >,
    mut turrets: Query<&mut Transform, (With<TankTurret>, Without<TankBody>, Without<TankRadar>)>,
    mut radar: Query<&mut Transform, (With<TankRadar>, Without<TankBody>, Without<TankTurret>)>,
    physics_state: Res<PhysicsState>,
    sprites: Res<TankAssets>,
    tank_ui_space_state: Res<TankUISpaceState>,
) {
    let mut tank_id_in_ui: HashSet<ObjUID> = HashSet::new();
    for (mut tank_transform, id_tank, entity, children) in query.iter_mut() {
        tank_id_in_ui.insert(id_tank.phy_id);

        match physics_state.tanks.get(&id_tank.phy_id) {
            None => commands.entity(entity).despawn_recursive(),
            Some(phy_tank) => {
                tank_transform.translation.x = phy_tank.position().translation.x;
                tank_transform.translation.y = phy_tank.position().translation.y;
                tank_transform.rotation =
                    Quat::from_rotation_z(phy_tank.position().rotation.angle());
                for child in children.iter() {
                    if let Ok(mut trans_radar)= radar.get_mut(*child) {
                        trans_radar.rotation = Quat::from_rotation_z(phy_tank.radar_position());
                    }
                    if let Ok(mut trans_turret) = turrets.get_mut(*child) {
                        trans_turret.rotation = Quat::from_rotation_z(
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
                .spawn(TankBodyBundle {
                    sprite_bundle: SpriteBundle {
                        texture: sprites.tank_body_sprite.clone(),
                        transform: Transform::IDENTITY
                            .with_translation(Vec3 {
                                x: 0.0,
                                y: 0.0,
                                z: TANK_BODY_Z,
                            })
                            .with_scale(Vec3::splat(tank_ui_space_state.tank_scaling_factor)),
                        ..default()
                    },
                    phy_id: PhysicalObjUID { phy_id },
                    marker: TankBody {},
                })
                .id();

            let turret = commands
                .spawn((
                    SpriteBundle {
                        texture: sprites.tank_turret_sprite.clone(),
                        transform: Transform::IDENTITY
                            .with_translation(Vec3 {
                                x: 0.0,
                                y: 0.0,
                                z: TANK_TURRET_Z,
                            })
                            .with_scale(Vec3::splat(tank_ui_space_state.tank_scaling_factor)),
                        ..default()
                    },
                    TankTurret {},
                ))
                .id();
            let radar = commands
                .spawn((
                    SpriteBundle {
                        texture: sprites.tank_radar_sprite.clone(),
                        transform: Transform::IDENTITY
                            .with_translation(Vec3 {
                                x: 0.0,
                                y: 0.0,
                                z: TANK_RADAR_Z,
                            })
                            .with_scale(Vec3::splat(tank_ui_space_state.tank_scaling_factor)),
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

fn tank_label(
    mut commands: Commands,
    mut text_query: Query<(&mut Transform, &PhysicalObjUID, Entity), With<Text>>,
    physics_state: Res<PhysicsState>,
    tank_ui_space_state: Res<TankUISpaceState>,
) {
    let mut text_in_ui = HashSet::new();
    // Update existing text and remove if tank is no longer present.
    for (mut transform, phy_uid, entity) in text_query.iter_mut() {
        text_in_ui.insert(phy_uid.phy_id);
        match physics_state.tanks.get(&phy_uid.phy_id) {
            None => commands.entity(entity).despawn_recursive(),
            Some(phy_tank) => {
                transform.translation.x = phy_tank.position().translation.x;
                transform.translation.y = phy_tank.position().translation.y;
            }
        }
    }

    // Add text for all tanks that doesn't have it
    for (&phy_id, tank) in physics_state.tanks.iter() {
        let tank_text_offset = TANK_TEXT_OFFSET * tank_ui_space_state.tank_scaling_factor;
        if !text_in_ui.contains(&phy_id) {
            commands.spawn(TankTextBundle {
                text_bundle: Text2dBundle {
                    transform: Transform::IDENTITY.with_translation(Vec3 {
                        x: tank.position().translation.x,
                        y: tank.position().translation.y,
                        z: TANK_TEXT_Z,
                    }),
                    text_anchor: bevy::sprite::Anchor::Custom(tank_text_offset),
                    text: Text::from_section(
                        tank.name.clone(),
                        TextStyle {
                            font_size: 10.0,
                            color: Color::Srgba(GOLD),
                            // If no font is specified, it will use the default font.
                            ..default()
                        },
                    ),

                    ..default()
                },
                phy_id: PhysicalObjUID { phy_id },
            });
        }
    }
}
