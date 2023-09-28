use crate::physics::{EntityId, GameStateReceiver, Point2, Real, SimulationState, UICommandSender};
use bevy::{math::Vec3Swizzles, prelude::*};
use std::collections::{HashMap, HashSet};
use std::sync::{Arc, Mutex};
mod camera_controller_plugin;
use camera_controller_plugin::{CameraController, CameraControllerPlugin};

const TIME_STEP: f32 = 1.0 / 60.0;
const BOUNDS: Vec2 = Vec2::new(1200.0, 640.0);
pub fn start_gui(
    rx_data: Box<dyn GameStateReceiver>,
    tx_ui_command: Box<dyn UICommandSender>,
    physical_scaling_factor: f32,
) {
    App::new()
        .add_plugins((DefaultPlugins, CameraControllerPlugin))
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
        //.add_systems(Update, spawn_spawn_tanks.after(get_physical_state))
        .add_systems(Update, bevy::window::close_on_esc)
        .run();
}

#[derive(Component)]
struct TankBody {}

#[derive(Component)]
struct TankTurret {}

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
    physical_scaling_factor:f32,
}

#[derive(Resource)]
struct Sprites {
    tank_body_sprite: Handle<Image>,
    tank_turret_sprite: Handle<Image>,
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
    commands.insert_resource(Sprites {
        tank_body_sprite,
        tank_turret_sprite,
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
    mut query: Query<(&mut Transform, &RigidBodyID, Entity), With<TankBody>>,
    physics_state: Res<PhysicsState>,
    sprites: Res<Sprites>,
) {
    let mut tank_id_in_ui: HashSet<EntityId> = HashSet::new();
    for (mut tank_transform, id_tank, entity) in query.iter_mut() {
        tank_id_in_ui.insert(id_tank.phy_id);

        match physics_state.tanks.get(&id_tank.phy_id) {
            None => commands.entity(entity).despawn_recursive(),
            Some(phy_tank) => {
                tank_transform.translation.x = phy_tank.position().translation.x;
                tank_transform.translation.y = phy_tank.position().translation.y;
                tank_transform.rotation =
                    Quat::from_rotation_z(phy_tank.position().rotation.angle());
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
                        transform: Transform::IDENTITY,
                        ..default()
                    },
                    TankTurret {},
                ))
                .id();
            // add the child to the parent
            commands.entity(tank_body).push_children(&[turret]);
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
        draw_polyline(&mut gizmos, tank.turret().shape_polyline(), physical_scaling_factor);
    }

    // Draw bullets
    for tank in physics_state.bullets.values() {
        let a = tank.shape_polyline();
        draw_polyline(&mut gizmos, a, physical_scaling_factor);
    }

    // Draw zero power limit
    gizmos.circle_2d(Vec2::ZERO, physics_state.zero_power_limit*physical_scaling_factor, Color::GREEN);
}
