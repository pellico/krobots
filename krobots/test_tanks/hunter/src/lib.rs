use core::f32;
mod util;
use std::{
    cell::RefCell,
    pin::pin,
    task::{Context, Waker},
};
// use futures::{self, task::LocalSpawn, FutureExt};
use f32::consts::PI;
use krobots::krobots::tank::*;
use wit_bindgen::generate;

use crate::util::*;

generate!({path:"../../wit"});

/// Wrap angle in the range ]pi,pi]
fn angle_wrapping(angle: f32) -> f32 {
    let mut angle_res = angle;
    loop {
        if angle_res > PI {
            angle_res -= 2.0 * PI;
        } else if angle_res <= -PI {
            angle_res += 2.0 * PI;
        } else {
            break;
        }
    }
    angle_res
}

type Real = f32;
pub struct PolarCoord {
    r: Real,
    p: Real,
}

#[derive(Clone, Copy)]
pub struct CartCoord {
    x: Real,
    y: Real,
}

const ENEMY_TANK_MIN_SIZE: Real = 3.0;
enum TankState {
    Search,
    Track(CartCoord),
    FireTracking(CartCoord),
}

pub struct TankController {
    last_status: TankStatus,
    configuration: SimulationConfig,
    state: TankState,
    /// Tank coordinates referred to power source
    pos: CartCoord,
    target_power_distance: Real,
}

impl TankController {
    /// Convert polar coordinates relative to tank to CartCoord relative to power source
    pub fn polar_2_cart(&self, pos: PolarCoord) -> CartCoord {
        let pos_rel_tank = (pos.r * pos.p.cos(), pos.r * pos.p.sin());
        CartCoord {
            x: pos_rel_tank.0 + self.pos.x,
            y: pos_rel_tank.1 + self.pos.y,
        }
    }

    pub fn execute_command(&mut self, command: Command) {
        let status = execute_command(command);
        self.update_status(status);
    }

    fn update_status(&mut self, status: TankStatus) {
        let r = status.power_source.r;
        let p = status.power_source.p;
        self.pos = CartCoord {
            x: -r * p.cos(),
            y: -r * p.sin(),
        };
        self.last_status = status;
    }

    /// Compute angle and distance of point in cartesian coordinates to polar reference respect tank position
    fn cart_2_polar(&self, enemy: &CartCoord) -> PolarCoord {
        let tank_pos = &self.pos;
        let pos_rel_tank_x = enemy.x - tank_pos.x;
        let pos_rel_tank_y = enemy.y - tank_pos.y;
        let angle = pos_rel_tank_y.atan2(pos_rel_tank_x);
        let distance = (pos_rel_tank_x.powi(2) + pos_rel_tank_y.powi(2)).sqrt();
        PolarCoord {
            r: distance,
            p: angle,
        }
    }
}

fn move_control(tank: &RefCell<TankController>) {
    let mut tank = tank.borrow_mut();
    match tank.state {
        TankState::Search => {
            //Go around power source keeping a fixed distance
            let delta_target_distance =
                tank.target_power_distance - tank.last_status.power_source.r;
            let delta_angle = (90 as Real)
                .min(90.0 * delta_target_distance.abs() / (100.0))
                .copysign(delta_target_distance);
            let target_angle =
                angle_wrapping(tank.last_status.power_source.p + (90.0 + delta_angle).to_radians());
            let target_power: Real = if tank.last_status.power_source.r > 300.0 {
                0.5
            } else {
                0.7
            };
            let delta_ang = angle_wrapping(target_angle - tank.last_status.angle);
            let angimp_set = (0.5 * delta_ang - 0.02 * tank.last_status.angvel)
                * (tank.last_status.angvel + 1.0).abs();
            // # If too big error angle reduce forward speed
            tank.execute_command(Command::SetEnginePower((
                target_power / (1.0 + 2.0 * delta_ang.abs()),
                angimp_set,
            )));
        }
        TankState::FireTracking(ref enemy_last_pos) | TankState::Track(ref enemy_last_pos) => {
            // Go in circle around enemy with a distance that allows to hit.
            let enemy_r_p = tank.cart_2_polar(enemy_last_pos);
            // Difference between expected distance and actual distance.
            let delta_target_distance = tank.configuration.bullet_max_range - 10.0 - enemy_r_p.r;
            // If at expected distance I have to move around my target.
            let target_angle = angle_wrapping(enemy_r_p.p);
            let target_power: Real = 0.7;
            let delta_ang = angle_wrapping(target_angle - tank.last_status.angle);
            let angimp_set = (0.5 * delta_ang - 0.02 * tank.last_status.angvel)
                * (tank.last_status.angvel + 1.0).abs();
            // If too big error angle reduce forward speed
            let power = -target_power * delta_target_distance / 9.0;
            tank.execute_command(Command::SetEnginePower((power, angimp_set)));
        }
    }
}

async fn track_and_fire(tank: &RefCell<TankController>) -> ! {
    let tank_ref = tank.borrow_mut();
    let mut track_and_fire_old_tank_angle = tank_ref.last_status.angle;
    let fire_min_distance = tank_ref.configuration.bullet_max_range - 10.0;
    let mut radar_width = tank_ref
        .configuration
        .radar_angle_increment_max
        .min(tank_ref.configuration.radar_width_max);
    let mut enemy_last_pos;
    drop(tank_ref);
    loop {
        loop {
            let mut tank_ref = tank.borrow_mut();
            let tank_angle_shift = tank_ref.last_status.angle - track_and_fire_old_tank_angle;
            track_and_fire_old_tank_angle = tank_ref.last_status.angle;
            let radar_delta =
                tank_ref.configuration.radar_angle_increment_max / 1.5 - tank_angle_shift;
            tank_ref.execute_command(Command::SetRadar((radar_delta, radar_width)));

            if !tank_ref.last_status.radar_result.tanks.is_empty() {
                let tank = &tank_ref.last_status.radar_result.tanks[0];
                enemy_last_pos = tank_ref.polar_2_cart(PolarCoord {
                    r: tank.distance,
                    p: tank_ref.last_status.angle + tank_ref.last_status.radar_result.angle,
                });
                tank_ref.state = TankState::Track(enemy_last_pos);
                break;
            } else {
                drop(tank_ref);
                pending_once().await;
            }
        }

        loop {
            let mut tank_ref = tank.borrow_mut();
            let enemy_polar = tank_ref.cart_2_polar(&enemy_last_pos);
            let delta_radar_2_enemy = angle_wrapping(
                enemy_polar.p
                    - tank_ref.last_status.angle
                    - tank_ref.last_status.radar_result.angle,
            );
            let min_radar_width = (ENEMY_TANK_MIN_SIZE / enemy_polar.r).atan();
            if radar_width > min_radar_width {
                radar_width /= 2.0;
            }
            //Check right
            let delta_radar_right = delta_radar_2_enemy - radar_width / 2.0;
            tank_ref.execute_command(Command::SetRadar((delta_radar_right, radar_width)));
            if tank_ref.last_status.radar_result.tanks.is_empty() {
                // Try left
                tank_ref.execute_command(Command::SetRadar((radar_width, radar_width)));
            }
            if tank_ref.last_status.radar_result.tanks.is_empty() {
                radar_width *= 3.0;
                if radar_width > tank_ref.configuration.radar_angle_increment_max {
                    tank_ref.state = TankState::Search;
                    break;
                }
            } else {
                let enemy_tank = &tank_ref.last_status.radar_result.tanks[0];
                let radar_angle = tank_ref.last_status.radar_result.angle;
                enemy_last_pos = tank_ref.polar_2_cart(PolarCoord {
                    r: enemy_tank.distance,
                    p: tank_ref.last_status.angle + radar_angle,
                });
                let min_radar_width = (ENEMY_TANK_MIN_SIZE / enemy_tank.distance).atan();

                tank_ref.execute_command(Command::SetCannotPosition(radar_angle));
                // If radar width is less or equal the min_radar_width required to have enough precise fire
                // and
                let delta_cannon_radar = angle_wrapping(
                    tank_ref.last_status.cannon_angle
                        - tank_ref.last_status.angle
                        -tank_ref.last_status.radar_result.angle
                        
                );
                if radar_width <= min_radar_width
                    && !tank_ref.last_status.radar_result.tanks.is_empty()
                    && tank_ref.last_status.radar_result.tanks[0].distance < fire_min_distance
                    && delta_cannon_radar.abs() < min_radar_width
                {
                    tank_ref.state = TankState::FireTracking(enemy_last_pos);
                    while tank_ref.last_status.cannon_temp < 320.0 {
                        tank_ref.execute_command(Command::FireCannon);
                        if !&tank_ref.last_status.radar_result.tanks.is_empty() {
                            let enemy_tank = &tank_ref.last_status.radar_result.tanks[0];
                            enemy_last_pos = tank_ref.polar_2_cart(PolarCoord {
                                r: enemy_tank.distance,
                                p: tank_ref.last_status.angle + radar_angle,
                            });
                            tank_ref.state = TankState::FireTracking(enemy_last_pos)
                        }
                    }
                } else {
                    if !&tank_ref.last_status.radar_result.tanks.is_empty() {
                        let enemy_tank = &tank_ref.last_status.radar_result.tanks[0];
                        enemy_last_pos = tank_ref.polar_2_cart(PolarCoord {
                            r: enemy_tank.distance,
                            p: tank_ref.last_status.angle + radar_angle,
                        });
                    }
                    tank_ref.state = TankState::Track(enemy_last_pos)
                }
            }
            drop(tank_ref);
            pending_once().await;
        }
        
        pending_once().await;
    }
}

impl Guest for TankController {
    fn run() {
        let configuration = get_simulation_config();
        let last_status = execute_command(Command::GetStatus);
        let state = TankState::Search;
        let r = last_status.power_source.r;
        let p = last_status.power_source.p;
        let pos = CartCoord {
            x: -r * p.cos(),
            y: -r * p.sin(),
        };

        let tank = RefCell::new(TankController {
            configuration,
            state,
            last_status,
            pos,
            target_power_distance: configuration.zero_power_limit - 200.0,
        });

        // let mut pin_control_future = pin!(move_control(&tank));
        let mut pin_track_and_fire_future = pin!(track_and_fire(&tank));
        let noop_waker = Waker::noop();
        let mut context = Context::from_waker(noop_waker);
        loop {
            move_control(&tank);
            let _ = pin_track_and_fire_future.as_mut().poll(&mut context);
        }

        // let mut pool = futures::executor::LocalPool::new();

        // let mut spawner = pool.spawner();
        // spawner.spawn_local_obj(tank.move_control());
        //     tank.move_control();
        //     tank.track_and_fire();
    }
}

export!(TankController);
