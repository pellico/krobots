use core::f32;
use std::sync::OnceState;

use f32::consts::PI;
use krobots::krobots::tank::*;
use wit_bindgen::generate;
generate!({path:"../../wit"});

struct TankComponent;
export!(TankComponent);

fn wait_command_execution(command: Command) -> CommandResult {
    execute_command(command);
    let old_tick = get_status().tick;
    loop {
        let status = get_status();
        if old_tick != status.tick {
            return status.command_result;
        }
    }
}

fn wait_next_tick() {
    let old_tick = get_status().tick;
    while (get_status().tick == old_tick) {}
}

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

impl Guest for TankComponent {
    fn run() {
        // At this distance from power source turn back to power source
        let turn_back_distance = 150.0;
        let mut status = get_status();
        // Initial tank body forward movement power
        let forward_power = 0.9;
        // Angle of power source from tank position in world coordinates. (not referred to tank direction)
        let mut target_angle = status.power_source.p;
        // Distance of power source from tank position
        let mut last_power_distance = status.power_source.r;
        let mut delta_ang;
        let mut angimp_set;
        wait_command_execution(Command::SetEnginePower((forward_power, 0.0)));

        loop {
            wait_command_execution(Command::SetRadar((-0.17, 0.17)));
            let mut radar_result = get_status().radar_result;
            wait_command_execution(Command::SetCannotPosition(radar_result.angle));
            status = get_status();
            delta_ang = angle_wrapping(target_angle - status.angle);
            angimp_set = (0.5 * delta_ang - 0.01 * status.angvel) * status.angvel.abs();
            //If too big error angle reduce forward speed
            wait_command_execution(Command::SetEnginePower((
                forward_power / (1.0 + 10.0 * delta_ang.abs()),
                angimp_set,
            )));
            status=get_status();
            if status.power_source.r > turn_back_distance && last_power_distance <= turn_back_distance {
                loop {
                    let vel_direction = status.velocity.r * (status.velocity.p -status.angle).cos();
                    if vel_direction < 0.1 {
                        break;
                    }
                    wait_command_execution(Command::SetEnginePower((-0.002*vel_direction,0.0)));
                    status=get_status();
                }
                target_angle=status.power_source.p;

            }
            last_power_distance=status.power_source.r;
        }
       
    }
}
