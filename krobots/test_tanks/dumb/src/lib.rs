use core::f32;
use f32::consts::PI;
use krobots::krobots::tank::*;
use wit_bindgen::generate;
generate!({path:"../../wit"});

struct TankComponent;
export!(TankComponent);



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
        let mut status =execute_command(Command::GetStatus);
        // Initial tank body forward movement power
        let forward_power = 0.9;
        // Angle of power source from tank position in world coordinates. (not referred to tank direction)
        let mut target_angle = status.power_source.p;
        // Distance of power source from tank position
        let mut last_power_distance = status.power_source.r;
        let mut delta_ang;
        let mut angimp_set;
        let simulation_config = get_simulation_config();
        execute_command(Command::SetEnginePower((forward_power, 0.0)));

        loop {
             let mut radar_result=execute_command(Command::SetRadar((-0.17, 0.17))).radar_result;
            execute_command(Command::SetCannotPosition(radar_result.angle));
            if !radar_result.tanks.is_empty() {
                for _ in 0..35 {
                   radar_result=execute_command(Command::SetRadar((0.0, 0.01))).radar_result;
                   radar_result=execute_command(Command::SetCannotPosition(radar_result.angle)).radar_result; 
                   if !radar_result.tanks.is_empty() && radar_result.tanks[0].distance < simulation_config.bullet_max_range {
                    radar_result=execute_command(Command::FireCannon).radar_result;
                   }
                }

            }
            status = execute_command(Command::GetStatus);
            delta_ang = angle_wrapping(target_angle - status.angle);
            angimp_set = (0.5 * delta_ang - 0.01 * status.angvel) * status.angvel.abs();
            //If too big error angle reduce forward speed
            execute_command(Command::SetEnginePower((
                forward_power / (1.0 + 10.0 * delta_ang.abs()),
                angimp_set,
            )));
            status = execute_command(Command::GetStatus);
            if status.power_source.r > turn_back_distance
                && last_power_distance <= turn_back_distance
            {
                loop {
                    let vel_direction =
                        status.velocity.r * (status.velocity.p - status.angle).cos();
                    if vel_direction < 0.1 {
                        break;
                    }
                    execute_command(Command::SetEnginePower((-0.002 * vel_direction, 0.0)));
                    status = execute_command(Command::GetStatus);
                }
                target_angle = status.power_source.p;
            }
            last_power_distance = status.power_source.r;
        }
    }
}
