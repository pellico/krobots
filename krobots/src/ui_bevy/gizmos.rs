use super::*;
use crate::physics::{Point2, Real};
use bevy::color::palettes::css::{GREEN,YELLOW};
use rapier2d::math::Vector;

fn draw_polyline(gizmos: &mut Gizmos, polyline: &[Vector], scaling_factor: f32) {
    let polyline_size = polyline.len();
    let poly_vec: Vec<Vec2> = polyline
        .iter()
        .map(|&x| x * scaling_factor)
        .collect();
    gizmos.linestrip_2d(poly_vec,  Color::Srgba( bevy::color::palettes::css::RED));
    //Close shape
    gizmos.line_2d(
        polyline[polyline_size - 1] * scaling_factor,
        polyline[0] * scaling_factor,
        Color::Srgba( bevy::color::palettes::css::RED),
    );
}

pub(super) fn gizmos(mut gizmos: Gizmos, physics_state: Res<PhysicsState>) {
    let physical_scaling_factor = 1.0;
    // Draw tank and turret
    for tank in physics_state.tanks.values() {
        // Draw tank
        draw_polyline(&mut gizmos, tank.shape_polyline(), physical_scaling_factor);
        draw_polyline(
            &mut gizmos,
            tank.turret().shape_polyline(),
            physical_scaling_factor,
        );
        // Draw radar range

        let scaled_radar_range = tank.radar_range() * physical_scaling_factor;
        gizmos
            .arc_2d(
                Isometry2d::new(tank.position().translation.into(),Rot2::radians(tank.position().rotation.angle() + tank.radar_position() - PI / 2.0 - tank.radar_width() *0.5)),
                
                tank.radar_width(),
                scaled_radar_range,
                Color::Srgba(GREEN),
            )
            .resolution(10);

        // Draw side of radar detection area
        let v1 = tank.position().translation * physical_scaling_factor;
        let (min_angle, max_angle) = tank.min_max_radar_angle();

        let v2 = Vector::from_angle(min_angle).rotate(Vector::new(scaled_radar_range, 0.0))
            + v1;
        let v3 = Vector::from_angle(max_angle).rotate(Vector::new(scaled_radar_range, 0.0))
            + v1;

        gizmos.line_2d(v1, v2, Color::Srgba(YELLOW));
        gizmos.line_2d(v1, v3, Color::Srgba(YELLOW));
    }

    // Draw bullets
    for tank in physics_state.bullets.values() {
        let a = tank.shape_polyline();
        draw_polyline(&mut gizmos, a, physical_scaling_factor);
    }

    // Draw zero power limit
    gizmos
        .circle_2d(
            Vec2::ZERO,
            physics_state.zero_power_limit * physical_scaling_factor,
            Color::Srgba(GREEN),
        )
        .resolution(64);
}
