use super::*;
use crate::physics::{Point2, Real};
use bevy::prelude::*;

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
                tank.position().translation.into(),
                -tank.position().rotation.angle() - tank.radar_position() + PI / 2.0,
                tank.radar_width(),
                scaled_radar_range,
                Color::GREEN,
            )
            .segments(10);

        // Draw side of radar detection area
        let v1 = tank.position().translation.vector * physical_scaling_factor;
        let (min_angle, max_angle) = tank.min_max_radar_angle();

        let v2 = (nalgebra::Isometry2::rotation(min_angle)
            * nalgebra::vector![scaled_radar_range, 0.0])
            + v1;
        let v3 = (nalgebra::Isometry2::rotation(max_angle)
            * nalgebra::vector![scaled_radar_range, 0.0])
            + v1;

        gizmos.line_2d(v1.into(), v2.into(), Color::YELLOW);
        gizmos.line_2d(v1.into(), v3.into(), Color::YELLOW);
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
            Color::GREEN,
        )
        .segments(64);
}
