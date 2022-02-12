use rapier2d::prelude::*;
use std::f32::consts::PI;

pub fn wrap_value<T: PartialOrd + Copy>(value: T, lower: T, upper: T) -> T {
    if value > upper {
        upper
    } else if value < lower {
        lower
    } else {
        value
    }
}

/*
Velocity of a point of rigidbody
# Arguments

* `x` - x coordinates relative to rigid body
* `y` - y coordinates relative to rigid body
* `body` - rigidbody
*/
pub fn get_velocity_at_point(x: f32, y: f32, rigid_body: &RigidBody) -> Vector<Real> {
    let point_relative = Point::new(x, y);
    let point_world = rigid_body.position() * point_relative;
    rigid_body.velocity_at_point(&point_world)
}

/// Wrap angle in range ]-PI,PI]
///
/// ```
/// let result = angle_wrapping(PI);
/// assert_eq!(result, -PI);
/// ```
pub fn angle_wrapping(angle: f32) -> f32 {
    let mut angle_res = angle;
    loop {
        if angle_res > PI {
            angle_res = angle_res - 2.0 * PI
        } else if angle_res <= -PI {
            angle_res = angle_res + 2.0 * PI
        } else {
            break;
        }
    }
    angle_res
}