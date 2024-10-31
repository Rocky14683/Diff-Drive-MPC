use nalgebra as na;
use raylib::prelude::*;

pub struct Chassis {
    pub pose: na::Matrix3x1<f32>,
    pub velocity: na::Matrix3x1<f32>,
    pub wheel_speed: na::Matrix2x1<f32>,
    pub track_width: f32,
    pub wheel_r: f32,
    pub car_dims: na::Matrix4x3<f32>,
    pub car_points: na::Matrix4x3<f32>,
}

impl Chassis {
    pub fn new(x: f32, y: f32) -> Self {
        let x = na::Matrix3x1::new(x, y, 0.0);
        let x_dot = na::Matrix3x1::zeros();
        let wheel_speed = na::Matrix2x1::zeros();
        let b = 50.0;
        let r = 1.0;
        let car_dims = na::Matrix4x3::new(
            -b / 2f32, b / 2f32, 1.0,
            -b / 2f32, -b / 2f32, 1.0,
            b / 2f32, -b / 2f32, 1.0,
            b / 2f32, b / 2f32, 1.0,
        );
        let car_points = na::Matrix4x3::zeros();

        let mut car = Chassis {
            pose: x,
            velocity: x_dot,
            wheel_speed,
            track_width: b,
            wheel_r: r,
            car_dims,
            car_points,
        };
        car.get_transformed_pts();
        car
    }

    pub fn set_wheel_velocity(&mut self, lw_speed: f32, rw_speed: f32) {
        self.wheel_speed = na::Matrix2x1::new(rw_speed, lw_speed);
        self.velocity = self.forward_kinematics();
    }

    pub fn set_robot_velocity(&mut self, linear_velocity: f32, angular_velocity: f32) {
        self.velocity = na::Matrix3x1::new(linear_velocity, 0.0, angular_velocity);
        self.wheel_speed = self.inverse_kinematics();
    }

    pub fn update_state(&mut self, dt: f32) {
        let a = na::Matrix3::identity();
        let b = na::Matrix3x2::new(
            (self.pose[(2, 0)] + std::f32::consts::PI / 2.0).sin() * dt, 0.0,
            (self.pose[(2, 0)] + std::f32::consts::PI / 2.0).cos() * dt, 0.0,
            0.0, dt,
        );

        let vel = na::Matrix2x1::new(self.velocity[(0, 0)], self.velocity[(2, 0)]);
        self.pose = a * self.pose + b * vel;
    }

    pub fn update(&mut self, dt: f32) {
        const MAX_WHEEL_ROT_SPEED_RAD: f32 = 3.0;
        const MIN_WHEEL_ROT_SPEED_RAD: f32 = -3.0;

        self.wheel_speed = self.wheel_speed.map(|v| v.min(MAX_WHEEL_ROT_SPEED_RAD).max(MIN_WHEEL_ROT_SPEED_RAD));
        self.velocity = self.forward_kinematics();
        self.update_state(dt);
        self.wheel_speed = self.inverse_kinematics();
    }

    pub fn get_state(&self) -> (na::Matrix3x1<f32>, na::Matrix3x1<f32>) {
        (self.pose, self.velocity)
    }

    pub fn forward_kinematics(&self) -> na::Matrix3x1<f32> {
        let kine_mat = na::Matrix3x2::new(
            self.wheel_r / 2.0, self.wheel_r / 2.0,
            0.0, 0.0,
            self.wheel_r / (2.0 * self.track_width), -self.wheel_r / (2.0 * self.track_width),
        );

        kine_mat * self.wheel_speed
    }

    pub fn inverse_kinematics(&self) -> na::Matrix2x1<f32> {
        let ikine_mat = na::Matrix2x3::new(
            1.0 / self.wheel_r, 0.0, self.track_width / self.wheel_r,
            1.0 / self.wheel_r, 0.0, -self.track_width / self.wheel_r,
        );

        ikine_mat * self.velocity
    }

    pub fn get_transformed_pts(&mut self) {
        let rot_mat = na::Matrix3::new(
            self.pose[(2, 0)].cos(), self.pose[(2, 0)].sin(), self.pose[(0, 0)],
            -self.pose[(2, 0)].sin(), self.pose[(2, 0)].cos(), self.pose[(1, 0)],
            0.0, 0.0, 1.0,
        );

        self.car_points = self.car_dims * rot_mat.transpose();
    }

    pub fn get_points(&mut self) -> na::Matrix4x3<f32> {
        self.get_transformed_pts();
        self.car_points
    }

    pub fn draw(&mut self, d: &mut RaylibDrawHandle) {
        self.update(0.1);
        let points = self.get_points();
        let str = format!("x: {:.2}, y: {:.2}, theta: {:.2}", self.pose[(0, 0)], self.pose[(1, 0)], 180f32 - (self.pose[(2, 0)].to_degrees() % 360.0).abs());
        d.draw_text(&str, 20, 20, 20, Color::BLACK);

        for i in 0..4 {
            let x = points[(i, 0)] as i32;
            let y = points[(i, 1)] as i32;
            d.draw_circle(x, y, 4.0, Color::RED);
            let color = if i == 0 {
                Color::BLACK
            } else {
                Color::RED
            };
            d.draw_line_ex(Vector2::new(x as f32, y as f32), Vector2::new(points[((i + 1) % 4, 0)], points[((i + 1) % 4, 1)]), 4.0, color);
        }
    }
}