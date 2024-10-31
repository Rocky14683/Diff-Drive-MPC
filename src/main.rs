mod chassis;
mod mpc_controller;

use raylib as rl;
use nalgebra as na;
use nalgebra::Point;
use raylib::drawing::RaylibDraw;
use trajectory::Trajectory;
use raylib::core::math::Vector2;

fn main() {
    println!("Hello, world!");
    let (mut rl, thread) = rl::init()
        .size(1400, 800)
        .title("Hello, world!")
        .build();

    let times = vec![0.0_f64, 1.0, 3.0, 4.0];
    let points = vec![
        Vector2::new(100.0, 100.0),
        Vector2::new(400.0, 30.0),
        Vector2::new(600.0, 800.0),
        Vector2::new(1000.0, 400.0),
    ];

    let traj_points = vec![
        vec![100.0, 100.0],
        vec![400.0, 70.0],
        vec![600.0, 660.0],
        vec![1200.0, 400.0],
    ];


    let traj = trajectory::CubicSpline::new(times, traj_points).unwrap();
    let mut way_pts: Vec<Vec<f64>> = Vec::new();
    for i in 0..400 {
        let t = i as f64 * 0.01_f64;
        let p = traj.position(t).unwrap();
        way_pts.push(p);
        // let v = ip.velocity(t).unwrap();
        // let a = ip.acceleration(t).unwrap();
    }
    let mut car = chassis::Chassis::new(200.0, 200.0);
    while !rl.window_should_close() {
        let mut d = rl.begin_drawing(&thread);



        for i in 0..400 {
            d.draw_circle(way_pts[i][0] as i32, way_pts[i][1] as i32, 2.0, raylib::color::Color::BLACK);
        }

        // d.draw_spline_bezier_cubic(&points, 2.0, raylib::color::Color::RED);

        // display mouse position
        let mouse_pos = d.get_mouse_position();
        d.draw_text(&format!("x: {:.2}, y: {:.2}", mouse_pos.x, mouse_pos.y), 20, 20, 20, raylib::color::Color::BLACK);



        // car.set_wheel_velocity(-0., 0.5);
        // car.draw(&mut d);
        d.clear_background(raylib::color::Color::WHITE);
    }
}
