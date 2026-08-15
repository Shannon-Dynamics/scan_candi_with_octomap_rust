//! Watch the drone fly the scan orbit in the MuJoCo viewer.
//!
//! This is the simulation itself, in real time — the temple, the drone, and
//! the flight path it follows while scanning. The occupancy map it builds is
//! not shown here; that lives in the Rerun recording produced by `live_scan`.
//!
//!   cargo run --release -p candi-sim --example fly_orbit
//!
//! Close the window to exit. Optional arguments:
//!   --speed <n>   waypoints per second (default 24)
//!   --loop        restart the orbit instead of stopping at the end

use candi_sim::orbit::OrbitPlan;
use candi_sim::scene_path;
use mujoco_rs::prelude::*;
use mujoco_rs::viewer::MjViewer;
use std::time::{Duration, Instant};

const RINGS: usize = 4;
const POINTS_PER_RING: usize = 72;
const RADIUS_FACTOR: f64 = 1.5;

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let repeat = args.iter().any(|a| a == "--loop");
    let speed: f64 = args
        .iter()
        .position(|a| a == "--speed")
        .and_then(|i| args.get(i + 1))
        .and_then(|v| v.parse().ok())
        .unwrap_or(24.0);

    let model = MjModel::from_xml(scene_path()).expect("could not load candi_scene.xml");
    let mut data = MjData::new(&model);
    data.forward();

    let (lo, hi) = candi_sim::body_aabb(&model, &data, "candi").expect("candi body not found");
    let dims = [hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]];
    let centre = [
        (lo[0] + hi[0]) / 2.0,
        (lo[1] + hi[1]) / 2.0,
        (lo[2] + hi[2]) / 2.0,
    ];
    let heights = OrbitPlan::heights_for(lo[2], hi[2], RINGS);
    let plan = OrbitPlan::generate(centre, dims, &heights, RADIUS_FACTOR, POINTS_PER_RING);

    let mocap_id = model
        .body("drone")
        .expect("drone body not found")
        .view(&model)
        .mocapid[0] as usize;

    println!(
        "candi {:.2} x {:.2} x {:.2} m | orbit r={:.2} m | {} waypoints",
        dims[0],
        dims[1],
        dims[2],
        plan.radius,
        plan.len()
    );
    println!("flying at {speed} waypoints/s — close the window to exit");

    let mut viewer = MjViewer::launch_passive(&model, 0).expect("could not launch the viewer");

    let dwell = Duration::from_secs_f64(1.0 / speed);
    let mut index = 0usize;
    let mut last_step = Instant::now();
    let mut last_ring = usize::MAX;

    while viewer.running() {
        if last_step.elapsed() >= dwell {
            last_step = Instant::now();

            if index >= plan.len() {
                if repeat {
                    index = 0;
                } else {
                    // Hold on the final pose rather than snapping back.
                    viewer.sync_data(&mut data);
                    let _ = viewer.render();
                    std::thread::sleep(Duration::from_millis(16));
                    continue;
                }
            }

            let wp = plan.waypoints[index];
            if wp.ring != last_ring {
                println!("ring {} at height {:.2} m", wp.ring, wp.pos[2]);
                last_ring = wp.ring;
            }

            data.mocap_pos_mut()[mocap_id] = wp.pos;
            data.mocap_quat_mut()[mocap_id] = wp.quat;
            data.forward();
            index += 1;
        }

        viewer.sync_data(&mut data);
        viewer.render().expect("viewer render failed");
        std::thread::sleep(Duration::from_millis(8));
    }

    println!("viewer closed");
}
