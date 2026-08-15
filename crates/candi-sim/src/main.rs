//! The simulation loop.
//!
//! Flies the scripted orbit, renders a depth frame at every waypoint and
//! converts it into a world-frame point cloud.
//!
//! Run with: cargo run --release -p candi-sim

use candi_sim::depth_to_cloud::{CameraPose, Intrinsics, depth_to_cloud};
use candi_sim::orbit::OrbitPlan;
use candi_sim::{depth_camera_options, scene_path};
use mujoco_rs::prelude::*;
use mujoco_rs::renderer::MjRenderer;

const WIDTH: usize = 640;
const HEIGHT: usize = 480;
const FOVY_DEG: f64 = 60.0;

/// The original plan specified 10 m, which cannot work for this candi and was raised
/// deliberately. The temple is a stepped pyramid 14.95 m wide: its upper
/// terraces are set back from the base, so from anywhere on the 16.48 m orbit
/// the summit sits ~16.5 m away. At 10 m the scan returned nothing above
/// z = 3.06 m — a ring of wall with no roof, and no orbit radius fixes it
/// because shrinking the orbit just puts the base edge in the way.
/// 20 m covers the summit while still stopping short of the far side.
const MAX_RANGE: f32 = 20.0;
const SUBSAMPLE: usize = 4;

const RINGS: usize = 4;
const POINTS_PER_RING: usize = 72;
const RADIUS_FACTOR: f64 = 1.5;

fn main() {
    let path = scene_path();
    println!("Loading {}", path.display());
    let model = MjModel::from_xml(&path).expect("could not load candi_scene.xml");
    let mut data = MjData::new(&model);
    data.forward();

    // Orbit geometry comes from the candi's measured bounds, not constants,
    // so swapping the mesh reshapes the flight path automatically.
    let (lo, hi) = candi_sim::body_aabb(&model, &data, "candi").expect("candi body not found");
    let dims = [hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]];
    let centre = [
        (lo[0] + hi[0]) / 2.0,
        (lo[1] + hi[1]) / 2.0,
        (lo[2] + hi[2]) / 2.0,
    ];

    let heights = OrbitPlan::heights_for(lo[2], hi[2], RINGS);
    let plan = OrbitPlan::generate(centre, dims, &heights, RADIUS_FACTOR, POINTS_PER_RING);

    println!(
        "candi: {:.2} x {:.2} x {:.2} m, centre [{:.2}, {:.2}, {:.2}]",
        dims[0], dims[1], dims[2], centre[0], centre[1], centre[2]
    );
    println!(
        "orbit: {} waypoints, radius {:.2} m, heights {:?}",
        plan.len(),
        plan.radius,
        plan.heights
            .iter()
            .map(|h| (h * 100.0).round() / 100.0)
            .collect::<Vec<_>>()
    );

    for (label, idx) in [("first", 0), ("second", 1), ("last", plan.len() - 1)] {
        let wp = plan.waypoints[idx];
        println!(
            "  {label:>6} waypoint: pos [{:.2}, {:.2}, {:.2}] quat [{:.3}, {:.3}, {:.3}, {:.3}]",
            wp.pos[0], wp.pos[1], wp.pos[2], wp.quat[0], wp.quat[1], wp.quat[2], wp.quat[3]
        );
    }

    let mocap_id = model
        .body("drone")
        .expect("drone body not found")
        .view(&model)
        .mocapid[0];
    assert!(mocap_id >= 0, "drone is not a mocap body");
    let mocap_id = mocap_id as usize;

    let cam_id = model
        .name_to_id(MjtObj::mjOBJ_CAMERA, "drone_cam")
        .expect("drone_cam not found");

    let mut renderer = MjRenderer::builder()
        .width(WIDTH as u32)
        .height(HEIGHT as u32)
        .rgb(false)
        .depth(true)
        .camera(MjvCamera::new_fixed(cam_id))
        .opts(depth_camera_options())
        .build(&model)
        .expect("failed to initialize the renderer");

    let intrinsics = Intrinsics::from_fovy(WIDTH, HEIGHT, FOVY_DEG);
    println!(
        "camera: {WIDTH}x{HEIGHT}, fovy {FOVY_DEG} deg, focal {:.1} px",
        intrinsics.focal
    );
    println!("cloud : subsample {SUBSAMPLE}, max range {MAX_RANGE} m\n");

    // The scene also contains a ground plane, and the camera legitimately sees
    // it, so points are split before checking: anything at floor level is
    // expected to lie outside the candi, everything else must be on it.
    // A slack of one voxel absorbs surface noise.
    const SLACK: f32 = 0.1;
    const FLOOR_Z: f32 = 0.05;
    let mut total_points = 0usize;
    let mut floor_points = 0usize;
    let mut structure_points = 0usize;
    let mut outside = 0usize;
    let mut cloud_lo = [f32::INFINITY; 3];
    let mut cloud_hi = [f32::NEG_INFINITY; 3];

    let start = std::time::Instant::now();

    for (i, wp) in plan.waypoints.iter().enumerate() {
        data.mocap_pos_mut()[mocap_id] = wp.pos;
        data.mocap_quat_mut()[mocap_id] = wp.quat;
        // Kinematic drone: forward() propagates the mocap pose into xpos and
        // the camera frame. Nothing here needs integration, so no step().
        data.forward();

        renderer.sync_data(&mut data).expect("sync_data failed");
        renderer.render().expect("render failed");

        let camera = CameraPose {
            pos: data.cam_xpos()[cam_id],
            mat: data.cam_xmat()[cam_id],
        };
        let depth = renderer.depth_flat().expect("depth rendering disabled");
        let cloud = depth_to_cloud(depth, intrinsics, camera, MAX_RANGE, SUBSAMPLE);

        for p in &cloud {
            for a in 0..3 {
                cloud_lo[a] = cloud_lo[a].min(p[a]);
                cloud_hi[a] = cloud_hi[a].max(p[a]);
            }
            if p[2] < FLOOR_Z {
                floor_points += 1;
                continue;
            }
            structure_points += 1;
            let out = (0..3).any(|a| p[a] < lo[a] as f32 - SLACK || p[a] > hi[a] as f32 + SLACK);
            if out {
                outside += 1;
            }
        }
        total_points += cloud.len();

        if i % POINTS_PER_RING == 0 || i == plan.len() - 1 {
            let sample: Vec<String> = cloud
                .iter()
                .step_by(cloud.len().max(1) / 3 + 1)
                .take(3)
                .map(|p| format!("[{:.2}, {:.2}, {:.2}]", p[0], p[1], p[2]))
                .collect();
            println!(
                "wp {i:>3} ring {} pos [{:>6.2},{:>6.2},{:>5.2}] -> {:>5} pts  {}",
                wp.ring,
                wp.pos[0],
                wp.pos[1],
                wp.pos[2],
                cloud.len(),
                sample.join(" ")
            );
        }
    }

    let elapsed = start.elapsed();
    println!("\n--- summary ---");
    println!(
        "{} waypoints in {:.2}s ({:.1} ms/frame, {:.1} Hz)",
        plan.len(),
        elapsed.as_secs_f64(),
        elapsed.as_secs_f64() * 1000.0 / plan.len() as f64,
        plan.len() as f64 / elapsed.as_secs_f64()
    );
    println!(
        "{total_points} points total, {:.0} per frame average",
        total_points as f64 / plan.len() as f64
    );
    println!(
        "cloud bbox: [{:.2}, {:.2}, {:.2}] .. [{:.2}, {:.2}, {:.2}]",
        cloud_lo[0], cloud_lo[1], cloud_lo[2], cloud_hi[0], cloud_hi[1], cloud_hi[2]
    );
    println!(
        "candi bbox: [{:.2}, {:.2}, {:.2}] .. [{:.2}, {:.2}, {:.2}]",
        lo[0], lo[1], lo[2], hi[0], hi[1], hi[2]
    );

    println!(
        "{floor_points} floor points ({:.1}%), {structure_points} structure points ({:.1}%)",
        100.0 * floor_points as f64 / total_points.max(1) as f64,
        100.0 * structure_points as f64 / total_points.max(1) as f64
    );

    let pct = 100.0 * outside as f64 / structure_points.max(1) as f64;
    println!("{outside} structure points ({pct:.2}%) outside the candi bbox +/- {SLACK} m");

    let top = cloud_hi[2];
    let coverage = 100.0 * (top - lo[2] as f32) / (hi[2] - lo[2]) as f32;
    println!("highest point {top:.2} m — {coverage:.0}% of the candi's height reached");

    if pct < 1.0 && coverage > 90.0 {
        println!("VERDICT: cloud lands on the candi surface and covers its full height");
    } else if pct >= 1.0 {
        println!("VERDICT: too many stray points — check the camera frame convention");
    } else {
        println!("VERDICT: surface is clean but the upper structure is out of range");
    }
}
