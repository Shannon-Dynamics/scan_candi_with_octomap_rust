//! Headless counterpart to `view_scene`.
//!
//! Loads scene/candi_scene.xml, prints the scale report, and renders two RGB
//! views to PNG so the scene can be checked without opening a window:
//!   - out/scene_drone_cam.png : what the drone's depth camera sees
//!   - out/scene_overview.png  : a free camera pulled back over the whole scene
//!
//! Only one MjRenderer is built — winit permits a single event loop per
//! process — so the two views come from re-pointing the same renderer.
//!
//! Run with: cargo run -p candi-sim --example scene_shot

use candi_sim::scene_path;
use mujoco_rs::prelude::*;
use mujoco_rs::renderer::MjRenderer;

const WIDTH: u32 = 640;
const HEIGHT: u32 = 480;

fn main() {
    let path = scene_path();
    println!("Loading {}", path.display());

    let model = MjModel::from_xml(&path).expect("could not load candi_scene.xml");
    let mut data = MjData::new(&model);

    candi_sim::report_scene(&model, &mut data);

    let out_dir = candi_sim::workspace_root().join("out");
    std::fs::create_dir_all(&out_dir).expect("could not create out/");

    let drone_cam = model
        .name_to_id(MjtObj::mjOBJ_CAMERA, "drone_cam")
        .expect("camera 'drone_cam' not found");

    let mut renderer = MjRenderer::builder()
        .width(WIDTH)
        .height(HEIGHT)
        .rgb(true)
        .depth(true)
        .camera(MjvCamera::new_fixed(drone_cam))
        .opts(candi_sim::depth_camera_options())
        .build(&model)
        .expect("failed to initialize the renderer");

    // View 1 — through the drone's camera, with the drone's own body hidden.
    renderer.sync_data(&mut data).expect("sync_data failed");
    renderer.render().expect("render failed");

    let depth = renderer.depth_flat().expect("depth disabled");
    let finite: Vec<f32> = depth.iter().copied().filter(|v| v.is_finite()).collect();
    let near = finite.iter().copied().fold(f32::INFINITY, f32::min);
    let far = finite.iter().copied().fold(f32::NEG_INFINITY, f32::max);
    let centre = depth[(HEIGHT as usize / 2) * WIDTH as usize + WIDTH as usize / 2];
    println!("drone_cam depth: centre={centre:.2} m, nearest={near:.2} m, farthest={far:.2} m");

    let p1 = out_dir.join("scene_drone_cam.png");
    renderer.save_rgb(&p1).expect("save_rgb failed");
    println!("wrote {}", p1.display());

    // View 2 — free camera, pulled back far enough to frame everything. The
    // drone is shown here, so restore the default geom groups.
    renderer.set_opts(MjvOption::default());

    let mut overview = MjvCamera::new_free(&model);
    overview.distance = 45.0;
    overview.azimuth = 135.0;
    overview.elevation = -20.0;
    overview.lookat = [0.0, 0.0, 6.0];
    renderer.set_camera(overview);

    renderer.sync_data(&mut data).expect("sync_data failed");
    renderer.render().expect("render failed");

    let p2 = out_dir.join("scene_overview.png");
    renderer.save_rgb(&p2).expect("save_rgb failed");
    println!("wrote {}", p2.display());
}
