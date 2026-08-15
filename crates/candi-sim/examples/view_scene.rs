//! Load scene/candi_scene.xml and open the viewer.
//!
//! Visual confirmation only: no orbit, no depth, no ROS. Just checks that
//! both bodies load and sit at sensible scale relative to each other.
//!
//! Run with: cargo run -p candi-sim --example view_scene
//!
//! This opens a window and blocks until you close it. For a non-interactive
//! check (CI, or a headless shell) use the `scene_shot` example instead,
//! which renders the same scene to PNG files and exits.

use candi_sim::scene_path;
use mujoco_rs::prelude::*;
use mujoco_rs::viewer::MjViewer;
use std::time::Duration;

fn main() {
    let path = scene_path();
    println!("Loading {}", path.display());

    let model = MjModel::from_xml(&path).expect("could not load candi_scene.xml");
    let mut data = MjData::new(&model);

    candi_sim::report_scene(&model, &mut data);

    let mut viewer = MjViewer::launch_passive(&model, 0).expect("could not launch the viewer");

    println!("Viewer open — close the window to exit.");
    while viewer.running() {
        data.step();
        viewer.sync_data(&mut data);
        viewer.render().expect("viewer render failed");
        std::thread::sleep(Duration::from_secs_f64(0.01));
    }
}
