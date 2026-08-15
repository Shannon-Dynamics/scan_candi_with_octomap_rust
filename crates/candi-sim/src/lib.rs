//! Shared helpers for the candi drone-scan simulation.

pub mod depth_to_cloud;
pub mod orbit;
pub mod ros_bridge;

use mujoco_rs::prelude::*;
use std::ops::Deref;
use std::path::{Path, PathBuf};

/// Workspace root, derived from this crate's manifest directory
/// (`<root>/crates/candi-sim` -> `<root>`).
pub fn workspace_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .ancestors()
        .nth(2)
        .expect("crate should live at <root>/crates/candi-sim")
        .to_path_buf()
}

/// Absolute path to the MJCF scene for the real scanned geometry.
///
/// This scene loads the converted temple mesh, which is not distributed with
/// the repository — see `assets/README.md`. Use [`demo_scene_path`] for a
/// scene that works in a fresh clone.
pub fn scene_path() -> PathBuf {
    workspace_root().join("scene").join("candi_scene.xml")
}

/// Absolute path to the self-contained demo scene.
///
/// Built from MuJoCo primitives with no external assets, so it runs
/// immediately after a clone. This is what the Quick Demo uses.
pub fn demo_scene_path() -> PathBuf {
    workspace_root()
        .join("assets")
        .join("demo")
        .join("demo_scene.xml")
}

/// Geom group carrying the drone's own body. Kept on its own group so the
/// depth camera can be told to ignore it.
pub const DRONE_GEOM_GROUP: usize = 2;

/// Visualization options for the depth camera.
///
/// The camera sits at the drone body's origin, so the drone's own rotors fall
/// inside its frustum and land in the depth buffer roughly 0.14 m out. Left in,
/// every frame would inject a shell of occupied voxels that travels with the
/// drone and corrupts the map. Hiding [`DRONE_GEOM_GROUP`] removes the drone
/// from this camera while leaving the candi (group 0) and floor untouched.
pub fn depth_camera_options() -> MjvOption {
    let mut opts = MjvOption::default();
    opts.geomgroup[DRONE_GEOM_GROUP] = 0;
    opts
}

/// World-space axis-aligned bounding box of every geom attached to `body_name`.
///
/// Mesh geoms are measured from their actual vertices; primitives use MuJoCo's
/// per-geom AABB. Two traps are handled here, both of which produced badly
/// wrong numbers for the candi before being tracked down:
///
/// 1. `geom_rbound` is a bounding *sphere* about the geom origin. For a wide,
///    flat temple it reported a 26.7 m cube instead of 14.9 x 14.9 x 6.0 m.
/// 2. The compiler rewrites mesh vertices into the mesh's principal-axis frame
///    — for the candi that tipped the 6 m height onto X and inflated the
///    footprint to 17.9 m. `mesh_vert` and `geom_aabb` are both expressed in
///    that rotated frame, so they must be mapped back through
///    `mesh_quat`/`mesh_pos` first: `v_geom = R(mesh_quat) * v + mesh_pos`.
///
/// Left uncorrected, either one drives the orbit radius off
/// the true 16.5 m (to 34.7 m and 24.1 m respectively).
///
/// Returns `None` if the body does not exist or carries no geoms.
pub fn body_aabb<M: Deref<Target = MjModel>>(
    model: &MjModel,
    data: &MjData<M>,
    body_name: &str,
) -> Option<([f64; 3], [f64; 3])> {
    let body_id = model.name_to_id(MjtObj::mjOBJ_BODY, body_name)? as i32;

    let mut lo = [f64::INFINITY; 3];
    let mut hi = [f64::NEG_INFINITY; 3];
    let mut found = false;

    for (geom_id, &owner) in model.geom_bodyid().iter().enumerate() {
        if owner != body_id {
            continue;
        }
        found = true;

        let origin = data.geom_xpos()[geom_id];
        let rot = data.geom_xmat()[geom_id]; // row-major 3x3

        // Collect the geom's extreme points in the geom's own frame.
        let mut local_points: Vec<[f64; 3]> = Vec::new();

        if model.geom_type()[geom_id] == MjtGeom::mjGEOM_MESH {
            let mesh_id = model.geom_dataid()[geom_id] as usize;
            let adr = model.mesh_vertadr()[mesh_id] as usize;
            let num = model.mesh_vertnum()[mesh_id] as usize;

            // Vertices stay in the compiler's principal-axis frame: geom_xmat
            // and geom_xpos already fold in mesh_quat/mesh_pos, which is why
            // the scene renders upright. Re-applying them here would rotate
            // the mesh twice.
            for v in &model.mesh_vert()[adr..adr + num] {
                local_points.push([v[0] as f64, v[1] as f64, v[2] as f64]);
            }
        } else {
            let aabb = model.geom_aabb()[geom_id];
            let (c, h) = ([aabb[0], aabb[1], aabb[2]], [aabb[3], aabb[4], aabb[5]]);
            for sx in [-1.0, 1.0] {
                for sy in [-1.0, 1.0] {
                    for sz in [-1.0, 1.0] {
                        local_points.push([c[0] + sx * h[0], c[1] + sy * h[1], c[2] + sz * h[2]]);
                    }
                }
            }
        }

        for p in local_points {
            for axis in 0..3 {
                let r = &rot[axis * 3..axis * 3 + 3];
                let w = origin[axis] + r[0] * p[0] + r[1] * p[1] + r[2] * p[2];
                lo[axis] = lo[axis].min(w);
                hi[axis] = hi[axis].max(w);
            }
        }
    }

    found.then_some((lo, hi))
}

/// Print a scale sanity-check for the loaded scene: how big the candi is, where
/// the drone sits, and how far apart they are. Runs `forward()` first so geom
/// world positions are populated.
pub fn report_scene<M: Deref<Target = MjModel>>(model: &MjModel, data: &mut MjData<M>) {
    data.forward();

    println!("--- scene ---");
    println!(
        "bodies={} geoms={} cameras={} mocap={}",
        model.ffi().nbody,
        model.ffi().ngeom,
        model.ffi().ncam,
        model.ffi().nmocap
    );

    match body_aabb(model, data, "candi") {
        Some((lo, hi)) => {
            let dims = [hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]];
            let centre = [
                (lo[0] + hi[0]) / 2.0,
                (lo[1] + hi[1]) / 2.0,
                (lo[2] + hi[2]) / 2.0,
            ];
            let radius = 0.5 * (dims[0].powi(2) + dims[1].powi(2) + dims[2].powi(2)).sqrt();
            println!(
                "candi: {:.2} x {:.2} x {:.2} m, centre [{:.2}, {:.2}, {:.2}], \
                 bounding sphere r={:.2} m",
                dims[0], dims[1], dims[2], centre[0], centre[1], centre[2], radius
            );
            println!(
                "candi: orbit radius at 1.5x bounding sphere = {:.2} m",
                1.5 * radius
            );

            if let Some(drone_id) = model.name_to_id(MjtObj::mjOBJ_BODY, "drone") {
                let p = data.xpos()[drone_id];
                let d = ((p[0] - centre[0]).powi(2)
                    + (p[1] - centre[1]).powi(2)
                    + (p[2] - centre[2]).powi(2))
                .sqrt();
                println!("drone: at [{:.2}, {:.2}, {:.2}]", p[0], p[1], p[2]);
                println!("drone: {d:.2} m from the candi centre");
            }
        }
        None => println!("candi: body not found or has no geoms"),
    }

    match body_aabb(model, data, "drone") {
        Some((lo, hi)) => println!(
            "drone: bounding box {:.2} x {:.2} x {:.2} m",
            hi[0] - lo[0],
            hi[1] - lo[1],
            hi[2] - lo[2]
        ),
        None => println!("drone: body not found or has no geoms"),
    }

    println!("stat: extent={:.3} m", model.stat().extent);
    println!("-------------");
}
