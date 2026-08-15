//! The Quick Demo: simulation, occupancy mapping and visualization in one
//! process, with no ROS 2 in the loop.
//!
//! This is the piece you actually watch, and the shortest path to seeing
//! `octomap-core` work. It builds **two** occupancy maps from the same points:
//!
//! * the `octomap-core` octree — the library this repository demonstrates;
//! * this project's hash grid — kept as an independent implementation to check
//!   the octree against, since two separately written maps agreeing on a voxel
//!   count is worth more than either one asserting it alone.
//!
//! ```text
//!   MuJoCo scene ─► depth camera ─► point cloud ─┬─► octomap-core OcTree ─┐
//!                                                │                        ├─► Rerun
//!                                                └─► hash grid ───────────┘
//! ```
//!
//! Writes `out/candi_scan.rrd`, which the Rerun viewer replays:
//!
//!   cargo run --release -p candi-sim --bin live_scan
//!   rerun out/candi_scan.rrd
//!
//! Flags:
//!
//!   --scene <path>      MJCF to scan. Defaults to the self-contained demo
//!                       scene, which needs no external assets.
//!   --connect           Stream into a running Rerun viewer instead of writing
//!                       a file.
//!   --mesh [path]       Overlay the source geometry in the recording.
//!   --octree-no-carve   Insert endpoints only, so both maps do the same work.

use std::path::{Path, PathBuf};

use candi_octomap_node::OccupancyMap;
use candi_octomap_node::palette;
use candi_sim::depth_to_cloud::{CameraPose, Intrinsics, depth_to_cloud};
use candi_sim::orbit::OrbitPlan;
use candi_sim::{demo_scene_path, depth_camera_options, workspace_root};
use mujoco_rs::prelude::*;
use mujoco_rs::renderer::MjRenderer;
use octomap_core::{OcTree, Point3, PointCloud};

const WIDTH: usize = 640;
const HEIGHT: usize = 480;
const FOVY_DEG: f64 = 60.0;
const MAX_RANGE: f32 = 20.0;
const SUBSAMPLE: usize = 4;

const RESOLUTION: f32 = 0.1;
const RINGS: usize = 4;
const POINTS_PER_RING: usize = 72;
const RADIUS_FACTOR: f64 = 1.5;

/// Push a map snapshot every N waypoints. The simulation steps at a rate well
/// above the 10 Hz the pipeline publishes at, so this only controls how many
/// frames the recording carries, not whether the pipeline keeps up.
const LOG_EVERY: usize = 4;

/// Free-space carving in the **hash grid**, off.
///
/// A hash grid stores every empty voxel it crosses individually, so carving
/// this scene would cost millions of entries to erase nothing — MuJoCo's depth
/// buffer has no spurious returns to remove. The octree does not have that
/// problem, because a uniform region prunes into a single node, which is why
/// carving is on by default on that side. Real sensor data is where carving
/// earns its cost for either structure.
const CARVE_FREE: bool = false;

/// Drop returns below this height as ground.
///
/// Without it the scan is 56% floor: the camera sees the plane out to the 20 m
/// range in every direction, so the map becomes a 31 m blue disc with the
/// temple lost in the middle of it. Real `octomap_server` deployments carry the
/// same filter for the same reason. At 0.15 m this costs the bottom voxel or
/// two of the candi's base and nothing else.
const GROUND_Z: f32 = 0.15;

#[derive(Clone, Copy)]
enum Projection {
    /// Looking down: x across, y up the image, colour by height.
    Top,
    /// Looking along -y: x across, z up the image, colour by height.
    Side,
}

/// Render occupied voxels as an orthographic projection, using the same height
/// gradient as the 3D view. Nearest voxel wins, so the result reads as a solid
/// silhouette rather than a haze of points.
fn write_projection(
    path: &std::path::Path,
    voxels: &[[f32; 3]],
    proj: Projection,
    z_min: f32,
    z_max: f32,
) -> Result<(), Box<dyn std::error::Error>> {
    const PX: usize = 700;

    if voxels.is_empty() {
        return Ok(());
    }

    let axes = match proj {
        Projection::Top => (0usize, 1usize),
        Projection::Side => (0usize, 2usize),
    };

    let mut lo = [f32::INFINITY; 2];
    let mut hi = [f32::NEG_INFINITY; 2];
    for v in voxels {
        for (i, &a) in [axes.0, axes.1].iter().enumerate() {
            lo[i] = lo[i].min(v[a]);
            hi[i] = hi[i].max(v[a]);
        }
    }

    // One shared scale on both axes so the structure keeps its proportions.
    let span = (hi[0] - lo[0]).max(hi[1] - lo[1]).max(1e-3);
    let scale = (PX - 20) as f32 / span;

    let mut rgb = vec![18u8; PX * PX * 3];
    // Depth buffer: for Top keep the highest voxel, for Side the nearest.
    let mut best = vec![f32::NEG_INFINITY; PX * PX];

    // Voxels are 0.1 m and land several pixels apart, so painting one pixel
    // each leaves a stipple instead of a surface. Draw each at its true size.
    let cell = ((RESOLUTION * scale).ceil() as usize).max(1);

    for v in voxels {
        let px = ((v[axes.0] - lo[0]) * scale) as usize + 10;
        // Image rows run downwards, so flip the vertical axis.
        let py = PX - 10 - ((v[axes.1] - lo[1]) * scale) as usize;

        let key = match proj {
            Projection::Top => v[2],
            Projection::Side => -v[1],
        };
        let c = palette::height_colour_u8(palette::normalize(v[2], z_min, z_max));

        for dy in 0..cell {
            let y = py + dy;
            if y >= PX {
                continue;
            }
            for dx in 0..cell {
                let x = px + dx;
                if x >= PX {
                    continue;
                }
                let idx = y * PX + x;
                if key <= best[idx] {
                    continue;
                }
                best[idx] = key;
                rgb[idx * 3..idx * 3 + 3].copy_from_slice(&c);
            }
        }
    }

    let file = std::fs::File::create(path)?;
    let mut encoder = png::Encoder::new(std::io::BufWriter::new(file), PX as u32, PX as u32);
    encoder.set_color(png::ColorType::Rgb);
    encoder.set_depth(png::BitDepth::Eight);
    encoder.write_header()?.write_image_data(&rgb)?;
    Ok(())
}

/// Resolve `--mesh [path]`. Bare `--mesh` takes `default`.
///
/// Off unless asked for: a source mesh is far larger than the recording it
/// would be embedded in, so making it automatic would multiply the size of
/// every `.rrd` for a view most runs do not need.
fn mesh_arg(default: PathBuf) -> Option<PathBuf> {
    let args: Vec<String> = std::env::args().collect();
    let at = args.iter().position(|a| a == "--mesh")?;
    Some(match args.get(at + 1) {
        Some(next) if !next.starts_with("--") => PathBuf::from(next),
        _ => default,
    })
}

/// Occupied leaves of an octree, as `(centre, edge length)`.
///
/// This is the whole of what it takes to read a map back out of the library:
/// walk the leaves, keep the ones the sensor model calls occupied, and ask the
/// tree geometry where each one is and how big it is. Size matters because
/// pruning merges uniform blocks — one leaf can stand for eight base voxels,
/// or eight thousand.
fn occupied_cells(octree: &OcTree) -> Vec<([f32; 3], f32)> {
    let threshold = octree.sensor().occupancy_thres_log();
    let geometry = octree.geometry();
    octree
        .iter_leaves()
        .filter(|visit| visit.value().log_odds >= threshold)
        .map(|visit| {
            let centre = geometry.key_to_coord(visit.key());
            let size = geometry.node_size(visit.depth()) as f32;
            ([centre.x, centre.y, centre.z], size)
        })
        .collect()
}

/// How many base voxels the occupied leaves stand for.
///
/// The number to compare against a hash grid's entry count: that structure has
/// no notion of a merged node, so its count is always in base voxels.
fn occupied_voxel_equivalent(cells: &[([f32; 3], f32)], resolution: f32) -> u64 {
    cells
        .iter()
        .map(|(_, size)| {
            let side = (size / resolution).round() as u64;
            side * side * side
        })
        .sum()
}

/// Resolve `--scene <path>`, defaulting to the self-contained demo scene.
///
/// The default has to work in a fresh clone, and the scene that loads real
/// scanned geometry cannot: its mesh is not distributed. Pass
/// `--scene scene/candi_scene.xml` once those assets are in place.
fn scene_arg() -> PathBuf {
    let args: Vec<String> = std::env::args().collect();
    match args.iter().position(|a| a == "--scene") {
        Some(at) => match args.get(at + 1) {
            Some(path) if !path.starts_with("--") => PathBuf::from(path),
            _ => demo_scene_path(),
        },
        None => demo_scene_path(),
    }
}

/// Log the source geometry as a static overlay, so the reconstruction can be
/// read against the thing it was reconstructed from.
///
/// The default is `candi.obj` — the exact file the MJCF loads — because an OBJ
/// is raw vertices in whatever frame it was written, so it lands in the map
/// frame with nothing to correct. glTF is the other case: the format fixes +Y
/// as up and Blender's exporter rotates on the way out, which has to be undone
/// here since this recording is Z-up. That rotation is the whole reason the two
/// formats are not interchangeable at this call site.
fn log_mesh_overlay(
    rec: &rerun::RecordingStream,
    path: &Path,
) -> Result<(), Box<dyn std::error::Error>> {
    let gltf = matches!(
        path.extension()
            .and_then(|e| e.to_str())
            .map(str::to_ascii_lowercase)
            .as_deref(),
        Some("glb" | "gltf")
    );

    if gltf {
        // +90 deg about X, which sends glTF's (x, y, z) to (x, -z, y).
        rec.log_static(
            "candi/mesh",
            &rerun::Transform3D::from_rotation(rerun::RotationAxisAngle::new(
                [1.0, 0.0, 0.0],
                rerun::Angle::from_degrees(90.0),
            )),
        )?;
    }

    rec.log_static("candi/mesh", &rerun::Asset3D::from_file_path(path)?)?;
    Ok(())
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let connect = std::env::args().any(|a| a == "--connect");

    let out_dir = workspace_root().join("out");
    std::fs::create_dir_all(&out_dir)?;
    let rrd_path = out_dir.join("candi_scan.rrd");

    let rec = if connect {
        println!("Streaming to a running Rerun viewer on the default port.");
        rerun::RecordingStreamBuilder::new("candi_scan").connect_grpc()?
    } else {
        println!("Recording to {}", rrd_path.display());
        rerun::RecordingStreamBuilder::new("candi_scan").save(&rrd_path)?
    };

    // Rerun's default is Y-up; MuJoCo and this scene are Z-up.
    rec.log_static("/", &rerun::ViewCoordinates::RIGHT_HAND_Z_UP())?;

    if let Some(mesh) = mesh_arg(workspace_root().join("assets/candi_obj/candi.obj")) {
        log_mesh_overlay(&rec, &mesh)?;
        println!("mesh overlay: {}", mesh.display());
    }

    let scene = scene_arg();
    println!("scene: {}", scene.display());
    let model = MjModel::from_xml(&scene)?;
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

    println!(
        "candi {:.2} x {:.2} x {:.2} m | orbit r={:.2} m | {} waypoints",
        dims[0],
        dims[1],
        dims[2],
        plan.radius,
        plan.len()
    );

    let mocap_id = model
        .body("drone")
        .expect("drone body not found")
        .view(&model)
        .mocapid[0] as usize;
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
        .build(&model)?;

    let intrinsics = Intrinsics::from_fovy(WIDTH, HEIGHT, FOVY_DEG);
    let mut map = OccupancyMap::new(RESOLUTION);

    // The library this repository demonstrates. Same resolution and the same
    // points as the hash grid above, so the two are directly comparable rather
    // than merely similar. The sensor model is left at octomap-core's defaults
    // — prob_hit 0.7, prob_miss 0.4 — because those are what the hash grid was
    // written to.
    let mut octree = OcTree::new(RESOLUTION as f64)?;
    let octree_carves = !std::env::args().any(|a| a == "--octree-no-carve");
    let mut octree_time = std::time::Duration::ZERO;

    let mut path: Vec<[f32; 3]> = Vec::with_capacity(plan.len());

    // Colour voxels against the candi's own height range so the gradient always
    // spans the structure regardless of scale.
    let (z_min, z_max) = (lo[2] as f32, hi[2] as f32);

    let start = std::time::Instant::now();
    let mut insert_time = std::time::Duration::ZERO;
    let mut ground_dropped = 0usize;
    let mut cloud_total = 0usize;

    for (i, wp) in plan.waypoints.iter().enumerate() {
        data.mocap_pos_mut()[mocap_id] = wp.pos;
        data.mocap_quat_mut()[mocap_id] = wp.quat;
        data.forward();

        renderer.sync_data(&mut data)?;
        renderer.render()?;

        let camera = CameraPose {
            pos: data.cam_xpos()[cam_id],
            mat: data.cam_xmat()[cam_id],
        };
        let depth = renderer.depth_flat().expect("depth rendering disabled");
        let cloud = depth_to_cloud(depth, intrinsics, camera, MAX_RANGE, SUBSAMPLE);
        let structure: Vec<[f32; 3]> = cloud.iter().copied().filter(|p| p[2] >= GROUND_Z).collect();
        ground_dropped += cloud.len() - structure.len();

        let origin = [
            camera.pos[0] as f32,
            camera.pos[1] as f32,
            camera.pos[2] as f32,
        ];
        let t0 = std::time::Instant::now();
        map.insert_point_cloud(&structure, origin, true, CARVE_FREE);
        insert_time += t0.elapsed();

        // The same points, into octomap-core. `insert_point_cloud` traces each
        // ray, so it records the free space between the sensor and the surface
        // as well as the surface itself — the distinction between "free" and
        // "never observed" that an occupancy map exists to keep.
        let sensor = Point3::new(origin[0], origin[1], origin[2]);
        let scan: PointCloud = structure
            .iter()
            .map(|p| Point3::new(p[0], p[1], p[2]))
            .collect();
        let t1 = std::time::Instant::now();
        if octree_carves {
            octree.insert_point_cloud(&scan, sensor, MAX_RANGE as f64, false, true);
        } else {
            // Endpoints only, matching what the hash grid does. There is no
            // flag for this on insert_point_cloud: tracing rays is what that
            // call is, so skipping them means updating each endpoint directly.
            for &p in scan.iter() {
                octree.update_node_at(p, true);
            }
        }
        octree_time += t1.elapsed();

        cloud_total += cloud.len();
        path.push([wp.pos[0] as f32, wp.pos[1] as f32, wp.pos[2] as f32]);

        rec.set_time_sequence("frame", i as i64);

        // The drone marker and its trail update every frame; the voxel set is
        // large, so it goes out less often.
        rec.log(
            "drone/position",
            &rerun::Points3D::new([*path.last().unwrap()])
                .with_colors([[255u8, 255, 80]])
                .with_radii([0.35]),
        )?;
        rec.log("drone/path", &rerun::LineStrips3D::new([path.clone()]))?;

        if i % LOG_EVERY == 0 || i == plan.len() - 1 {
            let centres = map.occupied_centres();
            let colours: Vec<[u8; 3]> = centres
                .iter()
                .map(|c| palette::height_colour_u8(palette::normalize(c[2], z_min, z_max)))
                .collect();

            rec.log(
                "hashgrid/occupied",
                &rerun::Points3D::new(centres.iter().copied())
                    .with_colors(colours)
                    .with_radii([RESOLUTION * 0.5]),
            )?;

            let cells = occupied_cells(&octree);
            let colours: Vec<[u8; 3]> = cells
                .iter()
                .map(|(c, _)| palette::height_colour_u8(palette::normalize(c[2], z_min, z_max)))
                .collect();
            // Radius follows node size: a pruned octree node stands for a cube
            // larger than one voxel, and drawing it at the base resolution
            // would make the map look sparser than it is.
            rec.log(
                "octree/occupied",
                &rerun::Points3D::new(cells.iter().map(|(c, _)| *c))
                    .with_colors(colours)
                    .with_radii(cells.iter().map(|(_, size)| size * 0.5)),
            )?;
        }

        if i % 72 == 0 || i == plan.len() - 1 {
            let s = map.stats();
            println!(
                "wp {i:>3} ring {} | cloud {:>5} pts | map {:>7} occupied voxels",
                wp.ring,
                cloud.len(),
                s.occupied
            );
        }
    }

    let elapsed = start.elapsed();
    let stats = map.stats();

    println!("\n--- summary ---");
    println!(
        "{} waypoints in {:.2}s ({:.1} ms/frame)",
        plan.len(),
        elapsed.as_secs_f64(),
        elapsed.as_secs_f64() * 1000.0 / plan.len() as f64
    );
    println!(
        "cloud: {cloud_total} points, {ground_dropped} dropped as ground ({:.1}%)",
        100.0 * ground_dropped as f64 / cloud_total.max(1) as f64
    );

    // Both maps, from the same points, side by side. The row to read is
    // "occupied voxels at <resolution>": two implementations written
    // independently should land on the same number, and if they do not, that
    // is a correctness problem rather than a curiosity.
    let cells = occupied_cells(&octree);
    let equivalent = occupied_voxel_equivalent(&cells, RESOLUTION);
    let frames = plan.len().max(1) as f64;

    println!();
    println!("{:<30} {:>14} {:>14}", "", "octomap-core", "hash grid");
    println!(
        "{:<30} {:>14} {:>14}",
        "occupied leaves / entries",
        cells.len(),
        stats.occupied
    );
    println!(
        "{:<30} {:>14} {:>14}",
        format!("occupied voxels at {RESOLUTION} m"),
        equivalent,
        stats.occupied
    );
    println!(
        "{:<30} {:>14} {:>14}",
        "nodes / entries held",
        octree.len(),
        stats.total_voxels
    );
    println!(
        "{:<30} {:>11.1} ms {:>11.1} ms",
        "insertion, per frame",
        octree_time.as_secs_f64() * 1000.0 / frames,
        insert_time.as_secs_f64() * 1000.0 / frames,
    );
    println!(
        "\nfree-space carving: octomap-core {}, hash grid off",
        if octree_carves { "on" } else { "off" }
    );
    if octree_carves {
        println!(
            "The octree's node count includes free space; the hash grid's does not.\n\
             Pass --octree-no-carve for a like-for-like comparison."
        );
    }

    let occupied = map.occupied_centres();
    let mut vlo = [f32::INFINITY; 3];
    let mut vhi = [f32::NEG_INFINITY; 3];
    for c in &occupied {
        for a in 0..3 {
            vlo[a] = vlo[a].min(c[a]);
            vhi[a] = vhi[a].max(c[a]);
        }
    }
    println!(
        "occupied extent: [{:.2}, {:.2}, {:.2}] .. [{:.2}, {:.2}, {:.2}]",
        vlo[0], vlo[1], vlo[2], vhi[0], vhi[1], vhi[2]
    );

    // Orthographic projections of the finished map. The .rrd needs a viewer to
    // inspect; these PNGs make the reconstruction checkable on its own, which
    // is what answers the only question that matters here — does the shape read
    // as the structure that was scanned.
    write_projection(
        &out_dir.join("map_top.png"),
        &occupied,
        Projection::Top,
        z_min,
        z_max,
    )?;
    write_projection(
        &out_dir.join("map_side.png"),
        &occupied,
        Projection::Side,
        z_min,
        z_max,
    )?;
    println!("wrote out/map_top.png and out/map_side.png");

    // Propagated rather than ignored: a failed flush means the recording on
    // disk is short, and the run would otherwise report success.
    rec.flush_blocking()?;

    if !connect {
        println!("\nWrote {}", rrd_path.display());
        println!("View it with:  .tools\\rerun\\rerun.exe out\\candi_scan.rrd");
    }

    Ok(())
}
