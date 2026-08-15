//! The mapping node: both occupancy maps running side by side.
//!
//! Subscribes to the scan `candi_publisher` puts on the wire and folds it into
//! **two** occupancy maps from the same points:
//!
//! - `octomap-core` — the octree, a Rust port of OctoMap
//! - `candi-octomap-node` — the hash grid written for this project when the
//!   octree was unavailable
//!
//! Running both is the point. They were built to the same sensor model
//! (`prob_hit` 0.7, `prob_miss` 0.4, clamps that agree to two decimals), so
//! their answers are directly comparable, and the interesting question is what
//! the octree buys.
//!
//! # The measurement worth taking
//!
//! `live_scan` disables free-space carving, with this reasoning:
//!
//! > Turning it on would trace ~200 voxels per ray and store every empty voxel
//! > it crosses — for this scene that is a ~40 m box at 0.1 m, i.e. millions of
//! > entries and hundreds of megabytes, to remove nothing.
//!
//! That is true of a hash grid, which stores every free voxel individually. It
//! is not a property of occupancy mapping — an octree prunes a uniform region
//! into a single node, and mapped free space is about as uniform as volumes
//! get. So the octree carves by default here, and the summary at the end
//! reports what it cost. `--octree-no-carve` turns it off for a like-for-like
//! comparison against the hash grid.
//!
//! # Frames
//!
//! The cloud arrives already in the map frame — `depth_to_cloud` projects
//! through the camera pose before publishing — so the points need no
//! transform. `/tf` is read for one thing: `map → drone_cam` gives where the
//! rays started. Without it every ray would be traced from the origin and the
//! free space would be nonsense.

use std::collections::HashMap;
use std::error::Error;
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

use futures::StreamExt;

use candi_octomap_node::{palette, OccupancyMap};
use octomap_core::{OcTree, Point3};
use octomap_ros::pointcloud2::{Cloud, FieldRef};
use octomap_ros::{msg as octomap_payload, voxels, ScanFilter, Transform3};

use r2r::octomap_msgs::msg::Octomap as OctomapMsg;
use r2r::sensor_msgs::msg::PointCloud2;
use r2r::std_msgs::msg::Header;
use r2r::tf2_msgs::msg::TFMessage;
use r2r::{Context, Node, QosProfile};

/// Matches `live_scan` so the maps are comparable frame for frame.
const RESOLUTION: f64 = 0.1;
const MAX_RANGE: f64 = 20.0;
const CAMERA_FRAME: &str = "drone_cam";

/// How often the map goes out on ROS and into the recording.
const PUBLISH_PERIOD: Duration = Duration::from_millis(500);

/// Height range used for the colour ramp, from the candi's own extent.
///
/// Fixed rather than tracking the map's bounds, so the colours mean the same
/// thing in every frame of the recording — a gradient that rescales as the map
/// grows makes the playback look like the structure is changing when it is not.
const Z_MIN: f32 = 0.0;
const Z_MAX: f32 = 6.0;

/// Default overlay geometry, resolved at compile time from this crate's
/// location so it does not depend on where the binary is launched from.
const DEFAULT_MESH: &str = concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../../assets/candi_obj/candi.obj"
);

/// What one scan cost, for the summary.
#[derive(Default)]
struct Timings {
    octree: Duration,
    hash_grid: Duration,
    frames: u64,
    points: u64,
}

/// Resolve `--mesh [path]`. Bare `--mesh` takes [`DEFAULT_MESH`].
///
/// Off unless asked for: the mesh adds 27 MB to every recording, and the
/// measurement runs produce one recording each.
fn mesh_arg() -> Option<PathBuf> {
    let args: Vec<String> = std::env::args().collect();
    let at = args.iter().position(|a| a == "--mesh")?;
    Some(match args.get(at + 1) {
        Some(next) if !next.starts_with("--") => PathBuf::from(next),
        _ => PathBuf::from(DEFAULT_MESH),
    })
}

/// Log the source geometry as a static overlay, so the map can be read against
/// the thing it was reconstructed from.
///
/// The default is `candi.obj` — the exact file the MJCF loads — because an OBJ
/// is raw vertices in whatever frame it was written, so it lands in the map
/// frame with nothing to correct. glTF is the other case: the format fixes +Y
/// as up and Blender's exporter rotates on the way out, which has to be undone
/// here since this recording is Z-up.
fn log_mesh_overlay(rec: &rerun::RecordingStream, path: &Path) -> Result<(), Box<dyn Error>> {
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

fn main() -> Result<(), Box<dyn Error>> {
    let carve = !std::env::args().any(|a| a == "--octree-no-carve");
    let rrd_path = std::env::args()
        .position(|a| a == "--out")
        .and_then(|i| std::env::args().nth(i + 1))
        .unwrap_or_else(|| "out/candi_ros2.rrd".to_string());

    let rec = rerun::RecordingStreamBuilder::new("candi_ros2").save(&rrd_path)?;
    // MuJoCo and this scene are Z-up; Rerun defaults to Y-up.
    rec.log_static("/", &rerun::ViewCoordinates::RIGHT_HAND_Z_UP())?;

    let ctx = Context::create()?;
    let mut node = Node::create(ctx, "candi_mapper", "")?;
    let logger = node.logger().to_string();

    let mut clouds = node.subscribe::<PointCloud2>("cloud", QosProfile::sensor_data())?;
    let mut transforms =
        node.subscribe::<TFMessage>("/tf", QosProfile::default().reliable().keep_last(100))?;
    let map_pub = node.create_publisher::<OctomapMsg>(
        "octomap_binary",
        QosProfile::default().reliable().transient_local().keep_last(1),
    )?;

    let mut octree = OcTree::new(RESOLUTION)?;
    let mut grid = OccupancyMap::new(RESOLUTION as f32);
    let mut origins: HashMap<String, Point3> = HashMap::new();
    let mut path: Vec<[f32; 3]> = Vec::new();
    let mut timings = Timings::default();

    r2r::log_info!(
        &logger,
        "candi_mapper up: resolution {RESOLUTION} m, max_range {MAX_RANGE} m, \
         octree carving {}",
        if carve { "on" } else { "off" }
    );
    r2r::log_info!(&logger, "recording to {rrd_path}");

    if let Some(mesh) = mesh_arg() {
        match log_mesh_overlay(&rec, &mesh) {
            // Not fatal: the overlay is a viewing aid, and a mapper that
            // refuses to start because an asset moved would cost the run.
            Err(e) => r2r::log_warn!(&logger, "no mesh overlay ({}): {e}", mesh.display()),
            Ok(()) => r2r::log_info!(&logger, "mesh overlay: {}", mesh.display()),
        }
    }

    let runtime = tokio::runtime::Builder::new_multi_thread()
        .enable_all()
        .build()?;

    runtime.block_on(async {
        let spin_handle = tokio::task::spawn_blocking(move || loop {
            node.spin_once(Duration::from_millis(10));
        });

        let mut ticker = tokio::time::interval(PUBLISH_PERIOD);
        let mut published = false;

        loop {
            tokio::select! {
                Some(message) = transforms.next() => {
                    for t in message.transforms {
                        origins.insert(
                            t.child_frame_id.clone(),
                            Point3::new(
                                t.transform.translation.x as f32,
                                t.transform.translation.y as f32,
                                t.transform.translation.z as f32,
                            ),
                        );
                    }
                }

                Some(cloud) = clouds.next() => {
                    // This scene publishes exactly one edge, map -> drone_cam,
                    // so the latest translation is the ray origin. A robot with
                    // a real kinematic chain would need a graph walk here.
                    let Some(&origin) = origins.get(CAMERA_FRAME) else {
                        r2r::log_warn!(
                            &logger,
                            "dropping a scan: no {CAMERA_FRAME} transform yet"
                        );
                        continue;
                    };

                    match integrate(
                        &cloud,
                        origin,
                        carve,
                        &mut octree,
                        &mut grid,
                        &mut timings,
                        &logger,
                    ) {
                        Some(count) => {
                            timings.frames += 1;
                            timings.points += count as u64;
                            path.push([origin.x, origin.y, origin.z]);
                        }
                        None => continue,
                    }
                }

                _ = ticker.tick() => {
                    if timings.frames == 0 {
                        continue;
                    }
                    if let Err(e) = publish(&octree, &map_pub) {
                        r2r::log_warn!(&logger, "could not publish the map: {e}");
                    }
                    if let Err(e) = record(&rec, &octree, &grid, &path, timings.frames) {
                        r2r::log_warn!(&logger, "could not log to rerun: {e}");
                    }
                    published = true;
                }

                _ = tokio::signal::ctrl_c() => break,
            }
        }

        // A final round, so the recording ends on the finished map rather than
        // whatever the last tick happened to catch.
        if published {
            let _ = publish(&octree, &map_pub);
            let _ = record(&rec, &octree, &grid, &path, timings.frames);
        }

        spin_handle.abort();
    });

    summarize(&octree, &grid, &timings, carve);
    let _ = rec.flush_blocking();
    println!("\nWrote {rrd_path}");

    Ok(())
}

/// Folds one cloud into both maps. Returns the number of points integrated.
fn integrate(
    message: &PointCloud2,
    origin: Point3,
    carve: bool,
    octree: &mut OcTree,
    grid: &mut OccupancyMap,
    timings: &mut Timings,
    logger: &str,
) -> Option<usize> {
    let fields: Vec<FieldRef<'_>> = message
        .fields
        .iter()
        .map(|f| FieldRef::new(&f.name, f.offset, f.datatype, f.count))
        .collect();

    let cloud = match Cloud::new(
        &fields,
        &message.data,
        message.width,
        message.height,
        message.point_step,
        message.row_step,
        message.is_bigendian,
    ) {
        Ok(cloud) => cloud,
        Err(e) => {
            r2r::log_error!(logger, "unusable cloud: {e}");
            return None;
        }
    };

    // The points are already in the map frame, so the transform is the
    // identity. The filter still runs: it is what drops the non-finite returns.
    let scan = ScanFilter::default().apply(&cloud, &Transform3::IDENTITY);
    if scan.is_empty() {
        return None;
    }

    let t0 = Instant::now();
    if carve {
        octree.insert_point_cloud(&scan, origin, MAX_RANGE, false, true);
    } else {
        // Endpoints only, to match what the hash grid is doing. Going through
        // update_node rather than insert_point_cloud is the only way to skip
        // the ray traversal — carving is not a flag on that call, it is what
        // the call does.
        for &p in scan.iter() {
            octree.update_node_at(p, true);
        }
    }
    timings.octree += t0.elapsed();

    // The hash grid takes the same points, and keeps its own settings: carving
    // off, which is what live_scan measured.
    let points: Vec<[f32; 3]> = scan.iter().map(|p| [p.x, p.y, p.z]).collect();
    let t1 = Instant::now();
    grid.insert_point_cloud(&points, [origin.x, origin.y, origin.z], true, false);
    timings.hash_grid += t1.elapsed();

    Some(scan.len())
}

/// Publishes the octree as an `octomap_msgs/Octomap`.
fn publish(octree: &OcTree, publisher: &r2r::Publisher<OctomapMsg>) -> Result<(), Box<dyn Error>> {
    let payload = octomap_payload::binary_payload(octree)?;
    publisher.publish(&OctomapMsg {
        header: Header {
            stamp: r2r::builtin_interfaces::msg::Time { sec: 0, nanosec: 0 },
            frame_id: "map".to_string(),
        },
        binary: payload.binary,
        id: payload.id.to_string(),
        resolution: payload.resolution,
        data: payload.into_i8(),
    })?;
    Ok(())
}

/// Logs both maps and the flight path into the recording.
fn record(
    rec: &rerun::RecordingStream,
    octree: &OcTree,
    grid: &OccupancyMap,
    path: &[[f32; 3]],
    frame: u64,
) -> Result<(), Box<dyn Error>> {
    rec.set_time_sequence("frame", frame as i64);

    // The octree's occupied leaves. A pruned node stands for a cube larger
    // than one voxel, so its radius follows its own size — drawing every leaf
    // at the base resolution would make merged regions look like sparse dust.
    let cells: Vec<voxels::Voxel> = voxels::occupied_voxels(octree).collect();
    let centres: Vec<[f32; 3]> = cells
        .iter()
        .map(|v| [v.center.x, v.center.y, v.center.z])
        .collect();
    let colours: Vec<[u8; 3]> = cells
        .iter()
        .map(|v| palette::height_colour_u8(palette::normalize(v.center.z, Z_MIN, Z_MAX)))
        .collect();
    let radii: Vec<f32> = cells.iter().map(|v| v.size as f32 * 0.5).collect();

    rec.log(
        "octree/occupied",
        &rerun::Points3D::new(centres)
            .with_colors(colours)
            .with_radii(radii),
    )?;

    // The hash grid, for comparison. Same points in, same palette, so any
    // visible difference is the mapping and not the rendering.
    let grid_centres = grid.occupied_centres();
    let grid_colours: Vec<[u8; 3]> = grid_centres
        .iter()
        .map(|c| palette::height_colour_u8(palette::normalize(c[2], Z_MIN, Z_MAX)))
        .collect();
    rec.log(
        "hashgrid/occupied",
        &rerun::Points3D::new(grid_centres.iter().copied())
            .with_colors(grid_colours)
            .with_radii([RESOLUTION as f32 * 0.5]),
    )?;

    if let Some(last) = path.last() {
        rec.log(
            "drone/position",
            &rerun::Points3D::new([*last])
                .with_colors([[255u8, 255, 80]])
                .with_radii([0.35]),
        )?;
    }
    rec.log("drone/path", &rerun::LineStrips3D::new([path.to_vec()]))?;

    Ok(())
}

/// The side-by-side numbers, printed once at the end.
fn summarize(octree: &OcTree, grid: &OccupancyMap, timings: &Timings, carve: bool) {
    let cells: Vec<voxels::Voxel> = voxels::occupied_voxels(octree).collect();

    // Leaf count and voxel count are different questions once pruning is in
    // play: one merged node can stand for eight, or eight thousand, base
    // voxels. The hash grid only has the second number, so both are reported.
    let equivalent: u64 = cells
        .iter()
        .map(|v| {
            let side = (v.size / RESOLUTION).round() as u64;
            side * side * side
        })
        .sum();

    let payload = octomap_payload::binary_payload(octree)
        .map(|p| p.data.len())
        .unwrap_or(0);
    let stats = grid.stats();

    println!("\n--- {} frames, {} points ---", timings.frames, timings.points);
    println!(
        "{:<28} {:>14} {:>14}",
        "", "octree", "hash grid"
    );
    println!(
        "{:<28} {:>14} {:>14}",
        "occupied leaves / entries",
        cells.len(),
        stats.occupied
    );
    println!(
        "{:<28} {:>14} {:>14}",
        "occupied voxels at 0.1 m", equivalent, stats.occupied
    );
    println!(
        "{:<28} {:>14} {:>14}",
        "nodes / entries held",
        octree.len(),
        stats.total_voxels
    );
    println!(
        "{:<28} {:>13} B {:>14}",
        "serialized (.bt payload)", payload, "n/a"
    );
    println!(
        "{:<28} {:>11.1} ms {:>11.1} ms",
        "insertion, per frame",
        timings.octree.as_secs_f64() * 1000.0 / timings.frames.max(1) as f64,
        timings.hash_grid.as_secs_f64() * 1000.0 / timings.frames.max(1) as f64,
    );
    println!(
        "\nfree-space carving: octree {}, hash grid off",
        if carve { "ON" } else { "off" }
    );
    if carve {
        println!(
            "The octree's node count includes free space; the hash grid's does not, \
             \nwhich is the comparison worth reading."
        );
    }
}
