//! Render the demo to animated PNGs.
//!
//! Produces two fallback animations that need no viewer and no video codec:
//!
//!   out/orbit.apng      the drone flying its scan orbit around the candi
//!   out/map_growth.apng the occupancy map filling in as the scan proceeds
//!
//! APNG rather than a video file because there is no encoder on this machine
//! and the `png` crate is already a dependency. Any browser plays the result.
//!
//!   cargo run --release -p candi-sim --example record_demo

use candi_octomap_node::OccupancyMap;
use candi_octomap_node::palette;
use candi_sim::depth_to_cloud::{CameraPose, Intrinsics, depth_to_cloud};
use candi_sim::orbit::OrbitPlan;
use candi_sim::{depth_camera_options, scene_path, workspace_root};
use mujoco_rs::prelude::*;
use mujoco_rs::renderer::MjRenderer;

/// Render size. Kept modest so the animations stay a few MB.
///
/// `--web` halves the frame count and trims the canvas, producing files small
/// enough to inline into a page as data URIs.
const W: usize = 512;
const H: usize = 384;
const WEB_W: usize = 384;
const WEB_H: usize = 288;

/// Depth pass matches the scan pipeline.
const DEPTH_W: usize = 640;
const DEPTH_H: usize = 480;
const FOVY_DEG: f64 = 60.0;
const MAX_RANGE: f32 = 20.0;
const SUBSAMPLE: usize = 4;
const GROUND_Z: f32 = 0.15;
const RESOLUTION: f32 = 0.1;

const RINGS: usize = 4;
const POINTS_PER_RING: usize = 72;
const RADIUS_FACTOR: f64 = 1.5;

/// Every Nth waypoint becomes a frame: 288 / 3 = 96 frames, ~8 s at 12 fps.
const FRAME_EVERY: usize = 3;
const FPS: u16 = 12;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let web = std::env::args().any(|a| a == "--web");
    let (w, h) = if web { (WEB_W, WEB_H) } else { (W, H) };
    let frame_every = if web { FRAME_EVERY * 2 } else { FRAME_EVERY };
    let suffix = if web { "_web" } else { "" };

    let out_dir = workspace_root().join("out");
    std::fs::create_dir_all(&out_dir)?;

    let model = MjModel::from_xml(scene_path())?;
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
    let cam_id = model
        .name_to_id(MjtObj::mjOBJ_CAMERA, "drone_cam")
        .expect("drone_cam not found");

    // One renderer only — winit permits a single event loop per process — so
    // the RGB overview and the depth pass take turns on it.
    let mut renderer = MjRenderer::builder()
        .width(W.max(DEPTH_W) as u32)
        .height(H.max(DEPTH_H) as u32)
        .rgb(true)
        .depth(true)
        .camera(MjvCamera::new_free(&model))
        .build(&model)?;

    let intrinsics = Intrinsics::from_fovy(DEPTH_W, DEPTH_H, FOVY_DEG);
    let mut map = OccupancyMap::new(RESOLUTION);
    let (z_min, z_max) = (lo[2] as f32, hi[2] as f32);

    let mut orbit_frames: Vec<Vec<u8>> = Vec::new();
    let mut growth_frames: Vec<Vec<u8>> = Vec::new();

    println!("rendering {} waypoints...", plan.len());

    for (i, wp) in plan.waypoints.iter().enumerate() {
        data.mocap_pos_mut()[mocap_id] = wp.pos;
        data.mocap_quat_mut()[mocap_id] = wp.quat;
        data.forward();

        // --- depth pass: feed the map ---
        renderer.set_opts(depth_camera_options());
        renderer.set_camera(MjvCamera::new_fixed(cam_id));
        renderer.sync_data(&mut data)?;
        renderer.render()?;

        let camera = CameraPose {
            pos: data.cam_xpos()[cam_id],
            mat: data.cam_xmat()[cam_id],
        };
        let depth = renderer.depth_flat().expect("depth disabled");
        let cloud = depth_to_cloud(depth, intrinsics, camera, MAX_RANGE, SUBSAMPLE);
        let structure: Vec<[f32; 3]> = cloud.into_iter().filter(|p| p[2] >= GROUND_Z).collect();
        let origin = [
            camera.pos[0] as f32,
            camera.pos[1] as f32,
            camera.pos[2] as f32,
        ];
        map.insert_point_cloud(&structure, origin, true, false);

        if i % frame_every != 0 {
            continue;
        }

        // --- rgb pass: the scene from outside, slowly turning ---
        let mut overview = MjvCamera::new_free(&model);
        overview.distance = 46.0;
        // A slow counter-rotation keeps the temple from looking static.
        overview.azimuth = 120.0 + 40.0 * (i as f64 / plan.len() as f64);
        overview.elevation = -22.0;
        overview.lookat = [centre[0], centre[1], centre[2]];
        renderer.set_camera(overview);
        renderer.set_opts(MjvOption::default());
        renderer.sync_data(&mut data)?;
        renderer.render()?;

        let rgb = renderer.rgb_flat().expect("rgb disabled");
        orbit_frames.push(crop_rgb(rgb, DEPTH_W, w, h));

        growth_frames.push(render_top_view(
            &map.occupied_centres(),
            lo,
            hi,
            z_min,
            z_max,
            w,
            h,
        ));

        if orbit_frames.len().is_multiple_of(16) {
            println!(
                "  {} frames, {} voxels",
                orbit_frames.len(),
                map.stats().occupied
            );
        }
    }

    let p1 = out_dir.join(format!("orbit{suffix}.apng"));
    write_apng(&p1, &orbit_frames, w, h)?;
    println!("wrote {} ({} frames)", p1.display(), orbit_frames.len());

    let p2 = out_dir.join(format!("map_growth{suffix}.apng"));
    write_apng(&p2, &growth_frames, w, h)?;
    println!("wrote {} ({} frames)", p2.display(), growth_frames.len());

    Ok(())
}

/// Take the top-left W x H window out of a larger RGB buffer.
fn crop_rgb(src: &[u8], src_w: usize, w: usize, h: usize) -> Vec<u8> {
    let mut out = vec![0u8; w * h * 3];
    for y in 0..h {
        let s = y * src_w * 3;
        let d = y * w * 3;
        out[d..d + w * 3].copy_from_slice(&src[s..s + w * 3]);
    }
    out
}

/// Top-down projection of the occupied voxels, framed on the candi's bounds so
/// the view never jumps as the map grows.
fn render_top_view(
    voxels: &[[f32; 3]],
    lo: [f64; 3],
    hi: [f64; 3],
    z_min: f32,
    z_max: f32,
    w: usize,
    h: usize,
) -> Vec<u8> {
    let mut rgb = vec![18u8; w * h * 3];

    let span = ((hi[0] - lo[0]).max(hi[1] - lo[1])) as f32;
    let scale = (h - 16) as f32 / span;
    let ox = (w as f32 - span * scale) / 2.0;
    let oy = 8.0;

    let mut best = vec![f32::NEG_INFINITY; w * h];
    let cell = ((RESOLUTION * scale).ceil() as usize).max(1);

    for v in voxels {
        let px = ((v[0] - lo[0] as f32) * scale + ox) as usize;
        let py = h - 1 - ((v[1] - lo[1] as f32) * scale + oy) as usize;
        let c = palette::height_colour_u8(palette::normalize(v[2], z_min, z_max));

        for dy in 0..cell {
            let y = py + dy;
            if y >= h {
                continue;
            }
            for dx in 0..cell {
                let x = px + dx;
                if x >= w {
                    continue;
                }
                let idx = y * w + x;
                if v[2] <= best[idx] {
                    continue;
                }
                best[idx] = v[2];
                rgb[idx * 3..idx * 3 + 3].copy_from_slice(&c);
            }
        }
    }

    rgb
}

fn write_apng(
    path: &std::path::Path,
    frames: &[Vec<u8>],
    w: usize,
    h: usize,
) -> Result<(), Box<dyn std::error::Error>> {
    if frames.is_empty() {
        return Ok(());
    }

    let file = std::fs::File::create(path)?;
    let mut encoder = png::Encoder::new(std::io::BufWriter::new(file), w as u32, h as u32);
    encoder.set_color(png::ColorType::Rgb);
    encoder.set_depth(png::BitDepth::Eight);
    encoder.set_animated(frames.len() as u32, 0)?;
    encoder.set_frame_delay(1, FPS)?;

    let mut writer = encoder.write_header()?;
    for frame in frames {
        writer.write_image_data(frame)?;
    }
    writer.finish()?;
    Ok(())
}
