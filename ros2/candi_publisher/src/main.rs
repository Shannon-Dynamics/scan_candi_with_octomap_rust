//! The candi scan, published as ROS 2 topics.
//!
//! Same flight, same depth camera, same projection as `live_scan`. The only
//! difference is where the points go: instead of straight into an occupancy
//! map in the same process, they leave as `sensor_msgs/PointCloud2` and the
//! mapping happens in `candi_mapper` on the other side of the middleware.
//!
//! ```text
//!   MuJoCo orbit ──► depth ──► depth_to_cloud ──► /cloud   (PointCloud2)
//!                                            └──► /tf      (map → drone_cam)
//! ```
//!
//! # Which frame the points are in
//!
//! `depth_to_cloud` already returns **world-frame** points — it applies the
//! camera pose during projection — so the cloud goes out with `frame_id: map`
//! and needs no transform on receipt. What the mapper still needs is where the
//! rays *started*, and that is what `/tf` carries: `map → drone_cam` is
//! published every frame, and its translation is the sensor origin.
//!
//! That split is unusual — normally a cloud arrives in the sensor's frame and
//! TF moves it — but it is what the data actually is, and converting the points
//! back into the camera frame just so the mapper could convert them out again
//! would lose precision to no purpose.
//!
//! # Pacing, not throttling
//!
//! [`RateLimiter`] exists to drop frames when the sim outruns the publish rate.
//! Here the opposite is wanted. The orbit is 288 waypoints that MuJoCo renders
//! in about a second; dropping to 10 Hz would publish ten of them and throw the
//! scan away. So the loop *waits* for each slot instead, and every waypoint is
//! published — 288 frames over ~29 s, which is what a 10 Hz sensor flying this
//! orbit would actually produce.

use std::error::Error;
use std::time::{Duration, Instant};

use candi_sim::depth_to_cloud::{depth_to_cloud, CameraPose, Intrinsics};
use candi_sim::orbit::OrbitPlan;
use candi_sim::ros_bridge::{CloudFrame, Transform, FRAME_MAP, PUBLISH_HZ};
use candi_sim::{depth_camera_options, scene_path};
use mujoco_rs::prelude::*;
use mujoco_rs::renderer::MjRenderer;

use r2r::builtin_interfaces::msg::Time;
use r2r::geometry_msgs::msg::{Quaternion, Transform as TransformMsg, TransformStamped, Vector3};
use r2r::sensor_msgs::msg::{PointCloud2, PointField};
use r2r::std_msgs::msg::Header;
use r2r::tf2_msgs::msg::TFMessage;
use r2r::{Clock, ClockType, Context, Node, QosProfile};

/// Camera and orbit settings, copied from `live_scan` so the two runs are
/// comparable frame for frame.
const WIDTH: usize = 640;
const HEIGHT: usize = 480;
const FOVY_DEG: f64 = 60.0;
const MAX_RANGE: f32 = 20.0;
const SUBSAMPLE: usize = 4;
const RINGS: usize = 4;
const POINTS_PER_RING: usize = 72;
const RADIUS_FACTOR: f64 = 1.5;

/// Drop returns below this height as ground.
///
/// Kept here rather than left to the mapper because it is a property of this
/// scene: the camera sees the ground plane out to the full 20 m range in every
/// direction, and without the cut the scan is 56% floor. The mapper has the
/// same knob, but filtering before transmission saves sending the points at all.
const GROUND_Z: f32 = 0.15;

/// The camera frame published on `/tf`, and the one the sensor origin is read
/// from.
const FRAME_CAMERA: &str = "drone_cam";

fn main() -> Result<(), Box<dyn Error>> {
    let wait_for_subscriber = !std::env::args().any(|a| a == "--no-wait");

    // Fly only the first N waypoints. The orbit is 288 of them, and a debug
    // build of the mapper cannot integrate that many in a reasonable time —
    // useful when the question is whether the pipeline works rather than what
    // the finished map looks like.
    let limit = std::env::args()
        .position(|a| a == "--waypoints")
        .and_then(|i| std::env::args().nth(i + 1))
        .and_then(|v| v.parse::<usize>().ok());

    let ctx = Context::create()?;
    let mut node = Node::create(ctx, "candi_publisher", "")?;
    let logger = node.logger().to_string();

    // Sensor data QoS: best-effort, which is what a real depth camera driver
    // publishes and what the mapper subscribes with.
    let cloud_pub = node.create_publisher::<PointCloud2>("cloud", QosProfile::sensor_data())?;
    let tf_pub =
        node.create_publisher::<TFMessage>("/tf", QosProfile::default().reliable().keep_last(100))?;
    let mut clock = Clock::create(ClockType::RosTime)?;

    // ---- the simulation, unchanged from live_scan -------------------------
    let model = MjModel::from_xml(&scene_path())?;
    let mut data = MjData::new(&model);
    data.forward();

    let (lo, hi) = candi_sim::body_aabb(&model, &data, "candi").ok_or("candi body not found")?;
    let dims = [hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]];
    let centre = [
        (lo[0] + hi[0]) / 2.0,
        (lo[1] + hi[1]) / 2.0,
        (lo[2] + hi[2]) / 2.0,
    ];
    let heights = OrbitPlan::heights_for(lo[2], hi[2], RINGS);
    let mut plan = OrbitPlan::generate(centre, dims, &heights, RADIUS_FACTOR, POINTS_PER_RING);

    if let Some(n) = limit {
        plan.waypoints.truncate(n);
    }

    let mocap_id = model
        .body("drone")
        .ok_or("drone body not found")?
        .view(&model)
        .mocapid[0] as usize;
    let cam_id = model
        .name_to_id(MjtObj::mjOBJ_CAMERA, FRAME_CAMERA)
        .ok_or("drone_cam not found")?;

    let mut renderer = MjRenderer::builder()
        .width(WIDTH as u32)
        .height(HEIGHT as u32)
        .rgb(false)
        .depth(true)
        .camera(MjvCamera::new_fixed(cam_id))
        .opts(depth_camera_options())
        .build(&model)?;

    let intrinsics = Intrinsics::from_fovy(WIDTH, HEIGHT, FOVY_DEG);

    r2r::log_info!(
        &logger,
        "candi {:.2} x {:.2} x {:.2} m | orbit r={:.2} m | {} waypoints at {PUBLISH_HZ} Hz",
        dims[0],
        dims[1],
        dims[2],
        plan.radius,
        plan.len()
    );

    // Publishing into a graph with no subscriber loses the whole scan, and a
    // best-effort publisher gives no indication that happened. Waiting is
    // cheap and turns a silent no-op into an obvious pause.
    if wait_for_subscriber {
        r2r::log_info!(&logger, "waiting for a subscriber on 'cloud' (--no-wait to skip)");
        while cloud_pub.get_inter_process_subscription_count()? == 0 {
            node.spin_once(Duration::from_millis(100));
        }
        r2r::log_info!(&logger, "subscriber found, starting the orbit");
    }

    let interval = Duration::from_secs_f64(1.0 / PUBLISH_HZ);
    let start = Instant::now();
    let mut points_sent = 0usize;
    let mut ground_dropped = 0usize;

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
        let depth = renderer.depth_flat().ok_or("depth rendering disabled")?;
        let cloud = depth_to_cloud(depth, intrinsics, camera, MAX_RANGE, SUBSAMPLE);

        let structure: Vec<[f32; 3]> = cloud.iter().copied().filter(|p| p[2] >= GROUND_Z).collect();
        ground_dropped += cloud.len() - structure.len();
        points_sent += structure.len();

        let stamp = now(&mut clock);
        let frame = CloudFrame::from_points(&structure);

        // The camera pose becomes map -> drone_cam. Only the translation is
        // load-bearing for mapping — it is the ray origin — but a real rotation
        // makes the frame usable in RViz and in any consumer that expects a
        // proper transform.
        let pose = Transform {
            translation: camera.pos,
            quat_wxyz: mat_to_quat_wxyz(&camera.mat),
        };

        tf_pub.publish(&tf_message(&pose, &stamp))?;
        cloud_pub.publish(&cloud_message(&frame, &stamp))?;

        node.spin_once(Duration::from_millis(0));

        if i % 72 == 0 || i == plan.len() - 1 {
            r2r::log_info!(
                &logger,
                "wp {i:>3} ring {} | {} points sent",
                wp.ring,
                structure.len()
            );
        }

        // Hold the cadence. Sleeping to an absolute deadline rather than for a
        // fixed duration keeps the rate from drifting as render times vary.
        let deadline = start + interval * (i as u32 + 1);
        if let Some(remaining) = deadline.checked_duration_since(Instant::now()) {
            std::thread::sleep(remaining);
        }
    }

    let elapsed = start.elapsed();
    r2r::log_info!(
        &logger,
        "done: {} frames, {points_sent} points, {ground_dropped} dropped as ground, {:.1} s ({:.2} Hz)",
        plan.len(),
        elapsed.as_secs_f64(),
        plan.len() as f64 / elapsed.as_secs_f64()
    );

    // The mapper publishes on a timer, so give the last frames time to arrive
    // and be integrated before this process exits and the topics disappear.
    for _ in 0..20 {
        node.spin_once(Duration::from_millis(100));
    }

    Ok(())
}

fn now(clock: &mut Clock) -> Time {
    clock
        .get_now()
        .map(|d| Clock::to_builtin_time(&d))
        .unwrap_or(Time { sec: 0, nanosec: 0 })
}

/// Wraps a packed cloud in a `sensor_msgs/PointCloud2`.
fn cloud_message(frame: &CloudFrame, stamp: &Time) -> PointCloud2 {
    let field = |name: &str, offset: u32| PointField {
        name: name.to_string(),
        offset,
        datatype: PointField::FLOAT32 as u8,
        count: 1,
    };

    PointCloud2 {
        header: Header {
            stamp: stamp.clone(),
            // The points are already world-frame; see the module docs.
            frame_id: FRAME_MAP.to_string(),
        },
        height: 1,
        width: frame.point_count,
        fields: vec![field("x", 0), field("y", 4), field("z", 8)],
        is_bigendian: false,
        point_step: CloudFrame::POINT_STEP,
        row_step: frame.row_step(),
        data: frame.data.clone(),
        // depth_to_cloud drops non-finite returns, so what is left is dense.
        is_dense: true,
    }
}

/// Wraps the camera pose in a `tf2_msgs/TFMessage`.
fn tf_message(pose: &Transform, stamp: &Time) -> TFMessage {
    let [x, y, z, w] = pose.quat_xyzw();

    TFMessage {
        transforms: vec![TransformStamped {
            header: Header {
                stamp: stamp.clone(),
                frame_id: FRAME_MAP.to_string(),
            },
            child_frame_id: FRAME_CAMERA.to_string(),
            transform: TransformMsg {
                translation: Vector3 {
                    x: pose.translation[0],
                    y: pose.translation[1],
                    z: pose.translation[2],
                },
                rotation: Quaternion { x, y, z, w },
            },
        }],
    }
}

/// Converts MuJoCo's row-major 3x3 camera matrix to a `[w, x, y, z]` quaternion.
///
/// Shepperd's method: pick the branch whose denominator is largest so the
/// square root never lands near zero. The naive single-branch formula loses
/// most of its precision when the trace approaches -1, which happens here
/// whenever the camera looks along -x.
fn mat_to_quat_wxyz(m: &[f64; 9]) -> [f64; 4] {
    let (m00, m01, m02) = (m[0], m[1], m[2]);
    let (m10, m11, m12) = (m[3], m[4], m[5]);
    let (m20, m21, m22) = (m[6], m[7], m[8]);
    let trace = m00 + m11 + m22;

    if trace > 0.0 {
        let s = (trace + 1.0).sqrt() * 2.0;
        [0.25 * s, (m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s]
    } else if m00 > m11 && m00 > m22 {
        let s = (1.0 + m00 - m11 - m22).sqrt() * 2.0;
        [(m21 - m12) / s, 0.25 * s, (m01 + m10) / s, (m02 + m20) / s]
    } else if m11 > m22 {
        let s = (1.0 + m11 - m00 - m22).sqrt() * 2.0;
        [(m02 - m20) / s, (m01 + m10) / s, 0.25 * s, (m12 + m21) / s]
    } else {
        let s = (1.0 + m22 - m00 - m11).sqrt() * 2.0;
        [(m10 - m01) / s, (m02 + m20) / s, (m12 + m21) / s, 0.25 * s]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: [f64; 4], b: [f64; 4]) {
        // q and -q are the same rotation, so compare up to sign.
        let flip = if a[0].signum() != b[0].signum() { -1.0 } else { 1.0 };
        for i in 0..4 {
            assert!(
                (a[i] - flip * b[i]).abs() < 1e-9,
                "got {a:?}, want {b:?} (up to sign)"
            );
        }
    }

    #[test]
    fn the_identity_matrix_is_the_identity_rotation() {
        let m = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];
        close(mat_to_quat_wxyz(&m), [1.0, 0.0, 0.0, 0.0]);
    }

    #[test]
    fn a_quarter_turn_about_z_round_trips() {
        // Rotating x onto y.
        let m = [0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0];
        let h = std::f64::consts::FRAC_1_SQRT_2;
        close(mat_to_quat_wxyz(&m), [h, 0.0, 0.0, h]);
    }

    #[test]
    fn the_negative_trace_branches_stay_accurate() {
        // 180 degrees about x, y and z in turn — each drives the trace to -1,
        // which is where the single-branch formula falls apart.
        close(
            mat_to_quat_wxyz(&[1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0]),
            [0.0, 1.0, 0.0, 0.0],
        );
        close(
            mat_to_quat_wxyz(&[-1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, -1.0]),
            [0.0, 0.0, 1.0, 0.0],
        );
        close(
            mat_to_quat_wxyz(&[-1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0]),
            [0.0, 0.0, 0.0, 1.0],
        );
    }

    #[test]
    fn every_result_is_a_unit_quaternion() {
        // A camera looking down at 45 degrees, the sort of pose this orbit
        // actually produces.
        let c = std::f64::consts::FRAC_1_SQRT_2;
        let m = [1.0, 0.0, 0.0, 0.0, c, -c, 0.0, c, c];
        let q = mat_to_quat_wxyz(&m);
        let norm = q.iter().map(|v| v * v).sum::<f64>().sqrt();
        assert!((norm - 1.0).abs() < 1e-12, "norm was {norm}");
    }

    #[test]
    fn a_cloud_message_describes_its_own_bytes() {
        let frame = CloudFrame::from_points(&[[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]);
        let msg = cloud_message(&frame, &Time { sec: 0, nanosec: 0 });

        assert_eq!(msg.width, 2);
        assert_eq!(msg.height, 1);
        assert_eq!(msg.point_step, 12);
        assert_eq!(msg.row_step, 24);
        assert_eq!(msg.data.len() as u32, msg.row_step * msg.height);
        assert_eq!(msg.header.frame_id, "map");

        let offsets: Vec<u32> = msg.fields.iter().map(|f| f.offset).collect();
        assert_eq!(offsets, [0, 4, 8]);
    }

    #[test]
    fn the_transform_message_puts_w_last() {
        let pose = Transform {
            translation: [1.0, 2.0, 3.0],
            quat_wxyz: [0.7071, 0.0, 0.0, 0.7071],
        };
        let msg = tf_message(&pose, &Time { sec: 0, nanosec: 0 });
        let t = &msg.transforms[0];

        assert_eq!(t.header.frame_id, "map");
        assert_eq!(t.child_frame_id, "drone_cam");
        assert_eq!(t.transform.rotation.w, 0.7071);
        assert_eq!(t.transform.rotation.z, 0.7071);
        assert_eq!(t.transform.rotation.x, 0.0);
        assert_eq!(t.transform.translation.z, 3.0);
    }
}
