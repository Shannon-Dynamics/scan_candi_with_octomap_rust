//! Validate that mujoco-rs covers what the candi drone-scan
//! pipeline needs, before anything is built on top of it.
//!
//! Three independent tests, each reported PASS/FAIL on its own:
//!   1. load an MJCF from a string and step the simulation
//!   2. drive a mocap body kinematically and read its pose back
//!   3. render an offscreen depth image and check it is metric
//!
//! Run with: cargo run -p candi-sim --example smoke_test

use mujoco_rs::prelude::*;
use mujoco_rs::renderer::MjRenderer;

/// Test 3 renders at the resolution the real pipeline will use.
const DEPTH_W: usize = 640;
const DEPTH_H: usize = 480;

fn main() {
    println!("=== mujoco-rs smoke test ===\n");

    let results = [
        ("Test 1 — load + step", test_load_and_step()),
        ("Test 2 — mocap body", test_mocap_body()),
        ("Test 3 — offscreen depth render", test_depth_render()),
    ];

    println!("\n=== Summary ===");
    for (name, outcome) in &results {
        let verdict = match outcome {
            Ok(()) => "PASS".to_string(),
            Err(why) => format!("FAIL — {why}"),
        };
        println!("{name}: {verdict}");
    }

    let depth_ok = results[2].1.is_ok();
    let all_ok = results.iter().all(|(_, r)| r.is_ok());

    println!();
    if all_ok {
        println!("Architecture decision: full Rust confirmed");
    } else if !depth_ok {
        println!(
            "Architecture decision: hybrid fallback needed — MuJoCo stays \
             Python, ROS 2 stays Rust"
        );
        if let Err(why) = &results[2].1 {
            println!("Depth render failed specifically because: {why}");
        }
    } else {
        println!(
            "Depth rendering works, but another test failed — see the \
             summary above before deciding."
        );
    }

    if !all_ok {
        std::process::exit(1);
    }
}

/// Test 1 — a plane and one free-jointed box, stepped 100 times.
/// PASS if nothing panics.
fn test_load_and_step() -> Result<(), String> {
    const MODEL: &str = r#"
<mujoco>
  <worldbody>
    <light pos="0 0 3"/>
    <geom name="floor" type="plane" size="5 5 0.1"/>
    <body name="box" pos="0 0 1">
      <freejoint/>
      <geom type="box" size="0.1 0.1 0.1" mass="1"/>
    </body>
  </worldbody>
</mujoco>
"#;

    let model = MjModel::from_xml_string(MODEL).map_err(|e| format!("model load failed: {e:?}"))?;
    let mut data = MjData::new(&model);

    for _ in 0..100 {
        data.step();
    }

    let box_info = data
        .body("box")
        .ok_or_else(|| "body 'box' not found".to_string())?;
    let z = box_info.view(&data).xpos[2];
    println!(
        "Test 1: stepped 100x, t={:.3}s, box z={:.4}m",
        data.time(),
        z
    );

    if !z.is_finite() {
        return Err(format!("box z is not finite ({z})"));
    }
    Ok(())
}

/// Test 2 — a mocap body commanded to a new position every step.
/// PASS if the body's world xpos tracks the command within 1e-3.
fn test_mocap_body() -> Result<(), String> {
    const MODEL: &str = r#"
<mujoco>
  <worldbody>
    <light pos="0 0 3"/>
    <geom name="floor" type="plane" size="5 5 0.1"/>
    <body name="drone" mocap="true" pos="0 0 1">
      <geom type="sphere" size="0.05" rgba="1 0 0 1" contype="0" conaffinity="0"/>
    </body>
  </worldbody>
</mujoco>
"#;

    let model = MjModel::from_xml_string(MODEL).map_err(|e| format!("model load failed: {e:?}"))?;
    let mut data = MjData::new(&model);

    let body = model
        .body("drone")
        .ok_or_else(|| "body 'drone' not found in model".to_string())?;
    let mocap_id = body.view(&model).mocapid[0];
    if mocap_id < 0 {
        return Err("body 'drone' is not a mocap body (mocapid < 0)".to_string());
    }
    let mocap_id = mocap_id as usize;

    let drone_info = data
        .body("drone")
        .ok_or_else(|| "body 'drone' not found in data".to_string())?;

    let mut worst_err = 0.0f64;

    // Walk the body +0.01 m in x per step and check it lands where commanded.
    for i in 0..100 {
        let commanded = [0.01 * (i + 1) as f64, 0.0, 1.0];
        data.mocap_pos_mut()[mocap_id] = commanded;
        data.step();

        let actual = drone_info.view(&data).xpos;

        let err = (0..3)
            .map(|k| (actual[k] - commanded[k]).abs())
            .fold(0.0f64, f64::max);
        worst_err = worst_err.max(err);

        if i == 0 || i == 99 {
            println!(
                "Test 2: step {i:>3} commanded [{:.3}, {:.3}, {:.3}] -> \
                 actual [{:.3}, {:.3}, {:.3}] (err {err:.2e})",
                commanded[0], commanded[1], commanded[2], actual[0], actual[1], actual[2]
            );
        }
    }

    println!("Test 2: worst tracking error over 100 steps = {worst_err:.3e} m");

    if worst_err > 1e-3 {
        return Err(format!("mocap tracking error {worst_err:.3e} exceeds 1e-3"));
    }
    Ok(())
}

/// Test 3 — camera 2 m above a plane, looking straight down.
/// PASS if the centre pixel's depth is within 5% of 2.0 m.
///
/// A flat plane normal to the view axis reads a uniform 2.0 in z-depth, which
/// is indistinguishable from a constant-fill bug. So the scene also holds a
/// 0.5 m tall block parked *off* the optical axis: the centre pixel still sees
/// bare floor at 2.0 m, while the block's top must show up as a 1.5 m minimum.
/// That proves the buffer tracks geometry without disturbing the centre check.
///
/// Only one renderer is ever built — winit allows a single event loop per
/// process, so a second `MjRenderer::build` in the same run fails with
/// `EventLoopError(RecreationAttempt)`.
fn test_depth_render() -> Result<(), String> {
    // A MuJoCo camera with identity orientation looks along its own -Z, so a
    // camera at z=2 with no rotation points straight down at the plane.
    const MODEL: &str = r#"
<mujoco>
  <visual>
    <global offwidth="640" offheight="480"/>
  </visual>
  <worldbody>
    <light pos="0 0 3"/>
    <geom name="floor" type="plane" size="5 5 0.1" rgba="0.6 0.6 0.6 1"/>
    <geom name="block" type="box" size="0.2 0.2 0.25" pos="0.7 0 0.25" rgba="1 0 0 1"/>
    <camera name="down_cam" pos="0 0 2" fovy="60"/>
  </worldbody>
</mujoco>
"#;

    let model = MjModel::from_xml_string(MODEL).map_err(|e| format!("model load failed: {e:?}"))?;
    let mut data = MjData::new(&model);

    let cam_id = model
        .camera("down_cam")
        .ok_or_else(|| "camera 'down_cam' not found".to_string())?
        .id;

    let mut renderer = MjRenderer::builder()
        .width(DEPTH_W as u32)
        .height(DEPTH_H as u32)
        .rgb(false)
        .depth(true)
        .camera(MjvCamera::new_fixed(cam_id))
        .build(&model)
        .map_err(|e| format!("renderer init failed: {e:?}"))?;

    data.forward();
    renderer
        .sync_data(&mut data)
        .map_err(|e| format!("sync_data failed: {e:?}"))?;
    renderer
        .render()
        .map_err(|e| format!("render failed: {e:?}"))?;

    let depth = renderer
        .depth_flat()
        .ok_or_else(|| "depth buffer is None (depth rendering disabled?)".to_string())?;

    if depth.len() != DEPTH_W * DEPTH_H {
        return Err(format!(
            "depth buffer is {} values, expected {}",
            depth.len(),
            DEPTH_W * DEPTH_H
        ));
    }

    let centre = depth[(DEPTH_H / 2) * DEPTH_W + DEPTH_W / 2];
    let finite: Vec<f32> = depth.iter().copied().filter(|v| v.is_finite()).collect();
    let min = finite.iter().copied().fold(f32::INFINITY, f32::min);
    let max = finite.iter().copied().fold(f32::NEG_INFINITY, f32::max);

    println!(
        "Test 3: {DEPTH_W}x{DEPTH_H} depth buffer, centre = {centre:.4} m \
         (expected ~2.0), range = [{min:.4}, {max:.4}]"
    );

    let expected = 2.0f32;
    let rel_err = (centre - expected).abs() / expected;
    if rel_err > 0.05 {
        return Err(format!(
            "centre depth {centre:.4} m is {:.1}% off the expected 2.0 m",
            rel_err * 100.0
        ));
    }

    println!(
        "Test 3: centre depth within {:.2}% of expected",
        rel_err * 100.0
    );

    // Geometry guard: the block's top face sits 0.5 m above the floor, so the
    // nearest depth in the whole buffer must read ~1.5 m.
    println!(
        "Test 3 (geometry guard): nearest = {min:.4} m (expected ~1.5, the \
         block top), farthest = {max:.4} m (expected ~2.0, the floor)"
    );

    if (min - 1.5).abs() / 1.5 > 0.05 {
        return Err(format!(
            "nearest depth {min:.4} m is not within 5% of the expected 1.5 m \
             — the block is not being resolved"
        ));
    }
    if (max - min).abs() < 0.1 {
        return Err(
            "depth buffer does not vary with geometry — looks like a constant fill".to_string(),
        );
    }

    println!("Test 3 (geometry guard): depth varies correctly with geometry");
    Ok(())
}
