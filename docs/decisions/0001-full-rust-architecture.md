# ADR-0001 — The whole runtime path is written in Rust

- **Status:** Accepted

## Context

The plan called for a full-Rust architecture: MuJoCo through `mujoco-rs`, ROS 2
through a Rust binding, occupancy mapping and visualization in Rust too. The
prepared fallback was **hybrid**: MuJoCo stays Python, only ROS 2 is Rust.

What decided it was whether `mujoco-rs` could actually (a) load and step a
model, (b) drive a mocap body precisely, and (c) render depth offscreen. Point
(c) was the doubtful one — if it failed, the whole architecture changed.

## Decision

Full Rust. Three smoke tests were run first
(`crates/candi-sim/examples/smoke_test.rs`) and all three passed, so there was
no reason to take the hybrid fallback.

## Evidence

| Test | Result | Evidence |
|---|---|---|
| Load + step | **PASS** | 100 × `data.step()`, t=0.200 s, the box falls to z=0.8058 m |
| Mocap body | **PASS** | Worst tracking error **0.0e0 m** over 100 steps (limit 1e-3) |
| Offscreen depth | **PASS** | Centre pixel **2.0000 m** against an expected 2.0 (0.00% difference) |

An anti-false-positive guard was added to the third test: a flat plane at a
uniform 2.0 m is indistinguishable from a constant-fill bug, so the scene got a
0.5 m block offset to `x=0.7`, off the optical axis. The centre pixel stayed at
2.0000 m while the nearest value read 1.5000 m — depth demonstrably follows the
geometry rather than filling a constant.

## Consequences

Four findings from this validation constrain every later phase:

1. **Depth is already metric and already flipped.** `MjRenderer` linearizes to
   metres and flips vertically inside `render()`. `depth_to_cloud()` must do
   neither.
2. **Depth is z-depth, not euclidean range.** Unprojection has to treat it as
   camera Z. Treated as range, the point cloud bows like a bowl.
3. **Background pixels read ≈ `far`, not zero.** The plan said "discard values
   near zero" — that does not apply to this crate.
4. **One renderer per process.** A second `MjRenderer::build()` fails with
   `EventLoopError(RecreationAttempt)` — a winit event-loop limit, not a MuJoCo
   one. Every binary creates its renderer once and reuses it, and the smoke test
   lives in `examples/` rather than `tests/`.

`renderer-winit-fallback` is required on Windows: true offscreen EGL is
Linux-only. The runtime also needs `mujoco.dll` on PATH.
