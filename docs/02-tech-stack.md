# 2. Tech stack

The whole runtime path is written in Rust. Python appears exactly once, offline,
inside Blender, to convert the source mesh.

---

## 2.1 Runtime

| Layer | Technology | Note |
|---|---|---|
| Language | **Rust** — `candi-sim` edition 2024, the ROS 2 crates edition 2021 | Two Cargo workspaces, four crates. Developed on 1.97.1 |
| Physics & sensor | **MuJoCo 3.9.0** via `mujoco-rs 5.0.0` | Offscreen depth + RGB rendering |
| Graphics backend | OpenGL via glutin / winit | Comes with `mujoco-rs`; on WSL it uses WSLg through `/dev/dxg` |
| Middleware | **ROS 2 Jazzy** via `r2r 0.9.5` | Message bindings built from `AMENT_PREFIX_PATH` at compile time |
| Occupancy mapping | `octomap-core` (octree) + `candi-octomap-node` (hash grid) | Two implementations, run side by side |
| 3D visualization | `rerun 0.35`, `sdk` feature only | Records `.rrd` |
| Image encoding | `png 0.18` | Static orthographic projections and APNG animations |
| Async runtime | `tokio 1` + `futures 0.3` | ROS 2 nodes only |
| Release profile | `lto = true`, `codegen-units = 1` | See the caveat in [`05-results.md`](05-results.md) |

## 2.2 The `mujoco-rs` features used, and why

```toml
mujoco-rs = { version = "5.0.0", features = [
    "renderer",                  # offscreen rendering — the core of the sensor
    "renderer-winit-fallback",   # required on Windows: true offscreen EGL is Linux-only
    "auto-download-mujoco",      # MuJoCo 3.9.0 downloads itself on the first build
    "viewer",                    # only for the view_scene example
] }
```

`renderer-winit-fallback` brings one constraint that shapes the design: a winit
event loop may be created **once per process**, and only on the main thread. A
second `MjRenderer::build()` in the same process fails with
`EventLoopError(RecreationAttempt)`. That is why the smoke test lives in
`examples/` rather than `tests/` — libtest runs tests on separate threads — and
why every binary creates its renderer once and reuses it.

## 2.3 External dependencies per crate

| Crate | Dependencies |
|---|---|
| `candi-sim` | `mujoco-rs 5.0.0`, `rerun 0.35` (sdk), `png 0.18`, `candi-octomap-node` |
| `candi-octomap-node` | **none** — plain Rust on `std` |
| `candi_publisher` | `candi-sim`, `mujoco-rs 5.0.0`, `r2r 0.9` |
| `candi_mapper` | `candi-octomap-node`, `octomap-core`, `octomap-ros`, `r2r 0.9`, `rerun 0.35`, `tokio 1`, `futures 0.3` |

`octomap-core` and `octomap-ros` are **path dependencies on the neighbouring
project**, not registry crates:

```toml
octomap-core = { path = "../../../octo_map_rust/crates/octomap-core" }
octomap-ros  = { path = "../../../octo_map_rust/crates/octomap-ros" }
```

That means the [`octo_map_rust`](https://github.com/Shannon-Dynamics/octo_map_rust)
checkout has to sit **beside this repository's folder** for `ros2/` to build:

```text
<parent>/octo_map_rust/
<parent>/scan_candi_with_octomap_rust/
```

When `octomap-core` is published to crates.io, these two lines become an
ordinary version requirement. The single-process path (`live_scan`) is entirely
unaffected either way — it does not touch the octree.

## 2.4 Toolchain outside Cargo

| Tool | Version | Role | Required? |
|---|---|---|---|
| Blender | 5.2.0 (portable build) | Runs `scripts/convert_glb.py` | Only if the source mesh changes |
| Rerun viewer | 0.35.0 prebuilt | Plays back `.rrd` | To look at the result |
| WSL Ubuntu | 24.04.2 LTS | Hosts ROS 2 Jazzy | ROS 2 path only |
| colcon / rosdep / vcstool | — | **Not used** | See [ADR-0005](decisions/0005-r2r-over-rclrs.md) |

Blender is installed as a **portable build** extracted into `.tools/` rather
than through an installer: the MSI fails with exit 1603 because it asks for
admin elevation that cannot be answered from a non-interactive session. The end
result is identical.

The Rerun viewer is used as a prebuilt binary at **exactly the SDK version**
(0.35.0). The `web_viewer` feature was tried, so recordings could be opened in a
browser without an install, but it pulls in the entire viewer stack
(`re_viewer`, wgpu, egui, `re_redap_client`) and compiling it ran out of memory
on a ~16 GB machine (`handle_alloc_error`).

## 2.5 Deliberately not used

| Candidate | Why not |
|---|---|
| `rclrs` | Its message crates on crates.io are 0.0.0 placeholders, so it still needs a full `ros2_rust` colcon workspace, which in turn needs administrative rights to install. [ADR-0005](decisions/0005-r2r-over-rclrs.md) |
| `bye_octomap_rs 0.1.1` | An unfinished port: `OcTree.rs`, `OcTreeBase.rs` and `OccupancyOcTreeBase.rs` are all 0 bytes. The occupancy mapping is exactly the part that is missing |
| `obj2mjcf` | The converted mesh has a single material and needs no collision decomposition — which is the only thing it would have contributed. [ADR-0008](decisions/0008-skip-obj2mjcf.md) |
| `octomap_server` (C++) | A prepared fallback that turned out to be unnecessary: the Rust mapping works and stays well inside the time budget |
| Python at runtime | A project rule. The only Python is `scripts/convert_glb.py`, offline |

## 2.6 Platforms with evidence behind them

| Platform | Single-process path | ROS 2 path |
|---|---|---|
| Windows 11 native | ✅ Demonstrated | ❌ `r2r` needs a ROS 2 installation |
| WSL Ubuntu 24.04 + Jazzy | ✅ Demonstrated | ✅ Demonstrated |

Headless MuJoCo rendering on WSL was the largest open question about this
stack, and the answer is positive: WSLg provides a GL context through `/dev/dxg`, and
a single 640×480 depth frame comes out with 307,200 values, 100% finite, spread
across 0.17–59.49 m. The failure mode to watch for is a uniform buffer — every
value at infinity, meaning no GL context. `ros2/smoke_run.sh` surfaces it
indirectly: it runs both nodes over 24 waypoints, and a map that comes back
empty is what a dead depth buffer looks like from the other end.
