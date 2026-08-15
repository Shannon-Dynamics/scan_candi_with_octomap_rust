# 6. Code tour — what each file does

This document describes the contents of each source file: its public functions,
the decisions embedded in it, and the tests that guard it. For the larger
picture see [`01-architecture.md`](01-architecture.md).

**45 unit tests** across the workspace: 18 in `candi-sim`, 27 in
`candi-octomap-node`.

---

## 6.1 `crates/candi-sim` — simulation and sensor

### [`src/lib.rs`](../crates/candi-sim/src/lib.rs) — scene utilities

| Item | What it does |
|---|---|
| `workspace_root()`, `scene_path()`, `demo_scene_path()` | Find the repository root, the mesh-based scene, and the self-contained demo scene, without depending on the cwd |
| `DRONE_GEOM_GROUP: usize = 2` | The drone's geom group — one constant shared by the MJCF and the code |
| `depth_camera_options() -> MjvOption` | An `MjvOption` that **hides group 2**, used only for depth rendering |
| `body_aabb()` | A body's true bounding box, measured from mesh vertices |
| `report_scene()` | Prints a scale report: temple bbox, bounding sphere, orbit radius, drone distance |

`depth_camera_options()` is the fix for the most dangerous bug this scene has
had.
Without it, the drone's own rotors read in the depth buffer at 0.14 m and every
frame injects a shell of occupied voxels that travels with the drone. The viewer
still uses the default `MjvOption`, so a human still sees the drone.

`body_aabb()` carries two traps that have already bitten:

1. It originally used `geom_rbound` for meshes — that is a **bounding sphere
   radius**, not a box. For a 14.95 × 14.95 × 6.00 m temple it reported a 26.73 m
   cube, which would have pushed the orbit radius from 16.5 m to 34.7 m. At that
   distance every point is cut off by `max_range` and the map is empty.
2. The MuJoCo compiler rewrites mesh vertices into the inertial principal-axis
   frame, **but `geom_xmat`/`geom_xpos` already fold in `mesh_quat`/`mesh_pos`**.
   Applying `mesh_quat` again by hand rotates the mesh twice. The correct
   sequence is: raw `mesh_vert`, then transform with `geom_xmat`/`geom_xpos`
   alone.

### [`src/orbit.rs`](../crates/candi-sim/src/orbit.rs) — the flight path

| Item | What it does |
|---|---|
| `struct Waypoint` | Position plus a quaternion facing the temple centre |
| `OrbitPlan::generate()` | 4 rings × 72 points from the temple bbox, radius 1.5 × the bounding sphere |
| `look_at_quat(from, to)` | The quaternion pointing **local +X** at a target — the `drone_cam` convention |
| `mat_to_quat`, `rotate`, `cross`, `norm`, `normalize` | Vector and quaternion algebra, dependency-free |

Ring heights are **derived from the bbox**, not constants: swapping the mesh
changes the flight path automatically. Ring directions alternate so the drone
does not have to fly all the way back between rings, and the top ring clears the
summit (105% of the height) so the drone can look down at the main stupa.

**6 unit tests.**

### [`src/depth_to_cloud.rs`](../crates/candi-sim/src/depth_to_cloud.rs) — depth → points

| Item | What it does |
|---|---|
| `struct CameraPose` | The camera pose, taken straight from `cam_xpos`/`cam_xmat` |
| `struct Intrinsics` | fovy, width, height → fx, fy, cx, cy |
| `depth_to_cloud()` | Depth buffer → `Vec<[f32;3]>` in the world frame |

It treats depth values as **camera z-depth**, not range —
`X = (u−cx)·Z/fx`. The regression guard `a_flat_wall_stays_flat` fails exactly
when someone changes it to range (the cloud bows like a bowl). Points are
discarded when `>= max_range`, not when close to zero: after linearization the
MuJoCo background reads ≈ `far`, not 0.

**6 unit tests.**

### [`src/ros_bridge.rs`](../crates/candi-sim/src/ros_bridge.rs) — the ROS byte layout

| Item | What it does |
|---|---|
| `FRAME_MAP`, `FRAME_DRONE`, `PUBLISH_HZ` | Frame and rate constants |
| `struct CloudFrame` | Assembles the PointCloud2 bytes: little-endian f32 xyz, `point_step` 12 |
| `struct Transform` | Pose, plus the MuJoCo quaternion reorder `[w,x,y,z]` → ROS `[x,y,z,w]` |
| `struct RateLimiter` | Throttles to 10 Hz |

**Entirely free of ROS types** — there is no `r2r` here. The node only pipes
fields and byte buffers through. That means this logic is tested without a ROS
installation.

**6 unit tests**, including `limiter_yields_the_requested_rate`, which simulates
one second at a 1 ms cadence and checks that exactly 10 publishes happen.

### [`src/bin/live_scan.rs`](../crates/candi-sim/src/bin/live_scan.rs) — the single-process demo

The constants at the top of the file are the pipeline's contract:
`MAX_RANGE = 20.0`, `SUBSAMPLE = 4`, `RESOLUTION = 0.1`, `GROUND_Z = 0.15`,
`CARVE_FREE = false`, `RINGS = 4`, `POINTS_PER_RING = 72`,
`RADIUS_FACTOR = 1.5`.

| Function | What it does |
|---|---|
| `write_projection()` | Orthographic projection of the map to PNG — `Projection::Top` and `Side` |
| `occupied_cells()` | Occupied octree leaves as (centre, edge length) |
| `occupied_voxel_equivalent()` | Leaf count → base-voxel count, for comparison with the hash grid |
| `scene_arg()` | Parses `--scene <path>`, defaulting to the demo scene |
| `mesh_arg()` | Parses `--mesh [path]` |
| `log_mesh_overlay()` | Logs the mesh as a static `Asset3D`; undoes the +90° X rotation for `.glb`/`.gltf` |
| `main()` | Load scene → orbit → render depth → cloud → insert → log to Rerun → write PNGs |

`write_projection()` exists because an `.rrd` needs a viewer to inspect. Those
two PNGs are what answer the question — does the shape read as the structure —
without opening anything.

This binary builds **both** maps: the `octomap-core` octree and the hash grid,
from the same points, printing the comparison on exit. `occupied_cells()` is
the whole of what it takes to read a map back out of the library — walk the
leaves, keep the occupied ones, ask the tree geometry where each is and how
large. `occupied_voxel_equivalent()` converts leaf counts to base voxels, which
is the number a hash grid can be compared against.

### [`src/main.rs`](../crates/candi-sim/src/main.rs)

Orbit and point cloud only, no mapping. Used to verify that points land on the
temple surface before the map layer was added.

### `examples/`

| File | What it does |
|---|---|
| [`smoke_test.rs`](../crates/candi-sim/examples/smoke_test.rs) | mujoco-rs validation: `test_load_and_step`, `test_mocap_body`, `test_depth_render`. In `examples/` rather than `tests/` because libtest runs tests on separate threads while a winit event loop must be on the main thread |
| [`scene_shot.rs`](../crates/candi-sim/examples/scene_shot.rs) | Headless scene render plus a scale report. Automatic verification without opening a window |
| [`view_scene.rs`](../crates/candi-sim/examples/view_scene.rs) | The interactive MuJoCo viewer |
| [`fly_orbit.rs`](../crates/candi-sim/examples/fly_orbit.rs) | Flies the orbit in the viewer |
| [`record_demo.rs`](../crates/candi-sim/examples/record_demo.rs) | Records the APNG/MP4 animations — the source of `media/video/orbit.mp4` and `map_growth.mp4`. `render_top_view()`, `write_apng()`, `crop_rgb()` |

---

## 6.2 `crates/candi-octomap-node` — mapping, with no dependencies

This crate has **no dependencies at all** — plain Rust on `std`. That is why it
can be fully tested in CI without MuJoCo, a GPU, or ROS 2.

### [`src/occupancy.rs`](../crates/candi-octomap-node/src/occupancy.rs) — the log-odds map

| Item | Value / contents |
|---|---|
| `PROB_HIT` | 0.7 |
| `PROB_MISS` | 0.4 |
| `CLAMP_MIN`, `CLAMP_MAX` | −2.0 .. 3.5 |
| `OCCUPANCY_THRESHOLD` | 0.0 log-odds (= p > 0.5) |
| `struct OccupancyMap` | A hash map keyed by integer grid coordinates |
| `struct MapStats` | Voxel counts, extent, statistics for the report |
| `trace_ray()` | 3D DDA ray casting for free space |

The probabilistic semantics follow OctoMap; the storage is a hash map rather
than an octree — the reasoning is in
[ADR-0004](decisions/0004-hash-grid-occupancy-map.md).

Free-space carving is **implemented and tested, but off** in `live_scan`:
MuJoCo's depth buffer has no spurious returns to erase, so for this scene it
would trace ~200 voxels per ray to remove none. It stays because it matters on
real sensor data — and because the measurement in
[`05-results.md`](05-results.md) rests on it.

**10 unit tests**, including `repeated_misses_clear_a_voxel` and
`surface_survives_repeated_observation_with_carving`.

### [`src/cloud_parser.rs`](../crates/candi-octomap-node/src/cloud_parser.rs) — PointCloud2 decoding

| Item | What it does |
|---|---|
| `INT8` … `FLOAT64` | PointField datatype constants |
| `struct Field` | Name, offset, datatype |
| `enum ParseError` | Structured errors — a truncated buffer is reported, not a panic |
| `parse_pointcloud2()` | Reads offsets **from `fields`**, rather than assuming a layout |
| `xyz_fields()`, `encode_xyz()` | The write side, used by the publisher and the tests |

Supports FLOAT32 and FLOAT64, and discards non-finite points. **10 unit tests.**

### [`src/palette.rs`](../crates/candi-octomap-node/src/palette.rs) — the height gradient

`STOPS` holds five colours: blue → cyan → green → yellow → red.
`height_colour_f32()`, `height_colour_u8()`, `normalize(z, z_min, z_max)`.
Shared by the Rerun points and the PNG projections, so the two cannot disagree.
**7 unit tests.**

---

## 6.3 `ros2/` — the two nodes

### [`candi_publisher/src/main.rs`](../ros2/candi_publisher/src/main.rs)

Its pipeline constants are **identical** to `live_scan`'s (`MAX_RANGE = 20.0`,
`SUBSAMPLE = 4`, `GROUND_Z = 0.15`, 4 × 72 waypoints), so the two paths compare
the same thing.

| Function | What it does |
|---|---|
| `main()` | Loads the scene, flies the orbit, renders, publishes; waits for a subscriber unless `--no-wait` |
| `now(clock)` | ROS timestamps |
| `cloud_message()` | `CloudFrame` → `sensor_msgs/PointCloud2` |
| `tf_message()` | `Transform` → `tf2_msgs/TFMessage`, frame `map → drone_cam` |
| `mat_to_quat_wxyz()` | MuJoCo rotation matrix → quaternion |

This binary adds **transport and nothing else**: orbit planning, depth
projection and the byte layout all come from the already-tested `candi-sim`.

### [`candi_mapper/src/main.rs`](../ros2/candi_mapper/src/main.rs)

**This is where `octomap-core` is used.**

| Function | What it does |
|---|---|
| `main()` | The node, `/cloud` + `/tf` subscriptions, a 500 ms publish timer |
| `integrate()` | **One stream of points → two maps**: `OcTree` and `OccupancyMap` |
| `publish()` | `OcTree` → `octomap_msgs/Octomap` (binary, latched) |
| `record()` | Rerun: `octree/occupied`, `hashgrid/occupied`, `drone/position`, `drone/path` |
| `summarize()` | The comparison table printed on exit — the source of the numbers in [`05-results.md`](05-results.md) |
| `struct Timings` | Accumulated insertion time per map |
| `mesh_arg()`, `log_mesh_overlay()` | The `--mesh` overlay, as in `live_scan` |

`Z_MIN = 0.0`, `Z_MAX = 6.0` pin the colour gradient's range. A gradient that
adapted to the map's bounds would make playback look as though the structure
were changing when it is not.

`integrate()` is the heart of the experiment: both maps are given the **same**
points, in the same process, in the same run. Not two separate runs whose
numbers are placed side by side afterwards.

### Scripts

| File | What it does |
|---|---|
| [`run_demo.sh`](../ros2/run_demo.sh) | Builds both nodes, starts the mapper first, flies the orbit, writes the `.rrd` |
| [`measure.sh`](../ros2/measure.sh) | Runs the release binaries and writes `out/measure_*.log` |
| [`build_all.sh`](../ros2/build_all.sh) | Builds only, with the right environment |
| [`smoke_run.sh`](../ros2/smoke_run.sh) | A short end-to-end run over 24 waypoints, debug build — for checking the pipeline rather than producing a map |

Every script sources ROS **before** `set -u`; the other order dies with
`AMENT_TRACE_SETUP_FILES: unbound variable`.

---

## 6.4 Outside Rust

| File | What it is |
|---|---|
| [`scene/candi_scene.xml`](../scene/candi_scene.xml) | MJCF: the static temple mesh, the Skydio X2 as a mocap body, `drone_cam`, a pinned `<statistic extent="17">`, drone geoms in group 2 |
| [`scripts/convert_glb.py`](../scripts/convert_glb.py) | Headless Blender: import glTF, decimate to <200k triangles, scale, recentre, export OBJ (`forward=Y, up=Z`) + GLB. `ensure_mtl_texture()` adds the `map_Kd` line Blender's exporter omits |
