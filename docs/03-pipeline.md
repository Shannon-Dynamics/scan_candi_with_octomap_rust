# 3. The pipeline, step by step

The order: asset → scene → flight path → depth → point cloud → transport →
occupancy map → visualization.

---

## 3.0 The scene

The default scene is [`assets/demo/demo_scene.xml`](../assets/demo/README.md):
a stepped pyramid of MuJoCo primitives, 15 × 15 × 6 m, committed and covered by
this repository's licence. It needs no conversion and no download, which is what
lets a fresh clone run the scan.

Everything below describes the alternative — scanning a real mesh, which the
pipeline supports through `--scene` and which needs the offline conversion step
in 3.1. The two paths differ only in where the geometry comes from.

---

## 3.1 The asset (offline, only for a mesh-based scene)

[`scripts/convert_glb.py`](../scripts/convert_glb.py) runs inside headless
Blender: import `candi.glb`, decimate **2,135,354 → 199,999** triangles, scale
uniformly to 6 m tall, recentre on the origin, drop to z=0, then export OBJ (for
MuJoCo) and GLB (for the Rerun overlay).

| Stage | Value |
|---|---|
| Source triangles | 2,135,354 |
| Decimate ratio | 0.0937 |
| Resulting triangles | **199,999** (just under the 200k budget) |
| Scale factor | 7.864386 |
| Recentring | (0.003, −0.011, +3.030) → sits at z=0, centred in XY |
| Final bbox | **14.946 × 14.946 × 6.000 m** |
| Bounding sphere | r = 10.986 m |

The OBJ export forces `forward_axis="Y", up_axis="Z"`. Blender's exporter
defaults to Y-up, which **cancels** the Y-up → Z-up conversion the glTF importer
already did, and hands MuJoCo a temple lying on its side. This bug happened, and
the fix was verified by measuring the exported OBJ vertices directly.

Why 6 m tall rather than Borobudur's real 35 m: see
[ADR-0002](decisions/0002-mesh-scale-6m.md).

This step only needs repeating if the source mesh changes. The procedure is in
[`runbooks/asset-conversion.md`](runbooks/asset-conversion.md).

---

## 3.2 The MuJoCo scene

[`scene/candi_scene.xml`](../scene/candi_scene.xml) loads the temple mesh as a
static body (`contype=0 conaffinity=0` — no collision needed), the Skydio X2
drone as a **mocap body**, and a `drone_cam` camera attached to it. The
menagerie model's freejoint, collision geoms, actuators and sensors are removed:
a mocap body must have no joints, and the rotors are never run.

Two details that decide whether the result is correct:

- **`<statistic extent="17">` is pinned explicitly.** `MjRenderer` caches
  `near = znear × extent` and `far = zfar × extent` at `build()`. Letting MuJoCo
  guess the extent means the depth scale changes silently whenever the scene
  geometry changes.
- **The drone's geoms are in group 2, and the depth camera hides that group.**
  Without this the drone's own rotors land in the depth buffer at around
  **0.14 m** and inject a shell of occupied voxels that travels with the drone —
  permanently corrupting the map. After the fix, the nearest distance rose from
  0.14 m to 8.00 m, exactly the distance to the near face of the temple. The
  viewer still shows the drone because it uses the default `MjvOption`.

The `drone_cam` camera uses `fovy=60` and `xyaxes="0 -1 0  0 0 1"`, so its view
direction is **local +X**. That convention is what the orbit planner uses: a
waypoint quaternion only has to point +X at the temple's centre.

---

## 3.3 The flight path

`OrbitPlan::generate()` in [`crates/candi-sim/src/orbit.rs`](../crates/candi-sim/src/orbit.rs)
produces **4 rings × 72 points = 288 waypoints** at a radius of **16.48 m**
(1.5 × the bounding sphere), with heights **derived from the temple's bbox** —
not constants, so swapping the mesh changes the flight path automatically.

| Quantity | Value |
|---|---|
| Rings | 4 |
| Points per ring | 72 (5° apart) |
| Radius | 16.48 m = 1.5 × bounding sphere |
| Heights | [1.57, 3.15, 4.72, 6.30] m |

Ring directions alternate so the drone does not have to fly all the way back
when it changes ring. The top ring deliberately clears the summit (105% of the
height) so the drone can look **down** at the main stupa — without it the top is
only ever seen from the side.

---

## 3.4 Depth → point cloud

[`crates/candi-sim/src/depth_to_cloud.rs`](../crates/candi-sim/src/depth_to_cloud.rs)
relies on three properties of MuJoCo's depth buffer, all confirmed by the
smoke-test example:

1. **The values are already in metres.** `MjRenderer` linearizes inside
   `render()`, so `depth_flat()` holds metres directly — no manual
   linearization.
2. **Row 0 is the top row.** The vertical flip is already done inside the crate.
3. **The contents are z-depth, not range.** Pinhole unprojection has to treat
   the value as camera Z (`X = (u−cx)·Z/fx`). Treated as range, the cloud bows
   like a bowl. There is a regression guard, `a_flat_wall_stays_flat`, that
   fails on exactly that mistake.

Another consequence of that linearization: **background pixels read ≈ `far`, not
zero**. The original plan said "discard values near zero (MuJoCo background)" —
which does not apply to this crate. The correct filter discards values
`>= max_range`.

Points with `z < 0.15 m` are discarded as floor — see
[ADR-0007](decisions/0007-ground-plane-filter.md) for the reasoning and the
numbers.

---

## 3.5 ROS 2 transport

| Topic | Type | Direction | QoS |
|---|---|---|---|
| `cloud` | `sensor_msgs/PointCloud2` | publisher → mapper | best-effort (sensor data) |
| `/tf` | `tf2_msgs/TFMessage` | publisher → mapper | reliable, keep_last(100) |
| `octomap_binary` | `octomap_msgs/Octomap` | mapper → anyone | reliable, transient_local |

The PointCloud2 layout is assembled by
[`crates/candi-sim/src/ros_bridge.rs`](../crates/candi-sim/src/ros_bridge.rs):
x/y/z fields as FLOAT32, `point_step` 12, `frame_id` = `map`, `is_dense` = true.
`Transform` handles the MuJoCo quaternion order `[w,x,y,z]` → ROS `[x,y,z,w]`,
and `RateLimiter` caps the rate at 10 Hz.

Reading it back happens in
[`crates/candi-octomap-node/src/cloud_parser.rs`](../crates/candi-octomap-node/src/cloud_parser.rs):
offsets are read from `fields` rather than assumed from the layout; FLOAT32 and
FLOAT64 are both supported; non-finite points are discarded; a truncated buffer
is reported as an error rather than panicking.

**The publisher waits for a subscriber before it starts flying** (`--no-wait`
skips that). Publishing into a graph with no subscriber loses the entire scan,
and a best-effort publisher gives no sign that it happened.

---

## 3.6 Occupancy mapping

Two implementations run side by side — in `live_scan` and in `candi_mapper`
alike — from the same points:

| | |
|---|---|
| `octomap-core` | The octree — a Rust port of OctoMap C++ 1.10.0 whose `.bt` output is byte-identical to the C++ reference's |
| `candi-octomap-node` | The hash grid written for this project while the octree was not yet available |

Both are configured to the same sensor model — `prob_hit` 0.7, `prob_miss` 0.4,
clamps `[−2.0, 3.5]`, occupied threshold at `p > 0.5`, discretized insertion —
so their answers are directly comparable rather than merely similar.

The hash grid stores integer grid coordinates in a hash map rather than an
octree. At this scene's scale (a ~40 m box at 0.1 m where only the surface is
ever touched) a hash map is simpler to get right and quicker to query; the
octree's advantage is memory compaction over much larger volumes — which is what
[`05-results.md`](05-results.md) ended up measuring.

Free-space carving is implemented in both (3D DDA ray casting). It is **on by
default for the octree** and **off for the hash grid**, in both paths: a hash
grid stores every empty voxel it crosses individually, while an octree prunes a
uniform region into one node. `--octree-no-carve` turns it off for a
like-for-like comparison. The reasoning, and the measurement behind it, is in
[ADR-0009](decisions/0009-dual-map-comparison.md).

---

## 3.7 Visualization

An `.rrd` recording contains these entities:

| Entity | Contents |
|---|---|
| `octree/occupied` | The octree's occupied leaves; the radius follows node size, because one pruned node stands for a cube larger than a single voxel |
| `hashgrid/occupied` | The comparison map, same palette, fixed 0.1 m radius |
| `drone/position` | A bright yellow point `[255,255,80]` |
| `drone/path` | The flight trace as `LineStrips3D` |
| `candi/mesh` | *Optional* (`--mesh`): the source mesh as a static `Asset3D` |

The axes are set to `RIGHT_HAND_Z_UP` because Rerun defaults to Y-up while this
scene is Z-up.

Voxel colour uses a height gradient **blue → cyan → green → yellow → red**
([`palette.rs`](../crates/candi-octomap-node/src/palette.rs), 7 unit tests) over
a **fixed 0–6 m** range. A gradient that adapted to the map's bounds would make
playback look as though the structure were changing when it is not.

The mesh overlay is deliberately **off** by default: the mesh file is 27 MB
while a recording without it is 23 MB. The formats are also not
interchangeable — an OBJ is raw vertices in whatever frame it was written (and
`candi.obj` is exactly the file the MJCF loads, so it necessarily aligns with
the map), while glTF fixes +Y as up and Blender's exporter rotates on the way
out, so that +90° rotation about X has to be undone again on the Rerun side. The
code only undoes it for paths ending in `.glb`/`.gltf`.

Besides the `.rrd`, `live_scan` also writes two orthographic PNG projections
([`media/img/map_top.png`](../media/img/map_top.png),
[`media/img/map_side.png`](../media/img/map_side.png)) so the reconstruction can
be judged without opening a viewer at all. That is what answers the central
question: does the shape read as the temple.
