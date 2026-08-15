# 1. Architecture

There are **two paths** running the same pipeline, and both are maintained. Not
because one replaces the other, but because they answer different questions.

---

## 1.1 The single-process path — `live_scan`

```text
MuJoCo orbit ─► depth 640×480 ─► depth_to_cloud ─┬─► octomap-core OcTree ──┐
                                                 │                         ├─► .rrd
                                                 └─► OccupancyMap ─────────┘  + PNGs
                                                     (hash grid)
```

There is no ROS 2 in it, so it runs anywhere — Windows without a ROS
installation included — and it needs no external assets: its default scene is
[`assets/demo/demo_scene.xml`](../assets/demo/README.md), built from MuJoCo
primitives and committed. One command, one process, no middleware that can fail
mid-presentation.

**Both occupancy maps are built here**, from the same points: the
`octomap-core` octree that this repository exists to demonstrate, and this
project's hash grid, kept as an independently written implementation to check it
against. The octree carves free space by default; `--octree-no-carve` makes the
two do identical work, and they then agree exactly on the occupied-voxel count.

Code: [`crates/candi-sim/src/bin/live_scan.rs`](../crates/candi-sim/src/bin/live_scan.rs).

## 1.2 The ROS 2 path — two nodes

```text
candi_publisher                                  candi_mapper
  MuJoCo orbit                                     octomap-core   (octree)
  depth 640×480          /cloud  ──────────────►   OccupancyMap   (hash grid)
  depth_to_cloud         /tf     ──────────────►         │
                                            ┌────────────┴────────────┐
                                     /octomap_binary            candi_ros2.rrd
                                     (RViz, ROS consumers)      (Rerun)
```

The points leave the process as ROS 2 topics, and the mapping happens on the
other side of the middleware. This is what demonstrates that the system connects
to existing robotics tooling: `octomap_binary` is the standard OctoMap message,
so RViz's Octomap display and any existing consumer can read this map without
knowing it was built in Rust.

The mapper builds **two maps at once** from the same points — the octree and the
hash grid — so their answers can be compared directly rather than merely
resembling each other. The full reasoning is in
[ADR-0009](decisions/0009-dual-map-comparison.md), the result in
[`05-results.md`](05-results.md).

Code: [`ros2/candi_publisher/src/main.rs`](../ros2/candi_publisher/src/main.rs) and
[`ros2/candi_mapper/src/main.rs`](../ros2/candi_mapper/src/main.rs).

---

## 1.3 Why ROS 2 has its own Cargo workspace

Everything under `ros2/` needs ROS 2 sourced at build time: `r2r` builds its
message bindings from `AMENT_PREFIX_PATH` at compile time. If those crates were
in the root workspace, `cargo build` at the top level would fail on any machine
without ROS — including the Windows side where the simulation was developed. So
`ros2/Cargo.toml` stands alone and the root `Cargo.toml` is untouched.

Split into two crates rather than two binaries in one, because the mapper has no
reason to compile MuJoCo and the publisher has no reason to compile Rerun. On a
machine where disk is the binding constraint, that is worth an extra manifest.

See [ADR-0006](decisions/0006-separate-ros2-workspace.md).

---

## 1.4 The crate split

| Crate | Contents | External dependencies |
|---|---|---|
| `candi-sim` | MuJoCo scene, flight path, depth → point cloud, PointCloud2 byte layout, the `live_scan` demo | `mujoco-rs`, `rerun`, `png`, `octomap-core` |
| `candi-octomap-node` | Hash-grid occupancy map, PointCloud2 parser, height palette | **none** — plain Rust on `std` |
| `candi_publisher` | ROS 2 node: orbit → `/cloud` + `/tf` | `r2r`, `candi-sim` |
| `candi_mapper` | ROS 2 node: octree + hash grid + Rerun + `octomap_binary` | `r2r`, `rerun`, `octomap-core`, `octomap-ros`, `candi-octomap-node` |

`candi-octomap-node` deliberately has no dependencies at all. The practical
consequence: all the mapping, parsing and colouring logic can be tested in CI
without MuJoCo, without a GPU and without a ROS 2 installation — which is what
[`.github/workflows/ci.yml`](../.github/workflows/ci.yml) does.

The same holds for the two modules in `candi-sim` that deal with ROS:
`ros_bridge.rs` and `cloud_parser.rs` are **free of ROS types**. One assembles a
byte buffer, the other reads it; the node only pipes fields and buffers through.
That means the logic is tested without a ROS installation, and when the
transport was connected all that remained was wiring.

---

## 1.5 Frame boundaries

The point cloud is published in the **map** frame, not the camera's.
`depth_to_cloud` already projects through the camera pose, so the points are in
the world frame from birth; converting them back to the camera frame just so the
mapper could convert them out again would lose precision for nothing.

What `/tf` carries is `map → drone_cam`, and the mapper reads one thing from it:
the **translation**, which is where each ray started. Without it every ray would
be traced from the origin and the free space would be wrong.

The camera pose itself comes straight from `data.cam_xpos()` / `cam_xmat()`,
rather than being reassembled from the drone pose plus an MJCF offset — one
source of truth for the frame convention rather than two that can quietly
disagree.
