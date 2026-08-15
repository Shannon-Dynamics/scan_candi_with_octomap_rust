# Scanning tutorial

Running the scan end to end, and understanding what comes out of it.

The first half needs nothing but a clone and a Rust toolchain. ROS 2 and custom
assets come afterwards, and only if you want them.

Every command here is one this repository actually accepts. Where something
does not exist, it says so rather than inventing a flag.

If your interest is the mapping library rather than this application, read
[`octo_map_rust`'s tutorial](https://github.com/Shannon-Dynamics/octo_map_rust/blob/main/docs/TUTORIAL.md)
first — this document assumes you know what an occupancy map is.

| Part | |
|---|---|
| 1 | [Run the Quick Demo](#1-run-the-quick-demo) |
| 2 | [Inspect the point cloud](#2-inspect-the-point-cloud) |
| 3 | [Read the occupancy map](#3-read-the-occupancy-map) |
| 4 | [Understand the comparison](#4-understand-the-comparison) |
| 5 | [Change the mapping parameters](#5-change-the-mapping-parameters) |
| 6 | [Export the output](#6-export-the-output) |
| 7 | [Scan your own geometry](#7-scan-your-own-geometry) |
| 8 | [The ROS 2 path](#8-the-ros-2-path) |

---

## 1. Run the Quick Demo

Two repositories, side by side — the mapping library is a path dependency until
it is published:

```bash
git clone https://github.com/Shannon-Dynamics/octo_map_rust
git clone https://github.com/Shannon-Dynamics/scan_candi_with_octomap_rust
cd scan_candi_with_octomap_rust
```

MuJoCo needs an absolute download directory at build time and its library on
the loader path at run time:

```powershell
# Windows
$env:MUJOCO_DOWNLOAD_DIR = "$PWD\.mujoco"
$env:Path = "$PWD\.mujoco\mujoco-3.9.0\bin;$env:Path"
```

```bash
# Linux / WSL
export MUJOCO_DOWNLOAD_DIR="$PWD/.mujoco"
export LD_LIBRARY_PATH="$MUJOCO_DOWNLOAD_DIR/mujoco-3.9.0/lib:${LD_LIBRARY_PATH:-}"
```

Then:

```bash
cargo run --release --config 'profile.release.lto=false' -p candi-sim --bin live_scan
```

The first build downloads MuJoCo (~19.5 MB) and compiles the Rerun SDK, which
takes a while. The `lto=false` override matters: with LTO on, one of Rerun's
dependencies takes impractically long to compile
([`runbooks/troubleshooting.md`](runbooks/troubleshooting.md)).

There is no asset step. The scene is
[`assets/demo/demo_scene.xml`](../assets/demo/README.md), which is committed and
built entirely from MuJoCo primitives: four square terraces, four staircases and
a domed summit, 15 × 15 × 6 m.

A healthy run prints the scene it loaded, the derived orbit, a progress line per
ring, and a summary. On this hardware the carving run takes about 30 seconds.

## 2. Inspect the point cloud

Before looking at the map, look at what fed it. The summary line to read is:

```text
cloud: 2909956 points, 1228568 dropped as ground (42.2%)
```

Two things are worth noticing.

**Most of a scan is floor.** The depth camera sees the ground plane out to
`max_range` in every direction. Without the `GROUND_Z` filter the map is mostly
a wide disc with the structure sunk in the middle of it — the reason for
[ADR-0007](decisions/0007-ground-plane-filter.md).

**The point count is a design choice, not a given.** `SUBSAMPLE = 4` keeps every
fourth pixel on each axis, so a 640×480 frame contributes about 10,000 points
rather than 307,200. Raising it is the cheapest way to shorten a scan.

To watch the cloud rather than the map, open the recording (part 6) and look at
the drone path and the voxels appearing along it, ring by ring.

## 3. Read the occupancy map

The scan writes two orthographic projections that need no viewer:

| File | View |
|---|---|
| `out/map_top.png` | From above |
| `out/map_side.png` | From the side |

What to look for, and what each thing tells you:

- **Four concentric square bands, blue through green to orange.** Colour is
  height, not confidence. The bands are the terraces, and their edges being
  sharp means the resolution is fine enough to resolve a 0.5 m step.
- **Four notches in the outline, one per side.** The staircases. They are the
  smallest features in the scene, so they are the test of whether the
  resolution is fine enough to see anything.
- **A red patch in the middle.** The domed summit.
- **A small unobserved spot at the very centre.** No waypoint looks straight
  down at the apex, so no ray ever reaches it. An occupancy map reports that as
  *unknown* rather than guessing, and this is the clearest illustration in the
  demo of why the library keeps free and unknown apart.

Reference images: [`../media/demo/README.md`](../media/demo/README.md).

If the shape is a wide flat disc, the ground filter is off. If it bows outward
like a bowl, depth is being treated as range rather than z-depth — there is a
regression test, `a_flat_wall_stays_flat`, that catches exactly that.

## 4. Understand the comparison

The summary ends with two maps side by side, built from the same points in the
same run:

```text
                                 octomap-core      hash grid
occupied leaves / entries               47244          76920
occupied voxels at 0.1 m                47244          76920
nodes / entries held                  1233989          76920
insertion, per frame                  88.3 ms         0.7 ms

free-space carving: octomap-core on, hash grid off
```

`octomap-core` is the library this repository demonstrates. The hash grid in
`crates/candi-octomap-node` is an independently written implementation kept as
a check on it ([ADR-0009](decisions/0009-dual-map-comparison.md)).

They differ here because only the octree is carving free space. A ray passing
through a voxel erases what one viewpoint saw as occupied when others prove the
space empty — so carving *reduces* the occupied count, and the 1.2 million
nodes it holds are mostly free space.

For a like-for-like run:

```bash
cargo run --release --config 'profile.release.lto=false' -p candi-sim --bin live_scan -- --octree-no-carve
```

```text
                                 octomap-core      hash grid
occupied leaves / entries               60288          76920
occupied voxels at 0.1 m                76920          76920
```

**76,920 on both sides, exactly.** Two implementations, different data
structures, same input, same answer — that is mutual validation rather than a
resemblance, and it is the measurement most worth reading in this project.

The leaf/voxel gap on the octree side is **pruning**: 60,288 leaves stand for
76,920 base voxels, because one merged node can represent eight voxels or eight
thousand. A hash grid cannot tell those two numbers apart, which is why both are
reported.

That run also finishes in a fraction of the time, because tracing every ray is
what carving does. The per-frame line in the table is there for the same reason
as the rest of it: so a later run can be checked against an earlier one. It is
not a property of either implementation.

## 5. Change the mapping parameters

The parameters are compile-time constants at the top of
[`../crates/candi-sim/src/bin/live_scan.rs`](../crates/candi-sim/src/bin/live_scan.rs).
There is no configuration file: changing one means editing the file and
rebuilding.

| Constant | Default | What it controls |
|---|---|---|
| `RESOLUTION` | `0.1` | Voxel edge length, metres. Halving it is roughly an 8× change in map size |
| `MAX_RANGE` | `20.0` | Metres. Points beyond this are dropped, and rays stop there — [ADR-0003](decisions/0003-max-range-20m.md) |
| `SUBSAMPLE` | `4` | Take every 4th pixel on each axis |
| `GROUND_Z` | `0.15` | Drop returns below this height — [ADR-0007](decisions/0007-ground-plane-filter.md) |
| `CARVE_FREE` | `false` | Free-space carving in the **hash grid** |
| `RINGS`, `POINTS_PER_RING` | `4`, `72` | The orbit: 288 waypoints |
| `RADIUS_FACTOR` | `1.5` | Orbit radius as a multiple of the structure's bounding sphere |
| `LOG_EVERY` | `4` | How often a map snapshot goes into the recording |

Which to change first, if the scan is not what you want:

- Map too coarse to see detail → lower `RESOLUTION`, and expect a slower run
  and a larger recording.
- Structure missing at the top → raise `MAX_RANGE`. The upper terraces of a
  stepped shape are further from the orbit than the base is.
- Too slow → raise `SUBSAMPLE` before touching anything else.
- The base of the structure is being eaten → lower `GROUND_Z`, at the cost of
  more floor in the map.

The octree's carving is a flag rather than a constant, because it is the one
you will want to toggle between runs: `--octree-no-carve`.

## 6. Export the output

| Output | Written by | Format |
|---|---|---|
| `out/candi_scan.rrd` | `live_scan` | Rerun recording, 288 frames |
| `out/map_top.png`, `out/map_side.png` | `live_scan` | PNG, 700×700 |
| `out/candi_ros2.rrd` | `candi_mapper` | Rerun recording |
| `octomap_binary` topic | `candi_mapper` | `octomap_msgs/Octomap` |
| `out/measure_*.log` | `ros2/measure.sh` | The comparison table as text |

To replay the recording:

```bash
rerun out/candi_scan.rrd
```

The viewer version has to match the SDK exactly (0.35.0). Entity paths:
`octree/occupied`, `hashgrid/occupied`, `drone/position`, `drone/path`, and
`candi/mesh` when `--mesh` is passed.

There is **no `.bt` file export** in either binary. The ROS 2 mapper builds the
same payload a `.bt` file wraps and publishes it as a message; writing one to
disk is a few lines with `octomap_core::io` if you need it.

`out/` is gitignored. The copies that belong in the repository live in
[`../media/`](../media/README.md).

## 7. Scan your own geometry

The demo scene is deliberately synthetic. To map something else, point the same
pipeline at another MJCF:

```bash
cargo run --release -p candi-sim --bin live_scan -- --scene path/to/scene.xml
```

The scene has to define a body named `candi` (the structure), a mocap body
named `drone`, and a camera named `drone_cam` on that body. Everything else —
orbit radius, ring heights, waypoint count — is derived from the structure's
bounding box, so a different shape produces a different flight path
automatically.

To scan a mesh rather than primitives, convert it first with headless Blender:

```bash
blender --background --python scripts/convert_glb.py
```

Defaults assume `assets/candi.glb`. Overrides go after Blender's own `--`
separator: `--glb`, `--obj-dir`, `--dae`, `--max-tris`, `--target-height`. The
procedure, the numbers to expect and the failure modes are in
[`runbooks/asset-conversion.md`](runbooks/asset-conversion.md).

Then check the scene before scanning it:

```bash
cargo run -p candi-sim --example scene_shot
```

It prints the structure's bounding box and the orbit radius derived from it. If
the dimensions look transposed — a tall narrow footprint where you expect a wide
one — the mesh was exported Y-up and is lying on its side.

## 8. The ROS 2 path

The same pipeline, split across two nodes with the middleware in between. This
is the path that publishes a standard `octomap_msgs/Octomap`, so RViz and any
existing OctoMap consumer can read the map.

Prerequisites: WSL Ubuntu 24.04 or Linux with ROS 2 Jazzy, and the same
side-by-side checkout ([`04-running.md`](04-running.md)).

```bash
source /opt/ros/jazzy/setup.bash

# A short run first — 24 waypoints on a debug build, to check the pipeline.
./ros2/smoke_run.sh

# The real thing.
./ros2/run_demo.sh                 # the octree carves free space
./ros2/run_demo.sh --no-carve      # endpoints only, like-for-like
```

`run_demo.sh` starts the mapper first on purpose: the publisher waits for a
subscriber before it flies, and a best-effort cloud topic with no subscriber
would drop the opening frames silently.

By hand, in two terminals:

```bash
./ros2/target/release/candi_mapper --out out/candi_ros2.rrd [--mesh] [--octree-no-carve]
./ros2/target/release/candi_publisher [--waypoints N] [--no-wait]
```

To take the measurement:

```bash
./ros2/measure.sh 288             # endpoints only
./ros2/measure.sh 288 --carve     # the octree also traces free space
```

Both write `out/measure_<mode>_<waypoints>.log`. The recorded numbers, from the
mesh-based scene, are in [`05-results.md`](05-results.md).

### Seeing it in RViz

With the mapper running, add an **Octomap** display on the `octomap_binary`
topic. Its QoS is `transient_local`, so an RViz started afterwards still
receives the latest map. Nothing in RViz needs to know the map was built in
Rust — that is the point of publishing the standard message.

---

## If something goes wrong

[`runbooks/troubleshooting.md`](runbooks/troubleshooting.md) first. The failures
in this project mostly do not name their cause: a full host drive surfaces as
impossible compile errors inside dependencies, and a depth camera that can see
the drone's own body surfaces as a map that follows the drone around.
