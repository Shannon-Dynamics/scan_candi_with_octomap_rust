# The candi scan over ROS 2

The same flight, depth camera and projection as `live_scan`, but the points
leave the process as ROS 2 topics and the mapping happens on the other side of
the middleware.

Both paths use the `octomap-core` library; this one adds the middleware, and
with it the standard `octomap_msgs/Octomap` output that existing OctoMap
tooling can read. For the shortest route to seeing the library work, use the
Quick Demo in the [README](../README.md#quick-demo) instead — it needs no ROS 2.

```text
candi_publisher                              candi_mapper
  MuJoCo orbit                                 octomap-core   (octree)
  depth 640x480          /cloud  ──────────►   OccupancyMap   (hash grid)
  depth_to_cloud         /tf     ──────────►         │
                                          ┌──────────┴──────────┐
                                   /octomap_binary        candi_ros2.rrd
                                   (RViz, ROS)            (Rerun)
```

## Running it

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
./run_demo.sh                # octree carves free space
./run_demo.sh --no-carve     # endpoints only, like-for-like with the hash grid
```

That builds both nodes, starts the mapper, flies the orbit, and writes
`out/candi_ros2.rrd`. The mapper prints the comparison table on exit.

Manually, in two terminals:

```bash
./target/release/candi_mapper --out ../out/candi_ros2.rrd
./target/release/candi_publisher
```

The publisher waits for a subscriber before it starts flying, so the mapper has
to be up first. `--no-wait` skips that if you are pointing something else at the
topic.

`--mesh` on the mapper overlays the source geometry in the recording, as a static
`Asset3D` at `candi/mesh`, so the voxels can be read against the thing they were
reconstructed from. Off by default: it adds 27 MB to every recording. Bare
`--mesh` takes `assets/candi_obj/candi.obj` — the file the MJCF itself loads, so
it lands in the map frame with nothing to correct. `--mesh <path>` takes another
file, and a `.glb`/`.gltf` gets a +90° rotation about X on the way in, because
glTF fixes +Y as up and Rerun does not undo that.

Both numbers below came from `./measure.sh 288` and `./measure.sh 288 --carve`,
which run the release binaries and write `out/measure_*.log`.

## Why this is a separate workspace

Everything here needs ROS 2 sourced to build — `r2r` generates its message
bindings from `AMENT_PREFIX_PATH` at build time. Folding these crates into the
root workspace would make `cargo build` at the top level fail on any machine
without ROS, including the Windows side where the scan was developed. The root
`Cargo.toml` is untouched and `live_scan` still runs exactly as before.

Two crates rather than two binaries in one, because the mapper has no reason to
compile MuJoCo and the publisher has no reason to compile Rerun.

## Two maps, on purpose

The mapper builds **both** occupancy maps from the same points:

| | |
|---|---|
| `octomap-core` | The octree — a Rust port of OctoMap, from the [`octo_map_rust`](https://github.com/Shannon-Dynamics/octo_map_rust) repository, expected as a sibling checkout at `../../../octo_map_rust` |
| `candi-octomap-node` | The hash grid written for this project when the octree was unavailable |

They were built to the same sensor model — `prob_hit` 0.7, `prob_miss` 0.4, and
clamps that agree with OctoMap's defaults to two decimals — so their answers are
directly comparable rather than merely similar.

### The measurement this exists to take

`live_scan` disables free-space carving, and says why:

> Turning it on would trace ~200 voxels per ray and store every empty voxel it
> crosses — for this scene that is a ~40 m box at 0.1 m, i.e. millions of
> entries and hundreds of megabytes, to remove nothing.

That is true of a hash grid, which stores every free voxel individually. It is
not a property of occupancy mapping. An octree prunes a uniform region into a
single node, and mapped free space is about as uniform as a volume gets — so
the cost that made carving unaffordable is the data structure's, not the
method's.

So the octree carves by default here and the summary reports what it actually
cost. `--no-carve` turns it off for the like-for-like run.

The summary separates two numbers that a hash grid cannot tell apart:

- **occupied leaves** — nodes the octree holds
- **occupied voxels at 0.1 m** — what those leaves stand for, since one pruned
  node can represent eight or eight thousand base voxels

The second is the one to compare against the hash grid's count. The `live_scan`
baseline is 56,063 occupied voxels at 0.1 m.

### What it measured — 288 waypoints, release build

Endpoints only, so both maps do the same work:

| | octree | hash grid |
|---|---:|---:|
| occupied leaves / entries | 53,937 | 56,065 |
| **occupied voxels @ 0.1 m** | **56,065** | **56,065** |
| nodes / entries held | 72,947 | 56,065 |
| `.bt` payload | 38,020 B | — |
| insertion per frame | 0.7 ms | 0.4 ms |

Two implementations written separately, given the same points, agreeing exactly.
That is mutual validation rather than a resemblance.

With the octree carving free space:

| | octree | hash grid |
|---|---:|---:|
| occupied leaves / entries | 38,152 | 56,065 |
| occupied voxels @ 0.1 m | 38,152 | 56,065 |
| nodes / entries held | 1,492,583 | 56,065 |
| `.bt` payload | 479,666 B | — |
| insertion per frame | 80.4 ms | 0.5 ms |

So: 1.49 M nodes, and **480 KB serialized** — not the hundreds of megabytes that
made carving unaffordable for a hash grid. The cost belonged to the data
structure. What carving does cost is time, 115× per frame, which still fits the
100 ms budget at 10 Hz but no longer with room to spare. These timings are from a
build with LTO off (`--config 'profile.release.lto=false'`, because `arrow-ipc`
never finished linking with it on), so they are an upper bound.

The drop from 56,065 to 38,152 occupied voxels is carving working, not structure
lost: a ray passing through erases a voxel that one viewpoint saw as occupied
while others prove it empty.

## Frames

The cloud is published in the **map** frame, not the camera's. `depth_to_cloud`
already projects through the camera pose, so the points are world-frame by the
time they exist; converting them back to the camera frame just so the mapper
could convert them out again would lose precision for nothing.

What `/tf` carries is `map → drone_cam`, and the mapper reads one thing from
it: the translation, which is where the rays started. Without it every ray
would be traced from the origin and the free space would be wrong.

## Interface

| Topic | Type | Direction |
|---|---|---|
| `cloud` | `sensor_msgs/PointCloud2` | publisher → mapper, best-effort |
| `/tf` | `tf2_msgs/TFMessage` | publisher → mapper, reliable |
| `octomap_binary` | `octomap_msgs/Octomap` | mapper → anything, latched |

`octomap_binary` is the standard OctoMap message, so RViz's octomap display and
any existing consumer can read this map without knowing it was built in Rust.

## Environment

Both are set by `run_demo.sh`, but worth knowing if you run the binaries
directly:

- `LD_LIBRARY_PATH` must include `.../mujoco-3.9.0/lib` — the Linux equivalent
  of the `$env:Path` line in the root README.
- `CARGO_HOME`, `CARGO_TARGET_DIR` and `ROS_HOME` are pointed at `D:`. The WSL
  VHDX sits on a full `C:`, and when a sparse VHDX cannot grow, ext4 remounts
  read-only mid-build. That surfaces as impossible compile errors — unresolved
  imports inside dependencies — rather than anything mentioning disk. Three
  builds were lost to it before the cause was found.
