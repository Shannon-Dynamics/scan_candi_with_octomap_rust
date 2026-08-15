# Runbook — running the ROS 2 path on WSL

**When to use it:** running the pipeline over DDS, taking the octree/hash-grid
measurement, or feeding the map to RViz.

**Prerequisites:**

- WSL **Ubuntu 24.04** with **ROS 2 Jazzy** in `/opt/ros/jazzy`
- A Rust toolchain inside WSL
- **An `octo_map_rust` checkout beside this repository's folder** —
  `candi_mapper` loads it through the path
  `../../../octo_map_rust/crates/octomap-core`
- Enough free space on the Windows drive holding the VHDX (see
  [`troubleshooting.md`](troubleshooting.md) — this is not a formality)

## Steps

```bash
# 1. Source ROS BEFORE any set -u
source /opt/ros/jazzy/setup.bash

# 2. A short end-to-end run over 24 waypoints, to check the pipeline
./ros2/smoke_run.sh

# 3. The full demo — build, start the mapper, fly the orbit
./ros2/run_demo.sh                 # the octree carves free space
./ros2/run_demo.sh --no-carve      # endpoints only, like-for-like

# 4. Playback
rerun out/candi_ros2.rrd
```

Manually, in two terminals. **The mapper has to come up first** — the publisher
waits for a subscriber before it flies:

```bash
# Terminal 1
./ros2/target/release/candi_mapper --out out/candi_ros2.rrd

# Terminal 2
./ros2/target/release/candi_publisher
```

The available flags:

| Binary | Flag | What it does |
|---|---|---|
| `candi_mapper` | `--out <path>` | Where the `.rrd` recording goes |
| | `--mesh [path]` | Overlays the source mesh (+27 MB) |
| | `--octree-no-carve` | Turns carving off, for a like-for-like comparison |
| `candi_publisher` | `--waypoints N` | Fly N waypoints instead of 288 |
| | `--no-wait` | Do not wait for a subscriber |

## Taking the measurement

```bash
./ros2/measure.sh 288             # endpoints only
./ros2/measure.sh 288 --carve     # the octree also traces free space
```

Writes `out/measure_endpoints_288.log` and `out/measure_carved_288.log`, and
prints the comparison table. The correct numbers are in
[`../05-results.md`](../05-results.md).

## Environment

The scripts set these; they are required if you run the binaries directly:

```bash
export MUJOCO_DOWNLOAD_DIR="$PWD/.mujoco"
export LD_LIBRARY_PATH="$MUJOCO_DOWNLOAD_DIR/mujoco-3.9.0/lib:${LD_LIBRARY_PATH:-}"
```

If the WSL VHDX is on a full drive, also point cargo's writes elsewhere before
building — the reasoning is in [`troubleshooting.md`](troubleshooting.md):

```bash
export CARGO_HOME=/mnt/<drive>/.cargo-wsl
export CARGO_TARGET_DIR=/mnt/<drive>/.cargo-target
export ROS_HOME=/mnt/<drive>/.ros
```

## If it fails

| Symptom | Cause | What to do |
|---|---|---|
| `AMENT_TRACE_SETUP_FILES: unbound variable` | `set -u` active before sourcing ROS | Source first, `set -u` afterwards |
| Dozens of unresolved imports inside dependency crates | ext4 remounted read-only because the VHDX could not grow | Free space on the host drive, `wsl --shutdown`, rebuild. See [`troubleshooting.md`](troubleshooting.md) |
| `Unsupported archive identifier` while parsing an rlib | A corrupt artefact from the failure above | Delete the target directory and rebuild |
| A uniform depth buffer, all infinity | No GL context under WSLg | Check that WSLg is active and `/dev/dxg` exists |
| The publisher flies the orbit but no map appears | The mapper was not up when the publisher started | Start the mapper first; the publisher waits for a subscriber by default |
| `arrow-ipc` never finishes compiling | LTO is on | Add `--config 'profile.release.lto=false'` |

## Verification

A correct run prints the comparison table when the mapper exits. For 288
waypoints, endpoints only:

| | octree | hash grid |
|---|---:|---:|
| occupied voxels @ 0.1 m | **56,065** | **56,065** |
| insertion per frame | 0.7 ms | 0.4 ms |

The number that must match exactly across the two columns is **occupied
voxels**. If they differ, one of the maps changed behaviour, and that is what to
investigate first — not the timings.

For RViz: run the mapper, then add an Octomap display on the `octomap_binary`
topic. Its QoS is `transient_local`, so an RViz started later still receives the
latest map.
