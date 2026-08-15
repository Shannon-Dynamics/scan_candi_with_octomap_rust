# 4. Running it

There are two paths. The single-process path needs no ROS 2 and runs on Windows;
the ROS 2 path needs WSL with Jazzy sourced.

---

## 4.1 Prerequisites

| Path | Required | Optional |
|---|---|---|
| Single process | A Rust toolchain, and an `octo_map_rust` checkout beside this repository | Rerun viewer 0.35.0, to play back the `.rrd` |
| ROS 2 | The above, plus WSL Ubuntu 24.04 or Linux with ROS 2 Jazzy | — |
| Scanning your own mesh | The above, plus Blender 5.x | — |

MuJoCo downloads itself on the first build. **No assets are needed for the
default scene**: `assets/demo/demo_scene.xml` is committed and built from
primitives. The mesh-based scene needs assets that are not in the repository —
see [`assets/README.md`](../assets/README.md).

### The sibling checkout

Both paths depend on `octomap-core` by relative path, so the two repositories
have to sit side by side:

```text
<workspace>/octo_map_rust/                      git clone .../octo_map_rust
<workspace>/scan_candi_with_octomap_rust/       this repository
```

If they are not, `cargo build` fails on a path that does not exist. When
`octomap-core` is published to crates.io this requirement disappears.

---

## 4.2 Windows — the single-process path

```powershell
$env:MUJOCO_DOWNLOAD_DIR = "$PWD\.mujoco"
$env:Path = "$PWD\.mujoco\mujoco-3.9.0\bin;$env:Path"   # mujoco.dll at runtime

# The Quick Demo: the committed demo scene, both maps, octree carving on.
cargo run --release --config 'profile.release.lto=false' -p candi-sim --bin live_scan
.\.tools\rerun\rerun.exe out\candi_scan.rrd                  # 3D playback, 288 frames
```

Produces `out/candi_scan.rrd` plus `out/map_top.png` and `out/map_side.png`.

Flags on `live_scan`:

| Flag | Effect |
|---|---|
| `--scene <path>` | Scan another MJCF. Defaults to `assets/demo/demo_scene.xml` |
| `--octree-no-carve` | Endpoints only, so both maps do identical work, and the run is much shorter |
| `--connect` | Stream into a running Rerun viewer instead of writing a file |
| `--mesh [path]` | Overlay the source geometry in the recording |

`--config 'profile.release.lto=false'` is effectively required: with LTO on,
one of Rerun's dependencies takes impractically long to compile
([`runbooks/troubleshooting.md`](runbooks/troubleshooting.md)).

`mujoco.dll` **must be on PATH at runtime** — without it the binary fails to
load the library with no clear message.

### The other entry points

| Command | What it does |
|---|---|
| `cargo run -p candi-sim --example smoke_test` | Validates mujoco-rs: load+step, mocap tracking, offscreen depth |
| `cargo run -p candi-sim --example scene_shot` | Renders the scene headless with a scale report, writes PNGs to `out/` |
| `cargo run -p candi-sim --example view_scene` | The interactive MuJoCo viewer |
| `cargo run -p candi-sim --example fly_orbit` | Flies the orbit in the viewer, no mapping |
| `cargo run -p candi-sim --example record_demo` | Records the APNG/MP4 animations |
| `cargo run --release -p candi-sim` | Orbit and point cloud only, no mapping |
| `cargo test` | The unit test suite — 45 tests |

`cargo test` needs the same two environment variables as everything else:
`MUJOCO_DOWNLOAD_DIR` at build time and `mujoco.dll` on PATH at run time.
`candi-sim`'s test binary links MuJoCo, so without the DLL it exits with
`STATUS_DLL_NOT_FOUND` (`-1073741515`) before running a single test. Only
`cargo test -p candi-octomap-node` is free of that — it has no dependencies at
all, which is why it is the crate CI covers.

The full procedure is in
[`runbooks/windows-live-scan.md`](runbooks/windows-live-scan.md).

---

## 4.3 WSL — the ROS 2 path

```bash
source /opt/ros/jazzy/setup.bash
cd <path to this repository>
./ros2/run_demo.sh                 # the octree carves free space
./ros2/run_demo.sh --no-carve      # endpoints only, like-for-like with the hash grid
```

That script builds both nodes, brings the mapper up first, flies the orbit, then
writes `out/candi_ros2.rrd`. The mapper prints the comparison table on exit.

Manually, in two terminals — **the mapper has to come up first**, because the
publisher waits for a subscriber before it flies:

```bash
./ros2/target/release/candi_mapper --out out/candi_ros2.rrd [--mesh] [--octree-no-carve]
./ros2/target/release/candi_publisher [--waypoints N] [--no-wait]
```

Measuring octree against hash grid on the release binaries:

```bash
./ros2/measure.sh 288             # endpoints only
./ros2/measure.sh 288 --carve     # the octree also traces free space
```

A short end-to-end run first — 24 of the 288 waypoints on a debug build, for
checking that the pipeline works before committing to a long run:

```bash
./ros2/smoke_run.sh [waypoints] [--carve]
```

### Environment variables that matter

The scripts set these, but they are worth knowing if you run the binaries
directly:

| Variable | Value | Why |
|---|---|---|
| `LD_LIBRARY_PATH` | includes `.../mujoco-3.9.0/lib` | The Linux equivalent of the `$env:Path` line above |
| `CARGO_HOME`, `CARGO_TARGET_DIR`, `ROS_HOME` | pointed at a Windows drive mount | The WSL VHDX sits on a full `C:`; see [`runbooks/troubleshooting.md`](runbooks/troubleshooting.md) |

Sourcing ROS **must happen before `set -u`**. `/opt/ros/jazzy/setup.bash` reads
variables that are not set, so a script with `set -euo pipefail` dies with
`AMENT_TRACE_SETUP_FILES: unbound variable`.

The full procedure is in [`runbooks/wsl-ros2.md`](runbooks/wsl-ros2.md).

---

## 4.4 Looking at the result

```powershell
.\.tools\rerun\rerun.exe out\candi_ros2.rrd
```

The prebuilt viewer at **0.35.0** is what is used — the version has to match the
SDK exactly.

To see the map in RViz: run the mapper, then add an Octomap display on the
`octomap_binary` topic. Its QoS is `transient_local`, so an RViz that starts
late still receives the latest map.

If you would rather not open a viewer at all, the two orthographic PNG
projections are enough to judge the shape — `out/map_top.png` and
`out/map_side.png`.

---

## 4.5 Release builds and LTO

`Cargo.toml` declares `lto = true`, but the release build behind the numbers in
[`05-results.md`](05-results.md) was run with:

```bash
cargo build --release --config 'profile.release.lto=false'
```

With LTO on, crates like `arrow-ipc` (a Rerun dependency) never finished
compiling in the time available. The flag is passed on the command line rather
than by editing `Cargo.toml`, so that the manifest still states the intent. The
consequence is that every timing figure is an upper bound.
