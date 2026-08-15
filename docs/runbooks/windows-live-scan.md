# Runbook — running the scan on Windows (the single-process path)

**When to use it:** a demo, a quick regression check, or rebuilding the backup
recording. Needs no ROS 2 at all.

**Prerequisites:** a Rust toolchain. The converted temple mesh in
`assets/candi_obj/` (see [`../../assets/README.md`](../../assets/README.md)).
MuJoCo downloads itself on the first build.

## Steps

```powershell
# 1. Environment. Both are required.
$env:MUJOCO_DOWNLOAD_DIR = "$PWD\.mujoco"
$env:Path = "$PWD\.mujoco\mujoco-3.9.0\bin;$env:Path"

# 2. Check the environment before a long run (optional, ~10 seconds)
cargo run -p candi-sim --example smoke_test
# Expect: three PASS lines

# 3. Check the scene and its scale without opening a window
cargo run -p candi-sim --example scene_shot
# Expect: temple bbox 14.95 x 14.95 x 6.00, orbit radius 16.48 m
# Writes out/scene_overview.png and out/scene_drone_cam.png

# 4. The full scan
cargo run --release -p candi-sim --bin live_scan
# ~1 second for 288 waypoints

# 5. Playback
.\.tools\rerun\rerun.exe out\candi_scan.rrd
```

The first build takes several minutes (MuJoCo downloads 19.5 MB, the Rerun SDK
is large). Later builds are fast.

## Variants

| Command | What it does |
|---|---|
| `... --bin live_scan -- --mesh` | Overlays the source mesh in the recording (+27 MB) |
| `... --bin live_scan -- --mesh <path>` | Overlays a different file; `.glb`/`.gltf` gets an X rotation applied |
| `cargo run --release -p candi-sim` | Orbit and point cloud only, no mapping |
| `cargo run -p candi-sim --example view_scene` | The interactive viewer |
| `cargo run -p candi-sim --example record_demo` | Records APNG/MP4 into `out/` |
| `cargo test` | 45 unit tests — needs the same two environment variables |

## If it fails

| Symptom | Cause | What to do |
|---|---|---|
| The binary exits at start with no clear message | `mujoco.dll` is not on PATH | Repeat step 1 — PATH does not persist between shell sessions |
| `cargo test` exits with `-1073741515` before any test runs | Same thing: `candi-sim`'s test binary links MuJoCo | Repeat step 1, or run `cargo test -p candi-octomap-node`, which has no dependencies |
| `EventLoopError(RecreationAttempt)` | Two `MjRenderer::build()` calls in one process | A winit event-loop limit. Create the renderer once and reuse it |
| `memory allocation of N bytes failed` | Out of memory, usually with `--mesh` on a ~16 GB machine | Run without `--mesh`; the overlay adds 27 MB per recording |
| The `.rrd` is overwritten and only a few hundred bytes | A run failed partway, after opening the file | The old recording is gone. Copy a good `.rrd` somewhere safe before an experimental run |
| The map is a wide blue disc with the temple sunk in it | The ground filter is not active | `GROUND_Z` must be 0.15 — see [ADR-0007](../decisions/0007-ground-plane-filter.md) |
| A shell of voxels travels with the drone | The depth camera can see the drone's own rotors | The drone's geoms must be in group 2, and `depth_camera_options()` must hide it |

## Verification

A correct run over 288 waypoints produces:

| What appears | Correct value |
|---|---|
| `out/candi_scan.rrd` | ~23.5 MB, 288 frames |
| `out/map_top.png` | Stepped square base, four staircases, concentric stupas |
| `out/map_side.png` | Stepped-pyramid silhouette |
| Occupied voxels printed | **56,063** at 0.1 m |
| Extent | [−7.45, −7.45, 0.15] .. [7.45, 7.45, 6.05] |
| Insertion | 0.3–0.4 ms/frame |

If the voxel count is far from 56,063, something in the pipeline changed —
check `MAX_RANGE`, `GROUND_Z`, `SUBSAMPLE` and the resolution before going on.
