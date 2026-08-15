# Changelog

The format follows [Keep a Changelog](https://keepachangelog.com/) and the
project follows [semantic versioning](https://semver.org/).

## [Unreleased]

Nothing has been released yet. This section describes the repository as it
stands.

### Added

- **A Quick Demo that needs no external assets.** `assets/demo/demo_scene.xml`
  is a self-contained MJCF scene built from MuJoCo primitives — a stepped
  pyramid with terraces, staircases and a domed summit — so a fresh clone can
  run the scan immediately. It is original work under this repository's licence.
- **`octomap-core` in the single-process path.** `live_scan` now builds the
  library's octree *and* the project's hash grid from the same points, prints a
  comparison table, and logs both to Rerun. Seeing the mapping library work no
  longer requires ROS 2.
- `--scene <path>` on `live_scan`, defaulting to the demo scene, so the same
  pipeline can be pointed at any MJCF.
- `--octree-no-carve` on `live_scan`, which makes both maps do identical work.
  They then agree exactly on the occupied-voxel count.
- `docs/SCANNING_TUTORIAL.md` — the workflow end to end, starting from a fresh
  clone rather than from prepared assets.
- `CONTRIBUTING.md`, `SECURITY.md`, `ROADMAP.md`, `deny.toml`.
- `media/demo/` — the projection images the Quick Demo produces, with an
  explanation of how to read them.
- Cargo metadata for every crate: `description`, `license`, `repository`, and
  `publish = false`, since these are application crates.

### Changed

- The README is structured around the Quick Demo: requirements, clone, build,
  run, what you should see. The technology stack is split by which demo needs
  it, and an environment support matrix distinguishes what has been verified
  from what has not.
- Free-space carving is on by default for the octree in both paths, which is
  what makes free space visible in the demo. The hash grid keeps it off — it
  stores every empty voxel individually, and the octree does not.
- All documentation is in English.
- CI requests read-only permissions and pins third-party actions to commit
  SHAs. It also syntax-checks the shell scripts.
- `Cargo.lock` is committed for both workspaces.

### Fixed

- **The `octomap-core` dependency did not resolve.** It pointed at a directory
  name that does not exist under the current repository layout, so nothing
  under `ros2/` could build. The expected side-by-side checkout is now
  documented in the manifest, the README and `docs/04-running.md`.
- **Formatting and Clippy failures across the workspace**, including in the
  crate CI checks. Among them: `rec.flush_blocking()`'s `Result` was discarded,
  so a failed flush reported success while leaving a truncated recording. It is
  propagated now.
- **The `ros2/*.sh` scripts defaulted to absolute paths from one machine.** The
  MuJoCo directory now defaults to the repository, and the cargo directories
  are left alone with the disk-space rationale documented as an override.

### Security

- The `PointCloud2` decoder in `candi-octomap-node` reports a truncated or
  inconsistent buffer as a `ParseError` rather than panicking; this is covered
  by unit tests.
- The README states which parts of this repository are `unsafe`-free and which
  link native code through FFI, rather than inheriting the mapping library's
  claim wholesale.

### Documentation

- Every claim about `octomap-core` usage is checked against the source it
  describes, and the code samples are abridged from it rather than written from
  memory.
- Provenance of the media derived from the non-redistributable mesh is stated
  in `media/README.md`.

### Known limitations

The public demo uses a synthetic scene; the original scan assets are not
redistributable; the sensor has no noise model; the ROS 2 path is Linux-only;
timings were taken with LTO disabled and are upper bounds; and there is a
2-voxel difference between the Windows and WSL paths that has not been traced.

---

## Earlier work — ROS 2 integration

### Added

- `ros2/candi_publisher` — flies the orbit in MuJoCo and publishes
  `sensor_msgs/PointCloud2` on `/cloud` and `tf2_msgs/TFMessage` on `/tf`.
  Waits for a subscriber before flying; `--no-wait` skips that.
- `ros2/candi_mapper` — subscribes to both topics and builds **two** occupancy
  maps from the same points: the `octomap-core` octree and the project's hash
  grid. Publishes `octomap_msgs/Octomap` on `octomap_binary` (latched) and
  records `out/candi_ros2.rrd`.
- `ros2/run_demo.sh`, `ros2/measure.sh`, `ros2/build_all.sh`,
  `ros2/smoke_run.sh` — build, demo and measurement drivers.
- `--mesh` on `live_scan` and `candi_mapper` — logs the source geometry as a
  static `Asset3D` at `candi/mesh`. Off by default; it adds 27 MB per
  recording.

### Measured

- Octree and hash grid **agree exactly on 56,065 occupied voxels** at 0.1 m from
  1,138,259 points — two independently written implementations, same input, same
  answer.
- Free-space carving in the octree: 1,492,583 nodes held but only **480 KB**
  serialized, versus the "hundreds of megabytes" that made carving unaffordable
  for a hash grid. The cost belonged to the data structure, not to the method.
  Time cost is real: 80.4 ms/frame versus 0.7 ms, ~115×.
- MuJoCo headless rendering **works under WSLg** via `/dev/dxg` — the largest
  open question about this stack. 307,200 finite depth values, 0.17–59.49 m.

### Fixed

- **A build failure that looked like `r2r` or WSL being unstable** was neither:
  a full host drive prevents the sparse WSL VHDX from growing, which remounts
  ext4 read-only mid-build and surfaces as impossible compile errors. Resolved
  by pointing `CARGO_HOME`, `CARGO_TARGET_DIR` and `ROS_HOME` at a host drive
  mount.

### Changed

- **`rclrs` → `r2r 0.9.5`.** `rclrs`'s message crates on crates.io are 0.0.0
  placeholders, so it still requires a full colcon `ros2_rust` workspace — and
  there is no passwordless sudo on this WSL install. `r2r` generates its bindings
  from the sourced Jazzy installation in one `cargo build`. See
  [ADR-0005](docs/decisions/0005-r2r-over-rclrs.md).
- Release builds run with `--config 'profile.release.lto=false'` on the command
  line rather than editing `Cargo.toml`, so the manifest keeps stating its intent.

---

## Earlier work — the simulation and the first maps

### Added

- Cargo workspace, `crates/candi-sim` and `crates/candi-octomap-node`.
- `examples/smoke_test.rs` — the mujoco-rs validation: load+step, mocap
  tracking, offscreen depth. All three passed, confirming the full-Rust
  architecture ([ADR-0001](docs/decisions/0001-full-rust-architecture.md)).
- `scripts/convert_glb.py` — Blender headless conversion: 2,135,354 → 199,999
  triangles, uniform scale to 6 m height, recentre, drop to z=0, export OBJ+GLB.
- `scene/candi_scene.xml` — MJCF with the real mesh, the Skydio X2 as a mocap
  body, and `drone_cam`.
- `src/orbit.rs`, `src/depth_to_cloud.rs`, `src/ros_bridge.rs`,
  `src/occupancy.rs`, `src/cloud_parser.rs`, `src/palette.rs`.
- `src/bin/live_scan.rs` — the integrated single-process demo.
- `examples/record_demo.rs` — APNG/MP4 output, `media/video/`.

### Measured

- Simulation 2.3 ms/frame (441 Hz); insertion 0.3–0.4 ms/frame against a 100 ms
  budget. None of the plan's three workload reductions were needed.
- 0.00% of structure points fall outside the candi bounding box; highest point
  6.03 m = 100% of the temple height.
- 56,063 occupied voxels at 0.1 m, extent [−7.45, −7.45, 0.15] .. [7.45, 7.45, 6.05].

### Changed

- **`max_range` 10 m → 20 m.** The candi's upper terraces are set back, putting
  the summit ~16.5 m from any orbit point. At 10 m nothing above z=3.06 m was
  ever captured, and no orbit radius fixes it.
  [ADR-0003](docs/decisions/0003-max-range-20m.md).
- **Occupancy map written for this project.** `crates/octomap-core/` did not
  exist; the closest crates.io candidate had 0-byte core files. OctoMap's
  probabilistic semantics over a hash grid.
  [ADR-0004](docs/decisions/0004-hash-grid-occupancy-map.md).
- **obj2mjcf skipped** — single material, no collision needed.
  [ADR-0008](docs/decisions/0008-skip-obj2mjcf.md).
- **Ubuntu 22.04 / Humble → Ubuntu 24.04 / Jazzy** — the other three WSL distros
  on this machine were already broken.

### Fixed

- **Drone saw its own rotors** at 0.14 m in the depth buffer, which would have
  injected a voxel shell travelling with the drone. Drone geoms moved to group 2,
  hidden from the depth camera only.
- **Temple standing on its side** — Blender's OBJ exporter defaults to Y-up,
  cancelling the glTF importer's Z-up conversion. Fixed with explicit
  `forward_axis="Y", up_axis="Z"`.
- **Bounding box 26.73 m instead of 14.95 × 14.95 × 6.00 m** — `body_aabb()` used
  `geom_rbound` (a bounding-sphere radius) for meshes. Would have inflated the
  orbit to 34.7 m, where every point is clipped by max range and the map comes
  out empty.
- **Double mesh rotation** — MuJoCo's compiler rewrites mesh vertices into the
  principal inertial frame, but `geom_xmat`/`geom_xpos` already include
  `mesh_quat`. Applying it again rotated the mesh twice.
- **Ground plane dominating the map** — 56.5% of points were floor, producing a
  31 m blue disc. Fixed with a `z < 0.15 m` filter, the same one real
  `octomap_server` deployments carry.
- **Collada export gone in Blender 5.x** — falls back to GLB, which suits the
  Rerun overlay better anyway.
- **MTL without `map_Kd`** — the OBJ exporter did not understand the glTF
  importer's material graph. Cosmetic only; the depth camera sees geometry.
