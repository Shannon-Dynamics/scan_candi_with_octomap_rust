# Contributing

Thanks for looking. This is a reference application, not a library: its job is
to show how [`octo_map_rust`](https://github.com/Shannon-Dynamics/octo_map_rust)
is used on a real spatial-mapping problem, and to be readable while doing it.

## Which repository does my change belong in?

There are two, and the split matters:

| Change | Repository |
|---|---|
| The octree, occupancy semantics, ray casting, file I/O, the library's public API | **[`octo_map_rust`](https://github.com/Shannon-Dynamics/octo_map_rust)** |
| ROS 2 message conversions, `PointCloud2` decoding in the library | **`octo_map_rust`** → `crates/octomap-ros` |
| The simulation, the orbit, depth projection, the MJCF scene | **here** → `crates/candi-sim` |
| The hash-grid map, the palette, this project's `PointCloud2` parser | **here** → `crates/candi-octomap-node` |
| The ROS 2 nodes for this demo | **here** → `ros2/` |
| "The library should also support X" | **`octo_map_rust`** |
| "The example should demonstrate X" | **here** |

Rule of thumb: if it would still be true for someone mapping a warehouse, it
belongs in the library. If it is about a temple, a drone or MuJoCo, it belongs
here.

## Getting set up

The single-process path needs nothing but Rust:

```bash
git clone https://github.com/Shannon-Dynamics/scan_candi_with_octomap_rust
cd scan_candi_with_octomap_rust

# Windows
$env:MUJOCO_DOWNLOAD_DIR = "$PWD\.mujoco"
$env:Path = "$PWD\.mujoco\mujoco-3.9.0\bin;$env:Path"

cargo build
cargo test                     # 45 tests; needs both variables above
cargo fmt --all
cargo clippy --workspace --all-targets -- -D warnings
```

If you only want to work on the mapping logic, `candi-octomap-node` has no
dependencies at all:

```bash
cargo test -p candi-octomap-node        # no MuJoCo, no GPU, no ROS
cargo clippy -p candi-octomap-node --all-targets -- -D warnings
```

That is also exactly what CI runs, and it is the crate to prefer for new logic
that does not have to touch the simulator.

**To run the scan** you also need the assets — see
[`assets/README.md`](assets/README.md). They are not committed.

**To work on the ROS 2 path** you need WSL with ROS 2 Jazzy and an
`octo_map_rust` checkout beside this repository:

```text
<parent>/octo_map_rust/
<parent>/scan_candi_with_octomap_rust/
```

See [`docs/04-running.md`](docs/04-running.md).

## The rules that keep this demo honest

1. **No Python in the runtime pipeline.** `scripts/convert_glb.py` is the only
   Python and it runs once, offline, inside Blender.
2. **The drone stays kinematic** — a mocap body, never physics-controlled. The
   flight path is an input, not a result.
3. **Keep both occupancy maps.** Deleting the hash grid deletes the comparison
   that validates the octree —
   [ADR-0009](docs/decisions/0009-dual-map-comparison.md).
4. **`ros2/` stays a separate Cargo workspace.** Folding it into the root breaks
   `cargo build` on any machine without ROS 2 —
   [ADR-0006](docs/decisions/0006-separate-ros2-workspace.md).
5. **Do not modify `octomap-core` from here.** It lives in the sibling
   repository and is a path dependency, not vendored.
6. **Every design change gets an ADR** in `docs/decisions/`, with the
   measurement that justified it. Nine exist; follow their format.
7. **This project measures rather than claims.** If you change the pipeline,
   re-run the scan and update the numbers in
   [`docs/05-results.md`](docs/05-results.md). If you cannot measure something,
   say so in the ADR rather than estimating.

## Submitting a change

1. Open an issue first for anything that changes the pipeline's behaviour or
   adds a dependency. Documentation fixes and bug fixes can go straight to a
   pull request.
2. Before pushing:

   ```bash
   cargo fmt --all -- --check
   cargo clippy --workspace --all-targets -- -D warnings
   cargo test
   ```

3. If the pipeline changed, run the scan and paste the before/after numbers into
   the pull request. The occupied-voxel count is the one that must not move for
   an unrelated reason.
4. Add a `CHANGELOG.md` entry under `## [Unreleased]`.
5. Update `docs/status/now.md` if you closed or opened something.

## Lockfile policy

`Cargo.lock` is **committed** for both workspaces. This is an application: the
point is that two people building it get the same dependency versions, and the
tree is large enough (hundreds of crates through Rerun and MuJoCo) that a
resolver difference is a real source of "works on my machine".

## Security checks

Not run in CI — the tree needs MuJoCo and a full resolve — so they are a
maintainer step:

```bash
cargo audit          # RustSec advisories
cargo deny check     # advisories, licences, sources, duplicates
```

`deny.toml` explains every exception it makes. Two transitive crates are
currently unmaintained rather than vulnerable; that is documented there rather
than silenced.

## Documentation

All documentation is written in English. `docs/archive/` is the exception: it is
a historical record kept verbatim in Indonesian, and it is not edited — the
wrong conclusions in it are part of the record.

Every command in a README or a runbook should run as written, and every number
should come from a run that happened. If you find one that does not, that is a
bug worth a pull request on its own.
