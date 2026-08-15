# Roadmap

Phases, not dates. This is a reference application, so "done" means it
demonstrates the library well and runs for someone who is not its author —
not that it grows into a product.

Nothing here promises a feature. Items marked *not started* have no
implementation behind them.

---

## Phase 1 — Runs for someone else

**Goal: a person who did not write this can build it, run it, and understand
what came out.**

| | Item |
|---|---|
| ✅ | Both pipeline paths working and measured |
| ✅ | The `octomap-core` dependency resolves against a documented checkout layout |
| ✅ | Documentation in English, every command checked against the code |
| ✅ | A tutorial covering the workflow end to end |
| ✅ | `cargo fmt`, `cargo clippy -D warnings` and the tests clean across the workspace |
| ✅ | CI on the dependency-free crate |
| ✅ | A committed demo scene, so a fresh clone can run the scan without the private mesh |
| ✅ | `Cargo.lock` committed for both workspaces |
| ⬜ | CI observed green on a real push |

## Phase 2 — A better demonstration of the library

**Goal: seeing `octomap-core` work should not require ROS 2.**

| | Item |
|---|---|
| ✅ | **The octree is built in `live_scan`**, beside the hash grid, from the same points |
| ✅ | The comparison table is printed from the single-process path |
| ⬜ | Export a `.bt` file from either path, so the map can be opened in `octovis` and other OctoMap tooling without a ROS graph |
| ⬜ | A short worked example that reads a committed point cloud and maps it — no MuJoCo, no ROS, runnable in CI |
| ⬜ | Depend on a published `octomap-core` from crates.io, removing the side-by-side checkout requirement |

## Phase 3 — Closer to a real sensor

**Goal: the parts of mapping that only matter with imperfect data.**

| | Item |
|---|---|
| ⬜ | A noise model on the depth render — Gaussian range noise, dropouts, and the occasional spurious return. Free-space carving earns its cost only against data like that |
| ⬜ | Sensor-frame publishing with a real TF chain, rather than points that are already in the map frame |
| ⬜ | Timing with LTO enabled, closing the caveat on every number in [`docs/05-results.md`](docs/05-results.md) |
| ⬜ | Tracing the 2-voxel Windows/WSL difference, or documenting it as inherent |

## Phase 4 — Broader applicability

**Goal: make it obvious this is a scanning workflow, not a Borobudur demo.**

| | Item |
|---|---|
| ⬜ | Move the pipeline constants out of `live_scan.rs` into a config file or CLI flags, so a different structure does not need a rebuild |
| ⬜ | A second scene, to prove the orbit derivation really is mesh-driven |
| ⬜ | Coverage analysis: which parts of the surface the orbit never sees, and what path would fix it |
| ⬜ | A screen recording of a Rerun playback session — the one media artefact still missing |

## Phase 5 — Robotics integration

**Goal: the parts a real deployment needs and this demo deliberately skips.**

| | Item |
|---|---|
| ⬜ | Consume a pose estimate rather than a known one, so the drone's position is a result rather than an input |
| ⬜ | Run against a recorded rosbag from real hardware |
| ⬜ | A launch file and parameters, instead of shell scripts |

Anything in this phase is a different project wearing this one's clothes. It is
listed so the boundary is explicit: **this repository does not do SLAM,
localization or planning, and adding them is a decision, not an increment.**

---

## Not on the roadmap

- **Making the simulation prettier.** The depth camera sees geometry; the
  rendering is a means.
- **Replacing the hash grid with the octree.** Deleting it deletes the
  comparison — [ADR-0009](docs/decisions/0009-dual-map-comparison.md).
- **Publishing these crates to crates.io.** They are an application. The library
  is what gets published, in the other repository.
