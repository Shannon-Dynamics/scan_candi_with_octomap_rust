# Architecture Decision Records

Every decision that changed the design is recorded here together with **the
measurement that justified it**. This project measures rather than claims; an
ADR without numbers has to say that the numbers are missing.

The format is in [`_template.md`](_template.md).

| ADR | Title | Phase | Status |
|---|---|---|---|
| [0001](0001-full-rust-architecture.md) | The whole runtime path is written in Rust | A.0 | Accepted |
| [0002](0002-mesh-scale-6m.md) | The temple is scaled to 6 m tall, not to life size | A.1 | Accepted |
| [0003](0003-max-range-20m.md) | `max_range` raised from 10 m to 20 m | B | Accepted |
| [0004](0004-hash-grid-occupancy-map.md) | The occupancy map is written here, on a hash grid | D | Accepted, extended by 0009 |
| [0005](0005-r2r-over-rclrs.md) | `r2r` as the ROS 2 binding rather than `rclrs` | C | Accepted |
| [0006](0006-separate-ros2-workspace.md) | `ros2/` is a separate Cargo workspace | C | Accepted |
| [0007](0007-ground-plane-filter.md) | Points below `z = 0.15 m` are discarded as floor | D | Accepted |
| [0008](0008-skip-obj2mjcf.md) | `obj2mjcf` skipped, the `<asset>` block written by hand | A.1 | Accepted |
| [0009](0009-dual-map-comparison.md) | The octree and the hash grid are both kept | D | Accepted |

## Decisions that do not stand alone as ADRs

Summarized here so they are not lost:

- **Ubuntu 22.04 / ROS 2 Humble → Ubuntu 24.04 / Jazzy.** Three other WSL
  distros on the machine were broken from the start (`ext4.vhdx` missing,
  `E_UNEXPECTED`, disk attach failures). `r2r` supports Jazzy, so the
  architecture did not change.
- **Blender installed as a portable build** rather than through the MSI
  installer, which fails with exit 1603 asking for admin elevation.
- **A prebuilt Rerun viewer** rather than the `web_viewer` feature — compiling
  it ran out of memory on a ~16 GB machine.
- **Release builds without LTO passed on the command line**, not by editing
  `Cargo.toml`, so the manifest keeps stating the intent.
- **Fallbacks that were never needed:** the three planned workload reductions
  (raise the subsample, lower the rate, coarsen the resolution) and the mapping
  fallback (the C++ `octomap_server`). Not one was required.
