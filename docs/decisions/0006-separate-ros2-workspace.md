# ADR-0006 — `ros2/` is a separate Cargo workspace

- **Status:** Accepted

## Context

`r2r` builds its message bindings from `AMENT_PREFIX_PATH` **at compile time**
([ADR-0005](0005-r2r-over-rclrs.md)). That means any crate depending on it only
builds on a machine with ROS 2 sourced.

The simulation was developed on Windows, which has no ROS 2 and will not get
one.

## Decision

`ros2/Cargo.toml` is its own workspace, outside the root workspace's `members`
list. The root `Cargo.toml` is untouched.

Inside it, **two crates** — `candi_publisher` and `candi_mapper` — rather than
two binaries in one crate.

## Evidence

If the ROS crates were in the root workspace, `cargo build` at the top level
would fail on every machine without ROS. That would take down `live_scan`, all
the examples and `cargo test` on the Windows side — the path actually used for
the demo.

The two-crate split: the mapper has no reason to compile MuJoCo, and the
publisher has no reason to compile Rerun. On a machine where disk is the binding
constraint — and this one was, see
[`runbooks/troubleshooting.md`](../runbooks/troubleshooting.md) — that is worth
an extra manifest.

## Consequences

- `cargo build` at the root **does not** build the ROS 2 nodes. That is
  deliberate.
- Shared code stays single-sourced: `candi_publisher` depends on the `candi-sim`
  crate by path, so the orbit, the depth projection and the PointCloud2 byte
  layout are not duplicated. The ROS nodes add transport and nothing else.
- `octomap-core` and `octomap-ros` enter as path dependencies on
  `../../../octo_map_rust/`, so **that repository has to sit beside this one**.
  If the octree is published to crates.io, this is the line that changes.
- There are two `Cargo.lock` files. That is the ordinary consequence of two
  workspaces, not a mistake.
