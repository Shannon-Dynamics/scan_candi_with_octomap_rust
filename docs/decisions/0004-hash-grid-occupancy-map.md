# ADR-0004 — The occupancy map is written here, on a hash grid

- **Status:** Accepted, extended by [ADR-0009](0009-dual-map-comparison.md)

## Context

The project notes named `crates/octomap-core/` as an existing, tested dependency
and **forbade changing it**.

That crate was not in this workspace, and it is not published on crates.io. The
closest candidate, `bye_octomap_rs 0.1.1`, turned out to be an unfinished port —
its core files are 0 bytes: `OccupancyOcTreeBase.rs`, `OcTree.rs`,
`OcTreeBase.rs`, `OcTreeIterator.rs`, `AbstractOccupancyOcTree.rs`. The
occupancy mapping is exactly the part that is missing.

Waiting for it was not an option.

## Decision

`crates/candi-octomap-node/src/occupancy.rs` was written for this project. Its
probabilistic semantics **follow OctoMap**; its storage is a **hash map keyed by
integer grid coordinates, not an octree**.

| Parameter | Value |
|---|---|
| `PROB_HIT` | 0.7 |
| `PROB_MISS` | 0.4 |
| Clamp | [−2.0, 3.5] |
| Occupied threshold | log-odds > 0 (p > 0.5) |
| Insertion | discretized |
| Free space | 3D DDA ray casting |

## Evidence

At this scene's scale — a ~40 m box at 0.1 m where only the surface is ever
touched — a hash map is simpler to get right and quicker to query. The octree's
advantage is memory compaction over much larger volumes, and at the time there
was no such volume here.

The result: **0.3–0.4 ms per frame** against a 100 ms budget, and 56,063
occupied voxels whose shape reads plainly as Borobudur.

The crate also has **no dependencies at all**, so the whole mapping logic can be
tested without MuJoCo, a GPU or ROS 2. **10 unit tests.**

## Consequences

This decision was not reversed when the octree finally became available in the
sibling `octo_map_rust` repository. Both are **kept and run side by side**,
because that comparison produces the most valuable measurement in the project —
see [ADR-0009](0009-dual-map-comparison.md) and
[`05-results.md`](../05-results.md).

The limit is clear and measured: in carving mode the hash grid stores every
empty voxel individually while the octree prunes them. That is a property of the
data structure, not a weakness of the implementation.
