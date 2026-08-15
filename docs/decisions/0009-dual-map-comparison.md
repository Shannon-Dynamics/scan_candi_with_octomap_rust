# ADR-0009 — The octree and the hash grid are both kept

- **Status:** Accepted

## Context

[ADR-0004](0004-hash-grid-occupancy-map.md) wrote an occupancy map here because
the octree it was meant to use did not exist yet. That octree now does exist, in
the neighbouring `octo_map_rust` project — a Rust port of OctoMap C++ 1.10.0
whose `.bt` output is byte-identical to the C++ reference's.

The obvious move was to replace the hash grid with the octree and delete the
old one.

## Decision

Both are kept. `candi_mapper` builds **two maps from the same stream of points,
in the same process, in the same run** (`integrate()`), and prints a comparison
table on exit (`summarize()`).

The octree **carves free space by default** in the ROS 2 path; `--no-carve` /
`--octree-no-carve` turns it off for a like-for-like comparison.

## Evidence

Two things that only keeping both can produce.

**First — cross-validation.** Endpoints only, 288 waypoints, 1,138,259 points:

| | octree | hash grid |
|---|---:|---:|
| **occupied voxels @ 0.1 m** | **56,065** | **56,065** |

Two implementations written separately, given the same points, producing
identical numbers. That is mutual validation, not a resemblance. The same
agreement had already appeared at 24 waypoints (16,384 voxels on both sides).

**Second — the reason for disabling carving turned out to be half wrong.**
`live_scan` disables it on the grounds of "millions of entries and hundreds of
megabytes, to erase nothing". What was measured:

| | octree | hash grid |
|---|---:|---:|
| nodes / entries held | 1,492,583 | 56,065 |
| `.bt` payload | **479,666 B** | — |
| insertion per frame | 80.4 ms | 0.5 ms |

The octree does hold 1.49 million nodes, but it serializes to **480 KB** — not
hundreds of megabytes. Uniform free space prunes into single nodes. **The cost
that made carving unaffordable belonged to the data structure, not to the
method.**

What stays expensive is time: 80.4 ms against 0.7 ms, roughly 115×. Still inside
the 100 ms budget at 10 Hz, but without much room.

## Consequences

- `candi_mapper` depends on **two** mapping implementations. That is
  deliberate; deleting either deletes the measurement.
- The timings above are **upper bounds** — a build without LTO, see
  [`05-results.md`](../05-results.md).
- The 56,065 (WSL) vs 56,063 (Windows) difference remains untraced; 0.003%,
  recorded in [`05-results.md`](../05-results.md).
- If only one map may ever remain, keep the octree — but the hash grid is still
  worth keeping as a test baseline, because it has no dependencies and can run
  in CI.
