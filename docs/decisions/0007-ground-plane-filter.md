# ADR-0007 — Points below `z = 0.15 m` are discarded as floor

- **Status:** Accepted

## Context

After `max_range` was raised to 20 m ([ADR-0003](0003-max-range-20m.md)), the
depth camera sees the ground plane out to that limit in every direction. The
first run produced a map **dominated by floor**: a 31 m blue disc with the
temple sunk in the middle of it, plus an aliasing grid pattern from pixel
sampling across a flat surface.

## Decision

Discard points with `z < 0.15 m` before insertion. The constant
`GROUND_Z = 0.15` is the same in `live_scan.rs`, `record_demo.rs` and
`candi_publisher/src/main.rs`.

## Evidence

| Quantity | Before the filter | After |
|---|---|---|
| Occupied voxels | 115,259 | **56,063** |
| XY extent | ±15.75 m (the floor) | **±7.45 m (the temple)** |
| Floor share of the points | 56.5% of 2,615,737 | discarded |

A 0.15 m threshold costs only a voxel or two of the temple's base — far less
than burying the whole structure inside a disc of floor.

Real `octomap_server` deployments carry the same filter for the same reason;
this is not a departure from common practice.

## Consequences

- Point-cloud quality verification has to **separate floor points from structure
  points** before testing "did any point stray outside the temple bbox". Floor
  points outside the bbox are legitimate. Once separated, 0.00% of structure
  points strayed.
- The filter is on the **publisher** side, not the mapper, so floor points never
  cross the middleware. That also means other consumers of the `cloud` topic
  will not receive them — if they are ever needed (to map the ground around the
  temple, say), the filter should move rather than the threshold change.
