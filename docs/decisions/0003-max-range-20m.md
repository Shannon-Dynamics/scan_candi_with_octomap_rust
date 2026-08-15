# ADR-0003 — `max_range` raised from 10 m to 20 m

- **Status:** Accepted

## Context

The project notes set `max_range = 10.0 m` as a measured parameter.
[ADR-0002](0002-mesh-scale-6m.md) chose the temple's scale from that number, and
the arithmetic looked safe: the orbit's distance to the nearest face is
16.48 − 7.47 = 9.0 m.

The first run passed surface verification — 0% of points strayed — but
the point cloud **never got above z = 3.06 m** although the temple is 6 m tall.
The result would have been a ring of walls with no roof.

## Decision

Raise `max_range` to **20 m**, and correct the project notes.

## Evidence

The cause was geometry, not a bug:

- This temple is a stepped pyramid; its upper terraces are **set back**.
- From any point on the orbit, the summit is **~16.5 m away**, not 9 m.
- At a 10 m `max_range` the top **can never be mapped**, however long the orbit
  runs.
- Shrinking the orbit radius does **not** help: the wide base blocks the view
  upward.

After raising it to 20 m:

| Quantity | Before | After |
|---|---|---|
| Highest point | 3.06 m (51% of the height) | **6.03 m (100%)** |
| Structure points outside the bbox | 0 | **0 (0.00%)** |

20 m reaches the summit while still stopping short of the far side of the
temple.

## Consequences

This is also what makes the ground filter mandatory: at 20 m the camera sees the
ground plane out to that limit in every direction, and 56.5% of points become
floor. See [ADR-0007](0007-ground-plane-filter.md).

The value appears as a `MAX_RANGE` constant in `live_scan.rs`, `main.rs`,
`record_demo.rs` and `candi_publisher/src/main.rs` — all four have to stay
equal, because if they diverge the two paths stop comparing the same thing.
