# ADR-0002 — The temple is scaled to 6 m tall, not to life size

- **Status:** Accepted

## Context

The source mesh `model.glb` is about 1.9 units across, not metres, so the scale
factor has to be chosen explicitly during conversion. The real Borobudur is
roughly 123 × 123 m and about 35 m tall.

The choice is not free: it is tied to `max_range` through the orbit geometry.
The orbit radius is 1.5 × the bounding sphere, and this mesh's proportions are
width ≈ 2.49 × height. So the drone's distance to the surface ≈ 1.5 × the
temple's height.

## Decision

Scale uniformly to a **height of 6 m** (which makes it 14.95 m wide).

## Evidence

With the 10 m `max_range` the plan specified, the temple height is limited to
≲ 6.7 m. Measured after conversion:

| Quantity | Value |
|---|---|
| Scale factor | 7.864386 |
| Final bbox | 14.946 × 14.946 × 6.000 m |
| Bounding sphere | r = 10.986 m |
| Orbit radius (1.5×) | 16.479 m |
| **Orbit distance to the surface** | **9.006 m** |

9.006 m fits under a 10 m `max_range` with about a metre to spare.

At life size (35 m tall) the orbit radius becomes ~53 m and **every point is cut
off by max range** — the map would be entirely empty.

## Consequences

The 9.006 m figure above turned out not to be the deciding one. It measures the
distance to the **nearest** face; what actually binds is the distance to the
**summit**, because the stepped pyramid's upper terraces are set back. That is
why `max_range` still had to be raised — see
[ADR-0003](0003-max-range-20m.md).

To scale the temple up later, `max_range` has to be raised first, not the other
way round.
