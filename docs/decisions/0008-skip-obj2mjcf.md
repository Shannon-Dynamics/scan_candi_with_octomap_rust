# ADR-0008 — `obj2mjcf` skipped, the `<asset>` block written by hand

- **Status:** Accepted

## Context

The asset pipeline was expected to run `obj2mjcf` on the converted OBJ to
produce the MJCF.

That tool does two things: split an OBJ per material, and build a convex
collision decomposition.

## Decision

Skipped. The `<asset>` block (mesh + texture + material) is written directly in
`scene/candi_scene.xml`.

## Evidence

- The OBJ that `convert_glb.py` produces has **one material**, so there is
  nothing to split.
- The temple uses `contype="0" conaffinity="0"` — no collision at all, because
  the drone moves kinematically and never hits anything. A convex decomposition
  would never be used.

What would be left is one more Python dependency, which collides with the
project rule of no Python in the runtime path.

No material is lost: the chain is `map_Kd` → `<texture>` → `<material>`,
verified through a `scene_shot` render.

## Consequences

If the temple ever needs collision — if the drone moves from mocap to physics
control, say — this decision has to be revisited, because a 199,999-triangle
mesh cannot be used directly as a collision geom.

While the drone stays kinematic, nothing is lost.
