# Demo scene

`demo_scene.xml` is the scene the Quick Demo runs against. It is committed, it
is the whole scene, and it needs nothing else — no meshes, no textures, no
conversion step, no download.

That is the point of it: a fresh clone can run the scan immediately.

## What it contains

A synthetic stepped pyramid, built from MuJoCo primitives:

| Element | Purpose |
|---|---|
| Four square terraces, each set back from the one below | Self-occlusion — the upper tiers are invisible from a low orbit, which is what makes a moving sensor worth having |
| A spherical summit | A curved surface reads very differently from a box in a voxel map, so it shows whether the reconstruction resolves shape or only bulk |
| Four staircases, one per side | The smallest features in the scene, and therefore the test of whether the resolution is fine enough |
| A ground plane | Gives the ground filter something to remove |

Overall size: **15 × 15 × 6 m**, which puts the derived orbit at a radius of
about 16.5 m.

The structure's body is named `candi` because that is the name the pipeline
looks up when it derives the flight path from the structure's bounding box.
Everything about the orbit — radius, ring heights, waypoint count — comes from
that bounding box rather than from constants, so a different scene produces a
different flight path automatically.

## What it is not

It is **not** a model of Borobudur, or of any real structure. It is a shape
chosen to exercise the mapping pipeline. Mapping real scanned geometry needs
`scene/candi_scene.xml` and the assets described in
[`../README.md`](../README.md), which are not distributed here.

## Licence

This file is part of this repository and is covered by the repository's
Apache-2.0 licence — see [`LICENSE.md`](LICENSE.md).

It contains no third-party geometry, textures or data. Nothing in it was
derived from a scan, a photograph, or an asset library.

## Using it

It is the default scene, so:

```bash
cargo run --release -p candi-sim --bin live_scan
```

To point the same pipeline at another scene:

```bash
cargo run --release -p candi-sim --bin live_scan -- --scene path/to/scene.xml
```

Any MJCF works, provided it defines a body named `candi`, a mocap body named
`drone`, and a camera named `drone_cam` on that body.
