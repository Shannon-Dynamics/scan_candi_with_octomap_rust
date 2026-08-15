# Licence for the demo scene

`demo_scene.xml` in this directory is original work, authored as part of this
repository, and is licensed under the **Apache License 2.0** — the same terms
as the repository's source code. See [`../../LICENSE`](../../LICENSE) for the
full text and [`../../NOTICE`](../../NOTICE) for third-party attributions.

Apache-2.0 does **not** apply to everything in the repository: the Borobudur
model and its derivatives are CC BY 4.0. This file covers this directory.

## Provenance

The scene is written from scratch in MJCF using MuJoCo's built-in primitive
shapes (boxes, a sphere, a plane) and built-in procedural textures. It embeds:

- no third-party meshes,
- no third-party textures or images,
- no scanned or photogrammetric data,
- no assets from any model library.

It may be redistributed, modified and used commercially under Apache-2.0.

## What this file does not cover

Other assets in this repository are third-party and carry their own terms.
None of them is covered by this file, and none is committed here:

- **The Borobudur model** and everything converted or rendered from it —
  `assets/candi_obj/`, `assets/candi_decimated.glb`, and the media under
  `media/img/`, `media/video/` and `media/apng/` — is licensed
  **CC BY 4.0**. See [`../borobudur/LICENSE.md`](../borobudur/LICENSE.md).
- **The Skydio X2 model** comes from MuJoCo Menagerie under **Apache-2.0** and
  ships with its own licence file, which travels with it.

See [`../README.md`](../README.md).
