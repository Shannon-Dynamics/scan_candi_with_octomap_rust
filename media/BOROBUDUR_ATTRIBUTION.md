# Attribution for Borobudur-derived media

The files under [`img/`](img/), [`video/`](video/) and [`apng/`](apng/) are
derived from a third-party 3D model and are licensed accordingly. This file is
the attribution that CC BY 4.0 requires to travel with them.

## Source

- **Title:** Candi Borobudur
- **Creator:** `PixForge` — see the note at the end of this file
- **Source:** Sketchfab
- **Original model:** <https://sketchfab.com/3d-models/candi-borobudur-d42b4ad2e4fd443e838df1d8df1830d0>
- **License:** Creative Commons Attribution 4.0 International (CC BY 4.0)
- **License text:** <https://creativecommons.org/licenses/by/4.0/>

## What was done to it

The model was converted, decimated and scaled for simulation
([`../assets/borobudur/LICENSE.md`](../assets/borobudur/LICENSE.md) lists the
steps), then loaded into a MuJoCo scene. Everything in these three directories
was produced by this project from that scene:

| Files | How they were produced |
|---|---|
| `img/scene_overview.png`, `img/scene_drone_cam.png` | Rendered from the simulated scene by `cargo run -p candi-sim --example scene_shot` |
| `img/map_top.png`, `img/map_side.png` | Orthographic projections of an occupancy map reconstructed from simulated depth images, written by `live_scan` |
| `video/orbit.mp4`, `apng/orbit*.apng` | A recording of the simulated flight around the model, by `cargo run -p candi-sim --example record_demo` |
| `video/map_growth.mp4`, `apng/map_growth*.apng` | The same recording of the occupancy map filling in as the scan proceeds |

So the rendering, the simulated sensing, the map reconstruction and the
animation are this project's work; the underlying geometry is not. Both the
model and these derivatives remain under CC BY 4.0.

## Licensing boundary

These files are **not** covered by the Apache-2.0 license that applies to this
repository's source code. Redistributing any of them requires keeping this
attribution, indicating that changes were made, and linking to the license.

The media under [`demo/`](demo/README.md) is a different matter: it is
generated from this repository's own synthetic scene and carries no
third-party obligation.

No endorsement of this project by the original creator is implied.

## Creator name

**`PixForge` is a placeholder.** The creator's name could not be
verified from any material available locally: the downloaded archive is the
"source" variant, which omits Sketchfab's `license.txt`, and the glTF files
were re-exported through a tool that dropped the `asset.copyright` field.

Replace it with the name shown on the model page before publishing. Until then
the attribution is incomplete and the CC BY 4.0 condition is not met.
