# Media

Every file here is produced by the pipeline, not edited by hand.

## Quick Demo output — [`demo/`](demo/README.md)

The two projection images the Quick Demo writes on a fresh clone. They come
from the self-contained scene in [`../assets/demo/`](../assets/demo/README.md),
so they are original work under this repository's licence, and they are what
the README shows.

## Recordings from the mesh-based scene

> **Provenance note.** The files in `video/`, `apng/` and `img/` were produced
> from the Borobudur mesh, which is **not distributed with this repository**
> and whose redistribution rights are not established. They are voxel
> reconstructions and renders derived from it rather than the mesh itself.
> Whether they may be published is a decision for the repository owner; the
> Quick Demo output in [`demo/`](demo/README.md) covers the same ground with
> assets this project owns outright.

### Video

| File | Size | Contents | Produced by |
|---|---|---|---|
| [`video/orbit.mp4`](video/orbit.mp4) | 853 KB | The drone flying a full orbit around the structure | `cargo run -p candi-sim --example record_demo` |
| [`video/map_growth.mp4`](video/map_growth.mp4) | 682 KB | The voxel map growing during the scan, seen from above | the same |

### Animations

APNG versions of the same, for pages that do not play video.

| File | Size | Note |
|---|---|---|
| [`apng/orbit.apng`](apng/orbit.apng) | 5.6 MB | Full resolution, 512×384 |
| [`apng/map_growth.apng`](apng/map_growth.apng) | 2.0 MB | Full resolution |
| [`apng/orbit_web.apng`](apng/orbit_web.apng) | 1.5 MB | 384×288, for the web |
| [`apng/map_growth_web.apng`](apng/map_growth_web.apng) | 870 KB | 384×288, for the web |

APNG was chosen because its encoder adds no dependency: the `png` crate already
in the tree can write it directly, with no ffmpeg on the build machine.

### Images

| File | Contents |
|---|---|
| [`img/map_top.png`](img/map_top.png) | The map from above — stepped base, four staircases, circular terraces, the main stupa at the centre |
| [`img/map_side.png`](img/map_side.png) | The same from the side — stepped-pyramid silhouette, blue→red height gradient |
| [`img/scene_overview.png`](img/scene_overview.png) | The MuJoCo scene from outside: structure and drone at true scale |
| [`img/scene_drone_cam.png`](img/scene_drone_cam.png) | The view from `drone_cam` |

The first two are written automatically by `live_scan` on every run
(`write_projection()`); the last two by `examples/scene_shot`.

Those PNG projections exist because an `.rrd` needs a viewer to inspect. They
are what answers the central question — does the shape read as the structure —
without opening anything.

## Colour

Every visualization uses the same height gradient — **blue → cyan → green →
yellow → red** — over a **fixed 0–6 m** range
([`palette.rs`](../crates/candi-octomap-node/src/palette.rs)). A gradient that
adapted to the map's bounds would make playback look as though the structure
were changing when it is not.

## What is not here

The `.rrd` recordings (23–155 MB each) are not committed. Regenerate one with
`cargo run --release -p candi-sim --bin live_scan`, then open it with the Rerun
viewer 0.35.0.
