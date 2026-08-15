# Media

Every file here is produced by the pipeline rather than edited by hand, but
they fall under **two different licenses** depending on which scene produced
them. Check which section a file is in before reusing it.

---

## Original project demo media — `demo/`

Generated from [`../assets/demo/demo_scene.xml`](../assets/demo/README.md), a
synthetic scene original to this repository. These follow the repository's own
licensing and carry no third-party obligation.

| File | Contents |
|---|---|
| [`demo/demo_map_top.png`](demo/demo_map_top.png) | The Quick Demo's occupancy map from above |
| [`demo/demo_map_side.png`](demo/demo_map_side.png) | The same map from the side |

How to read them, and why there is an unobserved patch at the summit:
[`demo/README.md`](demo/README.md).

---

## Borobudur-derived media — `img/`, `video/`, `apng/`

**Derived from a third-party model licensed CC BY 4.0.** Full attribution,
including what this project did to the model and what redistribution requires,
is in [`BOROBUDUR_ATTRIBUTION.md`](BOROBUDUR_ATTRIBUTION.md).

In short:

- **Title:** Candi Borobudur — **Creator:** `PixForge` — **Source:**
  Sketchfab —
  [model page](https://sketchfab.com/3d-models/candi-borobudur-d42b4ad2e4fd443e838df1d8df1830d0)
- **License:** [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/)
- The rendering, simulated depth sensing, map reconstruction and animation are
  this project's work; the underlying geometry is not.
- **Not** covered by the repository's Apache-2.0 source-code license.
- No endorsement by the original creator is implied.

### Video

| File | Size | Contents | Produced by |
|---|---|---|---|
| [`video/orbit.mp4`](video/orbit.mp4) | 856 KB | The drone flying a full orbit around the temple | `cargo run -p candi-sim --example record_demo` |
| [`video/map_growth.mp4`](video/map_growth.mp4) | 684 KB | The voxel map filling in during the scan, seen from above | the same |

### Animations

APNG versions of the same recordings, for pages that do not play video.

| File | Size | Note |
|---|---|---|
| [`apng/orbit.apng`](apng/orbit.apng) | 5.7 MB | Full resolution, 512×384 |
| [`apng/map_growth.apng`](apng/map_growth.apng) | 2.1 MB | Full resolution |
| [`apng/orbit_web.apng`](apng/orbit_web.apng) | 1.6 MB | 384×288, for the web |
| [`apng/map_growth_web.apng`](apng/map_growth_web.apng) | 872 KB | 384×288, for the web |

APNG was chosen because its encoder adds no dependency: the `png` crate already
in the tree can write it directly, with no ffmpeg on the build machine.

### Images

| File | Contents |
|---|---|
| [`img/map_top.png`](img/map_top.png) | The map from above — the stepped square base, four staircases, the circular terraces with their ring of perforated stupas, the main stupa at the centre |
| [`img/map_side.png`](img/map_side.png) | The same from the side — stepped-pyramid silhouette, blue→red height gradient |
| [`img/scene_overview.png`](img/scene_overview.png) | The MuJoCo scene from outside: temple and drone at true scale |
| [`img/scene_drone_cam.png`](img/scene_drone_cam.png) | The view from `drone_cam` |

The first two are written by `live_scan` on every run (`write_projection()`);
the last two by `examples/scene_shot`.

Those projections exist because an `.rrd` needs a viewer to inspect. They are
what answers the central question — does the reconstruction read as the
structure that was scanned — without opening anything.

---

## Colour

Every visualization uses the same height gradient — **blue → cyan → green →
yellow → red** — over a **fixed 0–6 m** range
([`palette.rs`](../crates/candi-octomap-node/src/palette.rs)). A gradient that
adapted to the map's bounds would make playback look as though the structure
were changing when it is not.

## What is not here

The `.rrd` recordings (23–155 MB each) are not committed. Regenerate one with
`cargo run --release -p candi-sim --bin live_scan`, then open it with the Rerun
viewer 0.35.0. A recording made from the Borobudur scene is a derivative of
that model; one made from the demo scene is not.
