# Quick Demo output

These two images are what `cargo run --release -p candi-sim --bin live_scan`
writes on a fresh clone, with no assets to prepare. They come from the
self-contained demo scene in [`../../assets/demo/`](../../assets/demo/README.md)
and are therefore covered by this repository's own licence, like the scene.

| File | View |
|---|---|
| [`demo_map_top.png`](demo_map_top.png) | The finished occupancy map from above |
| [`demo_map_side.png`](demo_map_side.png) | The same map from the side |

## What to look for

**From above:** four concentric square terraces, each a step higher than the
one outside it — blue at the base through cyan and green to orange at the top —
with four staircases breaking the outline, one per side, and the round summit
as a red patch in the middle.

**From the side:** the stepped profile, the notches where the staircases cut
through, and the dome on top. The colour is height, not confidence.

**The small unobserved patch at the very centre of the top view** is not a
defect. No waypoint ever looks straight down at the apex of the dome, so no ray
ever reaches it, and an occupancy map reports that as *unknown* rather than
guessing. It is the clearest illustration in the whole demo of why the library
distinguishes free from unknown.

Both images are orthographic projections written by the scan itself, so they
can be compared directly against a later run.
