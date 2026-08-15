# Borobudur asset

The scan this project is named after uses a 3D model of **Candi Borobudur**
published on Sketchfab under **CC BY 4.0**. Attribution and the licensing
boundary are in [`LICENSE.md`](LICENSE.md); read that before redistributing
anything derived from the model.

## This asset is optional

The repository does **not** need it. The default Quick Demo runs against
[`../demo/demo_scene.xml`](../demo/README.md) — a synthetic stepped pyramid
built from MuJoCo primitives, original to this project, committed, and covered
by the repository's own license. A fresh clone has a complete, runnable
demonstration without downloading or redistributing any third-party model.

Use the Borobudur asset when you want the scan the project was built around:
real scanned geometry, with the irregularity and self-occlusion that a
synthetic scene does not reproduce.

| | Default | Borobudur |
|---|---|---|
| Scene | `assets/demo/demo_scene.xml` | `scene/candi_scene.xml` |
| Provenance | Original to this repository | Third-party, CC BY 4.0 |
| In the repository | Committed | Not committed — you supply it |
| Preparation | None | Blender conversion, once |

## Which path uses it

Both, once the assets are in place, by pointing the scan at the mesh-based
scene:

```bash
# Single-process path
cargo run --release -p candi-sim --bin live_scan -- --scene scene/candi_scene.xml
```

The ROS 2 path uses `scene/candi_scene.xml` directly through
[`../../ros2/run_demo.sh`](../../ros2/README.md).

Neither path falls back automatically: without the converted mesh, MuJoCo fails
to load the scene. That is deliberate — a silent fallback to different geometry
would make two runs incomparable.

## Preparing it

1. Obtain the model from the Sketchfab page named in [`LICENSE.md`](LICENSE.md),
   under its CC BY 4.0 terms, and place the glTF file at `assets/candi.glb`.
2. Convert it with headless Blender 5.x:

   ```bash
   blender --background --python scripts/convert_glb.py
   ```

   Overrides go after Blender's own `--` separator: `--glb`, `--obj-dir`,
   `--dae`, `--max-tris`, `--target-height`.

3. Check the result before scanning:

   ```bash
   cargo run -p candi-sim --example scene_shot
   ```

   It must report a bounding box of **14.95 × 14.95 × 6.00 m** and an orbit
   radius of **16.48 m**. Different numbers mean the conversion went wrong, not
   the code — the failure modes are listed in
   [`../../docs/runbooks/asset-conversion.md`](../../docs/runbooks/asset-conversion.md).

The conversion writes into `assets/candi_obj/`, which is gitignored: the output
is 34 MB, and it is a derivative of a third-party model rather than something
this repository should redistribute by default.

## If you redistribute

Any of these carry the CC BY 4.0 obligation with them:

- the model itself, converted or not;
- `assets/candi_obj/`, `assets/candi_decimated.glb`;
- renders, recordings and animations of the scene;
- occupancy maps and voxel reconstructions built from it, including `.bt`
  payloads and Rerun recordings.

Keep the attribution from [`LICENSE.md`](LICENSE.md) with them, state that
changes were made, and link to the license. The media in
[`../../media/`](../../media/README.md) that this project already ships is
covered by [`../../media/BOROBUDUR_ATTRIBUTION.md`](../../media/BOROBUDUR_ATTRIBUTION.md).
