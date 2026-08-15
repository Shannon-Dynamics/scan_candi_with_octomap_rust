# Borobudur 3D Model Attribution

This directory, and every file in this repository derived from it, contains
material based on the following third-party asset:

- **Title:** Candi Borobudur
- **Creator:** `PixForge` — see "Creator attribution" below
- **Source:** Sketchfab
- **Original model:** <https://sketchfab.com/3d-models/candi-borobudur-d42b4ad2e4fd443e838df1d8df1830d0>
- **License:** Creative Commons Attribution 4.0 International (CC BY 4.0)
- **License text:** <https://creativecommons.org/licenses/by/4.0/>

The original model is licensed under CC BY 4.0.

## Changes made

The model is not used as published. For this project it was:

- exported to glTF and re-encoded (`candi-borobudur.zip` → `model.glb`);
- decimated from 2,135,354 triangles to 199,999, to stay inside a triangle
  budget the simulator can render per frame;
- scaled uniformly to 6 m tall, recentred on the origin, and dropped onto
  z = 0, so the flight path can be derived from its bounding box;
- exported to Wavefront OBJ with an explicit Z-up axis convention, with the
  base-colour texture written alongside it;
- assembled into a MuJoCo scene (`scene/candi_scene.xml`) as a static,
  collision-free body.

Everything produced downstream of that — rendered images, recorded videos and
animations, and the occupancy maps reconstructed from simulated depth
observations of it — is a derivative of the original model.

The steps are implemented in [`../../scripts/convert_glb.py`](../../scripts/convert_glb.py)
and described in
[`../../docs/runbooks/asset-conversion.md`](../../docs/runbooks/asset-conversion.md).

## Licensing boundary

**The Borobudur asset and its derivatives remain subject to CC BY 4.0.** They
are **not** covered by the Apache-2.0 license that applies to this
repository's original source code.

If you redistribute the model, a converted form of it, or any of the media
derived from it, CC BY 4.0 requires that you keep the attribution above,
indicate that changes were made, and link to the license.

The repository's own source code, and the original demo scene under
[`../demo/`](../demo/README.md), are Apache-2.0 and carry no such requirement.

No endorsement of this project by the original creator is implied.

## Creator attribution

**The exact Sketchfab creator name is not recorded in any material available
locally.** The downloaded archive is the "source" variant, which ships without
Sketchfab's usual `license.txt`, and both glTF files were re-exported through a
tool that dropped the `asset.copyright` field. Rather than guess at a name,
this file carries the placeholder `PixForge`.

**Before publishing, replace every `PixForge` placeholder in this
repository with the creator name shown on the model page.** CC BY 4.0
attribution is not satisfied until that is done. The occurrences are in this
file, in [`README.md`](README.md), and in
[`../../media/BOROBUDUR_ATTRIBUTION.md`](../../media/BOROBUDUR_ATTRIBUTION.md).
