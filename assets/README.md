# Assets

Two kinds of asset live here, under two different licenses.

| Directory | What it is | License | In the repository |
|---|---|---|---|
| [`demo/`](demo/README.md) | A synthetic scene, original to this project | Apache-2.0, like the source code | **Committed** |
| [`borobudur/`](borobudur/README.md) | Attribution for the third-party Borobudur model | Model is CC BY 4.0 | Documentation only — the model is not committed |
| `candi_obj/`, `candi_decimated.glb` | Conversion output from the Borobudur model | CC BY 4.0 derivative | Gitignored — you generate it |
| `skydio_x2/` | The drone model from MuJoCo Menagerie | Apache-2.0 | Gitignored — you fetch it |

**Nothing here is required for the default demo.** `assets/demo/` is committed
and self-contained, so a fresh clone can run the scan immediately. The rest is
for scanning the real temple instead.

---

## 1. The demo scene — committed, nothing to do

[`demo/demo_scene.xml`](demo/README.md) is built from MuJoCo primitives, embeds
no third-party geometry, and is what `live_scan` loads by default.

## 2. The Borobudur mesh — optional, third-party

The model of **Candi Borobudur** is published on Sketchfab under
**CC BY 4.0**. It is *not distributed in this repository*: the license permits
redistribution with attribution, but the model is large and the demo does not
need it, so it stays a local, opt-in step.

Obtain it from the page named in
[`borobudur/LICENSE.md`](borobudur/LICENSE.md), place the glTF at
`assets/candi.glb`, and convert it with Blender 5.x:

```bash
blender --background --python scripts/convert_glb.py
```

Correct output: **199,999 triangles**, bbox **14.946 × 14.946 × 6.000 m**. The
full procedure and its failure modes are in
[`../docs/runbooks/asset-conversion.md`](../docs/runbooks/asset-conversion.md);
what the license requires of you is in
[`borobudur/README.md`](borobudur/README.md).

`scene/candi_scene.xml` loads `candi_obj/candi.obj`, so the conversion has to
have run before that scene will open.

## 3. The Skydio X2 drone model — optional, third-party

Used only by `scene/candi_scene.xml`. From MuJoCo Menagerie, via a sparse
clone so the whole menagerie does not come with it:

```bash
git clone --depth 1 --filter=blob:none --sparse \
    https://github.com/google-deepmind/mujoco_menagerie.git /tmp/menagerie
cd /tmp/menagerie && git sparse-checkout set skydio_x2
cp -r /tmp/menagerie/skydio_x2 <workspace>/scan_candi_with_octomap_rust/assets/skydio_x2/
```

The model is **Apache-2.0** and its LICENSE file travels inside the folder.

In the scene it is a **mocap body**: the menagerie model's freejoint, collision
geoms, actuators and sensors are removed — a mocap body must have no joints,
and the rotors are never run. Its geoms are moved to **group 2** so the depth
camera can hide them; without that the drone sees its own rotors at 0.14 m.

The demo scene uses a plain box instead, which is why it needs no download.

## 4. MuJoCo

Nothing to prepare by hand. The `auto-download-mujoco` feature downloads MuJoCo
3.9.0 (19.5 MB) into `MUJOCO_DOWNLOAD_DIR` on the first build.

Worth remembering: `mujoco.dll` (Windows) or `libmujoco.so` (Linux) has to be on
PATH / `LD_LIBRARY_PATH` **at run time**, not only at build time.

---

## Verification

For the mesh-based scene, after conversion:

```powershell
cargo run -p candi-sim --example scene_shot
```

It must report a temple bbox of **14.95 × 14.95 × 6.00 m**, centre [0, 0, 3], an
orbit radius of **16.48 m**, and a drone bbox of 0.55 × 0.64 × 0.22 m. If the
numbers differ, the assets are wrong — not the code.
