# Assets

This folder is **empty in the repository**. Its contents are large, machine
generated, and partly not ours to redistribute — so all of it is gitignored and
prepared locally.

| What is needed | Size | Where it comes from |
|---|---|---|
| `candi_obj/candi.obj` + `.mtl` + `candi_basecolor.png` | ~34 MB | Output of `scripts/convert_glb.py` |
| `candi_decimated.glb` | 16 MB | Output of `scripts/convert_glb.py` — for the Rerun overlay |
| `skydio_x2/` | ~2 MB | MuJoCo Menagerie |

`scene/candi_scene.xml` loads `candi_obj/candi.obj` and `skydio_x2/`, so both
have to exist before anything can run.

---

## 1. The temple mesh

You need the source mesh `candi.glb` (Borobudur, supplied by the project owner —
**not distributed in this repository**) and Blender 5.x:

```bash
blender --background --python scripts/convert_glb.py
```

The full procedure, the correct numbers and the list of failure modes are in
[`../docs/runbooks/asset-conversion.md`](../docs/runbooks/asset-conversion.md).

Correct output: **199,999 triangles**, bbox **14.946 × 14.946 × 6.000 m**.

## 2. The Skydio X2 drone model

From MuJoCo Menagerie. A sparse clone, to avoid pulling the whole menagerie:

```bash
git clone --depth 1 --filter=blob:none --sparse \
    https://github.com/google-deepmind/mujoco_menagerie.git /tmp/menagerie
cd /tmp/menagerie && git sparse-checkout set skydio_x2
cp -r /tmp/menagerie/skydio_x2 <repo>/assets/skydio_x2/
```

The model is **Apache-2.0** licensed and its LICENSE file travels inside the
folder.

In `scene/candi_scene.xml` it is used as a **mocap body**: the menagerie
model's freejoint, collision geoms, actuators and sensors are all removed — a
mocap body must have no joints, and the rotors are never run. Its geoms are also
moved to **group 2** so the depth camera can hide them; without that the drone
sees its own rotors at 0.14 m.

## 3. MuJoCo

Nothing to prepare by hand. The `auto-download-mujoco` feature downloads MuJoCo
3.9.0 (19.5 MB) into `MUJOCO_DOWNLOAD_DIR` on the first build.

Worth remembering: `mujoco.dll` (Windows) or `libmujoco.so` (Linux) has to be on
PATH / `LD_LIBRARY_PATH` **at run time**, not only at build time.

---

## Verification

```powershell
cargo run -p candi-sim --example scene_shot
```

It must report a temple bbox of **14.95 × 14.95 × 6.00 m**, centre [0, 0, 3], an
orbit radius of **16.48 m**, and a drone bbox of 0.55 × 0.64 × 0.22 m. If the
numbers differ, the assets are wrong — not the code.
