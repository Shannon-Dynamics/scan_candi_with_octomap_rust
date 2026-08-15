# Runbook — converting the temple mesh

**When to use it:** only when the source mesh (`candi.glb`) changes. The
converted output is not in the repository, so this is also the step run when
setting up a new workspace.

**Prerequisites:** Blender 5.x. A portable build is enough — no installer, no
admin rights.

## Steps

```bash
blender --background --python scripts/convert_glb.py
```

To point it at a different file or change the targets, pass overrides after
Blender's own `--` separator: `--glb`, `--obj-dir`, `--dae`, `--max-tris`,
`--target-height`.

The script does, in order:

1. Import `candi.glb` through the glTF 2.0 importer (which handles the Y-up →
   Z-up conversion)
2. Print statistics: triangle count, textures, bounding box
3. If over 200,000 triangles: apply a Decimate modifier, printing before/after
4. Scale uniformly to 6 m tall, recentre in XY, drop to z=0
5. Export OBJ + MTL + texture into `assets/candi_obj/` with an **explicit
   `forward_axis="Y"`, `up_axis="Z"`**
6. Export a GLB to `assets/candi_decimated.glb`
7. `ensure_mtl_texture()` adds the `map_Kd` line to the MTL

## Verification

Correct output:

| Stage | Value |
|---|---|
| Source triangles | 2,135,354 |
| Decimate ratio | 0.0937 |
| Resulting triangles | **199,999** |
| Scale factor | 7.864386 |
| Recentring | (0.003, −0.011, +3.030) |
| **Final bbox** | **14.946 × 14.946 × 6.000 m** |
| Bounding sphere | r = 10.986 m |

The files produced:

| File | Contents |
|---|---|
| `assets/candi_obj/candi.obj` | 199,999 triangles, Z-up, metres (26.5 MB) |
| `assets/candi_obj/candi.mtl` | Material plus `map_Kd` |
| `assets/candi_obj/candi_basecolor.png` | A 2048×2048 atlas |
| `assets/candi_decimated.glb` | For the Rerun overlay |

The most convincing check is not the script's own output but `scene_shot`:

```powershell
cargo run -p candi-sim --example scene_shot
```

It must report a bbox of **14.95 × 14.95 × 6.00 m** and an orbit radius of
**16.48 m**.

## If it fails

| Symptom | Cause | What to do |
|---|---|---|
| The temple stands like a wall, tipped 90° | Blender's OBJ exporter defaults to **Y-up**, cancelling the glTF importer's conversion | `forward_axis="Y"`, `up_axis="Z"` must be explicit in the export call |
| `bpy.ops.wm.collada_export` does not exist | The Collada exporter was removed in Blender 5.x | The script already checks and falls back to GLB — which is better for the Rerun overlay anyway |
| The material comes out plain grey with no texture | The OBJ exporter does not understand the material graph the glTF importer built | `ensure_mtl_texture()` walks Principled BSDF → Base Color and adds `map_Kd`. Purely cosmetic: the depth camera only sees geometry |
| The Blender installer fails with exit 1603 | The MSI asks for admin elevation from a non-interactive session | Use the portable build (the official zip), extracted into `.tools/` |
| The bbox is reported as 26.73 m | `body_aabb()` used `geom_rbound` (a sphere radius) rather than vertices | Already fixed; if it reappears, check `body_aabb()` in `candi-sim/src/lib.rs` |
| The bbox is reported as 7.56 × 14.43 × 14.95 | `mesh_quat` applied twice | `geom_xmat`/`geom_xpos` already fold it in. Use raw `mesh_vert` and transform once |

## Why 6 m tall

Not an aesthetic choice — it is tied to `max_range`. See
[ADR-0002](../decisions/0002-mesh-scale-6m.md). To make the temple larger,
`max_range` has to be raised first, not the other way round.
