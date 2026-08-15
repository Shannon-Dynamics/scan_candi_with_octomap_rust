"""Convert candi.glb into simulation-ready assets.

Headless Blender script. This is the *only* Python in the project and it
runs once, offline — the runtime pipeline stays pure Rust.

Run with:
    blender --background --python scripts/convert_glb.py

Optional overrides (after a `--` separator):
    blender --background --python scripts/convert_glb.py -- --glb path/to/candi.glb

Outputs:
    assets/candi_obj/candi.obj  (+ .mtl + textures)   -> feeds obj2mjcf
    assets/candi.dae                                   -> optional rerun overlay

Scale: the GLB is authored in arbitrary units (~1.9 across), not metres, so
the model is scaled uniformly to TARGET_HEIGHT_M. 6 m is deliberate — the
orbit sits at 1.5x the bounding-sphere radius, which for this model's aspect
ratio puts the drone about 1.5x the height away from the surface. At 6 m that
is ~9 m. Making the candi taller
without also raising max_range would clip every point and yield an empty map.

Axes: glTF is Y-up, MuJoCo is Z-up. Blender's glTF importer performs that
conversion on import, so no manual rotation is applied here.
"""

import argparse
import os
import sys

import bpy
import mathutils

# Triangle budget. Above this the mesh is decimated — MuJoCo's convex/visual
# mesh handling gets slow well before the renderer does.
MAX_TRIS = 200_000

# Target height of the candi in metres, along Z after the Y-up -> Z-up import.
TARGET_HEIGHT_M = 6.0


def project_root() -> str:
    """Repo root, derived from this file's location (scripts/ lives at root)."""
    return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def parse_args() -> argparse.Namespace:
    """Parse args appearing after Blender's own `--` separator."""
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    root = project_root()

    # The source GLB has lived under a couple of names; take the first that
    # exists so the script runs with no arguments.
    candidates = [
        os.path.join(root, "candi.glb"),
        os.path.join(root, "3dCandi", "model.glb"),
    ]
    default_glb = next((p for p in candidates if os.path.isfile(p)), candidates[0])

    parser = argparse.ArgumentParser(prog="convert_glb.py")
    parser.add_argument("--glb", default=default_glb)
    parser.add_argument("--obj-dir", default=os.path.join(root, "assets", "candi_obj"))
    parser.add_argument("--dae", default=os.path.join(root, "assets", "candi.dae"))
    parser.add_argument("--max-tris", type=int, default=MAX_TRIS)
    parser.add_argument("--target-height", type=float, default=TARGET_HEIGHT_M)
    return parser.parse_args(argv)


def clear_scene() -> None:
    """Blender starts with a default cube/camera/light — drop them all."""
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(use_global=False)
    for block in (bpy.data.meshes, bpy.data.materials, bpy.data.images):
        for item in list(block):
            if item.users == 0:
                block.remove(item)


def mesh_objects() -> list:
    return [o for o in bpy.context.scene.objects if o.type == "MESH"]


def triangle_count() -> int:
    """Total triangles across all meshes, after modifiers are accounted for."""
    total = 0
    depsgraph = bpy.context.evaluated_depsgraph_get()
    for obj in mesh_objects():
        evaluated = obj.evaluated_get(depsgraph)
        mesh = evaluated.to_mesh()
        mesh.calc_loop_triangles()
        total += len(mesh.loop_triangles)
        evaluated.to_mesh_clear()
    return total


def world_bounding_box() -> tuple:
    """(min_xyz, max_xyz, dimensions) over every mesh, in world space."""
    lo = [float("inf")] * 3
    hi = [float("-inf")] * 3
    for obj in mesh_objects():
        for corner in obj.bound_box:
            world = obj.matrix_world @ mathutils.Vector(corner)
            for axis in range(3):
                lo[axis] = min(lo[axis], world[axis])
                hi[axis] = max(hi[axis], world[axis])
    dims = [hi[i] - lo[i] for i in range(3)]
    return lo, hi, dims


def count_textures() -> int:
    return len(bpy.data.images)


def scale_to_height(target_height: float) -> float:
    """Uniformly scale every mesh so the Z extent equals target_height.

    Also drops the model onto z=0 and centres it on the XY origin, so the
    orbit planner can assume the candi stands at the world origin.
    Returns the factor applied.
    """
    lo, hi, dims = world_bounding_box()
    if dims[2] <= 0.0:
        sys.exit("[convert_glb] ERROR: model has zero height, cannot scale.")

    factor = target_height / dims[2]
    print(f"[convert_glb] scaling by {factor:.6f} to reach {target_height} m height")

    bpy.ops.object.select_all(action="SELECT")
    bpy.context.view_layer.objects.active = mesh_objects()[0]
    bpy.ops.transform.resize(value=(factor, factor, factor))
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)

    # Re-measure, then centre on XY and sit on the ground plane.
    lo, hi, _ = world_bounding_box()
    offset = (
        -(lo[0] + hi[0]) / 2.0,
        -(lo[1] + hi[1]) / 2.0,
        -lo[2],
    )
    print(f"[convert_glb] recentring by ({offset[0]:.3f}, {offset[1]:.3f}, {offset[2]:.3f})")
    bpy.ops.transform.translate(value=offset)
    bpy.ops.object.transform_apply(location=True, rotation=False, scale=False)

    return factor


def decimate_to_budget(current: int, budget: int) -> None:
    """Apply a Decimate modifier to every mesh so the total lands under budget."""
    ratio = budget / float(current)
    print(f"[convert_glb] decimating with ratio {ratio:.4f}")

    for obj in mesh_objects():
        bpy.context.view_layer.objects.active = obj
        modifier = obj.modifiers.new(name="candi_decimate", type="DECIMATE")
        modifier.ratio = ratio
        bpy.ops.object.modifier_apply(modifier=modifier.name)


def base_color_images() -> list:
    """Every image feeding a material's Base Color, via the Principled BSDF."""
    found = []
    for mat in bpy.data.materials:
        if not mat.use_nodes or mat.node_tree is None:
            continue
        for node in mat.node_tree.nodes:
            if node.type != "BSDF_PRINCIPLED":
                continue
            socket = node.inputs.get("Base Color")
            if socket is None or not socket.is_linked:
                continue
            src = socket.links[0].from_node
            # glTF often routes the texture through a mix/gamma node first.
            if src.type != "TEX_IMAGE":
                for sub in src.inputs:
                    if sub.is_linked and sub.links[0].from_node.type == "TEX_IMAGE":
                        src = sub.links[0].from_node
                        break
            if src.type == "TEX_IMAGE" and src.image is not None:
                found.append((mat.name, src.image))
    return found


def ensure_mtl_texture(obj_path: str) -> None:
    """Guarantee the MTL references a base-color texture.

    Blender's OBJ exporter only emits `map_Kd` for node setups it recognises,
    and the glTF importer's graph does not always qualify — the first run
    produced a flat grey material with no texture at all. Rather than ship an
    untextured candi, save the base-color image next to the OBJ and patch the
    MTL by hand. Purely cosmetic: the depth camera driving the octomap only
    ever sees geometry.
    """
    mtl_path = os.path.splitext(obj_path)[0] + ".mtl"
    if not os.path.isfile(mtl_path):
        print("[convert_glb] WARNING: no MTL was written, skipping texture patch")
        return

    with open(mtl_path, "r", encoding="utf-8") as fh:
        mtl = fh.read()

    if "map_Kd" in mtl:
        print("[convert_glb] MTL already references a texture, no patch needed")
        return

    images = base_color_images()
    if not images:
        print("[convert_glb] WARNING: no base-color texture found; candi stays untextured")
        return

    mat_name, image = images[0]
    out_dir = os.path.dirname(obj_path)
    tex_name = "candi_basecolor.png"
    tex_path = os.path.join(out_dir, tex_name)

    print(f"[convert_glb] MTL has no map_Kd; saving {image.name} ({image.size[0]}x"
          f"{image.size[1]}) from material {mat_name!r} -> {tex_name}")
    image.filepath_raw = tex_path
    image.file_format = "PNG"
    image.save()

    # Neutralise Kd so the texture is not tinted down by the grey diffuse.
    patched = []
    for line in mtl.splitlines():
        patched.append("Kd 1.000000 1.000000 1.000000" if line.startswith("Kd ") else line)
    patched.append(f"map_Kd {tex_name}")

    with open(mtl_path, "w", encoding="utf-8") as fh:
        fh.write("\n".join(patched) + "\n")
    print(f"[convert_glb] patched {mtl_path}")


def export_overlay_mesh(dae_path: str) -> None:
    """Export a mesh for the optional rerun overlay.

    Blender 5.x ships without a working Collada exporter. `hasattr` is useless
    as a probe here — `bpy.ops` resolves attributes lazily, so the operator
    always looks present and only fails when called. Hence the try/except.

    The fallback is a GLB, which is the better overlay source anyway: rerun's
    Asset3D loads it directly and it carries textures inline, unlike Collada.
    """
    try:
        print(f"[convert_glb] exporting Collada -> {dae_path}")
        bpy.ops.wm.collada_export(filepath=dae_path)
        if os.path.isfile(dae_path):
            return
        print("[convert_glb] Collada exporter wrote nothing")
    except (AttributeError, RuntimeError, TypeError) as exc:
        print(f"[convert_glb] Collada export unavailable: {exc}")

    glb_path = os.path.splitext(dae_path)[0] + "_decimated.glb"
    print(f"[convert_glb] exporting GLB instead -> {glb_path}")
    bpy.ops.export_scene.gltf(filepath=glb_path, export_format="GLB")


def main() -> None:
    args = parse_args()

    if not os.path.isfile(args.glb):
        sys.exit(
            f"[convert_glb] ERROR: {args.glb} not found.\n"
            f"[convert_glb] Put candi.glb at the workspace root, or pass "
            f"-- --glb <path>."
        )

    clear_scene()

    print(f"[convert_glb] importing {args.glb}")
    bpy.ops.import_scene.gltf(filepath=args.glb)

    if not mesh_objects():
        sys.exit("[convert_glb] ERROR: the GLB contained no mesh objects.")

    tris = triangle_count()
    textures = count_textures()
    lo, hi, dims = world_bounding_box()

    print("[convert_glb] --- source stats ---")
    print(f"[convert_glb] triangles : {tris}")
    print(f"[convert_glb] textures  : {textures}")
    print(f"[convert_glb] bbox min  : {lo[0]:.3f} {lo[1]:.3f} {lo[2]:.3f} (m)")
    print(f"[convert_glb] bbox max  : {hi[0]:.3f} {hi[1]:.3f} {hi[2]:.3f} (m)")
    print(f"[convert_glb] bbox dims : {dims[0]:.3f} x {dims[1]:.3f} x {dims[2]:.3f} (m)")

    if tris > args.max_tris:
        print(f"[convert_glb] triangles {tris} exceed budget {args.max_tris}")
        decimate_to_budget(tris, args.max_tris)
        after = triangle_count()
        print(f"[convert_glb] triangles before={tris} after={after}")
    else:
        print(f"[convert_glb] triangles within budget ({args.max_tris}), no decimation")

    scale_to_height(args.target_height)

    lo, hi, dims = world_bounding_box()
    print("[convert_glb] --- final stats (metres, Z-up) ---")
    print(f"[convert_glb] bbox min  : {lo[0]:.3f} {lo[1]:.3f} {lo[2]:.3f}")
    print(f"[convert_glb] bbox max  : {hi[0]:.3f} {hi[1]:.3f} {hi[2]:.3f}")
    print(f"[convert_glb] bbox dims : {dims[0]:.3f} x {dims[1]:.3f} x {dims[2]:.3f} (m)")
    radius = 0.5 * (dims[0] ** 2 + dims[1] ** 2 + dims[2] ** 2) ** 0.5
    print(f"[convert_glb] bounding sphere r = {radius:.3f} m")
    print(f"[convert_glb] orbit radius (1.5x) = {1.5 * radius:.3f} m")
    print(f"[convert_glb] approx range to surface = {1.5 * radius - dims[0] / 2:.3f} m")

    os.makedirs(args.obj_dir, exist_ok=True)
    os.makedirs(os.path.dirname(args.dae), exist_ok=True)

    bpy.ops.object.select_all(action="SELECT")

    obj_path = os.path.join(args.obj_dir, "candi.obj")
    print(f"[convert_glb] exporting OBJ -> {obj_path}")
    # Blender 4.x renamed the OBJ exporter; fall back to the 3.x operator.
    # forward/up are NOT optional here. Blender's OBJ exporter defaults to the
    # Y-up convention, which would undo the Y-up -> Z-up conversion the glTF
    # importer just did and hand MuJoCo a temple rotated 90 degrees onto its
    # side. Y-forward / Z-up keeps Blender's world axes as-is.
    if hasattr(bpy.ops.wm, "obj_export"):
        bpy.ops.wm.obj_export(
            filepath=obj_path,
            export_selected_objects=False,
            export_materials=True,
            path_mode="COPY",
            forward_axis="Y",
            up_axis="Z",
        )
    else:
        bpy.ops.export_scene.obj(
            filepath=obj_path,
            use_materials=True,
            path_mode="COPY",
            axis_forward="Y",
            axis_up="Z",
        )

    ensure_mtl_texture(obj_path)
    export_overlay_mesh(args.dae)

    print("[convert_glb] done")
    print(f"[convert_glb] next: obj2mjcf --obj-dir {args.obj_dir} --save-mjcf "
          f"--output-dir assets/candi_mjcf/")


if __name__ == "__main__":
    main()
