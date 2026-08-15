# Runbook — environment traps

Each of these has a misleading symptom, and none points at its own cause.
Ordered by how much time it costs to diagnose from scratch.

---

## 1. The WSL VHDX cannot grow because the host drive is full

The most expensive failure in this list, and the least self-evident.

The VHDX is *sparse*: it grows as needed. The moment it needs a new block and
the host has no room, the result is an I/O error, and then ext4 does a
`remount-ro` (`errors=remount-ro`).

**The symptoms do not mention disk at all:**

- `df` **inside WSL** reports hundreds of gigabytes free — because what is full
  is the host
- Builds fail with impossible compile errors: **124 unresolved imports inside
  `futures-util`**
- Artefacts written during the failure are corrupt: `Unsupported archive
  identifier` while parsing an rlib, `invalid metadata files`, `only metadata
  stub found for rlib dependency`
- Not one message mentions disk, I/O, or read-only

The symptoms invite the conclusion that WSL itself is unstable. It is not; the
host drive is full.

**The fix:** point every build write at a host drive mount, so the VHDX never
has to grow.

```bash
export CARGO_HOME=/mnt/<drive>/.cargo-wsl
export CARGO_TARGET_DIR=/mnt/<drive>/.cargo-target
export ROS_HOME=/mnt/<drive>/.ros
```

**Recovering afterwards:** `wsl --shutdown`, free space on the host, delete the
corrupted target directory, rebuild. Consider `e2fsck` on the VHDX if the
symptoms persist.

---

## 2. LTO never finishes

With `lto = true`, crates like `arrow-ipc` (a Rerun dependency) never finish
compiling in the time available.

**The fix** — on the command line, **not** by editing `Cargo.toml`, so the
manifest keeps stating the intent:

```bash
cargo build --release --config 'profile.release.lto=false'
```

**The consequence, which must always be stated:** every timing figure in
[`../05-results.md`](../05-results.md) is an **upper bound**.

---

## 3. `set -u` collides with ROS

`/opt/ros/jazzy/setup.bash` reads variables that are not set. A script with
`set -euo pipefail` dies with:

```
AMENT_TRACE_SETUP_FILES: unbound variable
```

**The fix:** source ROS **before** `set -u`, not after. Every script in `ros2/`
already does this.

---

## 4. A nearly full Windows disk corrupts the target directory

Separate from the VHDX problem above, with similar symptoms. A host drive with
under a gigabyte free corrupts the Windows target directory, reporting `invalid
metadata files` and `only metadata stub found for rlib dependency`.

Deleting `target/debug` is usually enough to recover; a completed asset
conversion also makes the Blender install disposable.

**The lesson:** when a compile error looks impossible — a dependency that is
obviously fine suddenly having no contents — check disk space **before**
checking the code.

---

## 5. A broken WSL distro looks like a broken project

A WSL distribution whose `ext4.vhdx` is missing, or which fails to attach its
disk, produces errors during a build that read as though the build is at fault.
Check `wsl --list --verbose` and try a plain shell in the distribution before
diagnosing anything else.

The supported target is **Ubuntu 24.04 with ROS 2 Jazzy**. Jazzy rather than
Humble because it is 24.04's official pairing; `r2r` supports both, so the
choice does not affect the architecture.

---

## 6. Compiling Rerun's `web_viewer` runs out of memory

The `web_viewer` feature pulls in the whole viewer stack (`re_viewer`, wgpu,
egui, `re_redap_client`). Compiling it needs more memory than a 16 GB machine
has, and fails with `handle_alloc_error`.

**The fix:** the official prebuilt viewer (`.tools/rerun/rerun.exe`, 0.35.0) —
the version has to match the SDK **exactly**. No compilation at all.

---

## 7. The Blender installer fails with exit 1603

The MSI asks for admin elevation that cannot be answered from a
non-interactive session.

**The fix:** the portable build (the official zip, 386 MB) extracted into
`.tools/`. The end result is identical, with no admin rights needed.

---

## 8. MuJoCo traps that are not about the environment

Recorded here because their symptoms are equally unhelpful:

| Symptom | Cause |
|---|---|
| A shell of occupied voxels travels with the drone | The depth camera sees the drone's own rotors at 0.14 m. The drone's geoms must be in group 2, hidden from the depth camera |
| The depth scale changes silently after editing geometry | `MjRenderer` caches near/far from `<statistic extent>` at `build()`. Pin the extent explicitly |
| The point cloud bows like a bowl | Depth treated as range rather than z-depth. The `a_flat_wall_stays_flat` guard catches this |
| Background points are not discarded | The MuJoCo background reads ≈ `far` after linearization, not 0. Filter on `>= max_range` |
| `EventLoopError(RecreationAttempt)` | A second `MjRenderer::build()` in one process. A winit event-loop limit, not a MuJoCo one |
| The orbit radius balloons to 34.7 m and the map is empty | `body_aabb()` used `geom_rbound` (a bounding-sphere radius) for a mesh |
| The bbox reads 7.56 × 14.43 × 14.95 | `mesh_quat` applied twice; `geom_xmat`/`geom_xpos` already fold it in |
| The temple lies on its side in MuJoCo | Blender's OBJ exporter defaults to Y-up, cancelling the glTF importer's conversion |
