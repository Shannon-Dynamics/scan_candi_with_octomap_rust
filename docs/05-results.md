# 5. Validation and measured results

Every number here comes from a run that actually happened, not an estimate. The
raw log is named in each section. Release-build timings were taken **without
LTO** (see [§5.6](#56-methodology-notes)), so they are **upper bounds**.

---

## 5.1 Sensor and flight path

| Quantity | Value |
|---|---|
| Temple bounding box | 14.95 × 14.95 × 6.00 m |
| Temple centre | [0.00, 0.00, 3.00] |
| Bounding sphere | r = 10.99 m |
| Orbit radius | **16.48 m** (1.5 × the bounding sphere) |
| Waypoints | **288** (4 rings × 72 points) |
| Ring heights | [1.57, 3.15, 4.72, 6.30] m |
| Depth camera | 640×480, fovy 60°, subsample 4 |
| Drone bbox (Skydio X2) | 0.55 × 0.64 × 0.22 m |
| Centre-pixel depth | 11.89 m (hits the temple terrace) |
| Nearest depth | 5.20 m (the floor, = 3/tan30° — geometrically correct) |

### Point-cloud quality

| Quantity | Value |
|---|---|
| Total points, full orbit | 2,615,737 |
| Average per frame | **9,082** (below the initial 19k estimate) |
| Floor points | 1,477,445 (56.5%) |
| Structure points | 1,138,292 (43.5%) |
| **Structure points outside the bbox** | **0 (0.00%)** |
| **Highest point** | **6.03 m = 100% of the temple height** |
| Simulation rate | 2.3 ms/frame |

The verification separates floor points from structure points, because the scene
has a ground plane and the camera legitimately sees it — floor points outside the
temple bbox are correct. What is tested is the structure points, and not one of
them strayed.

The 6.03 m highest point closes a real defect: before `max_range`
was raised, the cloud never got above z = 3.06 m although the temple is 6 m
tall. See [ADR-0003](decisions/0003-max-range-20m.md).

---

## 5.2 The map — single-process path (`live_scan`, 288 waypoints)

| Quantity | Value |
|---|---|
| Total time | ~1.0 s (3.4 ms/frame) |
| **Insertion** | **0.3–0.4 ms/frame** (a 100 ms budget at 10 Hz) |
| Occupied voxels | **56,063** at 0.1 m resolution |
| Extent | [−7.45, −7.45, 0.15] .. [7.45, 7.45, 6.05] |
| Recording | `out/candi_scan.rrd`, 23.5 MB, 288 frames, 5 entity paths |

Because the 59 ms figure the plan was built around was never approached, **none
of the three planned workload reductions were needed** (raising the subsample,
lowering the rate, coarsening the resolution).

### The effect of the ground filter

| Quantity | Before the filter | After |
|---|---|---|
| Occupied voxels | 115,259 | **56,063** |
| XY extent | ±15.75 m (the floor) | **±7.45 m (the temple)** |

Without the filter the map is a 31 m blue disc with the temple sunk in the
middle of it. Details in
[ADR-0007](decisions/0007-ground-plane-filter.md).

### Shape verification

An `.rrd` needs a viewer to inspect, so orthographic PNG projections were added
so the reconstruction can be judged directly.

- [`media/img/map_top.png`](../media/img/map_top.png) — the stepped square base,
  **four staircases, one per side**, the circular terraces with their **ring of
  perforated stupas** as concentric dots, and the red main stupa at the centre.
- [`media/img/map_side.png`](../media/img/map_side.png) — the stepped-pyramid
  silhouette topped by the main stupa, across the full blue→red height gradient.

Both read plainly as Borobudur. The reconstruction is accurate, not a blob of
voxels.

Both images are derived from a third-party model under CC BY 4.0 and are not
covered by this repository's Apache-2.0 licence; attribution is in
[`media/BOROBUDUR_ATTRIBUTION.md`](../media/BOROBUDUR_ATTRIBUTION.md).

---

## 5.3 Octree against hash grid — 288 waypoints, release build

This is the measurement most worth reading in the whole project. Two runs, full
orbit, **1,138,259 points** each. Raw logs: `out/measure_endpoints_288.log` and
`out/measure_carved_288.log`.

### Endpoints only — like for like

Both maps do exactly the same work:

| | octree | hash grid |
|---|---:|---:|
| occupied leaves / entries | 53,937 | 56,065 |
| **occupied voxels @ 0.1 m** | **56,065** | **56,065** |
| nodes / entries held | 72,947 | 56,065 |
| `.bt` payload | 38,020 B | — |
| insertion per frame | 0.7 ms | 0.4 ms |

**Both maps agree exactly, on 56,065 voxels.** Two implementations written
separately, given the same points, producing identical numbers — mutual
validation rather than a resemblance. The agreement had already shown up at 24
waypoints (16,384 voxels on both sides), so it is not an artefact of one scale.

The gap between leaves and voxels is the octree's **pruning**: 53,937 leaves
stand for 56,065 base voxels, because one pruned node can represent eight voxels
or eight thousand. A hash grid cannot tell those two numbers apart, which is why
both are reported separately.

### With free-space carving in the octree

| | octree | hash grid |
|---|---:|---:|
| occupied leaves / entries | 38,152 | 56,065 |
| occupied voxels @ 0.1 m | 38,152 | 56,065 |
| nodes / entries held | **1,492,583** | 56,065 |
| `.bt` payload | **479,666 B** | — |
| insertion per frame | **80.4 ms** | 0.5 ms |

`live_scan` turns carving off, with this reasoning:

> Turning it on would trace ~200 voxels per ray and store every empty voxel it
> crosses — for this scene that is millions of entries and hundreds of
> megabytes, to remove nothing.

What was measured: that reasoning is **true of the hash grid, but is not a
property of the mapping method**. A hash grid stores every free voxel
individually; an octree prunes a uniform region into a single node, and mapped
free space is about as uniform as a volume gets. The result: 1.49 million nodes
— far more, certainly — but serializing to **480 KB**, not hundreds of
megabytes. **The cost that made carving unaffordable belonged to the data
structure, not to the method.**

What is not free is time: **80.4 ms per frame against 0.7 ms, roughly 115×**.
Still inside the 100 ms budget at 10 Hz, but without much room — and this is a
figure taken without LTO, so an upper bound.

Carving also drops the occupied-voxel count from 56,065 to 38,152. That is not
lost structure but the feature working: a ray passing through erases a voxel
that one viewpoint saw as occupied while others prove the space empty.

### The 24-waypoint run, debug build

The first numbers taken, recorded so the trail is complete. **Not valid as a
timing comparison** — it is a debug build — but both sides were measured under
the same conditions, so the ratio is indicative even though the magnitudes are
not.

| | octree | hash grid |
|---|---:|---:|
| occupied voxels @ 0.1 m | 16,384 | 16,384 |
| nodes / entries held | 23,962 | 16,384 |
| `.bt` payload | 15,156 B | — |
| insertion per frame | 18.5 ms | 3.9 ms |

---

## 5.4 Fitting the configured budget

The pipeline is configured to publish at 10 Hz, which allows 100 ms per frame.
This table exists so that a later run can be checked against it — a stage that
has moved is a signal to investigate, not a score:

| Stage | Time | Share of the budget |
|---|---:|---:|
| Simulation + render + unprojection | 2.3 ms | 2.3% |
| Hash-grid insertion | 0.4 ms | 0.4% |
| Octree insertion, endpoints only | 0.7 ms | 0.7% |
| Octree insertion, with carving | 80.4 ms | 80.4% |

Carving is the only configuration that comes near the budget, which is why it
is a flag rather than a constant: it is the one setting whose cost a deployment
has to think about.

---

## 5.5 A note on 56,065 vs 56,063

The ROS 2 path reports **56,065** voxels from 1,138,259 points; `live_scan` on
Windows reports **56,063** voxels from 1,138,292 points. Both paths use the same
orbit, camera, `max_range` and ground filter.

That difference — **33 points and 2 voxels** — has not been traced. The most
plausible explanation is floating-point rounding and depth rasterization
differing between OpenGL on Windows and under WSLg. At 0.003% it is small enough
not to change any conclusion, but it is recorded so that nobody assumes the two
numbers were meant to be identical.

---

## 5.6 Methodology notes

**The timings are upper bounds.** The release build was run with
`--config 'profile.release.lto=false'`, because with LTO on `arrow-ipc` never
finished compiling in the time available. `Cargo.toml` still declares
`lto = true`; the flag is passed on the command line so the manifest keeps
stating the intent. LTO was previously recorded as making a substantial
difference to these figures, which has not been re-verified here.

**What is being compared.** The octree/hash-grid comparison runs inside one
process (`candi_mapper`), from one stream of points, in the same run. It is not
two separate runs whose numbers were placed side by side afterwards.

**The unit tests pass** across the workspace, including the regression guards
`a_flat_wall_stays_flat` (fails if depth is treated as range),
`limiter_yields_the_requested_rate` (simulates one second at a 1 ms cadence and
checks that exactly 10 publishes happen), `repeated_misses_clear_a_voxel`, and
`surface_survives_repeated_observation_with_carving`.

**What has not been measured** is listed in the
[README](../README.md#current-limitations).
