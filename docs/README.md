# Documentation

These documents describe the system as it stands: what was built, how the parts
connect, how to run it, and the numbers that were actually measured.

New here? [`SCANNING_TUTORIAL.md`](SCANNING_TUTORIAL.md) walks the whole
workflow end to end. This index is for people working *on* the project.

---

## Reading order

| # | Document | For |
|---|---|---|
| 0 | [`SCANNING_TUTORIAL.md`](SCANNING_TUTORIAL.md) | Running the scan and understanding its output |
| 1 | [`01-architecture.md`](01-architecture.md) | Anyone arriving — the two paths and why both are maintained |
| 2 | [`02-tech-stack.md`](02-tech-stack.md) | Every dependency, its version and its role |
| 3 | [`03-pipeline.md`](03-pipeline.md) | Asset → scene → flight path → depth → point cloud → map → visualization |
| 4 | [`04-running.md`](04-running.md) | How to run each entry point, on Windows and under WSL |
| 5 | [`05-results.md`](05-results.md) | **Every measured number**, with its methodology |
| 6 | [`06-code-tour.md`](06-code-tour.md) | What each source file actually does |

## Reference material

| Folder | Contents |
|---|---|
| [`decisions/`](decisions/README.md) | 9 ADRs — the decisions that changed the design, with the measurements that justified them |
| [`runbooks/`](runbooks/README.md) | Operational procedures: running on Windows, running under WSL, converting assets, troubleshooting |
| [`../ROADMAP.md`](../ROADMAP.md) | What a next iteration would add |

## One screen

A simulated drone orbits Borobudur inside MuJoCo. At every waypoint it renders a
640×480 depth image, that image is unprojected into a world-frame point cloud,
and the cloud is folded into a probabilistic occupancy map. The end result is a
voxel reconstruction that replays in Rerun and publishes as a standard
`octomap_msgs/Octomap`.

Three properties define the shape of the system:

- **The drone is kinematic, not physics-controlled.** It is a MuJoCo mocap body
  moved along a precomputed path. No rotor dynamics, no controller — the flight
  path is an *input*, not a *result*.
- **The sensor is a depth camera, not a lidar.** The raw measurement is a
  640×480 offscreen render, so the pipeline's cost sits in rendering and
  unprojection rather than in ray tracing.
- **The map is probabilistic, not a pile of points.** Every voxel holds an
  occupancy log-odds, so repeated observations reinforce and contradicting ones
  erase.

The whole runtime path is written in Rust. Python appears once, offline, to
convert the source mesh through Blender.

## Where the library fits

This repository is a reference application for
[`octo_map_rust`](https://github.com/Shannon-Dynamics/octo_map_rust). The
library is used by **both paths** — `live_scan` and `ros2/candi_mapper` — and
both also build this project's own hash grid from the same points. Keeping two
implementations is deliberate: running them on identical input is what
validates each of them ([ADR-0009](decisions/0009-dual-map-comparison.md)).

The quickest way to see the library work is the Quick Demo in the
[README](../README.md#quick-demo): no ROS 2, no assets to prepare.

## What works today

| Capability | State |
|---|---|
| Self-contained demo scene, no assets to prepare | ✅ |
| Simulated depth sensing and point-cloud generation | ✅ |
| `octomap-core` occupancy mapping, both paths | ✅ |
| Independent hash-grid map for cross-validation | ✅ |
| Free-space carving, on by default in the octree | ✅ |
| Rerun recording and PNG projections | ✅ |
| ROS 2 publishing, `octomap_msgs/Octomap` out | ✅ (needs ROS 2 Jazzy) |
| Scanning a user-supplied mesh | ✅ (needs Blender for conversion) |
| Sensor noise model | ⬜ Not implemented |
| SLAM, localization, path planning | ⬜ Out of scope |

What a next iteration would add is in [`../ROADMAP.md`](../ROADMAP.md); the
limitations that stand today are listed in the
[README](../README.md#current-limitations).
