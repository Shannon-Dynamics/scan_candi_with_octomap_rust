# ADR-0005 — `r2r` as the ROS 2 binding rather than `rclrs`

- **Status:** Accepted

## Context

The plan named `rclrs` (ros2-rust). What turned up while trying to install it:

- `rclrs 0.7` itself only needs ROS 2 sourced — its `build.rs` supports
  "jazzy".
- **But its message crates are not available.** `sensor_msgs` and
  `geometry_msgs` on crates.io are **0.0.0** placeholders.
- So `rclrs` still requires a full `ros2_rust` colcon workspace: vcstool,
  rosdep, colcon-cargo, cargo-ament-build.
- Installing that toolchain needs administrative rights, which are not
  available in every environment this has to build in — a WSL install without
  passwordless sudo among them.

## Decision

`r2r 0.9.5`, which generates message bindings directly from a sourced ROS 2
installation through bindgen. One `cargo build`, no colcon.

## Evidence

The prerequisites were already present on WSL Ubuntu 24.04 / Jazzy: the message
packages exist (`sensor_msgs`, `geometry_msgs`, `tf2_msgs`,
`visualization_msgs`, `std_msgs`, `octomap_msgs`) and libclang is installed.

Two WSL build failures delayed the demonstration; their cause turned out to be
environmental rather than anything to do with `r2r` (see
[`runbooks/troubleshooting.md`](../runbooks/troubleshooting.md)). Once that was
resolved, `r2r` built its bindings in a single `cargo build` and the two nodes
talked over DDS.

## Consequences

- This repository needs **no colcon at all**. There is no `package.xml`, no
  `install/`, and no `setup.bash` of its own.
- `AMENT_PREFIX_PATH` has to be populated at build time, so ROS must be sourced
  first — and that is what forces `ros2/` to be a separate Cargo workspace
  ([ADR-0006](0006-separate-ros2-workspace.md)).
- If `ros2_rust` ever publishes real message crates, this decision is worth
  revisiting; until then `r2r` is the only route that does not require admin
  rights.
