# Security policy

## Scope

This repository is a simulation and a demonstration application. It has no
network service, no authentication, and no user data. The realistic security
surface is narrow, and it is worth naming precisely:

- **`crates/candi-octomap-node`** parses `sensor_msgs/PointCloud2` buffers.
  That is the one place code here reads bytes it did not produce, and a
  malformed buffer must produce a `ParseError` rather than a panic or an
  out-of-range read.
- **`ros2/`** subscribes to ROS 2 topics. Anything on the graph can publish to
  them.

The mapping library itself is a separate repository with its own policy:
[`octo_map_rust/SECURITY.md`](https://github.com/Shannon-Dynamics/octo_map_rust/blob/main/SECURITY.md).

## Reporting

Report privately through GitHub's **Report a vulnerability** button under the
[Security tab](https://github.com/Shannon-Dynamics/scan_candi_with_octomap_rust/security).
Please do not open a public issue for something exploitable.

> **Note for the repository owner:** private vulnerability reporting has to be
> enabled in the repository settings for that button to exist. Until it is,
> this section describes a route that is not yet open. No alternative contact
> is published here, because publishing one is the owner's decision to make.

Include the input that triggered it, which entry point you called, what
happened, and the commit and platform.

## What counts

In scope:

- A panic, a hang, or an out-of-range read reachable from a `PointCloud2`
  message — a truncated buffer, a `point_step` of zero, field offsets that do
  not fit the data.
- Unbounded memory growth from a small message.
- Anything in this repository's own code that could produce undefined
  behaviour.

Out of scope:

- Vulnerabilities in MuJoCo, `mujoco-rs`, Rerun, `r2r` or ROS 2 itself. Report
  those upstream. Issues in how this repository *uses* them are in scope.
- Resource use from legitimately large input: a full-resolution scan at a fine
  resolution is expensive by nature.
- The absence of authentication on ROS 2 topics. That is a property of the
  middleware's default configuration, not of this code.
