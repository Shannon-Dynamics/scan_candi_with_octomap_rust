#!/usr/bin/env bash
#
# Build everything the ROS 2 demo needs, in one detached run.
#
# Split out of the tooling that invokes it because this takes longer than most
# command timeouts allow, and a build killed halfway leaves a log that ends
# mid-compile with no error in it — which reads exactly like a crash. Launched
# with `setsid nohup`, it outlives whatever started it and the log is the
# progress report.
#
#   setsid nohup ./ros2/build_all.sh > out/build.log 2>&1 &
#   tail -f out/build.log

set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/.." && pwd)"

# CARGO_HOME is left alone by default. On a WSL install whose VHDX sits on a
# full C: drive, export it (and CARGO_TARGET_DIR) to a Windows drive mount
# before running this — see docs/runbooks/troubleshooting.md.
export MUJOCO_DOWNLOAD_DIR="${MUJOCO_DOWNLOAD_DIR:-$REPO/.mujoco}"

stamp() { date +%H:%M:%S; }
step() { printf '\n=== [%s] %s\n' "$(stamp)" "$1"; }

step "candi-sim (MuJoCo + Rerun), the existing single-process scan"
CARGO_TARGET_DIR="$REPO/target-wsl" \
  cargo build --release --manifest-path "$REPO/Cargo.toml" -p candi-sim --bin live_scan

step "the two ROS 2 nodes"
CARGO_TARGET_DIR="$HERE/target" \
  cargo build --release --manifest-path "$HERE/Cargo.toml"

step "unit tests"
CARGO_TARGET_DIR="$HERE/target" \
  cargo test --release --manifest-path "$HERE/Cargo.toml"

step "done"
# Free space where the build actually wrote, which is the thing that runs out
# first on a WSL install — see docs/runbooks/troubleshooting.md.
df -h "$REPO" | tail -1
