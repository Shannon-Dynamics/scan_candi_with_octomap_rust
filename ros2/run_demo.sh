#!/usr/bin/env bash
#
# The candi scan, over ROS 2.
#
#   source /opt/ros/jazzy/setup.bash
#   ./ros2/run_demo.sh                 # octree carves free space
#   ./ros2/run_demo.sh --no-carve      # endpoints only, matching the hash grid
#
# Starts the mapper first and the publisher second, on purpose: the publisher
# waits for a subscriber before it flies, so this ordering guarantees the whole
# orbit is captured. Started the other way round, a best-effort cloud topic
# would drop the opening frames into nothing and say so nowhere.

set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/.." && pwd)"

# Everything cargo writes goes to D:. The WSL VHDX lives on a full C: drive,
# and when it cannot grow, ext4 remounts read-only mid-build — which surfaces
# as impossible compile errors rather than a disk message. Keeping the writes
# off it avoids the whole failure mode.
# CARGO_HOME is left alone by default. On a WSL install whose VHDX sits on a
# full C: drive, export it (and CARGO_TARGET_DIR) to a Windows drive mount
# before running this — see docs/runbooks/troubleshooting.md.
export CARGO_TARGET_DIR="${CARGO_TARGET_DIR:-$HERE/target}"
export MUJOCO_DOWNLOAD_DIR="${MUJOCO_DOWNLOAD_DIR:-$REPO/.mujoco}"
export LD_LIBRARY_PATH="$MUJOCO_DOWNLOAD_DIR/mujoco-3.9.0/lib:${LD_LIBRARY_PATH:-}"

# ROS writes node logs under ~/.ros, which is inside that same VHDX.
export ROS_HOME="${ROS_HOME:-$REPO/out/ros_home}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-91}"

CARVE_ARG=""
if [[ "${1:-}" == "--no-carve" ]]; then
  CARVE_ARG="--octree-no-carve"
fi

if [[ -z "${ROS_DISTRO:-}" ]]; then
  echo "ROS 2 is not sourced. Run: source /opt/ros/<distro>/setup.bash" >&2
  exit 2
fi

mkdir -p "$REPO/out" "$ROS_HOME"

step() { printf '\n=== %s\n' "$1"; }

step "building both nodes"
cargo build --release --manifest-path "$HERE/Cargo.toml"

MAPPER="$CARGO_TARGET_DIR/release/candi_mapper"
PUBLISHER="$CARGO_TARGET_DIR/release/candi_publisher"

MAPPER_PID=""
cleanup() {
  if [[ -n "$MAPPER_PID" ]]; then
    # SIGINT, not SIGKILL: the mapper writes its summary and flushes the
    # recording on ctrl-c, and killing it outright loses both.
    kill -INT "$MAPPER_PID" 2>/dev/null || true
    wait "$MAPPER_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT

step "starting the mapper"
"$MAPPER" --out "$REPO/out/candi_ros2.rrd" $CARVE_ARG &
MAPPER_PID=$!

# Give it long enough to have its subscription up before the publisher looks.
sleep 3

step "flying the orbit"
"$PUBLISHER"

step "letting the mapper finish the last frames"
sleep 3
cleanup
MAPPER_PID=""

printf '\nWrote %s\n' "$REPO/out/candi_ros2.rrd"
printf 'View it with:  rerun %s\n' "$REPO/out/candi_ros2.rrd"
