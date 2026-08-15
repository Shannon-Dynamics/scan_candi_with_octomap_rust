#!/usr/bin/env bash
#
# A short end-to-end run of the two nodes, for checking the pipeline rather
# than producing a finished map.
#
#   ./ros2/smoke_run.sh [waypoints] [--carve]
#
# Defaults to 24 of the orbit's 288 waypoints and to endpoints-only insertion,
# because a debug build integrating carved rays over the full orbit takes long
# enough to hide whether anything is wrong. `run_demo.sh` is the real run.

set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/.." && pwd)"

WAYPOINTS="${1:-24}"
CARVE="--octree-no-carve"
if [[ "${2:-}" == "--carve" ]]; then
  CARVE=""
fi

# CARGO_HOME is left alone by default. On a WSL install whose VHDX sits on a
# full C: drive, export it (and CARGO_TARGET_DIR) to a Windows drive mount
# before running this — see docs/runbooks/troubleshooting.md.
export CARGO_TARGET_DIR="${CARGO_TARGET_DIR:-$HERE/target}"
export MUJOCO_DOWNLOAD_DIR="${MUJOCO_DOWNLOAD_DIR:-$REPO/.mujoco}"
export LD_LIBRARY_PATH="$MUJOCO_DOWNLOAD_DIR/mujoco-3.9.0/lib:${LD_LIBRARY_PATH:-}"
export ROS_HOME="${ROS_HOME:-$REPO/out/ros_home}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-91}"

mkdir -p "$REPO/out" "$ROS_HOME"

MAPPER_PID=""
cleanup() {
  if [[ -n "$MAPPER_PID" ]]; then
    kill -INT "$MAPPER_PID" 2>/dev/null || true
    wait "$MAPPER_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT

echo "=== building (debug) ==="
cargo build --manifest-path "$HERE/Cargo.toml" 2>&1 | tail -3

echo
echo "=== starting the mapper ==="
"$CARGO_TARGET_DIR/debug/candi_mapper" \
  --out "$REPO/out/candi_ros2.rrd" $CARVE \
  > "$REPO/out/mapper.log" 2>&1 &
MAPPER_PID=$!
sleep 5

echo "=== flying $WAYPOINTS waypoints ==="
"$CARGO_TARGET_DIR/debug/candi_publisher" --waypoints "$WAYPOINTS" 2>&1 | tail -12

echo
echo "=== letting the mapper drain ==="
sleep 6
cleanup
MAPPER_PID=""

echo
echo "=== mapper output ==="
tail -40 "$REPO/out/mapper.log"
