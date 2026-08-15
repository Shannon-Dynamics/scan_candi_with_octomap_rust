#!/usr/bin/env bash
#
# The octree-versus-hash-grid measurement, on release binaries.
#
#   ./ros2/measure.sh <waypoints> [--carve]
#
# Two runs make the comparison:
#
#   ./ros2/measure.sh 288            endpoints only — both maps do the same
#                                    work, so the numbers are like-for-like
#   ./ros2/measure.sh 288 --carve    the octree also traces free space, which
#                                    is what live_scan judged unaffordable
#
# The second is the one worth reading. `live_scan` disabled carving because a
# hash grid stores every empty voxel it crosses; an octree prunes uniform
# regions into single nodes, so the cost that made it unaffordable belongs to
# the data structure rather than to the method.

set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/.." && pwd)"

WAYPOINTS="${1:-288}"
CARVE="--octree-no-carve"
TAG="endpoints"
if [[ "${2:-}" == "--carve" ]]; then
  CARVE=""
  TAG="carved"
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
LOG="$REPO/out/measure_${TAG}_${WAYPOINTS}.log"

MAPPER_PID=""
cleanup() {
  if [[ -n "$MAPPER_PID" ]]; then
    # SIGINT rather than SIGKILL: the summary and the recording flush both
    # happen on the way out.
    kill -INT "$MAPPER_PID" 2>/dev/null || true
    wait "$MAPPER_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT

echo "=== $TAG, $WAYPOINTS waypoints ==="
"$CARGO_TARGET_DIR/release/candi_mapper" \
  --out "$REPO/out/candi_ros2_${TAG}.rrd" $CARVE \
  > "$LOG" 2>&1 &
MAPPER_PID=$!
sleep 4

"$CARGO_TARGET_DIR/release/candi_publisher" --waypoints "$WAYPOINTS" 2>&1 | tail -8

# The mapper integrates behind the publisher; give it room to catch up before
# asking for the summary, or the counts describe a half-finished map.
echo "=== draining ==="
sleep 10
cleanup
MAPPER_PID=""

echo
tail -30 "$LOG"
