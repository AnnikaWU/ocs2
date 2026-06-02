#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  export_moveit_collision_pairs.sh --urdf <robot.urdf> --srdf <robot.srdf> --output <collision_pairs.info> [options]

Options:
  --moveit-ws <path>       MoveIt workspace root. Default: /home/asterich/code/cpp/moveit_ws
  --ocs2-ws <path>         OCS2 workspace root. Default: /home/asterich/code/cpp/ocs2
  --config-pkg <path>      Existing MoveIt config package passed to collisions_updater.
  --trials <n>             Random trials for MoveIt never-colliding detection. Default: 10000
  --min-collision-fraction <f>
                           MoveIt threshold for always-colliding pairs. Default: MoveIt default.
  --xacro-arg <arg>        Additional xacro argument. May be repeated.
  --keep                   Keep disabled collision entries already present in the input SRDF.
  --default                Ask MoveIt to include default-colliding pairs in disabled collisions.
  --always                 Ask MoveIt to include always-colliding pairs in disabled collisions.
  --verbose                Ask MoveIt collisions_updater for verbose output.

This is a non-GUI replacement for the collision part of MoveIt Setup Assistant:
it runs moveit_setup_assistant/collisions_updater and converts the generated
SRDF disabled-collision matrix into OCS2 collisionLinkPairs to check.
EOF
}

MOVEIT_WS=/home/asterich/code/cpp/moveit_ws
OCS2_WS=/home/asterich/code/cpp/ocs2
TRIALS=10000
KEEP=()
DEFAULT=()
ALWAYS=()
VERBOSE=()
MIN_COLLISION_FRACTION=()
XACRO_ARGS=()
CONFIG_PKG=
URDF=
SRDF=
OUTPUT=

while [[ $# -gt 0 ]]; do
  case "$1" in
    --moveit-ws) MOVEIT_WS="$2"; shift 2 ;;
    --ocs2-ws) OCS2_WS="$2"; shift 2 ;;
    --config-pkg) CONFIG_PKG="$2"; shift 2 ;;
    --trials) TRIALS="$2"; shift 2 ;;
    --min-collision-fraction) MIN_COLLISION_FRACTION=(--min-collision-fraction "$2"); shift 2 ;;
    --xacro-arg) XACRO_ARGS+=(--xacro-args "$2"); shift 2 ;;
    --keep) KEEP=(--keep); shift ;;
    --default) DEFAULT=(--default); shift ;;
    --always) ALWAYS=(--always); shift ;;
    --verbose) VERBOSE=(--verbose); shift ;;
    --urdf) URDF="$2"; shift 2 ;;
    --srdf) SRDF="$2"; shift 2 ;;
    --output) OUTPUT="$2"; shift 2 ;;
    --help|-h) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

if [[ -z "$OUTPUT" || -z "$URDF" ]]; then
  usage >&2
  exit 2
fi
if [[ -z "$CONFIG_PKG" && -z "$SRDF" ]]; then
  echo "Either --srdf or --config-pkg is required for MoveIt collisions_updater" >&2
  usage >&2
  exit 2
fi

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
TMP_SRDF=$(mktemp --suffix=.srdf)
trap 'rm -f "$TMP_SRDF"' EXIT
export ROS_LOG_DIR=${ROS_LOG_DIR:-/tmp/ocs2_moveit_logs}
mkdir -p "$ROS_LOG_DIR"

set +u
source /opt/ros/jazzy/setup.bash
source "$OCS2_WS/install/setup.bash"
source "$MOVEIT_WS/install/setup.bash"
set -u

UPDATER_ARGS=(--output "$TMP_SRDF" --trials "$TRIALS")
if [[ -n "$CONFIG_PKG" ]]; then
  UPDATER_ARGS+=(--config-pkg "$CONFIG_PKG")
fi
if [[ -n "$URDF" ]]; then
  UPDATER_ARGS+=(--urdf "$URDF")
fi
if [[ -n "$SRDF" ]]; then
  UPDATER_ARGS+=(--srdf "$SRDF")
fi
UPDATER_ARGS+=(
  "${KEEP[@]}"
  "${DEFAULT[@]}"
  "${ALWAYS[@]}"
  "${VERBOSE[@]}"
  "${MIN_COLLISION_FRACTION[@]}"
  "${XACRO_ARGS[@]}"
)

set +e
ros2 run moveit_setup_assistant collisions_updater "${UPDATER_ARGS[@]}"
UPDATER_STATUS=$?
set -e
if [[ "$UPDATER_STATUS" -ne 0 ]]; then
  if [[ ! -s "$TMP_SRDF" ]]; then
    echo "collisions_updater failed before writing SRDF output" >&2
    exit "$UPDATER_STATUS"
  fi
  echo "collisions_updater exited with status $UPDATER_STATUS after writing SRDF; continuing" >&2
fi

"$SCRIPT_DIR/ocs2_collision_pairs.py" export \
  --urdf "$URDF" \
  --srdf "$TMP_SRDF" \
  --output "$OUTPUT"

"$SCRIPT_DIR/ocs2_collision_pairs.py" validate \
  --urdf "$URDF" \
  --pairs "$OUTPUT"
