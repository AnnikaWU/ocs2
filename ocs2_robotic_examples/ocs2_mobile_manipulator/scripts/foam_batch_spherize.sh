#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  foam_batch_spherize.sh --urdf <robot.urdf> --output-dir <dir> [options]

Options:
  --foam-root <path>       Foam checkout. Default: /home/asterich/code/cpp/foam
  --method <name>          Foam spherization method. Default: medial
  --depth <n>              Sphere-tree depth. Default: 1
  --branch <n>             Target branching factor. Default: 8
  --volume-heuristic-ratio <x>
                           Foam URDF target-sphere scaling. Default: 2.0
  --threads <n>            Foam worker threads. Default: 16
  --skip-mesh-json         Only write the spherized URDF, not per-mesh sphere JSON files.
  --skip-validation        Do not run structural mesh-vs-sphere URDF validation.

The script converts the URDF collision geometry to a sphere-based URDF and,
unless disabled, also exports one sphere JSON file per collision mesh. It is
headless and does not launch any visualizer.
EOF
}

FOAM_ROOT=/home/asterich/code/cpp/foam
METHOD=medial
DEPTH=1
BRANCH=8
VOLUME_HEURISTIC_RATIO=2.0
THREADS=16
SKIP_MESH_JSON=0
SKIP_VALIDATION=0
URDF=
OUTPUT_DIR=

while [[ $# -gt 0 ]]; do
  case "$1" in
    --foam-root) FOAM_ROOT="$2"; shift 2 ;;
    --method) METHOD="$2"; shift 2 ;;
    --depth) DEPTH="$2"; shift 2 ;;
    --branch) BRANCH="$2"; shift 2 ;;
    --volume-heuristic-ratio) VOLUME_HEURISTIC_RATIO="$2"; shift 2 ;;
    --threads) THREADS="$2"; shift 2 ;;
    --urdf) URDF="$2"; shift 2 ;;
    --output-dir) OUTPUT_DIR="$2"; shift 2 ;;
    --skip-mesh-json) SKIP_MESH_JSON=1; shift ;;
    --skip-validation) SKIP_VALIDATION=1; shift ;;
    --help|-h) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

if [[ -z "$URDF" || -z "$OUTPUT_DIR" ]]; then
  usage >&2
  exit 2
fi

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
mkdir -p "$OUTPUT_DIR/meshes"

PYTHON_CMD=(python3)
if [[ -n "${PYTHON:-}" ]]; then
  PYTHON_CMD=("$PYTHON")
elif [[ -x "$FOAM_ROOT/.venv/bin/python" ]] &&
     PYTHONPATH="$FOAM_ROOT${PYTHONPATH:+:$PYTHONPATH}" "$FOAM_ROOT/.venv/bin/python" -c 'import fire, trimesh' >/dev/null 2>&1; then
  PYTHON_CMD=("$FOAM_ROOT/.venv/bin/python")
elif command -v pixi >/dev/null 2>&1 && [[ -f "$FOAM_ROOT/pixi.toml" ]]; then
  PYTHON_CMD=(pixi run python)
fi

SPHERIZED_URDF="$OUTPUT_DIR/$(basename "${URDF%.urdf}")_spheres.urdf"
DATABASE="$OUTPUT_DIR/sphere_database.json"

(
  cd "$FOAM_ROOT"
  PYTHONPATH="$FOAM_ROOT${PYTHONPATH:+:$PYTHONPATH}" "${PYTHON_CMD[@]}" scripts/generate_sphere_urdf.py \
    --filename "$URDF" \
    --output "$SPHERIZED_URDF" \
    --database "$DATABASE" \
    --method "$METHOD" \
    --depth "$DEPTH" \
    --branch "$BRANCH" \
    --volume_heuristic_ratio "$VOLUME_HEURISTIC_RATIO" \
    --threads "$THREADS"
)

if [[ "$SKIP_MESH_JSON" -eq 0 ]]; then
  mapfile -t MESHES < <(
    cd "$FOAM_ROOT"
    PYTHONPATH="$FOAM_ROOT${PYTHONPATH:+:$PYTHONPATH}" "${PYTHON_CMD[@]}" - "$URDF" <<'PY'
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

urdf = Path(sys.argv[1]).resolve()
root = ET.parse(urdf).getroot()
for mesh in root.findall(".//collision/geometry/mesh"):
    filename = mesh.get("filename")
    if not filename:
        continue
    if filename.startswith("package://"):
        print(filename)
    else:
        print((urdf.parent / filename).resolve())
PY
)

  for mesh in "${MESHES[@]}"; do
    if [[ "$mesh" == package://* ]]; then
      echo "Skipping per-mesh JSON for package URI: $mesh" >&2
      continue
    fi
    safe_name=$(basename "${mesh%.*}")
    (
      cd "$FOAM_ROOT"
      PYTHONPATH="$FOAM_ROOT${PYTHONPATH:+:$PYTHONPATH}" "${PYTHON_CMD[@]}" scripts/generate_spheres.py \
        "$mesh" \
        --output "$OUTPUT_DIR/meshes/${safe_name}-spheres.json" \
        --method "$METHOD" \
        --depth "$DEPTH" \
        --branch "$BRANCH"
    )
  done
fi

if [[ "$SKIP_VALIDATION" -eq 0 ]]; then
  "$SCRIPT_DIR/ocs2_validate_sphere_urdf.py" \
    --mesh-urdf "$URDF" \
    --sphere-urdf "$SPHERIZED_URDF"
fi

echo "Spherized URDF: $SPHERIZED_URDF"
echo "Sphere database: $DATABASE"
