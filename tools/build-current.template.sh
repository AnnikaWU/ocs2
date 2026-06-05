#!/usr/bin/env bash

set -euo pipefail

# Template build helper for machines that do not share the local absolute paths.
# Expected layout by default:
#
#   <workspace>/
#     build/
#     install/
#     src/ocs2/
#
# Override any path below with the corresponding environment variable.

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "${script_dir}/.." && pwd)"
default_workspace_root="$(cd "${repo_root}/../.." && pwd)"

workspace_root="${OCS2_WORKSPACE_ROOT:-${default_workspace_root}}"
build_base="${OCS2_BUILD_BASE:-${workspace_root}/build}"
install_base="${OCS2_INSTALL_BASE:-${workspace_root}/install}"

ros_distro="${ROS_DISTRO:-jazzy}"
ros_setup="${OCS2_ROS_SETUP:-/opt/ros/${ros_distro}/setup.bash}"
openrobots_prefix="${OPENROBOTS_PREFIX:-/opt/openrobots}"

cmake_build_type="${CMAKE_BUILD_TYPE:-RelWithDebInfo}"
python3_executable="${Python3_EXECUTABLE:-${workspace_root}/.venv/bin/python3}"
pinocchio_dir="${PINOCCHIO_DIR:-${openrobots_prefix}/lib/cmake/pinocchio}"
hpp_fcl_dir="${HPP_FCL_DIR:-${openrobots_prefix}/lib/cmake/hpp-fcl}"
coal_dir="${COAL_DIR:-${openrobots_prefix}/lib/cmake/coal}"
parallel_workers="${COLCON_PARALLEL_WORKERS:-1}"
build_jobs="${CMAKE_BUILD_PARALLEL_LEVEL:-1}"
memory_limit_gb="${OCS2_BUILD_MEMORY_LIMIT_GB:-}"

if [[ ! -d "${workspace_root}" ]]; then
  echo "Workspace root not found: ${workspace_root}" >&2
  echo "Set OCS2_WORKSPACE_ROOT to your colcon workspace root." >&2
  exit 1
fi

if [[ ! -f "${ros_setup}" ]]; then
  echo "ROS setup not found: ${ros_setup}" >&2
  echo "Set ROS_DISTRO or OCS2_ROS_SETUP." >&2
  exit 1
fi

if [[ ! -x "${python3_executable}" ]]; then
  if command -v python3 >/dev/null 2>&1; then
    python3_executable="$(command -v python3)"
  else
    echo "Python3 not found. Set Python3_EXECUTABLE." >&2
    exit 1
  fi
fi

for cmake_package_dir in "${pinocchio_dir}" "${hpp_fcl_dir}" "${coal_dir}"; do
  if [[ ! -d "${cmake_package_dir}" ]]; then
    echo "Required CMake package directory not found: ${cmake_package_dir}" >&2
    echo "Set OPENROBOTS_PREFIX, PINOCCHIO_DIR, HPP_FCL_DIR, or COAL_DIR." >&2
    exit 1
  fi
done

# shellcheck disable=SC1090
set +u
source "${ros_setup}"
set -u

export CMAKE_PREFIX_PATH="${openrobots_prefix}:${CMAKE_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="${openrobots_prefix}/lib:${LD_LIBRARY_PATH:-}"
export CMAKE_BUILD_PARALLEL_LEVEL="${build_jobs}"
export MAKEFLAGS="-j${build_jobs} ${MAKEFLAGS:-}"
export NINJAFLAGS="-j${build_jobs} ${NINJAFLAGS:-}"

if [[ -d "${workspace_root}/.venv" ]]; then
  export VIRTUAL_ENV="${workspace_root}/.venv"
  export PATH="${VIRTUAL_ENV}/bin:${PATH}"
fi

if [[ -n "${memory_limit_gb}" ]]; then
  if [[ ! "${memory_limit_gb}" =~ ^[0-9]+$ ]]; then
    echo "OCS2_BUILD_MEMORY_LIMIT_GB must be an integer number of GB, got: ${memory_limit_gb}" >&2
    exit 1
  fi
  ulimit -v $((memory_limit_gb * 1024 * 1024))
fi

cmd=(
  colcon
  build
  --build-base "${build_base}"
  --install-base "${install_base}"
  --parallel-workers "${parallel_workers}"
  --cmake-args
    "-DCMAKE_BUILD_TYPE=${cmake_build_type}"
    "-DPython3_EXECUTABLE=${python3_executable}"
    "-Dpinocchio_DIR=${pinocchio_dir}"
    "-Dhpp-fcl_DIR=${hpp_fcl_dir}"
    "-Dcoal_DIR=${coal_dir}"
)

if [[ $# -gt 0 ]]; then
  cmd+=(--packages-select "$@")
fi

echo "Workspace root: ${workspace_root}"
echo "Build base:     ${build_base}"
echo "Install base:   ${install_base}"
echo "Build type:     ${cmake_build_type}"
echo "ROS setup:      ${ros_setup}"
echo "Python3:        ${python3_executable}"
echo "OpenRobots:     ${openrobots_prefix}"
echo "Pinocchio:      ${pinocchio_dir}"
echo "hpp-fcl:        ${hpp_fcl_dir}"
echo "coal:           ${coal_dir}"
echo "Package jobs:   ${parallel_workers}"
echo "Build jobs:     ${build_jobs}"
if [[ -n "${memory_limit_gb}" ]]; then
  echo "Memory limit:   ${memory_limit_gb} GB virtual memory per process tree"
else
  echo "Memory limit:   none"
fi
if [[ $# -gt 0 ]]; then
  echo "Packages:       $*"
else
  echo "Packages:       all"
fi

cd "${workspace_root}"
"${cmd[@]}"

echo
echo "Build completed."
echo "To use the result:"
echo "  source ${install_base}/setup.bash"
