#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 2 ]]; then
  echo "Usage: $0 <ocs2_collision_nextgen-build-dir> <output-dir>" >&2
  exit 2
fi

build_dir=$(realpath "$1")
output_dir=$(realpath -m "$2")
package_dir=$(realpath "$(dirname "${BASH_SOURCE[0]}")/..")
benchmark="${build_dir}/SphereKernelsBenchmark"
distance_object="${build_dir}/CMakeFiles/ocs2_collision_nextgen.dir/src/impl/SphereDistanceKernel.cpp.o"
jacobian_object="${build_dir}/CMakeFiles/ocs2_collision_nextgen.dir/src/impl/SphereJacobianKernel.cpp.o"

for tool in nm objdump readelf rg sha256sum; do
  command -v "${tool}" >/dev/null || {
    echo "Required tool not found: ${tool}" >&2
    exit 2
  }
done

for artifact in "${benchmark}" "${distance_object}" "${jacobian_object}"; do
  [[ -f "${artifact}" ]] || {
    echo "Expected build artifact not found: ${artifact}" >&2
    exit 2
  }
done

mkdir -p "${output_dir}"

sha256sum \
  "${package_dir}/include/ocs2_collision_nextgen/impl/simd/Vector.h" \
  "${package_dir}/src/impl/SphereDistanceKernel.cpp" \
  "${package_dir}/src/impl/SphereJacobianKernel.cpp" \
  "${package_dir}/benchmark/benchmarkSphereKernels.cpp" \
  "${benchmark}" "${distance_object}" "${jacobian_object}" \
  >"${output_dir}/kernel-provenance.sha256"

readelf -n "${benchmark}" >"${output_dir}/SphereKernelsBenchmark.notes.txt"
nm -C "${distance_object}" >"${output_dir}/SphereDistanceKernel.symbols.txt"
nm -C "${jacobian_object}" >"${output_dir}/SphereJacobianKernel.symbols.txt"
objdump -drwC -Mintel "${distance_object}" >"${output_dir}/SphereDistanceKernel.asm"
objdump -drwC -Mintel "${jacobian_object}" >"${output_dir}/SphereJacobianKernel.asm"

if rg -n 'simd::(Vector|Mask)' \
    "${output_dir}/SphereDistanceKernel.symbols.txt" \
    "${output_dir}/SphereJacobianKernel.symbols.txt" \
    >"${output_dir}/out-of-line-simd-wrapper-symbols.txt"; then
  echo "Out-of-line SIMD wrapper symbols found; inspect the evidence output." >&2
  exit 1
fi

rg -n 'vgather' "${output_dir}"/*.asm \
  >"${output_dir}/gather-instructions.txt" || true
rg -ni '(XMMWORD|YMMWORD) PTR \[rsp' "${output_dir}"/*.asm \
  >"${output_dir}/vector-stack-references.txt" || true

echo "Kernel evidence written to ${output_dir}"
