/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#pragma once

#include <cstddef>
#include <cstdint>

namespace ocs2 {
namespace collision_nextgen {
namespace impl {

struct SphereJacobianJointRun {
  size_t begin;
  size_t length;
  size_t firstJoint;
  size_t secondJoint;
};

/**
 * Computes distance Jacobians for sphere pairs.
 *
 * jointJacobians uses [joint][spatial row][padded dof] layout. The base pointer
 * must be 32-byte aligned and jointJacobianDofStride must be a multiple of
 * four. output is column-major [pair][dof], matching Eigen's default dynamic
 * matrix layout. firstJoint and secondJoint must contain nonnegative indices
 * below numJoints, and output must hold numPairs * numDofs values. Optional
 * jointRuns must form a contiguous partition of the input pair order and agree
 * with firstJoint/secondJoint. They enable reuse without reordering the output;
 * callers may omit them to use the generic dispatch.
 */
void computeSpherePairJacobians(
    const double *jointJacobians, size_t numJoints, size_t numDofs,
    size_t jointJacobianDofStride, const std::int64_t *firstJoint,
    const std::int64_t *secondJoint, const double *firstAngularX,
    const double *firstAngularY, const double *firstAngularZ,
    const double *secondAngularX, const double *secondAngularY,
    const double *secondAngularZ, const double *normalX, const double *normalY,
    const double *normalZ, size_t numPairs, double *output,
    const SphereJacobianJointRun *jointRuns = nullptr,
    size_t numJointRuns = 0);

inline size_t sphereJacobianDofStride(size_t numDofs) {
  constexpr size_t kPacketWidth = 4;
  return (numDofs + kPacketWidth - 1) & ~(kPacketWidth - 1);
}

} // namespace impl
} // namespace collision_nextgen
} // namespace ocs2
