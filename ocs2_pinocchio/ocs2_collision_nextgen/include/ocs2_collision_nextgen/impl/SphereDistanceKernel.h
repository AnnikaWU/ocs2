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

#include <cstdint>
#include <cstddef>

namespace ocs2 {
namespace collision_nextgen {
namespace impl {

void computeSpherePairDistances(

    // Per-object sphere centers in the world frame.
    const double* worldX, const double* worldY, const double* worldZ,

    // Per-pair sphere-object indices and radius sums.
    const std::int64_t* firstObject, const std::int64_t* secondObject, const double* radiusSum, size_t numPairs,

    // Per-pair signed-distance and optional first-to-second unit-normal outputs.
    double* distances, double* normalX, double* normalY, double* normalZ);

/**
 * Computes signed sphere distances and all coefficients needed by the
 * sphere-pair Jacobian in one pass. The object offsets are world-frame sphere
 * centers relative to their parent-joint origins.
 */
void computeSpherePairEvaluation(

    // Per-object sphere centers in the world frame.
    const double* worldX, const double* worldY, const double* worldZ,

    // Per-object world-frame vectors from the parent-joint origin to the sphere center.
    const double* objectOffsetX, const double* objectOffsetY, const double* objectOffsetZ,

    // Per-pair sphere-object indices into the arrays above; radiusSum[pair] is r_first + r_second.
    const std::int64_t* firstObject, const std::int64_t* secondObject, const double* radiusSum, size_t numPairs,

    // Per-pair signed-distance and first-to-second unit-normal outputs.
    double* distances, double* normalX, double* normalY, double* normalZ,

    // Per-pair normal cross objectOffset[firstObject[pair]] angular coefficients.
    double* firstAngularX, double* firstAngularY, double* firstAngularZ,

    // Per-pair normal cross objectOffset[secondObject[pair]] angular coefficients.
    double* secondAngularX, double* secondAngularY, double* secondAngularZ);

}  // namespace impl
}  // namespace collision_nextgen
}  // namespace ocs2
