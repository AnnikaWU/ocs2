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

#include <ocs2_collision_nextgen/impl/SphereDistanceKernel.h>

#include <ocs2_collision_nextgen/impl/simd/Vector.h>

#include <algorithm>
#include <cassert>
#include <cmath>

namespace ocs2 {
namespace collision_nextgen {
namespace impl {

namespace {

inline void computeSpherePairDistanceScalar(

    // Per-object sphere centers in the world frame.
    const double* worldX, const double* worldY, const double* worldZ,

    // Per-object world-frame vectors from the parent-joint origin to the sphere center.
    const double* objectOffsetX, const double* objectOffsetY, const double* objectOffsetZ,

    // Per-pair sphere-object indices into the arrays above; radiusSum[pair] is r_first + r_second.
    // i selects the pair evaluated by this scalar tail call.
    const std::int64_t* firstObject, const std::int64_t* secondObject, const double* radiusSum, size_t i,

    // Per-pair signed-distance and first-to-second unit-normal outputs.
    double* distances, double* normalX, double* normalY, double* normalZ,

    // Per-pair normal cross objectOffset[firstObject[pair]] angular coefficients.
    double* firstAngularX, double* firstAngularY, double* firstAngularZ,

    // Per-pair normal cross objectOffset[secondObject[pair]] angular coefficients.
    double* secondAngularX, double* secondAngularY, double* secondAngularZ) {
  constexpr double kEpsilon = 1e-12;
  const std::int64_t first = firstObject[i];
  const std::int64_t second = secondObject[i];
  const double dx = worldX[second] - worldX[first];
  const double dy = worldY[second] - worldY[first];
  const double dz = worldZ[second] - worldZ[first];
  const double centerDistance = std::sqrt(dx * dx + dy * dy + dz * dz);
  const double signedDistance = centerDistance - radiusSum[i];
  distances[i] = signedDistance;

  if (normalX != nullptr && normalY != nullptr && normalZ != nullptr) {
    const double invDistance = 1.0 / std::max(centerDistance, kEpsilon);
    const double nx = dx * invDistance;
    const double ny = dy * invDistance;
    const double nz = dz * invDistance;
    normalX[i] = nx;
    normalY[i] = ny;
    normalZ[i] = nz;

    if (firstAngularX != nullptr) {
      const double firstOffsetX = objectOffsetX[first];
      const double firstOffsetY = objectOffsetY[first];
      const double firstOffsetZ = objectOffsetZ[first];
      const double secondOffsetX = objectOffsetX[second];
      const double secondOffsetY = objectOffsetY[second];
      const double secondOffsetZ = objectOffsetZ[second];
      firstAngularX[i] = ny * firstOffsetZ - nz * firstOffsetY;
      firstAngularY[i] = nz * firstOffsetX - nx * firstOffsetZ;
      firstAngularZ[i] = nx * firstOffsetY - ny * firstOffsetX;
      secondAngularX[i] = ny * secondOffsetZ - nz * secondOffsetY;
      secondAngularY[i] = nz * secondOffsetX - nx * secondOffsetZ;
      secondAngularZ[i] = nx * secondOffsetY - ny * secondOffsetX;
    }
  }
}

void computeSpherePairDistancesImpl(

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
    double* secondAngularX, double* secondAngularY, double* secondAngularZ) {
  using NativePacket = simd::Vector<double, simd::NativeWidth<double>>;

  constexpr double kEpsilonValue = 1e-12;
  const NativePacket epsilon = NativePacket::set(kEpsilonValue);
  const NativePacket one = NativePacket::set(1.0);
  const bool computeNormal = normalX != nullptr && normalY != nullptr && normalZ != nullptr;
  const bool computeAngular = firstAngularX != nullptr;

  assert((normalX == nullptr) == (normalY == nullptr));
  assert((normalX == nullptr) == (normalZ == nullptr));
  assert(!computeAngular || computeNormal);
  assert(!computeAngular || (objectOffsetX != nullptr && objectOffsetY != nullptr && objectOffsetZ != nullptr));
  assert((firstAngularX == nullptr) == (firstAngularY == nullptr));
  assert((firstAngularX == nullptr) == (firstAngularZ == nullptr));
  assert((firstAngularX == nullptr) == (secondAngularX == nullptr));
  assert((firstAngularX == nullptr) == (secondAngularY == nullptr));
  assert((firstAngularX == nullptr) == (secondAngularZ == nullptr));

  size_t i = 0;
  for (; i + NativePacket::width <= numPairs; i += NativePacket::width) {
    const NativePacket firstX = NativePacket::gather(worldX, firstObject + i);
    const NativePacket firstY = NativePacket::gather(worldY, firstObject + i);
    const NativePacket firstZ = NativePacket::gather(worldZ, firstObject + i);
    const NativePacket secondX = NativePacket::gather(worldX, secondObject + i);
    const NativePacket secondY = NativePacket::gather(worldY, secondObject + i);
    const NativePacket secondZ = NativePacket::gather(worldZ, secondObject + i);

    const NativePacket dx = secondX - firstX;
    const NativePacket dy = secondY - firstY;
    const NativePacket dz = secondZ - firstZ;
    const NativePacket centerDistance = simd::sqrt(dx * dx + dy * dy + dz * dz);
    const NativePacket radiusSumPacket = NativePacket::loadUnaligned(radiusSum + i);
    const NativePacket signedDistance = centerDistance - radiusSumPacket;
    signedDistance.storeUnaligned(distances + i);

    if (computeNormal) {
      const NativePacket safeDistance = simd::max(centerDistance, epsilon);
      const NativePacket invDistance = one / safeDistance;
      const NativePacket nx = dx * invDistance;
      const NativePacket ny = dy * invDistance;
      const NativePacket nz = dz * invDistance;
      nx.storeUnaligned(normalX + i);
      ny.storeUnaligned(normalY + i);
      nz.storeUnaligned(normalZ + i);

      if (computeAngular) {
        const NativePacket firstOffsetX = NativePacket::gather(objectOffsetX, firstObject + i);
        const NativePacket firstOffsetY = NativePacket::gather(objectOffsetY, firstObject + i);
        const NativePacket firstOffsetZ = NativePacket::gather(objectOffsetZ, firstObject + i);
        const NativePacket secondOffsetX = NativePacket::gather(objectOffsetX, secondObject + i);
        const NativePacket secondOffsetY = NativePacket::gather(objectOffsetY, secondObject + i);
        const NativePacket secondOffsetZ = NativePacket::gather(objectOffsetZ, secondObject + i);
        (ny * firstOffsetZ - nz * firstOffsetY).storeUnaligned(firstAngularX + i);
        (nz * firstOffsetX - nx * firstOffsetZ).storeUnaligned(firstAngularY + i);
        (nx * firstOffsetY - ny * firstOffsetX).storeUnaligned(firstAngularZ + i);
        (ny * secondOffsetZ - nz * secondOffsetY).storeUnaligned(secondAngularX + i);
        (nz * secondOffsetX - nx * secondOffsetZ).storeUnaligned(secondAngularY + i);
        (nx * secondOffsetY - ny * secondOffsetX).storeUnaligned(secondAngularZ + i);
      }
    }
  }

  for (; i < numPairs; ++i) {
    computeSpherePairDistanceScalar(worldX, worldY, worldZ, objectOffsetX, objectOffsetY, objectOffsetZ, firstObject,
                                    secondObject, radiusSum, i, distances, normalX, normalY, normalZ, firstAngularX,
                                    firstAngularY, firstAngularZ, secondAngularX, secondAngularY, secondAngularZ);
  }
}

}  // namespace

void computeSpherePairDistances(

    // Per-object sphere centers in the world frame.
    const double* worldX, const double* worldY, const double* worldZ,

    // Per-pair sphere-object indices and radius sums.
    const std::int64_t* firstObject, const std::int64_t* secondObject, const double* radiusSum, size_t numPairs,

    // Per-pair signed-distance and optional first-to-second unit-normal outputs.
    double* distances, double* normalX, double* normalY, double* normalZ) {
  computeSpherePairDistancesImpl(worldX, worldY, worldZ, nullptr, nullptr, nullptr, firstObject, secondObject, radiusSum,
                                 numPairs, distances, normalX, normalY, normalZ, nullptr, nullptr, nullptr, nullptr,
                                 nullptr, nullptr);
}

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
    double* secondAngularX, double* secondAngularY, double* secondAngularZ) {
  computeSpherePairDistancesImpl(worldX, worldY, worldZ, objectOffsetX, objectOffsetY, objectOffsetZ, firstObject,
                                 secondObject, radiusSum, numPairs, distances, normalX, normalY, normalZ, firstAngularX,
                                 firstAngularY, firstAngularZ, secondAngularX, secondAngularY, secondAngularZ);
}

}  // namespace impl
}  // namespace collision_nextgen
}  // namespace ocs2
