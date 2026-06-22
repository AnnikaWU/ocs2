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

#include <cmath>

namespace ocs2 {
namespace collision_nextgen {
namespace impl {

namespace {

inline void computeSpherePairDistanceScalar(const double* worldX, const double* worldY, const double* worldZ,
                                            const std::int64_t* firstObject, const std::int64_t* secondObject,
                                            const double* radiusSum, size_t i, double* distances, double* normalX,
                                            double* normalY, double* normalZ) {
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
    if (centerDistance < kEpsilon) {
      normalX[i] = 1.0;
      normalY[i] = 0.0;
      normalZ[i] = 0.0;
    } else {
      const double invDistance = 1.0 / centerDistance;
      normalX[i] = dx * invDistance;
      normalY[i] = dy * invDistance;
      normalZ[i] = dz * invDistance;
    }
  }
}

void computeSpherePairDistancesImpl(const double* worldX, const double* worldY, const double* worldZ,
                                    const std::int64_t* firstObject, const std::int64_t* secondObject,
                                    const double* radiusSum, size_t numPairs, double* distances, double* normalX,
                                    double* normalY, double* normalZ) {
  using NativePacket = simd::Vector<double, simd::NativeWidth<double>>;

  constexpr double kEpsilonValue = 1e-12;
  const NativePacket epsilon = NativePacket::set(kEpsilonValue);
  const NativePacket one = NativePacket::set(1.0);
  const NativePacket zero = NativePacket::set(0.0);
  const bool computeNormal = normalX != nullptr && normalY != nullptr && normalZ != nullptr;

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
    const NativePacket radiusSumPacket = NativePacket::loadAligned(radiusSum + i);
    const NativePacket signedDistance = centerDistance - radiusSumPacket;
    signedDistance.storeUnaligned(distances + i);

    if (computeNormal) {
      const auto zeroDistanceMask = centerDistance < epsilon;
      const NativePacket safeDistance = simd::max(centerDistance, epsilon);
      const NativePacket invDistance = one / safeDistance;
      simd::select(zeroDistanceMask, one, dx * invDistance).storeAligned(normalX + i);
      simd::select(zeroDistanceMask, zero, dy * invDistance).storeAligned(normalY + i);
      simd::select(zeroDistanceMask, zero, dz * invDistance).storeAligned(normalZ + i);
    }
  }

  for (; i < numPairs; ++i) {
    computeSpherePairDistanceScalar(worldX, worldY, worldZ, firstObject, secondObject, radiusSum, i, distances, normalX, normalY,
                                    normalZ);
  }
}

}  // namespace

void computeSpherePairDistances(const double* worldX, const double* worldY, const double* worldZ, const std::int64_t* firstObject,
                                const std::int64_t* secondObject, const double* radiusSum, size_t numPairs, double* distances,
                                double* normalX, double* normalY, double* normalZ) {
  computeSpherePairDistancesImpl(worldX, worldY, worldZ, firstObject, secondObject, radiusSum, numPairs, distances, normalX, normalY,
                                 normalZ);
}

}  // namespace impl
}  // namespace collision_nextgen
}  // namespace ocs2
