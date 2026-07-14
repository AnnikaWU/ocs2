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

#include <ocs2_collision_nextgen/impl/SphereJacobianKernel.h>
#include <ocs2_collision_nextgen/impl/simd/Vector.h>

#include <algorithm>
#include <cassert>

namespace ocs2 {
namespace collision_nextgen {
namespace impl {

namespace {

struct KernelData {
  const double *jointJacobians;
  size_t numDofs;
  size_t dofStride;
  const std::int64_t *firstJoint;
  const std::int64_t *secondJoint;
  const double *firstAngularX;
  const double *firstAngularY;
  const double *firstAngularZ;
  const double *secondAngularX;
  const double *secondAngularY;
  const double *secondAngularZ;
  const double *normalX;
  const double *normalY;
  const double *normalZ;
  size_t numPairs;
  double *output;
  const SphereJacobianJointRun *jointRuns;
  size_t numJointRuns;
};

inline double jacobianValue(const KernelData &data, size_t joint, size_t row,
                            size_t dof) {
  return data.jointJacobians[(joint * 6 + row) * data.dofStride + dof];
}

[[maybe_unused]] void computeScalar(const KernelData &data) {
  for (size_t dof = 0; dof < data.numDofs; ++dof) {
    double *outputColumn = data.output + dof * data.numPairs;
    for (size_t pair = 0; pair < data.numPairs; ++pair) {
      const size_t first = static_cast<size_t>(data.firstJoint[pair]);
      const size_t second = static_cast<size_t>(data.secondJoint[pair]);
      outputColumn[pair] =
          data.normalX[pair] * (jacobianValue(data, second, 0, dof) -
                                jacobianValue(data, first, 0, dof)) +
          data.normalY[pair] * (jacobianValue(data, second, 1, dof) -
                                jacobianValue(data, first, 1, dof)) +
          data.normalZ[pair] * (jacobianValue(data, second, 2, dof) -
                                jacobianValue(data, first, 2, dof)) -
          data.secondAngularX[pair] * jacobianValue(data, second, 3, dof) -
          data.secondAngularY[pair] * jacobianValue(data, second, 4, dof) -
          data.secondAngularZ[pair] * jacobianValue(data, second, 5, dof) +
          data.firstAngularX[pair] * jacobianValue(data, first, 3, dof) +
          data.firstAngularY[pair] * jacobianValue(data, first, 4, dof) +
          data.firstAngularZ[pair] * jacobianValue(data, first, 5, dof);
    }
  }
}

template <size_t NumDofs>
void computeScalarPairMajor(const KernelData &data) {
  static_assert(NumDofs == 1 || NumDofs == 2,
                "Low-dimensional scalar specialization supports 1 or 2 dofs");
  for (size_t pair = 0; pair < data.numPairs; ++pair) {
    const size_t first = static_cast<size_t>(data.firstJoint[pair]);
    const size_t second = static_cast<size_t>(data.secondJoint[pair]);
    const double *firstRows =
        data.jointJacobians + first * 6 * data.dofStride;
    const double *secondRows =
        data.jointJacobians + second * 6 * data.dofStride;
    const double normalX = data.normalX[pair];
    const double normalY = data.normalY[pair];
    const double normalZ = data.normalZ[pair];
    const double firstAngularX = data.firstAngularX[pair];
    const double firstAngularY = data.firstAngularY[pair];
    const double firstAngularZ = data.firstAngularZ[pair];
    const double secondAngularX = data.secondAngularX[pair];
    const double secondAngularY = data.secondAngularY[pair];
    const double secondAngularZ = data.secondAngularZ[pair];

    for (size_t dof = 0; dof < NumDofs; ++dof) {
      double result = normalX * (secondRows[dof] - firstRows[dof]);
      result += normalY *
                (secondRows[data.dofStride + dof] -
                 firstRows[data.dofStride + dof]);
      result += normalZ *
                (secondRows[2 * data.dofStride + dof] -
                 firstRows[2 * data.dofStride + dof]);
      result -= secondAngularX * secondRows[3 * data.dofStride + dof];
      result -= secondAngularY * secondRows[4 * data.dofStride + dof];
      result -= secondAngularZ * secondRows[5 * data.dofStride + dof];
      result += firstAngularX * firstRows[3 * data.dofStride + dof];
      result += firstAngularY * firstRows[4 * data.dofStride + dof];
      result += firstAngularZ * firstRows[5 * data.dofStride + dof];
      data.output[dof * data.numPairs + pair] = result;
    }
  }
}

#if defined(OCS2_COLLISION_NEXTGEN_USE_AVX2)

using Packet = simd::Vector<double, 4>;

#if defined(_MSC_VER)
#define OCS2_COLLISION_NEXTGEN_FORCE_INLINE __forceinline
#elif defined(__GNUC__) || defined(__clang__)
#define OCS2_COLLISION_NEXTGEN_FORCE_INLINE inline __attribute__((always_inline))
#else
#define OCS2_COLLISION_NEXTGEN_FORCE_INLINE inline
#endif

OCS2_COLLISION_NEXTGEN_FORCE_INLINE Packet
computeDofPacket(const KernelData &data, size_t pair, size_t dof) {
  const size_t first = static_cast<size_t>(data.firstJoint[pair]);
  const size_t second = static_cast<size_t>(data.secondJoint[pair]);
  auto row = [&](size_t joint, size_t spatialRow) {
    return Packet::loadAligned(data.jointJacobians +
                               (joint * 6 + spatialRow) * data.dofStride + dof);
  };
  auto coefficient = [](double value) { return Packet::set(value); };

  Packet result = coefficient(data.normalX[pair]) *
                  (row(second, 0) - row(first, 0));
  result = result + coefficient(data.normalY[pair]) *
                        (row(second, 1) - row(first, 1));
  result = result + coefficient(data.normalZ[pair]) *
                        (row(second, 2) - row(first, 2));
  result = result - coefficient(data.secondAngularX[pair]) * row(second, 3);
  result = result - coefficient(data.secondAngularY[pair]) * row(second, 4);
  result = result - coefficient(data.secondAngularZ[pair]) * row(second, 5);
  result = result + coefficient(data.firstAngularX[pair]) * row(first, 3);
  result = result + coefficient(data.firstAngularY[pair]) * row(first, 4);
  return result + coefficient(data.firstAngularZ[pair]) * row(first, 5);
}

#undef OCS2_COLLISION_NEXTGEN_FORCE_INLINE

template <size_t ActiveDofs>
inline void storeTransposed(Packet row0, Packet row1, Packet row2,
                            Packet row3, size_t pair, size_t dof,
                            const KernelData &data) {
  static_assert(ActiveDofs >= 1 && ActiveDofs <= 4,
                "A transposed packet stores one to four dofs");
  simd::transpose4x4(row0, row1, row2, row3);
  row0.storeUnaligned(data.output + dof * data.numPairs + pair);
  if constexpr (ActiveDofs >= 2) {
    row1.storeUnaligned(data.output + (dof + 1) * data.numPairs + pair);
  }
  if constexpr (ActiveDofs >= 3) {
    row2.storeUnaligned(data.output + (dof + 2) * data.numPairs + pair);
  }
  if constexpr (ActiveDofs == 4) {
    row3.storeUnaligned(data.output + (dof + 3) * data.numPairs + pair);
  }
}

inline void storeTransposedDynamic(Packet row0, Packet row1, Packet row2,
                                   Packet row3, size_t pair, size_t dof,
                                   const KernelData &data) {
  const size_t activeDofs = std::min<size_t>(4, data.numDofs - dof);
  switch (activeDofs) {
  case 4:
    storeTransposed<4>(row0, row1, row2, row3, pair, dof, data);
    break;
  case 3:
    storeTransposed<3>(row0, row1, row2, row3, pair, dof, data);
    break;
  case 2:
    storeTransposed<2>(row0, row1, row2, row3, pair, dof, data);
    break;
  default:
    storeTransposed<1>(row0, row1, row2, row3, pair, dof, data);
    break;
  }
}

void computeAvx2(const KernelData &data) {
  size_t pair = 0;
  for (; pair + 4 <= data.numPairs; pair += 4) {
    for (size_t dof = 0; dof < data.numDofs; dof += 4) {
      storeTransposedDynamic(computeDofPacket(data, pair, dof),
                             computeDofPacket(data, pair + 1, dof),
                             computeDofPacket(data, pair + 2, dof),
                             computeDofPacket(data, pair + 3, dof), pair, dof,
                             data);
    }
  }

  alignas(32) double lanes[4];
  for (; pair < data.numPairs; ++pair) {
    for (size_t dof = 0; dof < data.numDofs; dof += 4) {
      computeDofPacket(data, pair, dof).storeAligned(lanes);
      const size_t activeDofs = std::min<size_t>(4, data.numDofs - dof);
      for (size_t lane = 0; lane < activeDofs; ++lane) {
        data.output[(dof + lane) * data.numPairs + pair] = lanes[lane];
      }
    }
  }
}

void computeAvx2Dof2(const KernelData &data) {
  size_t pair = 0;
  for (; pair + 4 <= data.numPairs; pair += 4) {
    const Packet row0 = computeDofPacket(data, pair, 0);
    const Packet row1 = computeDofPacket(data, pair + 1, 0);
    const Packet row2 = computeDofPacket(data, pair + 2, 0);
    const Packet row3 = computeDofPacket(data, pair + 3, 0);
    const Packet low01 = simd::permute<0, 4, 2, 6>(row0, row1);
    const Packet high01 = simd::permute<1, 5, 3, 7>(row0, row1);
    const Packet low23 = simd::permute<0, 4, 2, 6>(row2, row3);
    const Packet high23 = simd::permute<1, 5, 3, 7>(row2, row3);
    simd::permute<0, 1, 4, 5>(low01, low23).storeUnaligned(data.output + pair);
    simd::permute<0, 1, 4, 5>(high01, high23).storeUnaligned(data.output + data.numPairs + pair);
  }

  alignas(32) double lanes[4];
  for (; pair < data.numPairs; ++pair) {
    computeDofPacket(data, pair, 0).storeAligned(lanes);
    data.output[pair] = lanes[0];
    data.output[data.numPairs + pair] = lanes[1];
  }
}

void computeAvx2Dof7(const KernelData &data) {
  size_t pair = 0;
  for (; pair + 4 <= data.numPairs; pair += 4) {
    storeTransposed<4>(computeDofPacket(data, pair, 0),
                       computeDofPacket(data, pair + 1, 0),
                       computeDofPacket(data, pair + 2, 0),
                       computeDofPacket(data, pair + 3, 0), pair, 0, data);
    storeTransposed<3>(computeDofPacket(data, pair, 4),
                       computeDofPacket(data, pair + 1, 4),
                       computeDofPacket(data, pair + 2, 4),
                       computeDofPacket(data, pair + 3, 4), pair, 4, data);
  }

  alignas(32) double lanes[4];
  for (; pair < data.numPairs; ++pair) {
    computeDofPacket(data, pair, 0).storeAligned(lanes);
    for (size_t lane = 0; lane < 4; ++lane) {
      data.output[lane * data.numPairs + pair] = lanes[lane];
    }
    computeDofPacket(data, pair, 4).storeAligned(lanes);
    for (size_t lane = 0; lane < 3; ++lane) {
      data.output[(lane + 4) * data.numPairs + pair] = lanes[lane];
    }
  }
}

void computeAvx2RunGroupedDof7(const KernelData &data) {
  for (size_t runIndex = 0; runIndex < data.numJointRuns; ++runIndex) {
    const auto &run = data.jointRuns[runIndex];
    const size_t runEnd = run.begin + run.length;
    const double *firstRows =
        data.jointJacobians + run.firstJoint * 6 * data.dofStride;
    const double *secondRows =
        data.jointJacobians + run.secondJoint * 6 * data.dofStride;

    for (size_t dof = 0; dof < 7; ++dof) {
      const double linearX = secondRows[dof] - firstRows[dof];
      const double linearY = secondRows[data.dofStride + dof] -
                             firstRows[data.dofStride + dof];
      const double linearZ = secondRows[2 * data.dofStride + dof] -
                             firstRows[2 * data.dofStride + dof];
      const double firstAngularX = firstRows[3 * data.dofStride + dof];
      const double firstAngularY = firstRows[4 * data.dofStride + dof];
      const double firstAngularZ = firstRows[5 * data.dofStride + dof];
      const double secondAngularX = secondRows[3 * data.dofStride + dof];
      const double secondAngularY = secondRows[4 * data.dofStride + dof];
      const double secondAngularZ = secondRows[5 * data.dofStride + dof];

      const Packet linearXPacket = Packet::set(linearX);
      const Packet linearYPacket = Packet::set(linearY);
      const Packet linearZPacket = Packet::set(linearZ);
      const Packet firstAngularXPacket = Packet::set(firstAngularX);
      const Packet firstAngularYPacket = Packet::set(firstAngularY);
      const Packet firstAngularZPacket = Packet::set(firstAngularZ);
      const Packet secondAngularXPacket = Packet::set(secondAngularX);
      const Packet secondAngularYPacket = Packet::set(secondAngularY);
      const Packet secondAngularZPacket = Packet::set(secondAngularZ);

      double *output = data.output + dof * data.numPairs;
      size_t pair = run.begin;
      for (; pair + 4 <= runEnd; pair += 4) {
        Packet result = Packet::loadUnaligned(data.normalX + pair) * linearXPacket;
        result = result + Packet::loadUnaligned(data.normalY + pair) * linearYPacket;
        result = result + Packet::loadUnaligned(data.normalZ + pair) * linearZPacket;
        result = result - Packet::loadUnaligned(data.secondAngularX + pair) * secondAngularXPacket;
        result = result - Packet::loadUnaligned(data.secondAngularY + pair) * secondAngularYPacket;
        result = result - Packet::loadUnaligned(data.secondAngularZ + pair) * secondAngularZPacket;
        result = result + Packet::loadUnaligned(data.firstAngularX + pair) * firstAngularXPacket;
        result = result + Packet::loadUnaligned(data.firstAngularY + pair) * firstAngularYPacket;
        result = result + Packet::loadUnaligned(data.firstAngularZ + pair) * firstAngularZPacket;
        result.storeUnaligned(output + pair);
      }

      for (; pair < runEnd; ++pair) {
        output[pair] = data.normalX[pair] * linearX +
                       data.normalY[pair] * linearY +
                       data.normalZ[pair] * linearZ -
                       data.secondAngularX[pair] * secondAngularX -
                       data.secondAngularY[pair] * secondAngularY -
                       data.secondAngularZ[pair] * secondAngularZ +
                       data.firstAngularX[pair] * firstAngularX +
                       data.firstAngularY[pair] * firstAngularY +
                       data.firstAngularZ[pair] * firstAngularZ;
      }
    }
  }
}

#endif

} // namespace

void computeSpherePairJacobians(
    const double *jointJacobians, size_t numJoints, size_t numDofs,
    size_t jointJacobianDofStride, const std::int64_t *firstJoint,
    const std::int64_t *secondJoint, const double *firstAngularX,
    const double *firstAngularY, const double *firstAngularZ,
    const double *secondAngularX, const double *secondAngularY,
    const double *secondAngularZ, const double *normalX, const double *normalY,
    const double *normalZ, size_t numPairs, double *output,
    const SphereJacobianJointRun *jointRuns, size_t numJointRuns) {
  assert(reinterpret_cast<std::uintptr_t>(jointJacobians) % 32 == 0);
  assert(jointJacobianDofStride >= numDofs && jointJacobianDofStride % 4 == 0);
#ifndef NDEBUG
  if (numPairs > 0) {
    assert(firstJoint != nullptr && secondJoint != nullptr);
    for (size_t pair = 0; pair < numPairs; ++pair) {
      assert(firstJoint[pair] >= 0);
      assert(secondJoint[pair] >= 0);
      assert(static_cast<size_t>(firstJoint[pair]) < numJoints);
      assert(static_cast<size_t>(secondJoint[pair]) < numJoints);
    }
  }
  if (numJointRuns > 0) {
    assert(jointRuns != nullptr);
    size_t nextPair = 0;
    for (size_t runIndex = 0; runIndex < numJointRuns; ++runIndex) {
      const auto &run = jointRuns[runIndex];
      assert(run.begin == nextPair);
      assert(run.begin <= numPairs);
      assert(run.length > 0 && run.length <= numPairs - run.begin);
      assert(run.firstJoint < numJoints && run.secondJoint < numJoints);
      const size_t runEnd = run.begin + run.length;
      for (size_t pair = run.begin; pair < runEnd; ++pair) {
        assert(firstJoint[pair] == static_cast<std::int64_t>(run.firstJoint));
        assert(secondJoint[pair] ==
               static_cast<std::int64_t>(run.secondJoint));
      }
      nextPair = runEnd;
    }
    assert(nextPair == numPairs);
  }
#endif
  (void)numJoints;
  const KernelData data{jointJacobians, numDofs,        jointJacobianDofStride,
                        firstJoint,     secondJoint,    firstAngularX,
                        firstAngularY,  firstAngularZ,  secondAngularX,
                        secondAngularY, secondAngularZ, normalX,
                        normalY,        normalZ,        numPairs,
                        output,         jointRuns,      numJointRuns};
  if (numDofs == 1) {
    computeScalarPairMajor<1>(data);
    return;
  }
#if defined(OCS2_COLLISION_NEXTGEN_USE_AVX2)
  if (numDofs == 2) {
    computeAvx2Dof2(data);
    return;
  }
  const bool useRunGroupedDof7 =
      numDofs == 7 && jointJacobianDofStride == 8 && jointRuns != nullptr &&
      numJointRuns > 0 && numJointRuns <= numPairs / 4;
  if (useRunGroupedDof7) {
    computeAvx2RunGroupedDof7(data);
  } else if (numDofs == 7 && jointJacobianDofStride == 8) {
    computeAvx2Dof7(data);
  } else {
    computeAvx2(data);
  }
#else
  if (numDofs == 2) {
    computeScalarPairMajor<2>(data);
  } else {
    computeScalar(data);
  }
#endif
}

} // namespace impl
} // namespace collision_nextgen
} // namespace ocs2
