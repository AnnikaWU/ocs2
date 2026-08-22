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

#include <gtest/gtest.h>

#include <ocs2_collision_nextgen/impl/SphereCollisionModel.h>
#include <ocs2_collision_nextgen/impl/SphereDistanceKernel.h>
#include <ocs2_collision_nextgen/impl/SphereJacobianKernel.h>
#include <ocs2_collision_nextgen/impl/simd/Vector.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

using namespace ocs2::collision_nextgen::impl;

TEST(SphereDistanceKernel, HandlesDegenerateNormalsAndScalarTail) {
  constexpr size_t width = simd::NativeWidth<double>;
  const size_t numPairs = std::max(width + 1, size_t{13});
  const size_t numObjects = 2 * numPairs;
  AlignedVector<double> worldX(numObjects, 0.0);
  AlignedVector<double> worldY(numObjects, 0.0);
  AlignedVector<double> worldZ(numObjects, 0.0);
  AlignedVector<std::int64_t> first(numPairs);
  AlignedVector<std::int64_t> second(numPairs);
  AlignedVector<double> radius(numPairs, 0.0);
  AlignedVector<double> distance(numPairs);
  AlignedVector<double> normalX(numPairs);
  AlignedVector<double> normalY(numPairs);
  AlignedVector<double> normalZ(numPairs);

  for (size_t pair = 0; pair < numPairs; ++pair) {
    first[pair] = static_cast<std::int64_t>(2 * pair);
    second[pair] = static_cast<std::int64_t>(2 * pair + 1);
  }
  worldX[1] = 3.0;
  worldY[1] = 4.0;
  radius[0] = 1.0;
  if (numPairs > 2) {
    worldX[5] = 0.5e-12;
  }
  if (numPairs > 3) {
    worldX[7] = 2.0e-12;
  }
  worldX[2 * (numPairs - 1) + 1] = -1.0;

  computeSpherePairDistances(worldX.data(), worldY.data(), worldZ.data(),
                             first.data(), second.data(), radius.data(),
                             numPairs, distance.data(), normalX.data(),
                             normalY.data(), normalZ.data());

  EXPECT_DOUBLE_EQ(distance[0], 4.0);
  EXPECT_NEAR(normalX[0], 0.6, 1e-15);
  EXPECT_NEAR(normalY[0], 0.8, 1e-15);
  EXPECT_DOUBLE_EQ(normalZ[0], 0.0);
  EXPECT_DOUBLE_EQ(distance[1], 0.0);
  EXPECT_DOUBLE_EQ(normalX[1], 0.0);
  EXPECT_DOUBLE_EQ(normalY[1], 0.0);
  EXPECT_DOUBLE_EQ(normalZ[1], 0.0);
  if (numPairs > 2) {
    EXPECT_DOUBLE_EQ(normalX[2], 0.5);
  }
  if (numPairs > 3) {
    EXPECT_DOUBLE_EQ(normalX[3], 1.0);
  }
  EXPECT_DOUBLE_EQ(normalX[numPairs - 1], -1.0);
  for (size_t pair = 0; pair < numPairs; ++pair) {
    EXPECT_TRUE(std::isfinite(distance[pair]));
    EXPECT_TRUE(std::isfinite(normalX[pair]));
    EXPECT_TRUE(std::isfinite(normalY[pair]));
    EXPECT_TRUE(std::isfinite(normalZ[pair]));
  }

  std::fill(distance.begin(), distance.end(), 0.0);
  computeSpherePairDistances(
      worldX.data(), worldY.data(), worldZ.data(), first.data(), second.data(),
      radius.data(), numPairs, distance.data(), nullptr, nullptr, nullptr);
  EXPECT_DOUBLE_EQ(distance[0], 4.0);
  EXPECT_DOUBLE_EQ(distance[numPairs - 1], 1.0);
}

TEST(SphereDistanceKernel, FusedEvaluationMatchesDistancePathAndAngularReference) {
  constexpr size_t numPairs = 13;
  constexpr size_t numObjects = 2 * numPairs;
  AlignedVector<double> worldX(numObjects);
  AlignedVector<double> worldY(numObjects);
  AlignedVector<double> worldZ(numObjects);
  AlignedVector<double> offsetX(numObjects);
  AlignedVector<double> offsetY(numObjects);
  AlignedVector<double> offsetZ(numObjects);
  AlignedVector<std::int64_t> first(numPairs);
  AlignedVector<std::int64_t> second(numPairs);
  AlignedVector<double> radius(numPairs);
  for (size_t object = 0; object < numObjects; ++object) {
    const double value = static_cast<double>(object + 1);
    worldX[object] = 0.17 * value;
    worldY[object] = -0.11 * value + 0.02 * static_cast<double>(object % 3);
    worldZ[object] = 0.07 * value - 0.01 * static_cast<double>(object % 5);
    offsetX[object] = 0.013 * value;
    offsetY[object] = -0.019 * value;
    offsetZ[object] = 0.023 * value;
  }
  for (size_t pair = 0; pair < numPairs; ++pair) {
    first[pair] = static_cast<std::int64_t>(2 * pair);
    second[pair] = static_cast<std::int64_t>(2 * pair + 1);
    radius[pair] = 0.01 * static_cast<double>(pair + 1);
  }
  // Exercise the epsilon normal and the scalar packet tail.
  worldX[7] = worldX[6];
  worldY[7] = worldY[6];
  worldZ[7] = worldZ[6];

  AlignedVector<double> distances(numPairs), normalX(numPairs), normalY(numPairs), normalZ(numPairs);
  AlignedVector<double> firstAngularX(numPairs), firstAngularY(numPairs), firstAngularZ(numPairs);
  AlignedVector<double> secondAngularX(numPairs), secondAngularY(numPairs), secondAngularZ(numPairs);
  computeSpherePairEvaluation(
      worldX.data(), worldY.data(), worldZ.data(), offsetX.data(), offsetY.data(), offsetZ.data(), first.data(),
      second.data(), radius.data(), numPairs, distances.data(), normalX.data(), normalY.data(), normalZ.data(),
      firstAngularX.data(), firstAngularY.data(), firstAngularZ.data(), secondAngularX.data(), secondAngularY.data(),
      secondAngularZ.data());

  AlignedVector<double> separateDistances(numPairs), separateNormalX(numPairs), separateNormalY(numPairs),
      separateNormalZ(numPairs);
  computeSpherePairDistances(worldX.data(), worldY.data(), worldZ.data(), first.data(), second.data(), radius.data(),
                             numPairs, separateDistances.data(), separateNormalX.data(), separateNormalY.data(),
                             separateNormalZ.data());

  for (size_t pair = 0; pair < numPairs; ++pair) {
    EXPECT_DOUBLE_EQ(distances[pair], separateDistances[pair]);
    EXPECT_DOUBLE_EQ(normalX[pair], separateNormalX[pair]);
    EXPECT_DOUBLE_EQ(normalY[pair], separateNormalY[pair]);
    EXPECT_DOUBLE_EQ(normalZ[pair], separateNormalZ[pair]);
    const auto firstObject = static_cast<size_t>(first[pair]);
    const auto secondObject = static_cast<size_t>(second[pair]);
    EXPECT_DOUBLE_EQ(firstAngularX[pair],
                     normalY[pair] * offsetZ[firstObject] - normalZ[pair] * offsetY[firstObject]);
    EXPECT_DOUBLE_EQ(firstAngularY[pair],
                     normalZ[pair] * offsetX[firstObject] - normalX[pair] * offsetZ[firstObject]);
    EXPECT_DOUBLE_EQ(firstAngularZ[pair],
                     normalX[pair] * offsetY[firstObject] - normalY[pair] * offsetX[firstObject]);
    EXPECT_DOUBLE_EQ(secondAngularX[pair],
                     normalY[pair] * offsetZ[secondObject] - normalZ[pair] * offsetY[secondObject]);
    EXPECT_DOUBLE_EQ(secondAngularY[pair],
                     normalZ[pair] * offsetX[secondObject] - normalX[pair] * offsetZ[secondObject]);
    EXPECT_DOUBLE_EQ(secondAngularZ[pair],
                     normalX[pair] * offsetY[secondObject] - normalY[pair] * offsetX[secondObject]);
  }
}

TEST(SphereJacobianKernel, MatchesScalarReferenceWithPacketAndTail) {
  constexpr size_t width = simd::NativeWidth<double>;
  const size_t numPairs = std::max(width + 1, size_t{13});
  constexpr size_t numJoints = 4;
  for (const size_t numDofs :
       {size_t{1}, size_t{2}, size_t{3}, size_t{5}, size_t{6}, size_t{7},
        size_t{8}, size_t{11}}) {
    const size_t dofStride = sphereJacobianDofStride(numDofs);
    AlignedVector<double> jacobians(6 * dofStride * numJoints, 0.0);
    for (size_t row = 0; row < 6; ++row) {
      for (size_t dof = 0; dof < numDofs; ++dof) {
        for (size_t joint = 0; joint < numJoints; ++joint) {
          jacobians[(joint * 6 + row) * dofStride + dof] =
              0.1 * static_cast<double>(row + 1) +
              0.01 * static_cast<double>(dof) +
              0.001 * static_cast<double>(joint);
        }
      }
    }

    AlignedVector<std::int64_t> firstJoint(numPairs);
    AlignedVector<std::int64_t> secondJoint(numPairs);
    AlignedVector<double> firstX(numPairs), firstY(numPairs), firstZ(numPairs);
    AlignedVector<double> secondX(numPairs), secondY(numPairs),
        secondZ(numPairs);
    AlignedVector<double> firstAngularX(numPairs), firstAngularY(numPairs),
        firstAngularZ(numPairs);
    AlignedVector<double> secondAngularX(numPairs), secondAngularY(numPairs),
        secondAngularZ(numPairs);
    AlignedVector<double> normalX(numPairs), normalY(numPairs),
        normalZ(numPairs);
    AlignedVector<double> output(numPairs * numDofs);
    for (size_t pair = 0; pair < numPairs; ++pair) {
      firstJoint[pair] = pair < 8 ? 0 : 2;
      secondJoint[pair] = pair < 8 ? 1 : 3;
      firstX[pair] = 0.2 + 0.01 * static_cast<double>(pair);
      firstY[pair] = -0.3;
      firstZ[pair] = 0.4;
      secondX[pair] = -0.1;
      secondY[pair] = 0.5 + 0.02 * static_cast<double>(pair);
      secondZ[pair] = -0.2;
      normalX[pair] = 0.3;
      normalY[pair] = -0.4;
      normalZ[pair] = 0.5;
      firstAngularX[pair] =
          normalY[pair] * firstZ[pair] - normalZ[pair] * firstY[pair];
      firstAngularY[pair] =
          normalZ[pair] * firstX[pair] - normalX[pair] * firstZ[pair];
      firstAngularZ[pair] =
          normalX[pair] * firstY[pair] - normalY[pair] * firstX[pair];
      secondAngularX[pair] =
          normalY[pair] * secondZ[pair] - normalZ[pair] * secondY[pair];
      secondAngularY[pair] =
          normalZ[pair] * secondX[pair] - normalX[pair] * secondZ[pair];
      secondAngularZ[pair] =
          normalX[pair] * secondY[pair] - normalY[pair] * secondX[pair];
    }

    computeSpherePairJacobians(
        jacobians.data(), numJoints, numDofs, dofStride, firstJoint.data(),
        secondJoint.data(), firstAngularX.data(), firstAngularY.data(),
        firstAngularZ.data(), secondAngularX.data(), secondAngularY.data(),
        secondAngularZ.data(), normalX.data(), normalY.data(), normalZ.data(),
        numPairs, output.data());

    auto value = [&](size_t row, size_t dof, std::int64_t joint) {
      return jacobians[(static_cast<size_t>(joint) * 6 + row) * dofStride +
                       dof];
    };
    auto validateOutput = [&]() {
      for (size_t dof = 0; dof < numDofs; ++dof) {
        for (size_t pair = 0; pair < numPairs; ++pair) {
          const auto first = firstJoint[pair];
          const auto second = secondJoint[pair];
          const std::array<double, 3> firstCross{
              normalY[pair] * firstZ[pair] - normalZ[pair] * firstY[pair],
              normalZ[pair] * firstX[pair] - normalX[pair] * firstZ[pair],
              normalX[pair] * firstY[pair] - normalY[pair] * firstX[pair]};
          const std::array<double, 3> secondCross{
              normalY[pair] * secondZ[pair] - normalZ[pair] * secondY[pair],
              normalZ[pair] * secondX[pair] - normalX[pair] * secondZ[pair],
              normalX[pair] * secondY[pair] - normalY[pair] * secondX[pair]};
          double expected = 0.0;
          for (size_t axis = 0; axis < 3; ++axis) {
            const double normal =
                axis == 0 ? normalX[pair]
                          : (axis == 1 ? normalY[pair] : normalZ[pair]);
            expected +=
                normal * (value(axis, dof, second) - value(axis, dof, first));
            expected -= secondCross[axis] * value(axis + 3, dof, second);
            expected += firstCross[axis] * value(axis + 3, dof, first);
          }
          EXPECT_NEAR(output[dof * numPairs + pair], expected, 1e-15)
              << "numDofs=" << numDofs << " pair=" << pair
              << " dof=" << dof;
        }
      }
    };
    validateOutput();

    const std::array<SphereJacobianJointRun, 2> jointRuns{{
        {0, 8, 0, 1},
        {8, numPairs - 8, 2, 3},
    }};
    std::fill(output.begin(), output.end(), 0.0);
    computeSpherePairJacobians(
        jacobians.data(), numJoints, numDofs, dofStride, firstJoint.data(),
        secondJoint.data(), firstAngularX.data(), firstAngularY.data(),
        firstAngularZ.data(), secondAngularX.data(), secondAngularY.data(),
        secondAngularZ.data(), normalX.data(), normalY.data(), normalZ.data(),
        numPairs, output.data(), jointRuns.data(), jointRuns.size());
    validateOutput();

    std::vector<SphereJacobianJointRun> shortRuns;
    shortRuns.reserve(numPairs);
    for (size_t pair = 0; pair < numPairs; ++pair) {
      shortRuns.push_back({pair, 1, static_cast<size_t>(firstJoint[pair]),
                           static_cast<size_t>(secondJoint[pair])});
    }
    std::fill(output.begin(), output.end(), 0.0);
    computeSpherePairJacobians(
        jacobians.data(), numJoints, numDofs, dofStride, firstJoint.data(),
        secondJoint.data(), firstAngularX.data(), firstAngularY.data(),
        firstAngularZ.data(), secondAngularX.data(), secondAngularY.data(),
        secondAngularZ.data(), normalX.data(), normalY.data(), normalZ.data(),
        numPairs, output.data(), shortRuns.data(), shortRuns.size());
    validateOutput();
  }
}

TEST(SphereJacobianKernel, SupportsNonMinimalDofStrideWithoutWritingPadding) {
  constexpr size_t numPairs = 13;
  constexpr size_t numJoints = 2;
  constexpr size_t numDofs = 3;
  constexpr size_t dofStride = 8;
  constexpr size_t guardSize = 8;
  constexpr double canary = 1234567.25;

  AlignedVector<double> jacobians(6 * dofStride * numJoints, 0.0);
  for (size_t joint = 0; joint < numJoints; ++joint) {
    for (size_t row = 0; row < 6; ++row) {
      for (size_t dof = 0; dof < numDofs; ++dof) {
        jacobians[(joint * 6 + row) * dofStride + dof] =
            0.5 * static_cast<double>(joint + 1) +
            0.1 * static_cast<double>(row + 1) +
            0.01 * static_cast<double>(dof + 1);
      }
    }
  }

  AlignedVector<std::int64_t> firstJoint(numPairs, 0);
  AlignedVector<std::int64_t> secondJoint(numPairs, 1);
  AlignedVector<double> firstAngularX(numPairs);
  AlignedVector<double> firstAngularY(numPairs);
  AlignedVector<double> firstAngularZ(numPairs);
  AlignedVector<double> secondAngularX(numPairs);
  AlignedVector<double> secondAngularY(numPairs);
  AlignedVector<double> secondAngularZ(numPairs);
  AlignedVector<double> normalX(numPairs);
  AlignedVector<double> normalY(numPairs);
  AlignedVector<double> normalZ(numPairs);
  for (size_t pair = 0; pair < numPairs; ++pair) {
    const double offset = 0.01 * static_cast<double>(pair);
    firstAngularX[pair] = 0.2 + offset;
    firstAngularY[pair] = -0.3 + offset;
    firstAngularZ[pair] = 0.4 - offset;
    secondAngularX[pair] = -0.1 + offset;
    secondAngularY[pair] = 0.5 - offset;
    secondAngularZ[pair] = -0.2 - offset;
    normalX[pair] = 0.3 + offset;
    normalY[pair] = -0.4 + offset;
    normalZ[pair] = 0.5 - offset;
  }

  AlignedVector<double> guardedOutput(
      guardSize + dofStride * numPairs + guardSize, canary);
  double* output = guardedOutput.data() + guardSize;
  const SphereJacobianJointRun jointRun{0, numPairs, 0, 1};
  computeSpherePairJacobians(
      jacobians.data(), numJoints, numDofs, dofStride, firstJoint.data(),
      secondJoint.data(), firstAngularX.data(), firstAngularY.data(),
      firstAngularZ.data(), secondAngularX.data(), secondAngularY.data(),
      secondAngularZ.data(), normalX.data(), normalY.data(), normalZ.data(),
      numPairs, output, &jointRun, 1);

  auto value = [&](size_t joint, size_t row, size_t dof) {
    return jacobians[(joint * 6 + row) * dofStride + dof];
  };
  for (size_t dof = 0; dof < numDofs; ++dof) {
    for (size_t pair = 0; pair < numPairs; ++pair) {
      double expected =
          normalX[pair] * (value(1, 0, dof) - value(0, 0, dof));
      expected +=
          normalY[pair] * (value(1, 1, dof) - value(0, 1, dof));
      expected +=
          normalZ[pair] * (value(1, 2, dof) - value(0, 2, dof));
      expected -= secondAngularX[pair] * value(1, 3, dof);
      expected -= secondAngularY[pair] * value(1, 4, dof);
      expected -= secondAngularZ[pair] * value(1, 5, dof);
      expected += firstAngularX[pair] * value(0, 3, dof);
      expected += firstAngularY[pair] * value(0, 4, dof);
      expected += firstAngularZ[pair] * value(0, 5, dof);
      EXPECT_NEAR(output[dof * numPairs + pair], expected, 1e-15)
          << "pair=" << pair << " dof=" << dof;
    }
  }

  for (size_t index = 0; index < guardSize; ++index) {
    EXPECT_DOUBLE_EQ(guardedOutput[index], canary);
  }
  for (size_t index = guardSize + numDofs * numPairs;
       index < guardedOutput.size(); ++index) {
    EXPECT_DOUBLE_EQ(guardedOutput[index], canary) << "index=" << index;
  }
}
