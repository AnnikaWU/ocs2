#include <benchmark/benchmark.h>

#include "RawSphereDistanceKernel.h"

#include <ocs2_collision_nextgen/impl/SphereCollisionModel.h>
#include <ocs2_collision_nextgen/impl/SphereDistanceKernel.h>
#include <ocs2_collision_nextgen/impl/SphereJacobianKernel.h>
#include <ocs2_collision_nextgen/impl/simd/Vector.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

#if defined(OCS2_COLLISION_NEXTGEN_USE_AVX2)
#include <immintrin.h>
#endif

#if defined(_MSC_VER)
#define OCS2_COLLISION_BENCHMARK_NOINLINE __declspec(noinline)
#elif defined(__GNUC__) || defined(__clang__)
#define OCS2_COLLISION_BENCHMARK_NOINLINE __attribute__((noinline))
#else
#define OCS2_COLLISION_BENCHMARK_NOINLINE
#endif

namespace {

using ocs2::collision_nextgen::impl::AlignedVector;
using ocs2::collision_nextgen::impl::SphereJacobianJointRun;

constexpr double kNormalEpsilon = 1e-12;

struct DistanceData {
  explicit DistanceData(size_t numPairs)
      : numPairs(numPairs), numObjects(std::max<size_t>(32, numPairs / 3)) {
    worldX.resize(numObjects);
    worldY.resize(numObjects);
    worldZ.resize(numObjects);
    first.resize(numPairs);
    second.resize(numPairs);
    radiusSum.resize(numPairs);
    distance.resize(numPairs);
    normalX.resize(numPairs);
    normalY.resize(numPairs);
    normalZ.resize(numPairs);

    std::mt19937_64 generator(0x5eed);
    std::uniform_real_distribution<double> coordinate(-1.5, 1.5);
    std::uniform_real_distribution<double> radius(0.02, 0.2);
    for (size_t object = 0; object < numObjects; ++object) {
      worldX[object] = coordinate(generator);
      worldY[object] = coordinate(generator);
      worldZ[object] = coordinate(generator);
    }
    for (size_t pair = 0; pair < numPairs; ++pair) {
      first[pair] = static_cast<std::int64_t>((pair * 17 + 3) % numObjects);
      second[pair] = static_cast<std::int64_t>((pair * 29 + 11) % numObjects);
      if (second[pair] == first[pair]) {
        second[pair] = static_cast<std::int64_t>(
            (static_cast<size_t>(second[pair]) + 1) % numObjects);
      }
      radiusSum[pair] = radius(generator);
    }
  }

  size_t numPairs;
  size_t numObjects;
  AlignedVector<double> worldX, worldY, worldZ;
  AlignedVector<std::int64_t> first, second;
  AlignedVector<double> radiusSum, distance, normalX, normalY, normalZ;
};

void computeDistancesScalar(DistanceData &data, bool computeNormals) {
  for (size_t pair = 0; pair < data.numPairs; ++pair) {
    const size_t first = static_cast<size_t>(data.first[pair]);
    const size_t second = static_cast<size_t>(data.second[pair]);
    const double dx = data.worldX[second] - data.worldX[first];
    const double dy = data.worldY[second] - data.worldY[first];
    const double dz = data.worldZ[second] - data.worldZ[first];
    const double centerDistance = std::sqrt(dx * dx + dy * dy + dz * dz);
    data.distance[pair] = centerDistance - data.radiusSum[pair];
    if (computeNormals) {
      const double inverseDistance =
          1.0 / std::max(centerDistance, kNormalEpsilon);
      data.normalX[pair] = dx * inverseDistance;
      data.normalY[pair] = dy * inverseDistance;
      data.normalZ[pair] = dz * inverseDistance;
    }
  }
}

void computeDistancesSimd(DistanceData &data, bool computeNormals) {
  ocs2::collision_nextgen::impl::computeSpherePairDistances(
      data.worldX.data(), data.worldY.data(), data.worldZ.data(),
      data.first.data(), data.second.data(), data.radiusSum.data(),
      data.numPairs, data.distance.data(),
      computeNormals ? data.normalX.data() : nullptr,
      computeNormals ? data.normalY.data() : nullptr,
      computeNormals ? data.normalZ.data() : nullptr);
}

#if defined(OCS2_COLLISION_NEXTGEN_USE_AVX2)
void computeDistancesRawAvx2(DistanceData &data, bool computeNormals) {
  ocs2::collision_nextgen::benchmark_impl::computeSpherePairDistancesRaw(
      data.worldX.data(), data.worldY.data(), data.worldZ.data(),
      data.first.data(), data.second.data(), data.radiusSum.data(),
      data.numPairs, data.distance.data(),
      computeNormals ? data.normalX.data() : nullptr,
      computeNormals ? data.normalY.data() : nullptr,
      computeNormals ? data.normalZ.data() : nullptr);
}
#endif

void benchmarkDistance(benchmark::State &state, DistanceData &data, bool simd,
                       bool normals) {
  for (auto _ : state) {
    if (simd) {
      computeDistancesSimd(data, normals);
    } else {
      computeDistancesScalar(data, normals);
    }
    benchmark::DoNotOptimize(data.distance.data());
    benchmark::ClobberMemory();
  }
  state.SetItemsProcessed(state.iterations() *
                          static_cast<int64_t>(data.numPairs));
}

class DistanceFixture : public benchmark::Fixture {
public:
  void SetUp(const benchmark::State &state) override {
    data = std::make_unique<DistanceData>(static_cast<size_t>(state.range(0)));
  }
  void TearDown(const benchmark::State &) override { data.reset(); }

  std::unique_ptr<DistanceData> data;
};

BENCHMARK_DEFINE_F(DistanceFixture, Scalar)(benchmark::State &state) {
  benchmarkDistance(state, *data, false, false);
}
BENCHMARK_DEFINE_F(DistanceFixture, Simd)(benchmark::State &state) {
  benchmarkDistance(state, *data, true, false);
}
BENCHMARK_DEFINE_F(DistanceFixture, NormalScalar)(benchmark::State &state) {
  benchmarkDistance(state, *data, false, true);
}
BENCHMARK_DEFINE_F(DistanceFixture, NormalSimd)(benchmark::State &state) {
  benchmarkDistance(state, *data, true, true);
}

#if defined(OCS2_COLLISION_NEXTGEN_USE_AVX2)
BENCHMARK_DEFINE_F(DistanceFixture, RawAvx2)(benchmark::State &state) {
  for (auto _ : state) {
    computeDistancesRawAvx2(*data, false);
    benchmark::DoNotOptimize(data->distance.data());
    benchmark::ClobberMemory();
  }
  state.SetItemsProcessed(state.iterations() *
                          static_cast<int64_t>(data->numPairs));
}
BENCHMARK_DEFINE_F(DistanceFixture, NormalRawAvx2)(benchmark::State &state) {
  for (auto _ : state) {
    computeDistancesRawAvx2(*data, true);
    benchmark::DoNotOptimize(data->distance.data());
    benchmark::ClobberMemory();
  }
  state.SetItemsProcessed(state.iterations() *
                          static_cast<int64_t>(data->numPairs));
}
#endif

struct JacobianData {
  JacobianData(size_t numPairs, size_t numDofs, size_t numJoints)
      : numPairs(numPairs), numDofs(numDofs), numJoints(numJoints) {
    jointJacobians.resize(6 * numDofs * numJoints);
    dofStride = (numDofs + 3) & ~size_t{3};
    jointJacobiansTiled.resize(6 * dofStride * numJoints, 0.0);
    firstJoint.resize(numPairs);
    secondJoint.resize(numPairs);
    firstAngularX.resize(numPairs);
    firstAngularY.resize(numPairs);
    firstAngularZ.resize(numPairs);
    secondAngularX.resize(numPairs);
    secondAngularY.resize(numPairs);
    secondAngularZ.resize(numPairs);
    normalX.resize(numPairs);
    normalY.resize(numPairs);
    normalZ.resize(numPairs);
    output.resize(numPairs * numDofs);

    std::mt19937_64 generator(0xc0111510);
    std::uniform_real_distribution<double> value(-1.0, 1.0);
    for (double &coefficient : jointJacobians) {
      coefficient = value(generator);
    }
    for (size_t joint = 0; joint < numJoints; ++joint) {
      for (size_t row = 0; row < 6; ++row) {
        for (size_t dof = 0; dof < numDofs; ++dof) {
          jointJacobiansTiled[(joint * 6 + row) * dofStride + dof] =
              jointJacobians[(row * numDofs + dof) * numJoints + joint];
        }
      }
    }
    for (size_t pair = 0; pair < numPairs; ++pair) {
      firstJoint[pair] = static_cast<std::int64_t>((pair * 5 + 1) % numJoints);
      secondJoint[pair] = static_cast<std::int64_t>((pair * 7 + 3) % numJoints);
      firstAngularX[pair] = value(generator);
      firstAngularY[pair] = value(generator);
      firstAngularZ[pair] = value(generator);
      secondAngularX[pair] = value(generator);
      secondAngularY[pair] = value(generator);
      secondAngularZ[pair] = value(generator);
      normalX[pair] = value(generator);
      normalY[pair] = value(generator);
      normalZ[pair] = value(generator);
    }

    if (numPairs == 629 && numDofs == 7 && numJoints == 8) {
      static constexpr std::array<SphereJacobianJointRun, 17> kFrankaRuns{{
          {0, 18, 7, 0},     {18, 72, 7, 1},   {90, 36, 7, 2},
          {126, 45, 7, 3},   {171, 22, 0, 5},  {193, 6, 0, 6},
          {199, 18, 0, 7},   {217, 88, 1, 5},  {305, 24, 1, 6},
          {329, 72, 1, 7},   {401, 44, 2, 5},  {445, 12, 2, 6},
          {457, 36, 2, 7},   {493, 55, 3, 5},  {548, 25, 3, 7},
          {573, 44, 5, 7},   {617, 12, 6, 7},
      }};
      size_t coveredPairs = 0;
      for (const auto &run : kFrankaRuns) {
        if (run.begin != coveredPairs || run.firstJoint >= numJoints ||
            run.secondJoint >= numJoints) {
          throw std::runtime_error("Invalid Franka joint-pair run fixture");
        }
        const size_t runEnd = run.begin + run.length;
        if (runEnd > numPairs) {
          throw std::runtime_error("Franka joint-pair run exceeds fixture size");
        }
        for (size_t pair = run.begin; pair < runEnd; ++pair) {
          firstJoint[pair] = static_cast<std::int64_t>(run.firstJoint);
          secondJoint[pair] = static_cast<std::int64_t>(run.secondJoint);
        }
        coveredPairs = runEnd;
      }
      if (coveredPairs != numPairs) {
        throw std::runtime_error("Franka joint-pair runs do not cover fixture");
      }
    }

    if (numPairs > 0) {
      size_t runBegin = 0;
      for (size_t pair = 1; pair <= numPairs; ++pair) {
        const bool runEnds =
            pair == numPairs || firstJoint[pair] != firstJoint[runBegin] ||
            secondJoint[pair] != secondJoint[runBegin];
        if (runEnds) {
          jointPairRuns.push_back(
              {runBegin, pair - runBegin,
               static_cast<size_t>(firstJoint[runBegin]),
               static_cast<size_t>(secondJoint[runBegin])});
          runBegin = pair;
        }
      }
    }
  }

  size_t numPairs, numDofs, numJoints, dofStride;
  AlignedVector<double> jointJacobians, jointJacobiansTiled;
  AlignedVector<std::int64_t> firstJoint, secondJoint;
  AlignedVector<double> firstAngularX, firstAngularY, firstAngularZ;
  AlignedVector<double> secondAngularX, secondAngularY, secondAngularZ;
  AlignedVector<double> normalX, normalY, normalZ, output;
  std::vector<SphereJacobianJointRun> jointPairRuns;
};

double jacobianValue(const JacobianData &data, size_t row, size_t dof,
                     std::int64_t joint) {
  return data.jointJacobians[(row * data.numDofs + dof) * data.numJoints +
                             static_cast<size_t>(joint)];
}

OCS2_COLLISION_BENCHMARK_NOINLINE void
computeJacobiansScalar(JacobianData &data) {
  for (size_t dof = 0; dof < data.numDofs; ++dof) {
    for (size_t pair = 0; pair < data.numPairs; ++pair) {
      const auto first = data.firstJoint[pair];
      const auto second = data.secondJoint[pair];
      data.output[dof * data.numPairs + pair] =
          data.normalX[pair] * (jacobianValue(data, 0, dof, second) -
                                jacobianValue(data, 0, dof, first)) +
          data.normalY[pair] * (jacobianValue(data, 1, dof, second) -
                                jacobianValue(data, 1, dof, first)) +
          data.normalZ[pair] * (jacobianValue(data, 2, dof, second) -
                                jacobianValue(data, 2, dof, first)) -
          data.secondAngularX[pair] * jacobianValue(data, 3, dof, second) -
          data.secondAngularY[pair] * jacobianValue(data, 4, dof, second) -
          data.secondAngularZ[pair] * jacobianValue(data, 5, dof, second) +
          data.firstAngularX[pair] * jacobianValue(data, 3, dof, first) +
          data.firstAngularY[pair] * jacobianValue(data, 4, dof, first) +
          data.firstAngularZ[pair] * jacobianValue(data, 5, dof, first);
    }
  }
}

OCS2_COLLISION_BENCHMARK_NOINLINE void
computeJacobiansTiledScalar(JacobianData &data) {
  auto value = [&](size_t joint, size_t row, size_t dof) {
    return data.jointJacobiansTiled
        [(joint * 6 + row) * data.dofStride + dof];
  };
  for (size_t dof = 0; dof < data.numDofs; ++dof) {
    double *output = data.output.data() + dof * data.numPairs;
    for (size_t pair = 0; pair < data.numPairs; ++pair) {
      const size_t first = static_cast<size_t>(data.firstJoint[pair]);
      const size_t second = static_cast<size_t>(data.secondJoint[pair]);
      output[pair] =
          data.normalX[pair] *
              (value(second, 0, dof) - value(first, 0, dof)) +
          data.normalY[pair] *
              (value(second, 1, dof) - value(first, 1, dof)) +
          data.normalZ[pair] *
              (value(second, 2, dof) - value(first, 2, dof)) -
          data.secondAngularX[pair] * value(second, 3, dof) -
          data.secondAngularY[pair] * value(second, 4, dof) -
          data.secondAngularZ[pair] * value(second, 5, dof) +
          data.firstAngularX[pair] * value(first, 3, dof) +
          data.firstAngularY[pair] * value(first, 4, dof) +
          data.firstAngularZ[pair] * value(first, 5, dof);
    }
  }
}

[[maybe_unused]] OCS2_COLLISION_BENCHMARK_NOINLINE void
computeJacobiansUngrouped(JacobianData &data) {
  ocs2::collision_nextgen::impl::computeSpherePairJacobians(
      data.jointJacobiansTiled.data(), data.numJoints, data.numDofs,
      data.dofStride, data.firstJoint.data(), data.secondJoint.data(),
      data.firstAngularX.data(), data.firstAngularY.data(),
      data.firstAngularZ.data(), data.secondAngularX.data(),
      data.secondAngularY.data(), data.secondAngularZ.data(),
      data.normalX.data(), data.normalY.data(), data.normalZ.data(),
      data.numPairs, data.output.data());
}

OCS2_COLLISION_BENCHMARK_NOINLINE void
computeJacobiansSimd(JacobianData &data) {
  ocs2::collision_nextgen::impl::computeSpherePairJacobians(
      data.jointJacobiansTiled.data(), data.numJoints, data.numDofs,
      data.dofStride, data.firstJoint.data(), data.secondJoint.data(),
      data.firstAngularX.data(), data.firstAngularY.data(),
      data.firstAngularZ.data(), data.secondAngularX.data(),
      data.secondAngularY.data(), data.secondAngularZ.data(),
      data.normalX.data(), data.normalY.data(), data.normalZ.data(),
      data.numPairs, data.output.data(), data.jointPairRuns.data(),
      data.jointPairRuns.size());
}

OCS2_COLLISION_BENCHMARK_NOINLINE void
computeJacobiansGather(JacobianData &data) {
  using Packet = ocs2::collision_nextgen::impl::simd::Vector<
      double, ocs2::collision_nextgen::impl::simd::NativeWidth<double>>;
  for (size_t dof = 0; dof < data.numDofs; ++dof) {
    const double *linearX =
        data.jointJacobians.data() + (0 * data.numDofs + dof) * data.numJoints;
    const double *linearY =
        data.jointJacobians.data() + (1 * data.numDofs + dof) * data.numJoints;
    const double *linearZ =
        data.jointJacobians.data() + (2 * data.numDofs + dof) * data.numJoints;
    const double *angularX =
        data.jointJacobians.data() + (3 * data.numDofs + dof) * data.numJoints;
    const double *angularY =
        data.jointJacobians.data() + (4 * data.numDofs + dof) * data.numJoints;
    const double *angularZ =
        data.jointJacobians.data() + (5 * data.numDofs + dof) * data.numJoints;
    double *output = data.output.data() + dof * data.numPairs;
    size_t pair = 0;
    for (; pair + Packet::width <= data.numPairs; pair += Packet::width) {
      const Packet nx = Packet::loadAligned(data.normalX.data() + pair);
      const Packet ny = Packet::loadAligned(data.normalY.data() + pair);
      const Packet nz = Packet::loadAligned(data.normalZ.data() + pair);
      const Packet firstLinear =
          nx * Packet::gather(linearX, data.firstJoint.data() + pair) +
          ny * Packet::gather(linearY, data.firstJoint.data() + pair) +
          nz * Packet::gather(linearZ, data.firstJoint.data() + pair);
      const Packet secondLinear =
          nx * Packet::gather(linearX, data.secondJoint.data() + pair) +
          ny * Packet::gather(linearY, data.secondJoint.data() + pair) +
          nz * Packet::gather(linearZ, data.secondJoint.data() + pair);
      const Packet firstAngular =
          Packet::loadAligned(data.firstAngularX.data() + pair) *
              Packet::gather(angularX, data.firstJoint.data() + pair) +
          Packet::loadAligned(data.firstAngularY.data() + pair) *
              Packet::gather(angularY, data.firstJoint.data() + pair) +
          Packet::loadAligned(data.firstAngularZ.data() + pair) *
              Packet::gather(angularZ, data.firstJoint.data() + pair);
      const Packet secondAngular =
          Packet::loadAligned(data.secondAngularX.data() + pair) *
              Packet::gather(angularX, data.secondJoint.data() + pair) +
          Packet::loadAligned(data.secondAngularY.data() + pair) *
              Packet::gather(angularY, data.secondJoint.data() + pair) +
          Packet::loadAligned(data.secondAngularZ.data() + pair) *
              Packet::gather(angularZ, data.secondJoint.data() + pair);
      (secondLinear - firstLinear - secondAngular + firstAngular)
          .storeUnaligned(output + pair);
    }
    for (; pair < data.numPairs; ++pair) {
      const auto first = data.firstJoint[pair];
      const auto second = data.secondJoint[pair];
      output[pair] =
          data.normalX[pair] * (jacobianValue(data, 0, dof, second) -
                                jacobianValue(data, 0, dof, first)) +
          data.normalY[pair] * (jacobianValue(data, 1, dof, second) -
                                jacobianValue(data, 1, dof, first)) +
          data.normalZ[pair] * (jacobianValue(data, 2, dof, second) -
                                jacobianValue(data, 2, dof, first)) -
          data.secondAngularX[pair] * jacobianValue(data, 3, dof, second) -
          data.secondAngularY[pair] * jacobianValue(data, 4, dof, second) -
          data.secondAngularZ[pair] * jacobianValue(data, 5, dof, second) +
          data.firstAngularX[pair] * jacobianValue(data, 3, dof, first) +
          data.firstAngularY[pair] * jacobianValue(data, 4, dof, first) +
          data.firstAngularZ[pair] * jacobianValue(data, 5, dof, first);
    }
  }
}

#if defined(OCS2_COLLISION_NEXTGEN_USE_AVX2)

inline __m256d computeJacobianDofPacket(const JacobianData &data, size_t pair,
                                        size_t dof) {
  const size_t first = static_cast<size_t>(data.firstJoint[pair]);
  const size_t second = static_cast<size_t>(data.secondJoint[pair]);
  auto row = [&](size_t joint, size_t spatialRow) {
    return _mm256_load_pd(data.jointJacobiansTiled.data() +
                          (joint * 6 + spatialRow) * data.dofStride + dof);
  };
  auto coefficient = [](double value) { return _mm256_set1_pd(value); };

  __m256d result = _mm256_mul_pd(coefficient(data.normalX[pair]),
                                 _mm256_sub_pd(row(second, 0), row(first, 0)));
  result = _mm256_add_pd(
      result, _mm256_mul_pd(coefficient(data.normalY[pair]),
                            _mm256_sub_pd(row(second, 1), row(first, 1))));
  result = _mm256_add_pd(
      result, _mm256_mul_pd(coefficient(data.normalZ[pair]),
                            _mm256_sub_pd(row(second, 2), row(first, 2))));
  result = _mm256_sub_pd(
      result,
      _mm256_mul_pd(coefficient(data.secondAngularX[pair]), row(second, 3)));
  result = _mm256_sub_pd(
      result,
      _mm256_mul_pd(coefficient(data.secondAngularY[pair]), row(second, 4)));
  result = _mm256_sub_pd(
      result,
      _mm256_mul_pd(coefficient(data.secondAngularZ[pair]), row(second, 5)));
  result =
      _mm256_add_pd(result, _mm256_mul_pd(coefficient(data.firstAngularX[pair]),
                                          row(first, 3)));
  result =
      _mm256_add_pd(result, _mm256_mul_pd(coefficient(data.firstAngularY[pair]),
                                          row(first, 4)));
  return _mm256_add_pd(
      result,
      _mm256_mul_pd(coefficient(data.firstAngularZ[pair]), row(first, 5)));
}

struct JacobianDof7Packets {
  __m256d low;
  __m256d high;
};

inline JacobianDof7Packets
computeJacobianDof7Packets(const JacobianData &data, size_t pair) {
  const size_t first = static_cast<size_t>(data.firstJoint[pair]);
  const size_t second = static_cast<size_t>(data.secondJoint[pair]);
  const double *firstRows =
      data.jointJacobiansTiled.data() + first * 6 * data.dofStride;
  const double *secondRows =
      data.jointJacobiansTiled.data() + second * 6 * data.dofStride;
  auto row = [&](const double *rows, size_t spatialRow, size_t dof) {
    return _mm256_load_pd(rows + spatialRow * data.dofStride + dof);
  };

  const __m256d normalX = _mm256_set1_pd(data.normalX[pair]);
  JacobianDof7Packets result{
      _mm256_mul_pd(normalX,
                    _mm256_sub_pd(row(secondRows, 0, 0),
                                  row(firstRows, 0, 0))),
      _mm256_mul_pd(normalX,
                    _mm256_sub_pd(row(secondRows, 0, 4),
                                  row(firstRows, 0, 4)))};

  auto addDifference = [&](double coefficient, size_t spatialRow) {
    const __m256d scale = _mm256_set1_pd(coefficient);
    result.low = _mm256_add_pd(
        result.low,
        _mm256_mul_pd(scale,
                      _mm256_sub_pd(row(secondRows, spatialRow, 0),
                                    row(firstRows, spatialRow, 0))));
    result.high = _mm256_add_pd(
        result.high,
        _mm256_mul_pd(scale,
                      _mm256_sub_pd(row(secondRows, spatialRow, 4),
                                    row(firstRows, spatialRow, 4))));
  };
  auto addRow = [&](double coefficient, const double *rows,
                    size_t spatialRow) {
    const __m256d scale = _mm256_set1_pd(coefficient);
    result.low = _mm256_add_pd(
        result.low,
        _mm256_mul_pd(scale, row(rows, spatialRow, 0)));
    result.high = _mm256_add_pd(
        result.high,
        _mm256_mul_pd(scale, row(rows, spatialRow, 4)));
  };
  auto subtractRow = [&](double coefficient, const double *rows,
                         size_t spatialRow) {
    const __m256d scale = _mm256_set1_pd(coefficient);
    result.low = _mm256_sub_pd(
        result.low,
        _mm256_mul_pd(scale, row(rows, spatialRow, 0)));
    result.high = _mm256_sub_pd(
        result.high,
        _mm256_mul_pd(scale, row(rows, spatialRow, 4)));
  };

  addDifference(data.normalY[pair], 1);
  addDifference(data.normalZ[pair], 2);
  subtractRow(data.secondAngularX[pair], secondRows, 3);
  subtractRow(data.secondAngularY[pair], secondRows, 4);
  subtractRow(data.secondAngularZ[pair], secondRows, 5);
  addRow(data.firstAngularX[pair], firstRows, 3);
  addRow(data.firstAngularY[pair], firstRows, 4);
  addRow(data.firstAngularZ[pair], firstRows, 5);
  return result;
}

void storeTransposed(__m256d row0, __m256d row1, __m256d row2, __m256d row3,
                     size_t pair, size_t dof, JacobianData &data) {
  const __m256d unpackLow01 = _mm256_unpacklo_pd(row0, row1);
  const __m256d unpackHigh01 = _mm256_unpackhi_pd(row0, row1);
  const __m256d unpackLow23 = _mm256_unpacklo_pd(row2, row3);
  const __m256d unpackHigh23 = _mm256_unpackhi_pd(row2, row3);
  const __m256d columns[4] = {
      _mm256_permute2f128_pd(unpackLow01, unpackLow23, 0x20),
      _mm256_permute2f128_pd(unpackHigh01, unpackHigh23, 0x20),
      _mm256_permute2f128_pd(unpackLow01, unpackLow23, 0x31),
      _mm256_permute2f128_pd(unpackHigh01, unpackHigh23, 0x31)};
  const size_t activeDofs = std::min<size_t>(4, data.numDofs - dof);
  for (size_t lane = 0; lane < activeDofs; ++lane) {
    _mm256_storeu_pd(data.output.data() + (dof + lane) * data.numPairs + pair,
                     columns[lane]);
  }
}

OCS2_COLLISION_BENCHMARK_NOINLINE void
computeJacobiansTiled(JacobianData &data) {
  size_t pair = 0;
  for (; pair + 4 <= data.numPairs; pair += 4) {
    for (size_t dof = 0; dof < data.dofStride; dof += 4) {
      storeTransposed(computeJacobianDofPacket(data, pair, dof),
                      computeJacobianDofPacket(data, pair + 1, dof),
                      computeJacobianDofPacket(data, pair + 2, dof),
                      computeJacobianDofPacket(data, pair + 3, dof), pair, dof,
                      data);
    }
  }
  alignas(32) double lanes[4];
  for (; pair < data.numPairs; ++pair) {
    for (size_t dof = 0; dof < data.dofStride; dof += 4) {
      _mm256_store_pd(lanes, computeJacobianDofPacket(data, pair, dof));
      const size_t activeDofs = std::min<size_t>(4, data.numDofs - dof);
      for (size_t lane = 0; lane < activeDofs; ++lane) {
        data.output[(dof + lane) * data.numPairs + pair] = lanes[lane];
      }
    }
  }
}

OCS2_COLLISION_BENCHMARK_NOINLINE void
computeJacobiansPairMajor(JacobianData &data) {
  alignas(32) double lanes[4];
  for (size_t pair = 0; pair < data.numPairs; ++pair) {
    for (size_t dof = 0; dof < data.dofStride; dof += 4) {
      _mm256_store_pd(lanes, computeJacobianDofPacket(data, pair, dof));
      const size_t activeDofs = std::min<size_t>(4, data.numDofs - dof);
      for (size_t lane = 0; lane < activeDofs; ++lane) {
        data.output[(dof + lane) * data.numPairs + pair] = lanes[lane];
      }
    }
  }
}

OCS2_COLLISION_BENCHMARK_NOINLINE void
computeJacobiansDof2Specialized(JacobianData &data) {
  if (data.numDofs != 2 || data.dofStride != 4) {
    throw std::runtime_error(
        "2-DoF Jacobian benchmark requires 2 DoFs with stride 4");
  }

  size_t pair = 0;
  for (; pair + 4 <= data.numPairs; pair += 4) {
    const __m256d row0 = computeJacobianDofPacket(data, pair, 0);
    const __m256d row1 = computeJacobianDofPacket(data, pair + 1, 0);
    const __m256d row2 = computeJacobianDofPacket(data, pair + 2, 0);
    const __m256d row3 = computeJacobianDofPacket(data, pair + 3, 0);
    const __m256d low01 = _mm256_unpacklo_pd(row0, row1);
    const __m256d high01 = _mm256_unpackhi_pd(row0, row1);
    const __m256d low23 = _mm256_unpacklo_pd(row2, row3);
    const __m256d high23 = _mm256_unpackhi_pd(row2, row3);
    _mm256_storeu_pd(data.output.data() + pair,
                     _mm256_permute2f128_pd(low01, low23, 0x20));
    _mm256_storeu_pd(data.output.data() + data.numPairs + pair,
                     _mm256_permute2f128_pd(high01, high23, 0x20));
  }

  alignas(32) double lanes[4];
  for (; pair < data.numPairs; ++pair) {
    _mm256_store_pd(lanes, computeJacobianDofPacket(data, pair, 0));
    data.output[pair] = lanes[0];
    data.output[data.numPairs + pair] = lanes[1];
  }
}

OCS2_COLLISION_BENCHMARK_NOINLINE void
computeJacobiansDualPacketDof7(JacobianData &data) {
  if (data.numDofs != 7 || data.dofStride != 8) {
    throw std::runtime_error(
        "Dual-packet Jacobian benchmark requires 7 DoFs with stride 8");
  }

  size_t pair = 0;
  for (; pair + 4 <= data.numPairs; pair += 4) {
    const JacobianDof7Packets packet0 =
        computeJacobianDof7Packets(data, pair);
    const JacobianDof7Packets packet1 =
        computeJacobianDof7Packets(data, pair + 1);
    const JacobianDof7Packets packet2 =
        computeJacobianDof7Packets(data, pair + 2);
    const JacobianDof7Packets packet3 =
        computeJacobianDof7Packets(data, pair + 3);
    storeTransposed(packet0.low, packet1.low, packet2.low, packet3.low, pair,
                    0, data);
    storeTransposed(packet0.high, packet1.high, packet2.high, packet3.high,
                    pair, 4, data);
  }

  alignas(32) double low[4];
  alignas(32) double high[4];
  for (; pair < data.numPairs; ++pair) {
    const JacobianDof7Packets packets =
        computeJacobianDof7Packets(data, pair);
    _mm256_store_pd(low, packets.low);
    _mm256_store_pd(high, packets.high);
    for (size_t lane = 0; lane < 4; ++lane) {
      data.output[lane * data.numPairs + pair] = low[lane];
    }
    for (size_t lane = 0; lane < 3; ++lane) {
      data.output[(lane + 4) * data.numPairs + pair] = high[lane];
    }
  }
}

OCS2_COLLISION_BENCHMARK_NOINLINE void
computeJacobiansRunGroupedThreeAccumulators(JacobianData &data) {
  if (data.numDofs != 7 || data.dofStride != 8 ||
      data.jointPairRuns.empty()) {
    throw std::runtime_error(
        "Three-accumulator Jacobian benchmark requires run-grouped 7-DoF data");
  }

  for (const auto &run : data.jointPairRuns) {
    const size_t runEnd = run.begin + run.length;
    const double *firstRows = data.jointJacobiansTiled.data() +
                              run.firstJoint * 6 * data.dofStride;
    const double *secondRows = data.jointJacobiansTiled.data() +
                               run.secondJoint * 6 * data.dofStride;
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

      const __m256d linearXPacket = _mm256_set1_pd(linearX);
      const __m256d linearYPacket = _mm256_set1_pd(linearY);
      const __m256d linearZPacket = _mm256_set1_pd(linearZ);
      const __m256d firstAngularXPacket = _mm256_set1_pd(firstAngularX);
      const __m256d firstAngularYPacket = _mm256_set1_pd(firstAngularY);
      const __m256d firstAngularZPacket = _mm256_set1_pd(firstAngularZ);
      const __m256d secondAngularXPacket = _mm256_set1_pd(secondAngularX);
      const __m256d secondAngularYPacket = _mm256_set1_pd(secondAngularY);
      const __m256d secondAngularZPacket = _mm256_set1_pd(secondAngularZ);

      double *output = data.output.data() + dof * data.numPairs;
      size_t pair = run.begin;
      for (; pair + 4 <= runEnd; pair += 4) {
        __m256d linear = _mm256_mul_pd(
            _mm256_loadu_pd(data.normalX.data() + pair), linearXPacket);
        linear = _mm256_add_pd(
            linear, _mm256_mul_pd(
                        _mm256_loadu_pd(data.normalY.data() + pair),
                        linearYPacket));
        linear = _mm256_add_pd(
            linear, _mm256_mul_pd(
                        _mm256_loadu_pd(data.normalZ.data() + pair),
                        linearZPacket));

        __m256d secondAngular = _mm256_mul_pd(
            _mm256_loadu_pd(data.secondAngularX.data() + pair),
            secondAngularXPacket);
        secondAngular = _mm256_add_pd(
            secondAngular,
            _mm256_mul_pd(
                _mm256_loadu_pd(data.secondAngularY.data() + pair),
                secondAngularYPacket));
        secondAngular = _mm256_add_pd(
            secondAngular,
            _mm256_mul_pd(
                _mm256_loadu_pd(data.secondAngularZ.data() + pair),
                secondAngularZPacket));

        __m256d firstAngular = _mm256_mul_pd(
            _mm256_loadu_pd(data.firstAngularX.data() + pair),
            firstAngularXPacket);
        firstAngular = _mm256_add_pd(
            firstAngular,
            _mm256_mul_pd(
                _mm256_loadu_pd(data.firstAngularY.data() + pair),
                firstAngularYPacket));
        firstAngular = _mm256_add_pd(
            firstAngular,
            _mm256_mul_pd(
                _mm256_loadu_pd(data.firstAngularZ.data() + pair),
                firstAngularZPacket));

        _mm256_storeu_pd(
            output + pair,
            _mm256_add_pd(_mm256_sub_pd(linear, secondAngular),
                          firstAngular));
      }

      for (; pair < runEnd; ++pair) {
        const double linear = data.normalX[pair] * linearX +
                              data.normalY[pair] * linearY +
                              data.normalZ[pair] * linearZ;
        const double secondAngular =
            data.secondAngularX[pair] * secondAngularX +
            data.secondAngularY[pair] * secondAngularY +
            data.secondAngularZ[pair] * secondAngularZ;
        const double firstAngular =
            data.firstAngularX[pair] * firstAngularX +
            data.firstAngularY[pair] * firstAngularY +
            data.firstAngularZ[pair] * firstAngularZ;
        output[pair] = linear - secondAngular + firstAngular;
      }
    }
  }
}

#endif

void benchmarkJacobian(benchmark::State &state, JacobianData &data, bool simd) {
  for (auto _ : state) {
    if (simd) {
      computeJacobiansSimd(data);
    } else {
      computeJacobiansScalar(data);
    }
    benchmark::DoNotOptimize(data.output.data());
    benchmark::ClobberMemory();
  }
  state.SetItemsProcessed(state.iterations() *
                          static_cast<int64_t>(data.numPairs * data.numDofs));
}

class JacobianFixture : public benchmark::Fixture {
public:
  void SetUp(const benchmark::State &state) override {
    data = std::make_unique<JacobianData>(static_cast<size_t>(state.range(0)),
                                          static_cast<size_t>(state.range(1)),
                                          static_cast<size_t>(state.range(2)));
#if defined(OCS2_COLLISION_NEXTGEN_USE_AVX2)
    computeJacobiansScalar(*data);
    const std::vector<double> reference(data->output.begin(),
                                        data->output.end());
    auto validate = [&](const char *name) {
      for (size_t index = 0; index < reference.size(); ++index) {
        if (std::abs(reference[index] - data->output[index]) > 1e-12) {
          throw std::runtime_error(std::string(name) +
                                   " does not match the scalar reference");
        }
      }
    };
    computeJacobiansUngrouped(*data);
    validate("Ungrouped Jacobian kernel");
    computeJacobiansTiled(*data);
    validate("Experimental tiled Jacobian kernel");
    computeJacobiansPairMajor(*data);
    validate("Experimental pair-major Jacobian kernel");
    if (data->numDofs == 2 && data->dofStride == 4) {
      computeJacobiansDof2Specialized(*data);
      validate("Experimental fixed 2-DoF Jacobian kernel");
    }
    if (data->numDofs == 7 && data->dofStride == 8) {
      computeJacobiansDualPacketDof7(*data);
      validate("Experimental dual-packet 7-DoF Jacobian kernel");
    }
    computeJacobiansSimd(*data);
    validate("Production Jacobian kernel");
    computeJacobiansTiledScalar(*data);
    validate("Tiled-layout scalar Jacobian kernel");
    if (data->numDofs == 7 && data->dofStride == 8 &&
        data->jointPairRuns.size() <= data->numPairs / 4) {
      computeJacobiansRunGroupedThreeAccumulators(*data);
      validate("Experimental three-accumulator run-grouped Jacobian kernel");
    }
#endif
  }
  void TearDown(const benchmark::State &) override { data.reset(); }

  std::unique_ptr<JacobianData> data;
};

BENCHMARK_DEFINE_F(JacobianFixture, Scalar)(benchmark::State &state) {
  benchmarkJacobian(state, *data, false);
}
BENCHMARK_DEFINE_F(JacobianFixture, Simd)(benchmark::State &state) {
  benchmarkJacobian(state, *data, true);
}
BENCHMARK_DEFINE_F(JacobianFixture, TiledScalar)(benchmark::State &state) {
  for (auto _ : state) {
    computeJacobiansTiledScalar(*data);
    benchmark::DoNotOptimize(data->output.data());
    benchmark::ClobberMemory();
  }
  state.SetItemsProcessed(state.iterations() *
                          static_cast<int64_t>(data->numPairs * data->numDofs));
}
BENCHMARK_DEFINE_F(JacobianFixture, Gather)(benchmark::State &state) {
  for (auto _ : state) {
    computeJacobiansGather(*data);
    benchmark::DoNotOptimize(data->output.data());
    benchmark::ClobberMemory();
  }
  state.SetItemsProcessed(state.iterations() *
                          static_cast<int64_t>(data->numPairs * data->numDofs));
}

#if defined(OCS2_COLLISION_NEXTGEN_USE_AVX2)
BENCHMARK_DEFINE_F(JacobianFixture, Tiled)(benchmark::State &state) {
  for (auto _ : state) {
    computeJacobiansTiled(*data);
    benchmark::DoNotOptimize(data->output.data());
    benchmark::ClobberMemory();
  }
  state.SetItemsProcessed(state.iterations() *
                          static_cast<int64_t>(data->numPairs * data->numDofs));
}
BENCHMARK_DEFINE_F(JacobianFixture, PairMajor)(benchmark::State &state) {
  for (auto _ : state) {
    computeJacobiansPairMajor(*data);
    benchmark::DoNotOptimize(data->output.data());
    benchmark::ClobberMemory();
  }
  state.SetItemsProcessed(state.iterations() *
                          static_cast<int64_t>(data->numPairs * data->numDofs));
}
BENCHMARK_DEFINE_F(JacobianFixture, Dof2Specialized)(benchmark::State &state) {
  for (auto _ : state) {
    computeJacobiansDof2Specialized(*data);
    benchmark::DoNotOptimize(data->output.data());
    benchmark::ClobberMemory();
  }
  state.SetItemsProcessed(state.iterations() *
                          static_cast<int64_t>(data->numPairs * data->numDofs));
}
BENCHMARK_DEFINE_F(JacobianFixture, DualPacketDof7)(benchmark::State &state) {
  for (auto _ : state) {
    computeJacobiansDualPacketDof7(*data);
    benchmark::DoNotOptimize(data->output.data());
    benchmark::ClobberMemory();
  }
  state.SetItemsProcessed(state.iterations() *
                          static_cast<int64_t>(data->numPairs * data->numDofs));
}
BENCHMARK_DEFINE_F(JacobianFixture, Ungrouped)(benchmark::State &state) {
  for (auto _ : state) {
    computeJacobiansUngrouped(*data);
    benchmark::DoNotOptimize(data->output.data());
    benchmark::ClobberMemory();
  }
  state.SetItemsProcessed(state.iterations() *
                          static_cast<int64_t>(data->numPairs * data->numDofs));
}
BENCHMARK_DEFINE_F(JacobianFixture,
                   RunGroupedThreeAccumulators)(benchmark::State &state) {
  for (auto _ : state) {
    computeJacobiansRunGroupedThreeAccumulators(*data);
    benchmark::DoNotOptimize(data->output.data());
    benchmark::ClobberMemory();
  }
  state.SetItemsProcessed(state.iterations() *
                          static_cast<int64_t>(data->numPairs * data->numDofs));
}
#endif

void distanceArguments(benchmark::internal::Benchmark *benchmark) {
  for (const int64_t pairs : {20, 629, 630, 2048}) {
    benchmark->Arg(pairs);
  }
}

void jacobianArguments(benchmark::internal::Benchmark *benchmark) {
  for (const int64_t pairs : {20, 629, 630, 2048}) {
    benchmark->Args({pairs, 7, 10});
  }
  benchmark->Args({629, 7, 8});
}

void productionJacobianArguments(
    benchmark::internal::Benchmark *benchmark) {
  jacobianArguments(benchmark);
  for (const int64_t dofs : {1, 2, 3, 5, 6, 8, 11}) {
    benchmark->Args({629, dofs, 10});
  }
}

BENCHMARK_REGISTER_F(DistanceFixture, Scalar)->Apply(distanceArguments);
BENCHMARK_REGISTER_F(DistanceFixture, Simd)->Apply(distanceArguments);
BENCHMARK_REGISTER_F(DistanceFixture, NormalScalar)->Apply(distanceArguments);
BENCHMARK_REGISTER_F(DistanceFixture, NormalSimd)->Apply(distanceArguments);
#if defined(OCS2_COLLISION_NEXTGEN_USE_AVX2)
BENCHMARK_REGISTER_F(DistanceFixture, RawAvx2)->Apply(distanceArguments);
BENCHMARK_REGISTER_F(DistanceFixture, NormalRawAvx2)->Apply(distanceArguments);
#endif
BENCHMARK_REGISTER_F(JacobianFixture, Scalar)
    ->Apply(productionJacobianArguments);
BENCHMARK_REGISTER_F(JacobianFixture, Simd)
    ->Apply(productionJacobianArguments);
BENCHMARK_REGISTER_F(JacobianFixture, TiledScalar)
    ->Apply(productionJacobianArguments);
BENCHMARK_REGISTER_F(JacobianFixture, Gather)->Apply(jacobianArguments);
#if defined(OCS2_COLLISION_NEXTGEN_USE_AVX2)
BENCHMARK_REGISTER_F(JacobianFixture, Tiled)->Apply(jacobianArguments);
BENCHMARK_REGISTER_F(JacobianFixture, PairMajor)->Apply(jacobianArguments);
BENCHMARK_REGISTER_F(JacobianFixture, Dof2Specialized)->Args({629, 2, 10});
BENCHMARK_REGISTER_F(JacobianFixture, DualPacketDof7)->Args({629, 7, 8});
BENCHMARK_REGISTER_F(JacobianFixture, Ungrouped)
    ->Apply(productionJacobianArguments);
BENCHMARK_REGISTER_F(JacobianFixture, RunGroupedThreeAccumulators)
    ->Args({629, 7, 8});
#endif

#undef OCS2_COLLISION_BENCHMARK_NOINLINE

} // namespace
