#pragma once

#include <cstddef>
#include <cstdint>

namespace ocs2 {
namespace collision_nextgen {
namespace benchmark_impl {

void computeSpherePairDistancesRaw(const double *worldX, const double *worldY,
                                   const double *worldZ,
                                   const std::int64_t *firstObject,
                                   const std::int64_t *secondObject,
                                   const double *radiusSum, size_t numPairs,
                                   double *distances, double *normalX,
                                   double *normalY, double *normalZ);

} // namespace benchmark_impl
} // namespace collision_nextgen
} // namespace ocs2
