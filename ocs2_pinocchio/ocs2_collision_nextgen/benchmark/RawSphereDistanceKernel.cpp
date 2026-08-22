#include "RawSphereDistanceKernel.h"

#include <algorithm>
#include <cmath>

#if defined(OCS2_COLLISION_NEXTGEN_USE_AVX2)
#include <immintrin.h>
#endif

namespace ocs2 {
namespace collision_nextgen {
namespace benchmark_impl {

void computeSpherePairDistancesRaw(const double *worldX, const double *worldY,
                                   const double *worldZ,
                                   const std::int64_t *firstObject,
                                   const std::int64_t *secondObject,
                                   const double *radiusSum, size_t numPairs,
                                   double *distances, double *normalX,
                                   double *normalY, double *normalZ) {
  constexpr double kEpsilon = 1e-12;
  const bool computeNormals =
      normalX != nullptr && normalY != nullptr && normalZ != nullptr;
  size_t pair = 0;
#if defined(OCS2_COLLISION_NEXTGEN_USE_AVX2)
  const __m256d epsilon = _mm256_set1_pd(kEpsilon);
  const __m256d one = _mm256_set1_pd(1.0);
  for (; pair + 4 <= numPairs; pair += 4) {
    const __m256i first = _mm256_loadu_si256(
        reinterpret_cast<const __m256i *>(firstObject + pair));
    const __m256i second = _mm256_loadu_si256(
        reinterpret_cast<const __m256i *>(secondObject + pair));
    const __m256d dx = _mm256_sub_pd(_mm256_i64gather_pd(worldX, second, 8),
                                     _mm256_i64gather_pd(worldX, first, 8));
    const __m256d dy = _mm256_sub_pd(_mm256_i64gather_pd(worldY, second, 8),
                                     _mm256_i64gather_pd(worldY, first, 8));
    const __m256d dz = _mm256_sub_pd(_mm256_i64gather_pd(worldZ, second, 8),
                                     _mm256_i64gather_pd(worldZ, first, 8));
    const __m256d centerDistance = _mm256_sqrt_pd(_mm256_add_pd(
        _mm256_add_pd(_mm256_mul_pd(dx, dx), _mm256_mul_pd(dy, dy)),
        _mm256_mul_pd(dz, dz)));
    _mm256_storeu_pd(
        distances + pair,
        _mm256_sub_pd(centerDistance, _mm256_loadu_pd(radiusSum + pair)));
    if (computeNormals) {
      const __m256d inverseDistance =
          _mm256_div_pd(one, _mm256_max_pd(centerDistance, epsilon));
      _mm256_storeu_pd(normalX + pair, _mm256_mul_pd(dx, inverseDistance));
      _mm256_storeu_pd(normalY + pair, _mm256_mul_pd(dy, inverseDistance));
      _mm256_storeu_pd(normalZ + pair, _mm256_mul_pd(dz, inverseDistance));
    }
  }
#endif
  for (; pair < numPairs; ++pair) {
    const size_t first = static_cast<size_t>(firstObject[pair]);
    const size_t second = static_cast<size_t>(secondObject[pair]);
    const double dx = worldX[second] - worldX[first];
    const double dy = worldY[second] - worldY[first];
    const double dz = worldZ[second] - worldZ[first];
    const double centerDistance = std::sqrt(dx * dx + dy * dy + dz * dz);
    distances[pair] = centerDistance - radiusSum[pair];
    if (computeNormals) {
      const double inverseDistance = 1.0 / std::max(centerDistance, kEpsilon);
      normalX[pair] = dx * inverseDistance;
      normalY[pair] = dy * inverseDistance;
      normalZ[pair] = dz * inverseDistance;
    }
  }
}

} // namespace benchmark_impl
} // namespace collision_nextgen
} // namespace ocs2
