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

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstddef>

#include <Eigen/Core>

#include <ocs2_collision_nextgen/impl/simd/AlignedAllocator.h>

#if defined(OCS2_COLLISION_NEXTGEN_USE_AVX2)
#include <immintrin.h>
#endif

namespace ocs2 {
namespace collision_nextgen {
namespace impl {
namespace simd {

template <typename DataType, size_t DataWidth>
class Mask;

template <typename DataType, size_t DataWidth>
class Vector;

/** Returns {value[I0], value[I1], value[I2], value[I3]}. */
template <size_t I0, size_t I1, size_t I2, size_t I3>
inline Vector<double, 4> permute(const Vector<double, 4>& value);

/** Selects lanes from two packets, where indices 0..3 address lhs and 4..7 address rhs. */
template <size_t I0, size_t I1, size_t I2, size_t I3>
inline Vector<double, 4> permute(const Vector<double, 4>& lhs, const Vector<double, 4>& rhs);

/** Transposes four row packets into four column packets in place. */
inline void transpose4x4(Vector<double, 4>& row0, Vector<double, 4>& row1, Vector<double, 4>& row2,
                         Vector<double, 4>& row3);

template <typename DataType>
inline constexpr size_t NativeWidth = 1;

#if defined(OCS2_COLLISION_NEXTGEN_USE_AVX2)
template <>
inline constexpr size_t NativeWidth<double> = 4;
#endif

template <typename DataType, size_t DataWidth>
class alignas(kDefaultAlignment) Mask {
 public:
  using Scalar = DataType;
  static constexpr size_t width = DataWidth;

  Mask() { lanes_.fill(false); }
  explicit Mask(bool value) { lanes_.fill(value); }

  bool lane(size_t index) const { return lanes_[index]; }
  void setLane(size_t index, bool value) { lanes_[index] = value; }

 private:
  std::array<bool, DataWidth> lanes_;

  friend class Vector<DataType, DataWidth>;
  friend Mask operator||(const Mask& lhs, const Mask& rhs) {
    Mask result;
    for (size_t i = 0; i < DataWidth; ++i) {
      result.lanes_[i] = lhs.lanes_[i] || rhs.lanes_[i];
    }
    return result;
  }
  friend Mask operator&&(const Mask& lhs, const Mask& rhs) {
    Mask result;
    for (size_t i = 0; i < DataWidth; ++i) {
      result.lanes_[i] = lhs.lanes_[i] && rhs.lanes_[i];
    }
    return result;
  }
};

template <typename DataType, size_t DataWidth>
class alignas(kDefaultAlignment) Vector {
 public:
  using Scalar = DataType;
  using MaskType = Mask<DataType, DataWidth>;
  using EigenVector = Eigen::Matrix<DataType, DataWidth, 1>;
  static constexpr size_t width = DataWidth;

  Vector() { lanes_.fill(DataType{0}); }
  explicit Vector(DataType value) { lanes_.fill(value); }

  static Vector set(DataType value) { return Vector(value); }

  static Vector loadAligned(const DataType* ptr) { return loadUnaligned(ptr); }
  static Vector loadUnaligned(const DataType* ptr) {
    Vector result;
    for (size_t i = 0; i < DataWidth; ++i) {
      result.lanes_[i] = ptr[i];
    }
    return result;
  }

  static Vector gather(const DataType* base, const std::int64_t* indices) {
    Vector result;
    for (size_t i = 0; i < DataWidth; ++i) {
      result.lanes_[i] = base[indices[i]];
    }
    return result;
  }

  static Vector fromEigen(const EigenVector& values) {
    Vector result;
    for (size_t i = 0; i < DataWidth; ++i) {
      result.lanes_[i] = values[static_cast<Eigen::Index>(i)];
    }
    return result;
  }

  template <typename Derived>
  static Vector loadEigen(const Eigen::MatrixBase<Derived>& values, Eigen::Index offset = 0) {
    Vector result;
    for (size_t i = 0; i < DataWidth; ++i) {
      result.lanes_[i] = values.derived().data()[offset + static_cast<Eigen::Index>(i)];
    }
    return result;
  }

  void storeAligned(DataType* ptr) const { storeUnaligned(ptr); }
  void storeUnaligned(DataType* ptr) const {
    for (size_t i = 0; i < DataWidth; ++i) {
      ptr[i] = lanes_[i];
    }
  }

  template <typename Derived>
  void storeEigen(Eigen::MatrixBase<Derived>& values, Eigen::Index offset = 0) const {
    for (size_t i = 0; i < DataWidth; ++i) {
      values.derived().data()[offset + static_cast<Eigen::Index>(i)] = lanes_[i];
    }
  }

  EigenVector toEigen() const {
    EigenVector result;
    for (size_t i = 0; i < DataWidth; ++i) {
      result[static_cast<Eigen::Index>(i)] = lanes_[i];
    }
    return result;
  }

  DataType lane(size_t index) const { return lanes_[index]; }

  friend Vector operator+(const Vector& lhs, const Vector& rhs) {
    Vector result;
    for (size_t i = 0; i < DataWidth; ++i) {
      result.lanes_[i] = lhs.lanes_[i] + rhs.lanes_[i];
    }
    return result;
  }
  friend Vector operator-(const Vector& lhs, const Vector& rhs) {
    Vector result;
    for (size_t i = 0; i < DataWidth; ++i) {
      result.lanes_[i] = lhs.lanes_[i] - rhs.lanes_[i];
    }
    return result;
  }
  friend Vector operator*(const Vector& lhs, const Vector& rhs) {
    Vector result;
    for (size_t i = 0; i < DataWidth; ++i) {
      result.lanes_[i] = lhs.lanes_[i] * rhs.lanes_[i];
    }
    return result;
  }
  friend Vector operator/(const Vector& lhs, const Vector& rhs) {
    Vector result;
    for (size_t i = 0; i < DataWidth; ++i) {
      result.lanes_[i] = lhs.lanes_[i] / rhs.lanes_[i];
    }
    return result;
  }
  friend MaskType operator<(const Vector& lhs, const Vector& rhs) {
    MaskType result;
    for (size_t i = 0; i < DataWidth; ++i) {
      result.setLane(i, lhs.lanes_[i] < rhs.lanes_[i]);
    }
    return result;
  }

 private:
  std::array<DataType, DataWidth> lanes_;

  template <typename T, size_t W>
  friend Vector<T, W> min(const Vector<T, W>& lhs, const Vector<T, W>& rhs);
  template <typename T, size_t W>
  friend Vector<T, W> max(const Vector<T, W>& lhs, const Vector<T, W>& rhs);
  template <typename T, size_t W>
  friend Vector<T, W> sqrt(const Vector<T, W>& value);
  template <typename T, size_t W>
  friend Vector<T, W> sin(const Vector<T, W>& value);
  template <typename T, size_t W>
  friend Vector<T, W> cos(const Vector<T, W>& value);
  template <typename T, size_t W>
  friend Vector<T, W> select(const Mask<T, W>& mask, const Vector<T, W>& ifTrue, const Vector<T, W>& ifFalse);
};

template <typename DataType, size_t DataWidth>
Vector<DataType, DataWidth> min(const Vector<DataType, DataWidth>& lhs, const Vector<DataType, DataWidth>& rhs) {
  Vector<DataType, DataWidth> result;
  for (size_t i = 0; i < DataWidth; ++i) {
    result.lanes_[i] = std::min(lhs.lanes_[i], rhs.lanes_[i]);
  }
  return result;
}

template <typename DataType, size_t DataWidth>
Vector<DataType, DataWidth> max(const Vector<DataType, DataWidth>& lhs, const Vector<DataType, DataWidth>& rhs) {
  Vector<DataType, DataWidth> result;
  for (size_t i = 0; i < DataWidth; ++i) {
    result.lanes_[i] = std::max(lhs.lanes_[i], rhs.lanes_[i]);
  }
  return result;
}

template <typename DataType, size_t DataWidth>
Vector<DataType, DataWidth> clamp(const Vector<DataType, DataWidth>& value, const Vector<DataType, DataWidth>& low,
                                  const Vector<DataType, DataWidth>& high) {
  return min(max(value, low), high);
}

template <typename DataType, size_t DataWidth>
Vector<DataType, DataWidth> sqrt(const Vector<DataType, DataWidth>& value) {
  Vector<DataType, DataWidth> result;
  for (size_t i = 0; i < DataWidth; ++i) {
    result.lanes_[i] = std::sqrt(value.lanes_[i]);
  }
  return result;
}

template <typename DataType, size_t DataWidth>
Vector<DataType, DataWidth> sin(const Vector<DataType, DataWidth>& value) {
  Vector<DataType, DataWidth> result;
  for (size_t i = 0; i < DataWidth; ++i) {
    result.lanes_[i] = std::sin(value.lanes_[i]);
  }
  return result;
}

template <typename DataType, size_t DataWidth>
Vector<DataType, DataWidth> cos(const Vector<DataType, DataWidth>& value) {
  Vector<DataType, DataWidth> result;
  for (size_t i = 0; i < DataWidth; ++i) {
    result.lanes_[i] = std::cos(value.lanes_[i]);
  }
  return result;
}

template <typename DataType, size_t DataWidth>
Vector<DataType, DataWidth> select(const Mask<DataType, DataWidth>& mask, const Vector<DataType, DataWidth>& ifTrue,
                                   const Vector<DataType, DataWidth>& ifFalse) {
  Vector<DataType, DataWidth> result;
  for (size_t i = 0; i < DataWidth; ++i) {
    result.lanes_[i] = mask.lane(i) ? ifTrue.lanes_[i] : ifFalse.lanes_[i];
  }
  return result;
}

#if defined(OCS2_COLLISION_NEXTGEN_USE_AVX2)

template <>
class alignas(32) Mask<double, 4> {
 public:
  using Scalar = double;
  static constexpr size_t width = 4;

  Mask() : value_(_mm256_setzero_pd()) {}
  explicit Mask(bool value)
      : value_(_mm256_castsi256_pd(_mm256_set1_epi64x(value ? -1 : 0))) {}
  explicit Mask(__m256d value) : value_(value) {}

  bool lane(size_t index) const {
    return (_mm256_movemask_pd(value_) & (1 << index)) != 0;
  }
  void setLane(size_t index, bool value) {
    const __m256d laneMask = _mm256_castsi256_pd(
        _mm256_set_epi64x(index == 3 ? -1 : 0, index == 2 ? -1 : 0,
                          index == 1 ? -1 : 0, index == 0 ? -1 : 0));
    value_ = value ? _mm256_or_pd(value_, laneMask)
                   : _mm256_andnot_pd(laneMask, value_);
  }

 private:
  __m256d value_;

  friend class Vector<double, 4>;
  friend Mask operator||(const Mask& lhs, const Mask& rhs) { return Mask(_mm256_or_pd(lhs.value_, rhs.value_)); }
  friend Mask operator&&(const Mask& lhs, const Mask& rhs) { return Mask(_mm256_and_pd(lhs.value_, rhs.value_)); }
  friend Vector<double, 4> select(const Mask& mask, const Vector<double, 4>& ifTrue, const Vector<double, 4>& ifFalse);
};

template <>
class alignas(32) Vector<double, 4> {
 public:
  using Scalar = double;
  using MaskType = Mask<double, 4>;
  using EigenVector = Eigen::Matrix<double, 4, 1>;
  static constexpr size_t width = 4;

  Vector() : value_(_mm256_setzero_pd()) {}
  explicit Vector(double value) : value_(_mm256_set1_pd(value)) {}
  explicit Vector(__m256d value) : value_(value) {}

  static Vector set(double value) { return Vector(value); }
  static Vector loadAligned(const double* ptr) { return Vector(_mm256_load_pd(ptr)); }
  static Vector loadUnaligned(const double* ptr) { return Vector(_mm256_loadu_pd(ptr)); }
  static Vector gather(const double* base, const std::int64_t* indices) {
    const __m256i index = _mm256_set_epi64x(indices[3], indices[2], indices[1], indices[0]);
    return Vector(_mm256_i64gather_pd(base, index, 8));
  }

  static Vector fromEigen(const EigenVector& values) { return loadUnaligned(values.data()); }

  template <typename Derived>
  static Vector loadEigen(const Eigen::MatrixBase<Derived>& values, Eigen::Index offset = 0) {
    return loadUnaligned(values.derived().data() + offset);
  }

  void storeAligned(double* ptr) const { _mm256_store_pd(ptr, value_); }
  void storeUnaligned(double* ptr) const { _mm256_storeu_pd(ptr, value_); }

  template <typename Derived>
  void storeEigen(Eigen::MatrixBase<Derived>& values, Eigen::Index offset = 0) const {
    storeUnaligned(values.derived().data() + offset);
  }

  EigenVector toEigen() const {
    EigenVector result;
    storeUnaligned(result.data());
    return result;
  }

  double lane(size_t index) const {
    alignas(32) std::array<double, 4> lanes;
    storeAligned(lanes.data());
    return lanes[index];
  }

  friend Vector operator+(const Vector& lhs, const Vector& rhs) { return Vector(_mm256_add_pd(lhs.value_, rhs.value_)); }
  friend Vector operator-(const Vector& lhs, const Vector& rhs) { return Vector(_mm256_sub_pd(lhs.value_, rhs.value_)); }
  friend Vector operator*(const Vector& lhs, const Vector& rhs) { return Vector(_mm256_mul_pd(lhs.value_, rhs.value_)); }
  friend Vector operator/(const Vector& lhs, const Vector& rhs) { return Vector(_mm256_div_pd(lhs.value_, rhs.value_)); }
  friend MaskType operator<(const Vector& lhs, const Vector& rhs) {
    return MaskType(_mm256_cmp_pd(lhs.value_, rhs.value_, _CMP_LT_OQ));
  }

 private:
  __m256d value_;

  friend Vector min(const Vector& lhs, const Vector& rhs);
  friend Vector max(const Vector& lhs, const Vector& rhs);
  friend Vector sqrt(const Vector& value);
  friend Vector sin(const Vector& value);
  friend Vector cos(const Vector& value);
  friend Vector select(const Mask<double, 4>& mask, const Vector& ifTrue, const Vector& ifFalse);
  template <size_t I0, size_t I1, size_t I2, size_t I3>
  friend Vector<double, 4> permute(const Vector<double, 4>& value);
  template <size_t I0, size_t I1, size_t I2, size_t I3>
  friend Vector<double, 4> permute(const Vector<double, 4>& lhs, const Vector<double, 4>& rhs);
};

inline Vector<double, 4> min(const Vector<double, 4>& lhs, const Vector<double, 4>& rhs) {
  return Vector<double, 4>(_mm256_min_pd(lhs.value_, rhs.value_));
}

inline Vector<double, 4> max(const Vector<double, 4>& lhs, const Vector<double, 4>& rhs) {
  return Vector<double, 4>(_mm256_max_pd(lhs.value_, rhs.value_));
}

inline Vector<double, 4> sqrt(const Vector<double, 4>& value) {
  return Vector<double, 4>(_mm256_sqrt_pd(value.value_));
}

inline Vector<double, 4> sin(const Vector<double, 4>& value) {
  alignas(32) std::array<double, 4> lanes;
  value.storeAligned(lanes.data());
  for (double& lane : lanes) {
    lane = std::sin(lane);
  }
  return Vector<double, 4>::loadAligned(lanes.data());
}

inline Vector<double, 4> cos(const Vector<double, 4>& value) {
  alignas(32) std::array<double, 4> lanes;
  value.storeAligned(lanes.data());
  for (double& lane : lanes) {
    lane = std::cos(lane);
  }
  return Vector<double, 4>::loadAligned(lanes.data());
}

inline Vector<double, 4> select(const Mask<double, 4>& mask, const Vector<double, 4>& ifTrue, const Vector<double, 4>& ifFalse) {
  return Vector<double, 4>(_mm256_blendv_pd(ifFalse.value_, ifTrue.value_, mask.value_));
}

#endif

template <size_t I0, size_t I1, size_t I2, size_t I3>
inline Vector<double, 4> permute(const Vector<double, 4>& value) {
  static_assert(I0 < 4 && I1 < 4 && I2 < 4 && I3 < 4, "SIMD permutation indices must be smaller than four");
#if defined(OCS2_COLLISION_NEXTGEN_USE_AVX2)
  return Vector<double, 4>(_mm256_permute4x64_pd(value.value_, _MM_SHUFFLE(I3, I2, I1, I0)));
#else
  const std::array<double, 4> lanes{value.lane(I0), value.lane(I1), value.lane(I2), value.lane(I3)};
  return Vector<double, 4>::loadUnaligned(lanes.data());
#endif
}

template <size_t I0, size_t I1, size_t I2, size_t I3>
inline Vector<double, 4> permute(const Vector<double, 4>& lhs, const Vector<double, 4>& rhs) {
  static_assert(I0 < 8 && I1 < 8 && I2 < 8 && I3 < 8, "SIMD binary permutation indices must be smaller than eight");
#if defined(OCS2_COLLISION_NEXTGEN_USE_AVX2)
  if constexpr (I0 == 0 && I1 == 4 && I2 == 2 && I3 == 6) {
    return Vector<double, 4>(_mm256_unpacklo_pd(lhs.value_, rhs.value_));
  } else if constexpr (I0 == 1 && I1 == 5 && I2 == 3 && I3 == 7) {
    return Vector<double, 4>(_mm256_unpackhi_pd(lhs.value_, rhs.value_));
  } else if constexpr (I0 == 0 && I1 == 1 && I2 == 4 && I3 == 5) {
    return Vector<double, 4>(_mm256_permute2f128_pd(lhs.value_, rhs.value_, 0x20));
  } else if constexpr (I0 == 2 && I1 == 3 && I2 == 6 && I3 == 7) {
    return Vector<double, 4>(_mm256_permute2f128_pd(lhs.value_, rhs.value_, 0x31));
  } else {
    constexpr int control = _MM_SHUFFLE(I3 & 3, I2 & 3, I1 & 3, I0 & 3);
    constexpr int rhsMask = (I0 >= 4 ? 0x1 : 0) | (I1 >= 4 ? 0x2 : 0) | (I2 >= 4 ? 0x4 : 0) |
                            (I3 >= 4 ? 0x8 : 0);
    const __m256d lhsPermuted = _mm256_permute4x64_pd(lhs.value_, control);
    const __m256d rhsPermuted = _mm256_permute4x64_pd(rhs.value_, control);
    return Vector<double, 4>(_mm256_blend_pd(lhsPermuted, rhsPermuted, rhsMask));
  }
#else
  const std::array<double, 8> source{lhs.lane(0), lhs.lane(1), lhs.lane(2), lhs.lane(3),
                                     rhs.lane(0), rhs.lane(1), rhs.lane(2), rhs.lane(3)};
  const std::array<double, 4> lanes{source[I0], source[I1], source[I2], source[I3]};
  return Vector<double, 4>::loadUnaligned(lanes.data());
#endif
}

/**
 * In-place four-by-four transpose implemented as two lane-permutation stages.
 * Each bracket is one four-lane packet.
 *
 * Input packets:
 *
 *   row0 = [ a0 a1 a2 a3 ]
 *   row1 = [ b0 b1 b2 b3 ]
 *   row2 = [ c0 c1 c2 c3 ]
 *   row3 = [ d0 d1 d2 d3 ]
 *
 * Stage 1, interleave matching even/odd lanes from two rows:
 *
 *   low01  = [ a0 b0 a2 b2 ]   high01 = [ a1 b1 a3 b3 ]
 *   low23  = [ c0 d0 c2 d2 ]   high23 = [ c1 d1 c3 d3 ]
 *
 * Stage 2, combine the low and high 128-bit halves:
 *
 *   column0 = [ a0 b0 c0 d0 ]
 *   column1 = [ a1 b1 c1 d1 ]
 *   column2 = [ a2 b2 c2 d2 ]
 *   column3 = [ a3 b3 c3 d3 ]
 *
 * Output assignment: row0 = column0, ..., row3 = column3.
 */
inline void transpose4x4(Vector<double, 4>& row0, Vector<double, 4>& row1, Vector<double, 4>& row2,
                         Vector<double, 4>& row3) {
  const Vector<double, 4> low01 = permute<0, 4, 2, 6>(row0, row1);
  const Vector<double, 4> high01 = permute<1, 5, 3, 7>(row0, row1);
  const Vector<double, 4> low23 = permute<0, 4, 2, 6>(row2, row3);
  const Vector<double, 4> high23 = permute<1, 5, 3, 7>(row2, row3);
  const Vector<double, 4> column0 = permute<0, 1, 4, 5>(low01, low23);
  const Vector<double, 4> column1 = permute<0, 1, 4, 5>(high01, high23);
  const Vector<double, 4> column2 = permute<2, 3, 6, 7>(low01, low23);
  const Vector<double, 4> column3 = permute<2, 3, 6, 7>(high01, high23);
  row0 = column0;
  row1 = column1;
  row2 = column2;
  row3 = column3;
}

}  // namespace simd
}  // namespace impl
}  // namespace collision_nextgen
}  // namespace ocs2
