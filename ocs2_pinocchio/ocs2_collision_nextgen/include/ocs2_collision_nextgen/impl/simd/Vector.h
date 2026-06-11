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

#if defined(OCS2_COLLISION_NEXTGEN_USE_AVX2) && defined(__AVX2__)
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

template <typename DataType>
inline constexpr size_t NativeWidth = 1;

#if defined(OCS2_COLLISION_NEXTGEN_USE_AVX2) && defined(__AVX2__)
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

#if defined(OCS2_COLLISION_NEXTGEN_USE_AVX2) && defined(__AVX2__)

template <>
class alignas(32) Mask<double, 4> {
 public:
  using Scalar = double;
  static constexpr size_t width = 4;

  Mask() : value_(_mm256_setzero_pd()) {}
  explicit Mask(__m256d value) : value_(value) {}

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

}  // namespace simd
}  // namespace impl
}  // namespace collision_nextgen
}  // namespace ocs2
