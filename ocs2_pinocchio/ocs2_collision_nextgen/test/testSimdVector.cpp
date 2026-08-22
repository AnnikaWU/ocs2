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

#include <array>
#include <algorithm>
#include <cmath>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include <ocs2_collision_nextgen/impl/simd/Vector.h>

using namespace ocs2::collision_nextgen::impl;

TEST(SimdVector, ArithmeticMaskAndMath) {
  using Packet = simd::Vector<double, simd::NativeWidth<double>>;
  static_assert(Packet::width >= 1);

  alignas(simd::kDefaultAlignment) std::array<double, Packet::width> input{};
  alignas(simd::kDefaultAlignment) std::array<double, Packet::width> output{};
  for (size_t i = 0; i < Packet::width; ++i) {
    input[i] = -0.5 + static_cast<double>(i);
  }

  const Packet x = Packet::loadAligned(input.data());
  const Packet clamped = simd::clamp(x, Packet::set(-0.25), Packet::set(0.75));
  clamped.storeAligned(output.data());
  for (size_t i = 0; i < Packet::width; ++i) {
    EXPECT_DOUBLE_EQ(output[i], std::clamp(input[i], -0.25, 0.75));
  }

  const auto mask = x < Packet::set(0.0);
  simd::select(mask, Packet::set(10.0), Packet::set(20.0)).storeAligned(output.data());
  for (size_t i = 0; i < Packet::width; ++i) {
    EXPECT_DOUBLE_EQ(output[i], input[i] < 0.0 ? 10.0 : 20.0);
  }

  simd::sin(x).storeAligned(output.data());
  for (size_t i = 0; i < Packet::width; ++i) {
    EXPECT_NEAR(output[i], std::sin(input[i]), 1e-14);
  }

  simd::cos(x).storeAligned(output.data());
  for (size_t i = 0; i < Packet::width; ++i) {
    EXPECT_NEAR(output[i], std::cos(input[i]), 1e-14);
  }
}

TEST(SimdVector, EigenLoadStore) {
  using Packet = simd::Vector<double, simd::NativeWidth<double>>;

  Eigen::Matrix<double, Eigen::Dynamic, 1> values(Packet::width + 2);
  values.setZero();
  for (size_t i = 0; i < Packet::width; ++i) {
    values[1 + static_cast<Eigen::Index>(i)] = 0.25 * static_cast<double>(i + 1);
  }

  const Packet packet = Packet::loadEigen(values, 1);
  Eigen::Matrix<double, Eigen::Dynamic, 1> output = Eigen::Matrix<double, Eigen::Dynamic, 1>::Zero(Packet::width + 2);
  packet.storeEigen(output, 1);

  for (size_t i = 0; i < Packet::width; ++i) {
    EXPECT_DOUBLE_EQ(output[1 + static_cast<Eigen::Index>(i)], values[1 + static_cast<Eigen::Index>(i)]);
  }
}
