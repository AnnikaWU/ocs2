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

#include <cstdint>
#include <string>
#include <vector>

#include <Eigen/Core>

#include <ocs2_collision_nextgen/impl/simd/AlignedAllocator.h>
#include <ocs2_core/Types.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <pinocchio/multibody/fwd.hpp>

namespace pinocchio {
struct GeometryModel;
}  // namespace pinocchio

namespace ocs2 {
namespace collision_nextgen {
namespace impl {

using vector3_t = Eigen::Matrix<scalar_t, 3, 1>;

template <typename T>
using AlignedVector = std::vector<T, simd::AlignedAllocator<T>>;

struct SphereCollisionEvaluation {
  explicit SphereCollisionEvaluation(size_t numPairs = 0) { resize(numPairs); }

  void resize(size_t numPairs) {
    distances.resize(numPairs);
    firstAngularX.resize(numPairs);
    firstAngularY.resize(numPairs);
    firstAngularZ.resize(numPairs);
    secondAngularX.resize(numPairs);
    secondAngularY.resize(numPairs);
    secondAngularZ.resize(numPairs);
    normalX.resize(numPairs);
    normalY.resize(numPairs);
    normalZ.resize(numPairs);
  }

  vector_t distances;
  AlignedVector<double> firstAngularX;
  AlignedVector<double> firstAngularY;
  AlignedVector<double> firstAngularZ;
  AlignedVector<double> secondAngularX;
  AlignedVector<double> secondAngularY;
  AlignedVector<double> secondAngularZ;
  AlignedVector<double> normalX;
  AlignedVector<double> normalY;
  AlignedVector<double> normalZ;
};

class SphereCollisionModel final {
 public:
  SphereCollisionModel(const pinocchio::Model& model, const pinocchio::GeometryModel& geometryModel);

  size_t getNumObjects() const { return parentJoint_.size(); }
  size_t getNumPairs() const { return firstObject_.size(); }
  bool empty() const { return getNumPairs() == 0; }

  vector_t getDistances(const PinocchioInterface& pinocchioInterface) const;
  SphereCollisionEvaluation evaluate(const PinocchioInterface& pinocchioInterface) const;
  void evaluate(const PinocchioInterface& pinocchioInterface, SphereCollisionEvaluation& evaluation) const;

  size_t getFirstParentJoint(size_t pairIndex) const;
  size_t getSecondParentJoint(size_t pairIndex) const;

 private:
  struct WorldCenterScratch {
    AlignedVector<double> x;
    AlignedVector<double> y;
    AlignedVector<double> z;
    AlignedVector<double> offsetX;
    AlignedVector<double> offsetY;
    AlignedVector<double> offsetZ;
  };

  const WorldCenterScratch& computeWorldCenters(const PinocchioInterface& pinocchioInterface) const;
  void computeDistances(const WorldCenterScratch& worldCenters, double* distances, double* normalX, double* normalY,
                        double* normalZ) const;

  std::vector<std::string> objectNames_;
  std::vector<size_t> parentJoint_;
  AlignedVector<double> localX_;
  AlignedVector<double> localY_;
  AlignedVector<double> localZ_;
  AlignedVector<double> radius_;
  AlignedVector<std::int64_t> firstObject_;
  AlignedVector<std::int64_t> secondObject_;
  AlignedVector<double> pairRadiusSum_;
  mutable WorldCenterScratch worldCenterScratch_;
};

}  // namespace impl
}  // namespace collision_nextgen
}  // namespace ocs2
