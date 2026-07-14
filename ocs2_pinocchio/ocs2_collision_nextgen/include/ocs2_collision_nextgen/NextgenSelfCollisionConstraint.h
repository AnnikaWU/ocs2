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
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <ocs2_collision_nextgen/impl/SphereJacobianKernel.h>
#include <ocs2_collision_nextgen/impl/simd/AlignedAllocator.h>
#include <ocs2_core/constraint/StateConstraint.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_pinocchio_interface/PinocchioStateInputMapping.h>
#include <pinocchio/multibody/fwd.hpp>

namespace pinocchio {
struct GeometryModel;
}  // namespace pinocchio

namespace ocs2 {
namespace collision_nextgen {

namespace impl {
class SphereCollisionModel;
struct SphereCollisionEvaluation;
}  // namespace impl

class NextgenSelfCollisionConstraint : public StateConstraint {
 public:
  NextgenSelfCollisionConstraint(const PinocchioStateInputMapping<scalar_t>& mapping, const pinocchio::Model& model,
                                 const pinocchio::GeometryModel& geometryModel, scalar_t minimumDistance);
  NextgenSelfCollisionConstraint(const NextgenSelfCollisionConstraint& other);
  ~NextgenSelfCollisionConstraint() override;

  NextgenSelfCollisionConstraint* clone() const override = 0;

  size_t getNumConstraints(scalar_t time) const override;
  vector_t getValue(scalar_t time, const vector_t& state, const PreComputation& preComputation) const override;
  VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state,
                                                           const PreComputation& preComputation) const override;

 protected:
  virtual const PinocchioInterface& getPinocchioInterface(const PreComputation& preComputation) const = 0;

  std::unique_ptr<impl::SphereCollisionModel> sphereModelPtr_;
  scalar_t minimumDistance_ = 0.0;
  std::unique_ptr<PinocchioStateInputMapping<scalar_t>> mappingPtr_;

 private:
  void initializeJacobianStorage(const pinocchio::Model& model);

  std::unique_ptr<impl::SphereCollisionEvaluation> evaluationScratchPtr_;
  std::vector<size_t> activeJoints_;
  std::vector<std::int64_t> firstJoint_;
  std::vector<std::int64_t> secondJoint_;
  std::vector<impl::SphereJacobianJointRun> jointRuns_;
  mutable matrix_t jointJacobianTemporary_;
  mutable std::vector<scalar_t, impl::simd::AlignedAllocator<scalar_t>> jointJacobianCache_;
  size_t jointJacobianDofStride_ = 0;
  mutable matrix_t dfdqScratch_;
  mutable matrix_t dfdvScratch_;
};

}  // namespace collision_nextgen
}  // namespace ocs2
