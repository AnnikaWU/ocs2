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

#include <ocs2_collision_nextgen/NextgenSelfCollisionConstraint.h>

#include <ocs2_collision_nextgen/impl/SphereCollisionModel.h>
#include <ocs2_collision_nextgen/impl/SphereJacobianKernel.h>

#include <boost/dynamic_bitset.hpp>

#include <pinocchio/algorithm/jacobian.hpp>

#include <limits>
#include <memory>
#include <stdexcept>
#include <tuple>
#include <utility>

namespace ocs2 {
namespace collision_nextgen {

namespace {

std::int64_t checkedJointIndex(size_t joint) {
  if (joint > static_cast<size_t>(std::numeric_limits<std::int64_t>::max())) {
    throw std::runtime_error("[NextgenSelfCollisionConstraint] parent joint index does not fit into int64_t for SIMD gather.");
  }
  return static_cast<std::int64_t>(joint);
}

}  // namespace

NextgenSelfCollisionConstraint::NextgenSelfCollisionConstraint(const PinocchioStateInputMapping<scalar_t>& mapping,
                                                               const pinocchio::Model& model,
                                                               const pinocchio::GeometryModel& geometryModel,
                                                               scalar_t minimumDistance)
    : StateConstraint(ConstraintOrder::Linear),
      sphereModelPtr_(std::make_unique<impl::SphereCollisionModel>(model, geometryModel)),
      minimumDistance_(minimumDistance),
      mappingPtr_(mapping.clone()) {
  initializeJacobianStorage(model);
}

NextgenSelfCollisionConstraint::NextgenSelfCollisionConstraint(const NextgenSelfCollisionConstraint& other)
    : StateConstraint(other),
      sphereModelPtr_(std::make_unique<impl::SphereCollisionModel>(*other.sphereModelPtr_)),
      minimumDistance_(other.minimumDistance_),
      mappingPtr_(other.mappingPtr_->clone()),
      evaluationScratchPtr_(std::make_unique<impl::SphereCollisionEvaluation>(*other.evaluationScratchPtr_)),
      activeJoints_(other.activeJoints_),
      firstJoint_(other.firstJoint_),
      secondJoint_(other.secondJoint_),
      jointRuns_(other.jointRuns_),
      jointJacobianTemporary_(other.jointJacobianTemporary_),
      jointJacobianCache_(other.jointJacobianCache_),
      jointJacobianDofStride_(other.jointJacobianDofStride_),
      dfdqScratch_(other.dfdqScratch_),
      dfdvScratch_(other.dfdvScratch_) {}

NextgenSelfCollisionConstraint::~NextgenSelfCollisionConstraint() = default;

void NextgenSelfCollisionConstraint::initializeJacobianStorage(const pinocchio::Model& model) {
  const size_t numPairs = sphereModelPtr_->getNumPairs();
  const size_t numJoints = model.joints.size();
  evaluationScratchPtr_ = std::make_unique<impl::SphereCollisionEvaluation>(numPairs);
  firstJoint_.resize(numPairs);
  secondJoint_.resize(numPairs);

  boost::dynamic_bitset<> activeJointMask(numJoints);
  for (size_t pair = 0; pair < numPairs; ++pair) {
    const size_t first = sphereModelPtr_->getFirstParentJoint(pair);
    const size_t second = sphereModelPtr_->getSecondParentJoint(pair);
    if (first >= numJoints || second >= numJoints) {
      throw std::runtime_error("[NextgenSelfCollisionConstraint] parent joint index is outside the Pinocchio model.");
    }
    firstJoint_[pair] = checkedJointIndex(first);
    secondJoint_[pair] = checkedJointIndex(second);
    activeJointMask.set(first);
    activeJointMask.set(second);
  }
  if (numPairs > 0) {
    size_t runBegin = 0;
    for (size_t pair = 1; pair <= numPairs; ++pair) {
      const bool runEnds =
          pair == numPairs || firstJoint_[pair] != firstJoint_[runBegin] ||
          secondJoint_[pair] != secondJoint_[runBegin];
      if (runEnds) {
        jointRuns_.push_back(
            {runBegin, pair - runBegin,
             static_cast<size_t>(firstJoint_[runBegin]),
             static_cast<size_t>(secondJoint_[runBegin])});
        runBegin = pair;
      }
    }
  }
  for (size_t joint = activeJointMask.find_first(); joint != boost::dynamic_bitset<>::npos;
       joint = activeJointMask.find_next(joint)) {
    activeJoints_.push_back(joint);
  }

  jointJacobianTemporary_.resize(6, model.nv);
  jointJacobianDofStride_ = impl::sphereJacobianDofStride(model.nv);
  jointJacobianCache_.resize(6 * jointJacobianDofStride_ * numJoints, 0.0);
  // The kernel overwrites the first nv columns on every call. Keep the
  // possible nq > nv tail and the velocity Jacobian deterministically zero
  // without clearing the full pair-by-coordinate matrices in the hot path.
  dfdqScratch_.setZero(numPairs, model.nq);
  dfdvScratch_.setZero(numPairs, model.nq);
}

size_t NextgenSelfCollisionConstraint::getNumConstraints(scalar_t time) const {
  return sphereModelPtr_->getNumPairs();
}

vector_t NextgenSelfCollisionConstraint::getValue(scalar_t time, const vector_t& state, const PreComputation& preComputation) const {
  const auto& pinocchioInterface = getPinocchioInterface(preComputation);
  vector_t values = sphereModelPtr_->getDistances(pinocchioInterface);
  values.array() -= minimumDistance_;
  return values;
}

VectorFunctionLinearApproximation NextgenSelfCollisionConstraint::getLinearApproximation(scalar_t time, const vector_t& state,
                                                                                         const PreComputation& preComputation) const {
  const auto& pinocchioInterface = getPinocchioInterface(preComputation);
  mappingPtr_->setPinocchioInterface(pinocchioInterface);

  auto& evaluation = *evaluationScratchPtr_;
  sphereModelPtr_->evaluate(pinocchioInterface, evaluation);
  const auto& model = pinocchioInterface.getModel();

  VectorFunctionLinearApproximation constraint;
  constraint.f = evaluation.distances;
  constraint.f.array() -= minimumDistance_;

  const size_t numJoints = model.joints.size();
  const size_t numDofs = static_cast<size_t>(model.nv);
  for (const size_t joint : activeJoints_) {
    jointJacobianTemporary_.setZero();
    pinocchio::getJointJacobian(model, pinocchioInterface.getData(), joint, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED,
                                jointJacobianTemporary_);
    for (size_t row = 0; row < 6; ++row) {
      for (size_t dof = 0; dof < numDofs; ++dof) {
        jointJacobianCache_[(joint * 6 + row) * jointJacobianDofStride_ + dof] =
            jointJacobianTemporary_(static_cast<Eigen::Index>(row), static_cast<Eigen::Index>(dof));
      }
    }
  }

  impl::computeSpherePairJacobians(
      jointJacobianCache_.data(), numJoints, numDofs, jointJacobianDofStride_, firstJoint_.data(), secondJoint_.data(),
      evaluation.firstAngularX.data(), evaluation.firstAngularY.data(), evaluation.firstAngularZ.data(),
      evaluation.secondAngularX.data(), evaluation.secondAngularY.data(), evaluation.secondAngularZ.data(),
      evaluation.normalX.data(), evaluation.normalY.data(), evaluation.normalZ.data(),
      static_cast<size_t>(evaluation.distances.rows()), dfdqScratch_.data(),
      jointRuns_.data(), jointRuns_.size());

  if (const auto* stateOnlyMapping = dynamic_cast<const PinocchioStateOnlyJacobianMapping<scalar_t>*>(mappingPtr_.get())) {
    constraint.dfdx = stateOnlyMapping->getOcs2StateJacobian(state, dfdqScratch_, dfdvScratch_);
  } else {
    std::tie(constraint.dfdx, std::ignore) = mappingPtr_->getOcs2Jacobian(state, dfdqScratch_, dfdvScratch_);
  }
  return constraint;
}

}  // namespace collision_nextgen
}  // namespace ocs2
