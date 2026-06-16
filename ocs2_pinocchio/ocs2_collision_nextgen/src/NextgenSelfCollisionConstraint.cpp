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

#include <Eigen/Geometry>

#include <pinocchio/algorithm/jacobian.hpp>

#include <memory>
#include <stdexcept>
#include <tuple>
#include <utility>

namespace ocs2 {
namespace collision_nextgen {

namespace {

using vector3_t = impl::vector3_t;

}  // namespace

NextgenSelfCollisionConstraint::NextgenSelfCollisionConstraint(const PinocchioStateInputMapping<scalar_t>& mapping,
                                                               const pinocchio::Model& model,
                                                               const pinocchio::GeometryModel& geometryModel,
                                                               scalar_t minimumDistance)
    : StateConstraint(ConstraintOrder::Linear),
      sphereModelPtr_(std::make_unique<impl::SphereCollisionModel>(model, geometryModel)),
      minimumDistance_(minimumDistance),
      mappingPtr_(mapping.clone()) {}

NextgenSelfCollisionConstraint::NextgenSelfCollisionConstraint(const NextgenSelfCollisionConstraint& other)
    : StateConstraint(other),
      sphereModelPtr_(std::make_unique<impl::SphereCollisionModel>(*other.sphereModelPtr_)),
      minimumDistance_(other.minimumDistance_),
      mappingPtr_(other.mappingPtr_->clone()) {}

NextgenSelfCollisionConstraint::~NextgenSelfCollisionConstraint() = default;

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

  const auto evaluation = sphereModelPtr_->evaluate(pinocchioInterface);
  const auto& model = pinocchioInterface.getModel();

  VectorFunctionLinearApproximation constraint;
  constraint.f = evaluation.distances;
  constraint.f.array() -= minimumDistance_;

  matrix_t dfdq = matrix_t::Zero(evaluation.distances.rows(), model.nq);
  std::vector<matrix_t> jointJacobianCache(model.joints.size());
  std::vector<bool> jointJacobianCached(model.joints.size(), false);
  auto getCachedJointJacobian = [&](size_t joint) -> const matrix_t& {
    if (joint >= jointJacobianCache.size()) {
      throw std::runtime_error("[NextgenSelfCollisionConstraint] parent joint index is outside the Pinocchio model.");
    }
    if (!jointJacobianCached[joint]) {
      jointJacobianCache[joint].setZero(6, model.nv);
      pinocchio::getJointJacobian(model, pinocchioInterface.getData(), joint, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED,
                                  jointJacobianCache[joint]);
      jointJacobianCached[joint] = true;
    }
    return jointJacobianCache[joint];
  };

  for (int i = 0; i < evaluation.distances.rows(); ++i) {
    const auto pairIndex = static_cast<size_t>(i);
    const auto firstJoint = sphereModelPtr_->getFirstParentJoint(pairIndex);
    const auto secondJoint = sphereModelPtr_->getSecondParentJoint(pairIndex);
    const auto& firstJacobian = getCachedJointJacobian(firstJoint);
    const auto& secondJacobian = getCachedJointJacobian(secondJoint);
    const auto& normal = evaluation.normals[pairIndex];
    const vector3_t firstOffset = evaluation.firstCenters[pairIndex] - pinocchioInterface.getData().oMi[firstJoint].translation();
    const vector3_t secondOffset = evaluation.secondCenters[pairIndex] - pinocchioInterface.getData().oMi[secondJoint].translation();

    dfdq.row(i).leftCols(model.nv).noalias() = normal.transpose() * (secondJacobian.topRows(3) - firstJacobian.topRows(3)) -
                                               normal.cross(secondOffset).transpose() * secondJacobian.bottomRows(3) +
                                               normal.cross(firstOffset).transpose() * firstJacobian.bottomRows(3);
  }

  matrix_t dfdv = matrix_t::Zero(dfdq.rows(), dfdq.cols());
  std::tie(constraint.dfdx, std::ignore) = mappingPtr_->getOcs2Jacobian(state, dfdq, dfdv);
  return constraint;
}

}  // namespace collision_nextgen
}  // namespace ocs2
