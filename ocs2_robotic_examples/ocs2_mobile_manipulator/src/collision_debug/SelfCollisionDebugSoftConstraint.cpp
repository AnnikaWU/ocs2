/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  list of conditions and the following disclaimer in the documentation
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

#include <ocs2_mobile_manipulator/collision_debug/SelfCollisionDebugSoftConstraint.h>

#include <stdexcept>

namespace ocs2 {
namespace mobile_manipulator {

SelfCollisionDebugSoftConstraint::SelfCollisionDebugSoftConstraint(std::unique_ptr<StateConstraint> activeConstraintPtr,
                                                                   std::unique_ptr<PenaltyBase> penaltyFunction,
                                                                   std::shared_ptr<SelfCollisionDebugProbe> debugProbePtr)
    : activeConstraintPtr_(std::move(activeConstraintPtr)),
      penalty_(std::move(penaltyFunction)),
      debugProbePtr_(std::move(debugProbePtr)) {}

SelfCollisionDebugSoftConstraint::SelfCollisionDebugSoftConstraint(const SelfCollisionDebugSoftConstraint& other)
    : StateCost(other),
      activeConstraintPtr_(other.activeConstraintPtr_->clone()),
      penalty_(other.penalty_),
      debugProbePtr_(other.debugProbePtr_) {}

SelfCollisionDebugSoftConstraint* SelfCollisionDebugSoftConstraint::clone() const {
  return new SelfCollisionDebugSoftConstraint(*this);
}

bool SelfCollisionDebugSoftConstraint::isActive(scalar_t time) const {
  return activeConstraintPtr_->isActive(time);
}

scalar_t SelfCollisionDebugSoftConstraint::getValue(scalar_t time, const vector_t& state, const TargetTrajectories&,
                                                    const PreComputation& preComp) const {
  return penalty_.getValue(time, activeConstraintPtr_->getValue(time, state, preComp));
}

ScalarFunctionQuadraticApproximation SelfCollisionDebugSoftConstraint::getQuadraticApproximation(
    scalar_t time, const vector_t& state, const TargetTrajectories&, const PreComputation& preComp) const {
  ScalarFunctionQuadraticApproximation approximation;
  switch (activeConstraintPtr_->getOrder()) {
    case ConstraintOrder::Linear:
      approximation = penalty_.getQuadraticApproximation(time, activeConstraintPtr_->getLinearApproximation(time, state, preComp));
      break;
    case ConstraintOrder::Quadratic:
      approximation =
          penalty_.getQuadraticApproximation(time, activeConstraintPtr_->getQuadraticApproximation(time, state, preComp));
      break;
    default:
      throw std::runtime_error("[SelfCollisionDebugSoftConstraint] Unknown constraint Order");
  }

  if (debugProbePtr_ != nullptr) {
    debugProbePtr_->enqueue(time, state, preComp, approximation);
  }
  return approximation;
}

}  // namespace mobile_manipulator
}  // namespace ocs2
