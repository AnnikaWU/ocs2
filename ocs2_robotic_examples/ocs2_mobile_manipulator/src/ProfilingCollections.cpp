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

#include "ocs2_mobile_manipulator/ProfilingCollections.h"

#include <algorithm>
#include <utility>

#include "ocs2_mobile_manipulator/Profiling.h"

namespace ocs2 {
namespace mobile_manipulator {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ProfiledStateInputCostCollection::ProfiledStateInputCostCollection(std::string scope) : Base(std::move(scope)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t ProfiledStateInputCostCollection::getValue(scalar_t time, const vector_t& state, const vector_t& input,
                                                    const TargetTrajectories& targetTrajectories,
                                                    const PreComputation& preComp) const {
  scalar_t cost = 0.0;

  for (size_t i = 0; i < terms().size(); i++) {
    if (terms()[i]->isActive(time)) {
      profiling::ScopedTimer timer(scope(), "State-Input Cost Value", termName(i));
      cost += terms()[i]->getValue(time, state, input, targetTrajectories, preComp);
    }
  }

  return cost;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation ProfiledStateInputCostCollection::getQuadraticApproximation(
    scalar_t time, const vector_t& state, const vector_t& input, const TargetTrajectories& targetTrajectories,
    const PreComputation& preComp) const {
  const auto& costTerms = terms();
  const auto firstActive = std::find_if(costTerms.begin(), costTerms.end(),
                                        [time](const std::unique_ptr<StateInputCost>& costTerm) { return costTerm->isActive(time); });

  if (firstActive == costTerms.end()) {
    return ScalarFunctionQuadraticApproximation::Zero(state.rows(), input.rows());
  }

  const size_t firstActiveInd = std::distance(costTerms.begin(), firstActive);
  ScalarFunctionQuadraticApproximation cost;
  {
    // Match StateInputCostCollection behavior: initialize from the first active term, then accumulate.
    profiling::ScopedTimer timer(scope(), "State-Input Cost Quadratic", termName(firstActiveInd));
    cost = (*firstActive)->getQuadraticApproximation(time, state, input, targetTrajectories, preComp);
  }

  for (size_t i = firstActiveInd + 1; i < costTerms.size(); i++) {
    if (costTerms[i]->isActive(time)) {
      profiling::ScopedTimer timer(scope(), "State-Input Cost Quadratic", termName(i));
      cost += costTerms[i]->getQuadraticApproximation(time, state, input, targetTrajectories, preComp);
    }
  }

  return cost;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ProfiledStateCostCollection::ProfiledStateCostCollection(std::string scope) : Base(std::move(scope)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t ProfiledStateCostCollection::getValue(scalar_t time, const vector_t& state,
                                               const TargetTrajectories& targetTrajectories,
                                               const PreComputation& preComp) const {
  scalar_t cost = 0.0;

  for (size_t i = 0; i < terms().size(); i++) {
    if (terms()[i]->isActive(time)) {
      profiling::ScopedTimer timer(scope(), "State Cost Value", termName(i));
      cost += terms()[i]->getValue(time, state, targetTrajectories, preComp);
    }
  }

  return cost;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation ProfiledStateCostCollection::getQuadraticApproximation(
    scalar_t time, const vector_t& state, const TargetTrajectories& targetTrajectories, const PreComputation& preComp) const {
  const auto& costTerms = terms();
  const auto firstActive = std::find_if(costTerms.begin(), costTerms.end(),
                                        [time](const std::unique_ptr<StateCost>& costTerm) { return costTerm->isActive(time); });

  if (firstActive == costTerms.end()) {
    return ScalarFunctionQuadraticApproximation::Zero(state.rows());
  }

  const size_t firstActiveInd = std::distance(costTerms.begin(), firstActive);
  ScalarFunctionQuadraticApproximation cost;
  {
    // State-only terms leave input derivatives empty; preserve the base collection's accumulation shape.
    profiling::ScopedTimer timer(scope(), "State Cost Quadratic", termName(firstActiveInd));
    cost = (*firstActive)->getQuadraticApproximation(time, state, targetTrajectories, preComp);
  }

  for (size_t i = firstActiveInd + 1; i < costTerms.size(); i++) {
    if (costTerms[i]->isActive(time)) {
      profiling::ScopedTimer timer(scope(), "State Cost Quadratic", termName(i));
      const auto costTermApproximation = costTerms[i]->getQuadraticApproximation(time, state, targetTrajectories, preComp);
      cost.f += costTermApproximation.f;
      cost.dfdx += costTermApproximation.dfdx;
      cost.dfdxx += costTermApproximation.dfdxx;
    }
  }

  cost.dfdu = vector_t();
  cost.dfduu = matrix_t();
  cost.dfdux = matrix_t();

  return cost;
}

}  // namespace mobile_manipulator
}  // namespace ocs2
