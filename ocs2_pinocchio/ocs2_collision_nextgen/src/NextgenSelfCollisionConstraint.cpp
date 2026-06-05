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

#include <stdexcept>
#include <utility>

namespace ocs2 {
namespace collision_nextgen {

NextgenSelfCollisionConstraint::NextgenSelfCollisionConstraint(const PinocchioStateInputMapping<scalar_t>& mapping,
                                                               CollisionModelCache cache, scalar_t minimumDistance)
    : StateConstraint(ConstraintOrder::Linear),
      cache_(std::move(cache)),
      minimumDistance_(minimumDistance),
      mappingPtr_(mapping.clone()) {}

NextgenSelfCollisionConstraint::NextgenSelfCollisionConstraint(const NextgenSelfCollisionConstraint& other)
    : StateConstraint(other),
      cache_(other.cache_),
      minimumDistance_(other.minimumDistance_),
      mappingPtr_(other.mappingPtr_->clone()) {}

size_t NextgenSelfCollisionConstraint::getNumConstraints(scalar_t time) const {
  return cache_.getNumCollisionPairs();
}

vector_t NextgenSelfCollisionConstraint::getValue(scalar_t time, const vector_t& state, const PreComputation& preComputation) const {
  throwNotImplemented();
}

VectorFunctionLinearApproximation NextgenSelfCollisionConstraint::getLinearApproximation(scalar_t time, const vector_t& state,
                                                                                         const PreComputation& preComputation) const {
  throwNotImplemented();
}

void NextgenSelfCollisionConstraint::throwNotImplemented() const {
  throw std::runtime_error("[NextgenSelfCollisionConstraint] collision_nextgen backend is wired but distance evaluation is not implemented.");
}

}  // namespace collision_nextgen
}  // namespace ocs2
