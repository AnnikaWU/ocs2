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

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <ocs2_core/cost/StateCostCollection.h>
#include <ocs2_core/cost/StateInputCostCollection.h>

namespace ocs2 {
namespace mobile_manipulator {

/**
 * CRTP wrapper that keeps profiling local to the mobile manipulator package.
 *
 * The OCS2 Collection base does not expose term names in insertion order, so this wrapper mirrors
 * add/extract operations to keep termNames_ aligned with the protected terms_ vector.
 */
template <typename Derived, typename BaseCollection, typename Term>
class ProfiledCollection : public BaseCollection {
 public:
  explicit ProfiledCollection(std::string scope) : scope_(std::move(scope)) {}

  BaseCollection* clone() const override { return new Derived(static_cast<const Derived&>(*this)); }

  void add(std::string name, std::unique_ptr<Term> term) {
    termNames_.push_back(name);
    try {
      BaseCollection::add(std::move(name), std::move(term));
    } catch (...) {
      termNames_.pop_back();
      throw;
    }
  }

  void clear() {
    BaseCollection::clear();
    termNames_.clear();
  }

  std::unique_ptr<Term> extract(const std::string& name) {
    size_t index = 0;
    if (!this->getTermIndex(name, index)) {
      return nullptr;
    }

    auto term = BaseCollection::extract(name);
    if (term != nullptr) {
      termNames_.erase(termNames_.begin() + index);
    }
    return term;
  }

  bool erase(const std::string& name) { return extract(name) != nullptr; }

 protected:
  ProfiledCollection(const ProfiledCollection& other) : BaseCollection(other), scope_(other.scope_), termNames_(other.termNames_) {}

  const std::string& scope() const { return scope_; }
  const std::string& termName(size_t index) const { return termNames_[index]; }
  const std::vector<std::unique_ptr<Term>>& terms() const { return this->terms_; }

 private:
  std::string scope_;
  std::vector<std::string> termNames_;
};

class ProfiledStateInputCostCollection final
    : public ProfiledCollection<ProfiledStateInputCostCollection, StateInputCostCollection, StateInputCost> {
 public:
  using Base = ProfiledCollection<ProfiledStateInputCostCollection, StateInputCostCollection, StateInputCost>;

  explicit ProfiledStateInputCostCollection(std::string scope);

  scalar_t getValue(scalar_t time, const vector_t& state, const vector_t& input, const TargetTrajectories& targetTrajectories,
                    const PreComputation& preComp) const override;

  ScalarFunctionQuadraticApproximation getQuadraticApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                                 const TargetTrajectories& targetTrajectories,
                                                                 const PreComputation& preComp) const override;

 private:
  friend Base;
  ProfiledStateInputCostCollection(const ProfiledStateInputCostCollection& other) = default;
};

class ProfiledStateCostCollection final : public ProfiledCollection<ProfiledStateCostCollection, StateCostCollection, StateCost> {
 public:
  using Base = ProfiledCollection<ProfiledStateCostCollection, StateCostCollection, StateCost>;

  explicit ProfiledStateCostCollection(std::string scope);

  scalar_t getValue(scalar_t time, const vector_t& state, const TargetTrajectories& targetTrajectories,
                    const PreComputation& preComp) const override;

  ScalarFunctionQuadraticApproximation getQuadraticApproximation(scalar_t time, const vector_t& state,
                                                                 const TargetTrajectories& targetTrajectories,
                                                                 const PreComputation& preComp) const override;

 private:
  friend Base;
  ProfiledStateCostCollection(const ProfiledStateCostCollection& other) = default;
};

}  // namespace mobile_manipulator
}  // namespace ocs2
