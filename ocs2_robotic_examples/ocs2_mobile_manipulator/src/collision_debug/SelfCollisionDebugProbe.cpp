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

#include <ocs2_mobile_manipulator/collision_debug/SelfCollisionDebugProbe.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <ocs2_core/ComputationRequest.h>
#include <ocs2_core/penalties/MultidimensionalPenalty.h>
#include <ocs2_mobile_manipulator/FactoryFunctions.h>
#include <ocs2_mobile_manipulator/MobileManipulatorPinocchioMapping.h>
#include <ocs2_mobile_manipulator/MobileManipulatorPreComputation.h>
#include <ocs2_mobile_manipulator/constraint/MobileManipulatorNextgenSelfCollisionConstraint.h>
#include <ocs2_mobile_manipulator/constraint/MobileManipulatorSelfCollisionConstraint.h>
#include <ocs2_mobile_manipulator/third_party/moodycamel/readerwriterqueue.h>
#include <ocs2_self_collision/PinocchioGeometryInterface.h>

#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/multibody/model.hpp>

namespace ocs2 {
namespace mobile_manipulator {
namespace {

scalar_t maxAbsCoeff(const vector_t& value) {
  return value.size() == 0 ? 0.0 : value.array().abs().maxCoeff();
}

scalar_t maxAbsCoeff(const matrix_t& value) {
  return value.size() == 0 ? 0.0 : value.array().abs().maxCoeff();
}

scalar_t relativeDifference(scalar_t lhs, scalar_t rhs) {
  const scalar_t scale = std::max<scalar_t>({1.0, std::abs(lhs), std::abs(rhs)});
  return std::abs(lhs - rhs) / scale;
}

std::string threadIdToString(std::thread::id threadId) {
  std::ostringstream stream;
  stream << threadId;
  return stream.str();
}

std::string sanitizeForTsv(std::string text) {
  std::replace(text.begin(), text.end(), '\t', ' ');
  std::replace(text.begin(), text.end(), '\n', ' ');
  std::replace(text.begin(), text.end(), '\r', ' ');
  return text;
}

scalar_t constraintViolationSse(const vector_t& constraints) {
  scalar_t result = 0.0;
  for (Eigen::Index i = 0; i < constraints.size(); ++i) {
    if (constraints[i] < 0.0) {
      result += constraints[i] * constraints[i];
    }
  }
  return result;
}

scalar_t integrateTrapezoidal(const scalar_array_t& time, const scalar_array_t& values) {
  if (time.size() != values.size()) {
    throw std::invalid_argument("[SelfCollisionDebugProbe] trajectory time/value size mismatch.");
  }

  scalar_t integral = 0.0;
  for (size_t i = 1; i < time.size(); ++i) {
    const scalar_t dt = time[i] - time[i - 1];
    if (dt < 0.0) {
      throw std::invalid_argument("[SelfCollisionDebugProbe] MPC trajectory time must be nondecreasing.");
    }
    integral += 0.5 * dt * (values[i - 1] + values[i]);
  }
  return integral;
}

std::vector<size_t> linkPairWidths(const PinocchioInterface& pinocchioInterface,
                                   const PinocchioGeometryInterface& geometryInterface,
                                   const std::vector<std::pair<std::string, std::string>>& collisionLinkPairs) {
  const auto& model = pinocchioInterface.getModel();
  const auto& geometryModel = geometryInterface.getGeometryModel();

  auto countObjects = [&](const std::string& frameName) {
    size_t count = 0;
    for (const auto& object : geometryModel.geometryObjects) {
      if (model.frames[object.parentFrame].name == frameName) {
        ++count;
      }
    }
    return count;
  };

  std::vector<size_t> widths;
  widths.reserve(collisionLinkPairs.size());
  for (const auto& linkPair : collisionLinkPairs) {
    widths.push_back(countObjects(linkPair.first) * countObjects(linkPair.second));
  }
  return widths;
}

std::vector<scalar_t> linkPairMinimums(const vector_t& constraints, const std::vector<size_t>& widths) {
  const size_t expectedSize = std::accumulate(widths.begin(), widths.end(), size_t{0});
  if (expectedSize != static_cast<size_t>(constraints.size())) {
    throw std::runtime_error("[SelfCollisionDebugProbe] link-pair grouping does not match the constraint vector.");
  }

  std::vector<scalar_t> minimums;
  minimums.reserve(widths.size());
  Eigen::Index offset = 0;
  for (const size_t width : widths) {
    if (width == 0) {
      throw std::runtime_error("[SelfCollisionDebugProbe] link pair has no collision-object combinations.");
    }
    minimums.push_back(constraints.segment(offset, static_cast<Eigen::Index>(width)).minCoeff());
    offset += static_cast<Eigen::Index>(width);
  }
  return minimums;
}

}  // namespace

struct SelfCollisionDebugProbe::Slot {
  Slot(size_t slotIdParam, std::thread::id producerThreadIdParam, size_t queueCapacity)
      : slotId(slotIdParam), producerThreadId(producerThreadIdParam), queue(queueCapacity) {}

  size_t slotId = 0;
  std::thread::id producerThreadId;
  moodycamel::BlockingReaderWriterQueue<SelfCollisionDebugSample> queue;
  std::atomic<bool> stopRequested{false};
  std::thread worker;
  std::atomic<uint64_t> processedSamples{0};
  std::atomic<uint64_t> droppedSamples{0};
};

struct SelfCollisionDebugProbe::MpcEvaluator {
  struct StateMetrics {
    scalar_t activeCost = 0.0;
    scalar_t referenceCost = 0.0;
    scalar_t activeViolationSse = 0.0;
    scalar_t referenceViolationSse = 0.0;
    scalar_t activeMinConstraint = std::numeric_limits<scalar_t>::quiet_NaN();
    scalar_t referenceMinConstraint = std::numeric_limits<scalar_t>::quiet_NaN();
    scalar_t maxLinkPairMinAbsDiff = std::numeric_limits<scalar_t>::quiet_NaN();
  };

  MpcEvaluator(const std::string& activeUrdfFile, const std::string& referenceUrdfFile, ManipulatorModelType modelType,
               const std::vector<std::string>& removeJointNames,
               const std::vector<std::pair<std::string, std::string>>& collisionLinkPairs,
               const std::vector<std::pair<size_t, size_t>>& collisionObjectPairs, const ManipulatorModelInfo& modelInfo,
               scalar_t minimumDistance, const PenaltyBase& penaltyPrototype) {
    PinocchioInterface activePinocchioInterface = createPinocchioInterface(activeUrdfFile, modelType, removeJointNames);
    PinocchioGeometryInterface activeGeometryInterface(activePinocchioInterface, collisionLinkPairs, collisionObjectPairs);
    activeLinkPairWidths = linkPairWidths(activePinocchioInterface, activeGeometryInterface, collisionLinkPairs);
    activeConstraint = std::make_unique<MobileManipulatorNextgenSelfCollisionConstraint>(
        MobileManipulatorPinocchioMapping(modelInfo), activePinocchioInterface.getModel(),
        activeGeometryInterface.getGeometryModel(), minimumDistance);

    PinocchioInterface referencePinocchioInterface = createPinocchioInterface(referenceUrdfFile, modelType, removeJointNames);
    PinocchioGeometryInterface referenceGeometryInterface(referencePinocchioInterface, collisionLinkPairs, collisionObjectPairs);
    referenceLinkPairWidths = linkPairWidths(referencePinocchioInterface, referenceGeometryInterface, collisionLinkPairs);
    referenceConstraint = std::make_unique<MobileManipulatorSelfCollisionConstraint>(
        MobileManipulatorPinocchioMapping(modelInfo), std::move(referenceGeometryInterface), minimumDistance);

    preComputation = std::make_unique<MobileManipulatorPreComputation>(std::move(referencePinocchioInterface), modelInfo);
    activePenalty = std::make_unique<MultidimensionalPenalty>(std::unique_ptr<PenaltyBase>(penaltyPrototype.clone()));
    referencePenalty = std::make_unique<MultidimensionalPenalty>(std::unique_ptr<PenaltyBase>(penaltyPrototype.clone()));
    canCompareLogicalLinkPairs = collisionObjectPairs.empty();
    zeroInput = vector_t::Zero(modelInfo.inputDim);
  }

  StateMetrics evaluate(scalar_t time, const vector_t& state) {
    preComputation->request(Request::SoftConstraint, time, state, zeroInput);
    const vector_t activeValues = activeConstraint->getValue(time, state, *preComputation);
    const vector_t referenceValues = referenceConstraint->getValue(time, state, *preComputation);

    StateMetrics result;
    result.activeCost = activePenalty->getValue(time, activeValues);
    result.referenceCost = referencePenalty->getValue(time, referenceValues);
    result.activeViolationSse = constraintViolationSse(activeValues);
    result.referenceViolationSse = constraintViolationSse(referenceValues);
    if (activeValues.size() > 0) {
      result.activeMinConstraint = activeValues.minCoeff();
    }
    if (referenceValues.size() > 0) {
      result.referenceMinConstraint = referenceValues.minCoeff();
    }

    if (canCompareLogicalLinkPairs) {
      const auto activeMinimums = linkPairMinimums(activeValues, activeLinkPairWidths);
      const auto referenceMinimums = linkPairMinimums(referenceValues, referenceLinkPairWidths);
      if (activeMinimums.size() != referenceMinimums.size()) {
        throw std::runtime_error("[SelfCollisionDebugProbe] active/reference logical link-pair count mismatch.");
      }
      scalar_t maxDifference = 0.0;
      for (size_t i = 0; i < activeMinimums.size(); ++i) {
        maxDifference = std::max(maxDifference, std::abs(activeMinimums[i] - referenceMinimums[i]));
      }
      result.maxLinkPairMinAbsDiff = maxDifference;
    }
    return result;
  }

  std::unique_ptr<MobileManipulatorPreComputation> preComputation;
  std::unique_ptr<MobileManipulatorNextgenSelfCollisionConstraint> activeConstraint;
  std::unique_ptr<MobileManipulatorSelfCollisionConstraint> referenceConstraint;
  std::unique_ptr<MultidimensionalPenalty> activePenalty;
  std::unique_ptr<MultidimensionalPenalty> referencePenalty;
  vector_t zeroInput;
  std::vector<size_t> activeLinkPairWidths;
  std::vector<size_t> referenceLinkPairWidths;
  bool canCompareLogicalLinkPairs = false;

  size_t runIndex = 0;
  bool hasObservation = false;
  scalar_t observationTime = 0.0;
  StateMetrics observationMetrics;
  scalar_t realizedActiveCost = 0.0;
  scalar_t realizedReferenceCost = 0.0;
  scalar_t realizedActiveViolationSse = 0.0;
  scalar_t realizedReferenceViolationSse = 0.0;
  scalar_t cumulativePredictedSignedCostDiff = 0.0;
  scalar_t cumulativePredictedAbsCostDiff = 0.0;
};

SelfCollisionDebugProbe::SelfCollisionDebugProbe(SelfCollisionDebugSettings settings, std::string activeUrdfFile,
                                                 std::string referenceUrdfFile,
                                                 ManipulatorModelType modelType, std::vector<std::string> removeJointNames,
                                                 std::vector<std::pair<std::string, std::string>> collisionLinkPairs,
                                                 std::vector<std::pair<size_t, size_t>> collisionObjectPairs,
                                                 ManipulatorModelInfo modelInfo, scalar_t minimumDistance,
                                                 std::unique_ptr<PenaltyBase> penaltyPrototype)
    : settings_(std::move(settings)),
      activeUrdfFile_(std::move(activeUrdfFile)),
      referenceUrdfFile_(std::move(referenceUrdfFile)),
      modelType_(modelType),
      removeJointNames_(std::move(removeJointNames)),
      collisionLinkPairs_(std::move(collisionLinkPairs)),
      collisionObjectPairs_(std::move(collisionObjectPairs)),
      modelInfo_(std::move(modelInfo)),
      minimumDistance_(minimumDistance),
      penaltyPrototype_(std::move(penaltyPrototype)) {
  if (settings_.queueCapacity == 0) {
    throw std::invalid_argument("[SelfCollisionDebugProbe] queueCapacity must be positive.");
  }
  if (settings_.maxProducerThreads == 0) {
    throw std::invalid_argument("[SelfCollisionDebugProbe] maxProducerThreads must be positive.");
  }
  if (settings_.waitTimeoutUs <= 0) {
    throw std::invalid_argument("[SelfCollisionDebugProbe] waitTimeoutUs must be positive.");
  }
  if (penaltyPrototype_ == nullptr) {
    throw std::invalid_argument("[SelfCollisionDebugProbe] penaltyPrototype must not be null.");
  }

  PinocchioInterface referencePinocchioInterface = createPinocchioInterface(referenceUrdfFile_, modelType_, removeJointNames_);
  referenceGeometryInterfacePtr_ = std::make_unique<PinocchioGeometryInterface>(referencePinocchioInterface, collisionLinkPairs_,
                                                                               collisionObjectPairs_);
  mpcEvaluatorPtr_ =
      std::make_unique<MpcEvaluator>(activeUrdfFile_, referenceUrdfFile_, modelType_, removeJointNames_, collisionLinkPairs_,
                                     collisionObjectPairs_, modelInfo_, minimumDistance_, *penaltyPrototype_);

  if (!settings_.outputFile.empty()) {
    output_.open(settings_.outputFile, std::ios::out | std::ios::trunc);
    if (!output_.is_open()) {
      throw std::runtime_error("[SelfCollisionDebugProbe] Failed to open debug output file: " + settings_.outputFile);
    }
    writeHeader();
  }
  if (!settings_.mpcOutputFile.empty()) {
    mpcOutput_.open(settings_.mpcOutputFile, std::ios::out | std::ios::trunc);
    if (!mpcOutput_.is_open()) {
      throw std::runtime_error("[SelfCollisionDebugProbe] Failed to open MPC debug output file: " + settings_.mpcOutputFile);
    }
    mpcOutput_
        << "status\tmessage\tmpc_run\tobservation_time\ttrajectory_nodes\tsolver_active_total_cost\t"
           "inferred_reference_total_cost\ttotal_cost_signed_diff\ttotal_cost_abs_diff\tactive_self_collision_horizon_cost\t"
           "reference_self_collision_horizon_cost\tself_collision_horizon_signed_diff\tself_collision_horizon_abs_diff\t"
           "active_constraint_violation_sse\treference_constraint_violation_sse\tactive_min_constraint\t"
           "reference_min_constraint\tmax_link_pair_min_abs_diff\tobservation_active_cost\tobservation_reference_cost\t"
           "realized_active_cumulative_cost\trealized_reference_cumulative_cost\trealized_cumulative_signed_diff\t"
           "realized_active_violation_sse\trealized_reference_violation_sse\tcumulative_overlapping_horizon_signed_diff\t"
           "cumulative_overlapping_horizon_abs_diff\n";
  }

  std::cerr << "[SelfCollisionDebugProbe] active backend nextgen uses URDF: " << activeUrdfFile_ << '\n';
  std::cerr << "[SelfCollisionDebugProbe] reference backend pinocchio_fcl uses URDF: " << referenceUrdfFile_ << '\n';
  std::cerr << "[SelfCollisionDebugProbe] output file: "
            << (settings_.outputFile.empty() ? std::string("<disabled>") : settings_.outputFile) << '\n';
  std::cerr << "[SelfCollisionDebugProbe] MPC output file: "
            << (settings_.mpcOutputFile.empty() ? std::string("<disabled>") : settings_.mpcOutputFile) << '\n';
}

SelfCollisionDebugProbe::~SelfCollisionDebugProbe() {
  for (auto& slot : slots_) {
    slot->stopRequested.store(true, std::memory_order_release);
  }
  for (auto& slot : slots_) {
    if (slot->worker.joinable()) {
      slot->worker.join();
    }
  }

  uint64_t processed = 0;
  uint64_t perSlotDropped = 0;
  for (const auto& slot : slots_) {
    processed += slot->processedSamples.load(std::memory_order_relaxed);
    perSlotDropped += slot->droppedSamples.load(std::memory_order_relaxed);
  }
  std::cerr << "[SelfCollisionDebugProbe] processed=" << processed << ", dropped=" << droppedSamples_.load()
            << ", per-slot-dropped=" << perSlotDropped << ", producer-errors=" << producerErrors_.load() << '\n';
}

void SelfCollisionDebugProbe::enqueue(scalar_t time, const vector_t& state, const PreComputation& preComputation,
                                      const ScalarFunctionQuadraticApproximation& activeApproximation) noexcept {
  const std::thread::id producerThreadId = std::this_thread::get_id();
  const uint64_t callId = nextCallId_.fetch_add(1, std::memory_order_relaxed);

  try {
    Slot* slot = getOrCreateSlot(producerThreadId);
    if (slot == nullptr) {
      droppedSamples_.fetch_add(1, std::memory_order_relaxed);
      return;
    }

    SelfCollisionDebugSample sample;
    sample.callId = callId;
    sample.time = time;
    sample.state = state;
    sample.preComputation.reset(preComputation.clone());
    sample.producerThreadId = producerThreadId;
    sample.producerSlot = slot->slotId;
    sample.activeApproximation = activeApproximation;

    if (!slot->queue.try_enqueue(std::move(sample))) {
      slot->droppedSamples.fetch_add(1, std::memory_order_relaxed);
      droppedSamples_.fetch_add(1, std::memory_order_relaxed);
    }
  } catch (const std::exception& e) {
    recordProducerError(e.what());
  } catch (...) {
    recordProducerError("unknown producer-side exception");
  }
}

void SelfCollisionDebugProbe::startMpcRun(scalar_t currentTime, const vector_t& currentState) noexcept {
  if (mpcEvaluatorPtr_ == nullptr) {
    return;
  }

  try {
    auto& evaluator = *mpcEvaluatorPtr_;
    const auto currentMetrics = evaluator.evaluate(currentTime, currentState);

    if (evaluator.hasObservation) {
      const scalar_t dt = currentTime - evaluator.observationTime;
      if (dt < 0.0) {
        evaluator.realizedActiveCost = 0.0;
        evaluator.realizedReferenceCost = 0.0;
        evaluator.realizedActiveViolationSse = 0.0;
        evaluator.realizedReferenceViolationSse = 0.0;
      } else {
        evaluator.realizedActiveCost +=
            0.5 * dt * (evaluator.observationMetrics.activeCost + currentMetrics.activeCost);
        evaluator.realizedReferenceCost +=
            0.5 * dt * (evaluator.observationMetrics.referenceCost + currentMetrics.referenceCost);
        evaluator.realizedActiveViolationSse +=
            0.5 * dt * (evaluator.observationMetrics.activeViolationSse + currentMetrics.activeViolationSse);
        evaluator.realizedReferenceViolationSse +=
            0.5 * dt * (evaluator.observationMetrics.referenceViolationSse + currentMetrics.referenceViolationSse);
      }
    }

    evaluator.hasObservation = true;
    evaluator.observationTime = currentTime;
    evaluator.observationMetrics = currentMetrics;
    ++evaluator.runIndex;
  } catch (const std::exception& e) {
    recordProducerError(std::string("MPC observation evaluation failed: ") + e.what());
  } catch (...) {
    recordProducerError("MPC observation evaluation failed: unknown exception");
  }
}

void SelfCollisionDebugProbe::finishMpcRun(const scalar_array_t& timeTrajectory, const vector_array_t& stateTrajectory,
                                           scalar_t solverActiveTotalCost) noexcept {
  if (mpcEvaluatorPtr_ == nullptr || settings_.mpcOutputFile.empty()) {
    return;
  }

  try {
    if (timeTrajectory.size() != stateTrajectory.size() || timeTrajectory.empty()) {
      throw std::invalid_argument("[SelfCollisionDebugProbe] MPC time/state trajectory is empty or has mismatched sizes.");
    }

    auto& evaluator = *mpcEvaluatorPtr_;
    scalar_array_t activeCosts;
    scalar_array_t referenceCosts;
    scalar_array_t activeViolationSse;
    scalar_array_t referenceViolationSse;
    activeCosts.reserve(timeTrajectory.size());
    referenceCosts.reserve(timeTrajectory.size());
    activeViolationSse.reserve(timeTrajectory.size());
    referenceViolationSse.reserve(timeTrajectory.size());

    scalar_t activeMinConstraint = std::numeric_limits<scalar_t>::infinity();
    scalar_t referenceMinConstraint = std::numeric_limits<scalar_t>::infinity();
    scalar_t maxLinkPairMinAbsDiff = 0.0;
    bool hasLinkPairComparison = false;

    for (size_t i = 0; i < timeTrajectory.size(); ++i) {
      const auto metrics = evaluator.evaluate(timeTrajectory[i], stateTrajectory[i]);
      activeCosts.push_back(metrics.activeCost);
      referenceCosts.push_back(metrics.referenceCost);
      activeViolationSse.push_back(metrics.activeViolationSse);
      referenceViolationSse.push_back(metrics.referenceViolationSse);
      activeMinConstraint = std::min(activeMinConstraint, metrics.activeMinConstraint);
      referenceMinConstraint = std::min(referenceMinConstraint, metrics.referenceMinConstraint);
      if (std::isfinite(metrics.maxLinkPairMinAbsDiff)) {
        hasLinkPairComparison = true;
        maxLinkPairMinAbsDiff = std::max(maxLinkPairMinAbsDiff, metrics.maxLinkPairMinAbsDiff);
      }
    }

    const scalar_t activeSelfCollisionCost = integrateTrapezoidal(timeTrajectory, activeCosts);
    const scalar_t referenceSelfCollisionCost = integrateTrapezoidal(timeTrajectory, referenceCosts);
    const scalar_t activeConstraintViolationSse = integrateTrapezoidal(timeTrajectory, activeViolationSse);
    const scalar_t referenceConstraintViolationSse = integrateTrapezoidal(timeTrajectory, referenceViolationSse);
    const scalar_t signedDifference = activeSelfCollisionCost - referenceSelfCollisionCost;
    const scalar_t inferredReferenceTotalCost = solverActiveTotalCost - signedDifference;
    evaluator.cumulativePredictedSignedCostDiff += signedDifference;
    evaluator.cumulativePredictedAbsCostDiff += std::abs(signedDifference);

    const scalar_t nan = std::numeric_limits<scalar_t>::quiet_NaN();
    const scalar_t observationActiveCost = evaluator.hasObservation ? evaluator.observationMetrics.activeCost : nan;
    const scalar_t observationReferenceCost = evaluator.hasObservation ? evaluator.observationMetrics.referenceCost : nan;
    const scalar_t realizedSignedDifference = evaluator.realizedActiveCost - evaluator.realizedReferenceCost;

    std::lock_guard<std::mutex> lock(outputMutex_);
    if (mpcOutput_.is_open()) {
      mpcOutput_ << "ok\t\t" << evaluator.runIndex << '\t' << std::setprecision(17)
                 << (evaluator.hasObservation ? evaluator.observationTime : nan) << '\t' << timeTrajectory.size() << '\t'
                 << solverActiveTotalCost << '\t' << inferredReferenceTotalCost << '\t' << signedDifference << '\t'
                 << std::abs(signedDifference) << '\t' << activeSelfCollisionCost << '\t' << referenceSelfCollisionCost << '\t'
                 << signedDifference << '\t' << std::abs(signedDifference) << '\t' << activeConstraintViolationSse << '\t'
                 << referenceConstraintViolationSse << '\t' << activeMinConstraint << '\t' << referenceMinConstraint << '\t'
                 << (hasLinkPairComparison ? maxLinkPairMinAbsDiff : nan) << '\t' << observationActiveCost << '\t'
                 << observationReferenceCost << '\t' << evaluator.realizedActiveCost << '\t'
                 << evaluator.realizedReferenceCost << '\t' << realizedSignedDifference << '\t'
                 << evaluator.realizedActiveViolationSse << '\t' << evaluator.realizedReferenceViolationSse << '\t'
                 << evaluator.cumulativePredictedSignedCostDiff << '\t' << evaluator.cumulativePredictedAbsCostDiff << '\n';
      mpcOutput_.flush();
    }
  } catch (const std::exception& e) {
    std::lock_guard<std::mutex> lock(outputMutex_);
    if (mpcOutput_.is_open()) {
      mpcOutput_ << "mpc_error\t" << sanitizeForTsv(e.what()) << '\t' << mpcEvaluatorPtr_->runIndex << '\n';
      mpcOutput_.flush();
    }
  } catch (...) {
    std::lock_guard<std::mutex> lock(outputMutex_);
    if (mpcOutput_.is_open()) {
      mpcOutput_ << "mpc_error\tunknown MPC trajectory evaluation exception\t" << mpcEvaluatorPtr_->runIndex << '\n';
      mpcOutput_.flush();
    }
  }
}

void SelfCollisionDebugProbe::recordProducerError(const std::string& message) noexcept {
  producerErrors_.fetch_add(1, std::memory_order_relaxed);
  try {
    std::lock_guard<std::mutex> lock(outputMutex_);
    if (output_.is_open()) {
      output_ << "producer_error\t" << sanitizeForTsv(message) << "\t"
              << nextCallId_.load(std::memory_order_relaxed) << "\t\t" << threadIdToString(std::this_thread::get_id())
              << "\t" << std::numeric_limits<scalar_t>::quiet_NaN() << "\t0\t"
              << std::numeric_limits<scalar_t>::quiet_NaN() << "\t" << std::numeric_limits<scalar_t>::quiet_NaN()
              << "\t" << std::numeric_limits<scalar_t>::quiet_NaN() << "\t"
              << std::numeric_limits<scalar_t>::quiet_NaN()
              << "\t0\t0\t" << std::numeric_limits<scalar_t>::quiet_NaN() << "\t"
              << std::numeric_limits<scalar_t>::quiet_NaN() << "\t0\t0\t0\t0\t"
              << std::numeric_limits<scalar_t>::quiet_NaN() << "\t"
              << std::numeric_limits<scalar_t>::quiet_NaN() << '\n';
    }
  } catch (...) {
  }
}

SelfCollisionDebugProbe::Slot* SelfCollisionDebugProbe::getOrCreateSlot(std::thread::id producerThreadId) {
  std::lock_guard<std::mutex> lock(slotsMutex_);

  const auto existing = producerSlotMap_.find(producerThreadId);
  if (existing != producerSlotMap_.end()) {
    return slots_[existing->second].get();
  }

  if (slots_.size() >= settings_.maxProducerThreads) {
    return nullptr;
  }

  const size_t slotId = slots_.size();
  auto slot = std::make_unique<Slot>(slotId, producerThreadId, settings_.queueCapacity);
  Slot* slotPtr = slot.get();
  slotPtr->worker = std::thread([this, slotPtr]() { workerLoop(*slotPtr); });
  slots_.push_back(std::move(slot));
  producerSlotMap_.emplace(producerThreadId, slotId);

  std::cerr << "[SelfCollisionDebugProbe] started worker slot " << slotId
            << " for producer thread " << threadIdToString(producerThreadId) << '\n';
  return slotPtr;
}

void SelfCollisionDebugProbe::workerLoop(Slot& slot) noexcept {
  try {
    MobileManipulatorSelfCollisionConstraint referenceConstraint(MobileManipulatorPinocchioMapping(modelInfo_),
                                                                 *referenceGeometryInterfacePtr_, minimumDistance_);
    MultidimensionalPenalty referencePenalty(std::unique_ptr<PenaltyBase>(penaltyPrototype_->clone()));
    MobileManipulatorPreComputation fallbackPreComputation(createPinocchioInterface(referenceUrdfFile_, modelType_, removeJointNames_),
                                                           modelInfo_);
    const vector_t zeroInput = vector_t::Zero(modelInfo_.inputDim);
    const auto waitTimeout = std::chrono::microseconds(settings_.waitTimeoutUs);

    while (true) {
      SelfCollisionDebugSample sample;
      if (slot.queue.wait_dequeue_timed(sample, waitTimeout)) {
        try {
          PreComputation& preComputation = sample.preComputation != nullptr ? *sample.preComputation : fallbackPreComputation;
          preComputation.request(Request::SoftConstraint + Request::Approximation, sample.time, sample.state, zeroInput);

          ScalarFunctionQuadraticApproximation referenceApproximation;
          switch (referenceConstraint.getOrder()) {
            case ConstraintOrder::Linear:
              referenceApproximation = referencePenalty.getQuadraticApproximation(
                  sample.time, referenceConstraint.getLinearApproximation(sample.time, sample.state, preComputation));
              break;
            case ConstraintOrder::Quadratic:
              referenceApproximation = referencePenalty.getQuadraticApproximation(
                  sample.time, referenceConstraint.getQuadraticApproximation(sample.time, sample.state, preComputation));
              break;
            default:
              throw std::runtime_error("[SelfCollisionDebugProbe] Unknown reference constraint order.");
          }

          slot.processedSamples.fetch_add(1, std::memory_order_relaxed);
          writeComparisonLine(slot.slotId, sample, referenceApproximation, "ok", "");
        } catch (const std::exception& e) {
          writeStatusLine(slot.slotId, sample.callId, sample.time, sample.producerThreadId, &sample.activeApproximation,
                          "worker_exception", e.what());
        } catch (...) {
          writeStatusLine(slot.slotId, sample.callId, sample.time, sample.producerThreadId, &sample.activeApproximation,
                          "worker_exception", "unknown worker-side exception");
        }
        continue;
      }

      if (slot.stopRequested.load(std::memory_order_acquire) &&
          (!settings_.syncOnDestruction || slot.queue.size_approx() == 0)) {
        break;
      }
    }
  } catch (const std::exception& e) {
    writeStatusLine(slot.slotId, 0, std::numeric_limits<scalar_t>::quiet_NaN(), slot.producerThreadId, nullptr,
                    "worker_fatal", e.what());
  } catch (...) {
    writeStatusLine(slot.slotId, 0, std::numeric_limits<scalar_t>::quiet_NaN(), slot.producerThreadId, nullptr,
                    "worker_fatal", "unknown worker fatal exception");
  }
}

void SelfCollisionDebugProbe::writeHeader() {
  output_ << "status\tmessage\tcall_id\tproducer_slot\tproducer_thread_id\ttime\tstate_dim\tactive_cost\treference_cost\t"
             "cost_abs_diff\tcost_rel_diff\tactive_dfdx_size\treference_dfdx_size\tdfdx_linf\tdfdx_l2\t"
             "active_dfdxx_rows\tactive_dfdxx_cols\treference_dfdxx_rows\treference_dfdxx_cols\tdfdxx_linf\tdfdxx_fro\n";
}

void SelfCollisionDebugProbe::writeComparisonLine(size_t producerSlot, const SelfCollisionDebugSample& sample,
                                                  const ScalarFunctionQuadraticApproximation& referenceApproximation,
                                                  const std::string& status, const std::string& message) {
  const bool dfdxSameSize = sample.activeApproximation.dfdx.size() == referenceApproximation.dfdx.size();
  const bool dfdxxSameSize = sample.activeApproximation.dfdxx.rows() == referenceApproximation.dfdxx.rows() &&
                             sample.activeApproximation.dfdxx.cols() == referenceApproximation.dfdxx.cols();

  const scalar_t nan = std::numeric_limits<scalar_t>::quiet_NaN();
  const vector_t dfdxDiff = dfdxSameSize ? sample.activeApproximation.dfdx - referenceApproximation.dfdx : vector_t();
  const matrix_t dfdxxDiff = dfdxxSameSize ? sample.activeApproximation.dfdxx - referenceApproximation.dfdxx : matrix_t();

  std::lock_guard<std::mutex> lock(outputMutex_);
  if (!output_.is_open()) {
    return;
  }

  output_ << status << '\t' << sanitizeForTsv(message) << '\t' << sample.callId << '\t' << producerSlot << '\t'
          << threadIdToString(sample.producerThreadId) << '\t' << std::setprecision(17) << sample.time << '\t'
          << sample.state.size() << '\t' << sample.activeApproximation.f << '\t' << referenceApproximation.f << '\t'
          << std::abs(sample.activeApproximation.f - referenceApproximation.f) << '\t'
          << relativeDifference(sample.activeApproximation.f, referenceApproximation.f) << '\t'
          << sample.activeApproximation.dfdx.size() << '\t' << referenceApproximation.dfdx.size() << '\t'
          << (dfdxSameSize ? maxAbsCoeff(dfdxDiff) : nan) << '\t' << (dfdxSameSize ? dfdxDiff.norm() : nan) << '\t'
          << sample.activeApproximation.dfdxx.rows() << '\t' << sample.activeApproximation.dfdxx.cols() << '\t'
          << referenceApproximation.dfdxx.rows() << '\t' << referenceApproximation.dfdxx.cols() << '\t'
          << (dfdxxSameSize ? maxAbsCoeff(dfdxxDiff) : nan) << '\t' << (dfdxxSameSize ? dfdxxDiff.norm() : nan) << '\n';
}

void SelfCollisionDebugProbe::writeStatusLine(size_t producerSlot, uint64_t callId, scalar_t time,
                                              std::thread::id producerThreadId,
                                              const ScalarFunctionQuadraticApproximation* activeApproximation,
                                              const std::string& status, const std::string& message) {
  const scalar_t nan = std::numeric_limits<scalar_t>::quiet_NaN();
  const scalar_t activeCost = activeApproximation != nullptr ? activeApproximation->f : nan;
  const Eigen::Index activeDfdxSize = activeApproximation != nullptr ? activeApproximation->dfdx.size() : 0;
  const Eigen::Index activeDfdxxRows = activeApproximation != nullptr ? activeApproximation->dfdxx.rows() : 0;
  const Eigen::Index activeDfdxxCols = activeApproximation != nullptr ? activeApproximation->dfdxx.cols() : 0;

  std::lock_guard<std::mutex> lock(outputMutex_);
  if (!output_.is_open()) {
    return;
  }

  output_ << status << '\t' << sanitizeForTsv(message) << '\t' << callId << '\t' << producerSlot << '\t'
          << threadIdToString(producerThreadId) << '\t' << std::setprecision(17) << time << "\t0\t" << activeCost << '\t'
          << nan << '\t' << nan << '\t' << nan << '\t' << activeDfdxSize << "\t0\t" << nan << '\t' << nan << '\t'
          << activeDfdxxRows << '\t' << activeDfdxxCols << "\t0\t0\t" << nan << '\t' << nan << '\n';
}

}  // namespace mobile_manipulator
}  // namespace ocs2
