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
#include <sstream>
#include <stdexcept>
#include <utility>

#include <ocs2_core/ComputationRequest.h>
#include <ocs2_core/penalties/MultidimensionalPenalty.h>
#include <ocs2_mobile_manipulator/FactoryFunctions.h>
#include <ocs2_mobile_manipulator/MobileManipulatorPinocchioMapping.h>
#include <ocs2_mobile_manipulator/MobileManipulatorPreComputation.h>
#include <ocs2_mobile_manipulator/constraint/MobileManipulatorSelfCollisionConstraint.h>
#include <ocs2_mobile_manipulator/third_party/moodycamel/readerwriterqueue.h>
#include <ocs2_self_collision/PinocchioGeometryInterface.h>

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

SelfCollisionDebugProbe::SelfCollisionDebugProbe(SelfCollisionDebugSettings settings, std::string referenceUrdfFile,
                                                 ManipulatorModelType modelType, std::vector<std::string> removeJointNames,
                                                 std::vector<std::pair<std::string, std::string>> collisionLinkPairs,
                                                 std::vector<std::pair<size_t, size_t>> collisionObjectPairs,
                                                 ManipulatorModelInfo modelInfo, scalar_t minimumDistance,
                                                 std::unique_ptr<PenaltyBase> penaltyPrototype)
    : settings_(std::move(settings)),
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

  if (!settings_.outputFile.empty()) {
    output_.open(settings_.outputFile, std::ios::out | std::ios::trunc);
    if (!output_.is_open()) {
      throw std::runtime_error("[SelfCollisionDebugProbe] Failed to open debug output file: " + settings_.outputFile);
    }
    writeHeader();
  }

  std::cerr << "[SelfCollisionDebugProbe] reference backend pinocchio_fcl uses URDF: " << referenceUrdfFile_ << '\n';
  std::cerr << "[SelfCollisionDebugProbe] output file: "
            << (settings_.outputFile.empty() ? std::string("<disabled>") : settings_.outputFile) << '\n';
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
