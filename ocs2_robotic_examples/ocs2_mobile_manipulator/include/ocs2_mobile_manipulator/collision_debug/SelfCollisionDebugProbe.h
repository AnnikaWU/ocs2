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

#pragma once

#include <atomic>
#include <cstdint>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <ocs2_core/PreComputation.h>
#include <ocs2_core/Types.h>
#include <ocs2_core/penalties/penalties/PenaltyBase.h>

#include <ocs2_mobile_manipulator/ManipulatorModelInfo.h>

namespace ocs2 {

class PinocchioGeometryInterface;

namespace mobile_manipulator {

struct SelfCollisionDebugSettings {
  size_t queueCapacity = 256;
  size_t maxProducerThreads = 8;
  int waitTimeoutUs = 1000;
  bool syncOnDestruction = true;
  std::string outputFile = "/tmp/ocs2_self_collision_debug.tsv";
  std::string mpcOutputFile = "/tmp/ocs2_self_collision_debug_mpc.tsv";
};

struct SelfCollisionDebugSample {
  uint64_t callId = 0;
  scalar_t time = 0.0;
  vector_t state;
  std::unique_ptr<PreComputation> preComputation;
  std::thread::id producerThreadId;
  size_t producerSlot = 0;
  ScalarFunctionQuadraticApproximation activeApproximation;

  SelfCollisionDebugSample() = default;
  SelfCollisionDebugSample(SelfCollisionDebugSample&&) noexcept = default;
  SelfCollisionDebugSample& operator=(SelfCollisionDebugSample&&) noexcept = default;

  SelfCollisionDebugSample(const SelfCollisionDebugSample&) = delete;
  SelfCollisionDebugSample& operator=(const SelfCollisionDebugSample&) = delete;
};

class SelfCollisionDebugProbe final {
 public:
  SelfCollisionDebugProbe(SelfCollisionDebugSettings settings, std::string activeUrdfFile, std::string referenceUrdfFile,
                          ManipulatorModelType modelType,
                          std::vector<std::string> removeJointNames,
                          std::vector<std::pair<std::string, std::string>> collisionLinkPairs,
                          std::vector<std::pair<size_t, size_t>> collisionObjectPairs, ManipulatorModelInfo modelInfo,
                          scalar_t minimumDistance, std::unique_ptr<PenaltyBase> penaltyPrototype);
  ~SelfCollisionDebugProbe();

  SelfCollisionDebugProbe(const SelfCollisionDebugProbe&) = delete;
  SelfCollisionDebugProbe& operator=(const SelfCollisionDebugProbe&) = delete;

  void enqueue(scalar_t time, const vector_t& state, const PreComputation& preComputation,
               const ScalarFunctionQuadraticApproximation& activeApproximation) noexcept;
  void recordProducerError(const std::string& message) noexcept;

  /** Records the actually observed state at the beginning of an MPC run. */
  void startMpcRun(scalar_t currentTime, const vector_t& currentState) noexcept;

  /**
   * Re-evaluates the accepted MPC trajectory with both collision models.
   *
   * The solver total cost is the active-model PerformanceIndex::cost. Since all other terms are identical,
   * replacing the active self-collision integral with the reference integral gives the reference total cost
   * on exactly the same accepted trajectory.
   */
  void finishMpcRun(const scalar_array_t& timeTrajectory, const vector_array_t& stateTrajectory,
                    scalar_t solverActiveTotalCost) noexcept;

 private:
  struct Slot;
  struct MpcEvaluator;

  Slot* getOrCreateSlot(std::thread::id producerThreadId);
  void workerLoop(Slot& slot) noexcept;
  void writeHeader();
  void writeComparisonLine(size_t producerSlot, const SelfCollisionDebugSample& sample,
                           const ScalarFunctionQuadraticApproximation& referenceApproximation, const std::string& status,
                           const std::string& message);
  void writeStatusLine(size_t producerSlot, uint64_t callId, scalar_t time, std::thread::id producerThreadId,
                       const ScalarFunctionQuadraticApproximation* activeApproximation, const std::string& status,
                       const std::string& message);

  SelfCollisionDebugSettings settings_;
  std::string activeUrdfFile_;
  std::string referenceUrdfFile_;
  ManipulatorModelType modelType_;
  std::vector<std::string> removeJointNames_;
  std::vector<std::pair<std::string, std::string>> collisionLinkPairs_;
  std::vector<std::pair<size_t, size_t>> collisionObjectPairs_;
  ManipulatorModelInfo modelInfo_;
  scalar_t minimumDistance_ = 0.0;
  std::unique_ptr<PenaltyBase> penaltyPrototype_;
  std::unique_ptr<PinocchioGeometryInterface> referenceGeometryInterfacePtr_;
  std::unique_ptr<MpcEvaluator> mpcEvaluatorPtr_;

  std::atomic<uint64_t> nextCallId_{0};
  std::atomic<uint64_t> droppedSamples_{0};
  std::atomic<uint64_t> producerErrors_{0};

  mutable std::mutex slotsMutex_;
  std::unordered_map<std::thread::id, size_t> producerSlotMap_;
  std::vector<std::unique_ptr<Slot>> slots_;

  mutable std::mutex outputMutex_;
  std::ofstream output_;
  std::ofstream mpcOutput_;
};

}  // namespace mobile_manipulator
}  // namespace ocs2
