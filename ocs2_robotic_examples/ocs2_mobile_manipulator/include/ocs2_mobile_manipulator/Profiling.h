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

#include <atomic>
#include <chrono>
#include <iosfwd>
#include <mutex>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include <ocs2_core/Types.h>

namespace ocs2 {
namespace mobile_manipulator {
namespace profiling {

struct Entry {
  std::string scope;
  std::string category;
  std::string name;
  size_t calls = 0;
  scalar_t totalMilliseconds = 0.0;
  scalar_t totalPercent = 0.0;
  scalar_t averageMicroseconds = 0.0;
  scalar_t p90Microseconds = 0.0;
  scalar_t maxMicroseconds = 0.0;
};

/**
 * Lightweight process-local profiler for mobile manipulator cost and soft-constraint terms.
 *
 * Recording uses thread-local accumulators on the hot path. Summary collection swaps epochs and
 * merges inactive thread-local data so solver worker threads do not contend on a shared mutex for
 * every term evaluation.
 */
class Profiler {
 public:
  static Profiler& instance();

  void setEnabled(bool enabled);
  bool isEnabled() const { return enabled_.load(std::memory_order_relaxed); }
  void reset();

  void record(std::string_view scope, std::string_view category, std::string_view name,
              std::chrono::steady_clock::duration duration);

  std::vector<Entry> getEntries() const;

 private:
  struct Accumulator {
    size_t calls = 0;
    std::chrono::nanoseconds total = std::chrono::nanoseconds::zero();
    std::chrono::nanoseconds max = std::chrono::nanoseconds::zero();
    std::vector<std::chrono::nanoseconds> samples;
  };

  struct alignas(64) ThreadLocalData {
    ~ThreadLocalData();

    bool registered = false;
    // Two epochs let getEntries() retire one buffer while new records move to the other.
    std::unordered_map<std::string, Accumulator> accumulators[2];
    std::atomic_size_t activeRecords[2]{};
  };

  static std::string makeKey(std::string_view scope, std::string_view category, std::string_view name);
  static void addToAccumulator(Accumulator& accumulator, std::chrono::nanoseconds duration);
  static void mergeAccumulators(const std::unordered_map<std::string, Accumulator>& source,
                                std::unordered_map<std::string, Accumulator>& destination);
  static ThreadLocalData& getThreadLocalData();

  void registerThreadLocalData(ThreadLocalData& threadLocalData);
  void mergeAndUnregisterThreadLocalData(ThreadLocalData& threadLocalData);
  void rotateAndRetireInactiveEpoch() const;
  void waitForEpoch(size_t epochIndex) const;

  std::atomic_bool enabled_{false};
  std::atomic_bool resetInProgress_{false};
  mutable std::atomic_size_t activeEpoch_{0};
  mutable std::mutex mutex_;
  mutable std::vector<ThreadLocalData*> threadLocalDataPtrs_;
  mutable std::unordered_map<std::string, Accumulator> retiredAccumulators_;
};

class ScopedTimer {
 public:
  ScopedTimer(std::string_view scope, std::string_view category, std::string_view name);
  ~ScopedTimer();

 private:
  bool enabled_;
  std::string scope_;
  std::string category_;
  std::string name_;
  std::chrono::steady_clock::time_point startTime_;
};

/** Enables profiling for the node lifetime and prints one summary when the node exits. */
class SummaryGuard {
 public:
  explicit SummaryGuard(bool enabled);
  ~SummaryGuard();

 private:
  bool enabled_;
};

/** Prints per-MPC-run deltas for the profiled cost/constraint term timers. */
class MpcRunProfiler {
 public:
  explicit MpcRunProfiler(bool enabled);

  void start(scalar_t currentTime);
  void finish(bool controllerIsUpdated);

 private:
  bool enabled_;
  size_t runIndex_ = 0;
  scalar_t currentTime_ = 0.0;
  std::vector<Entry> beforeRunEntries_;
};

void printSummary(std::ostream& stream);

}  // namespace profiling
}  // namespace mobile_manipulator
}  // namespace ocs2
