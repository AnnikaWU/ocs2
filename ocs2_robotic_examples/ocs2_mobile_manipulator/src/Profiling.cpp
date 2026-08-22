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

#include "ocs2_mobile_manipulator/Profiling.h"

#include <algorithm>
#include <iostream>
#include <thread>
#include <unordered_map>

namespace ocs2 {
namespace mobile_manipulator {
namespace profiling {
namespace {

scalar_t toMilliseconds(std::chrono::nanoseconds duration) {
  return std::chrono::duration<scalar_t, std::milli>(duration).count();
}

scalar_t toMicroseconds(std::chrono::nanoseconds duration) {
  return std::chrono::duration<scalar_t, std::micro>(duration).count();
}

std::chrono::nanoseconds percentile90(std::vector<std::chrono::nanoseconds> samples) {
  if (samples.empty()) {
    return std::chrono::nanoseconds::zero();
  }
  const size_t index = (samples.size() * 9 + 9) / 10 - 1;
  std::nth_element(samples.begin(), samples.begin() + index, samples.end());
  return samples[index];
}

std::string entryKey(const Entry& entry) {
  std::string key;
  key.reserve(entry.scope.size() + entry.category.size() + entry.name.size() + 2);
  key += entry.scope;
  key += '\0';
  key += entry.category;
  key += '\0';
  key += entry.name;
  return key;
}

void addTotalPercent(std::vector<Entry>& entries) {
  scalar_t profiledTotalMilliseconds = 0.0;
  for (const auto& entry : entries) {
    profiledTotalMilliseconds += entry.totalMilliseconds;
  }

  for (auto& entry : entries) {
    entry.totalPercent =
        profiledTotalMilliseconds > 0.0 ? 100.0 * entry.totalMilliseconds / profiledTotalMilliseconds : 0.0;
  }
}

std::vector<Entry> diffEntries(const std::vector<Entry>& before, const std::vector<Entry>& after) {
  std::unordered_map<std::string, Entry> beforeByKey;
  beforeByKey.reserve(before.size());
  for (const auto& entry : before) {
    beforeByKey.emplace(entryKey(entry), entry);
  }

  std::vector<Entry> diff;
  diff.reserve(after.size());
  for (const auto& afterEntry : after) {
    Entry entry = afterEntry;
    const auto beforeIt = beforeByKey.find(entryKey(afterEntry));
    if (beforeIt != beforeByKey.end()) {
      const auto& beforeEntry = beforeIt->second;
      entry.calls = afterEntry.calls >= beforeEntry.calls ? afterEntry.calls - beforeEntry.calls : 0;
      entry.totalMilliseconds = afterEntry.totalMilliseconds - beforeEntry.totalMilliseconds;
    }

    if (entry.calls == 0 || entry.totalMilliseconds <= 0.0) {
      continue;
    }

    entry.averageMicroseconds = 1000.0 * entry.totalMilliseconds / static_cast<scalar_t>(entry.calls);
    entry.p90Microseconds = 0.0;
    entry.maxMicroseconds = 0.0;
    diff.push_back(std::move(entry));
  }

  addTotalPercent(diff);
  std::sort(diff.begin(), diff.end(), [](const Entry& lhs, const Entry& rhs) {
    return lhs.totalMilliseconds > rhs.totalMilliseconds;
  });
  return diff;
}

void printMpcRunSummary(std::ostream& stream, size_t runIndex, scalar_t currentTime, bool controllerIsUpdated,
                        const std::vector<Entry>& entries) {
  if (entries.empty()) {
    return;
  }

  stream << "\nMobile Manipulator MPC Run Profiling Summary #" << runIndex << " at t=" << currentTime << " [s]";
  if (!controllerIsUpdated) {
    stream << " (aborted)";
  }
  stream << ":\n";
  stream << "\tscope\tcategory\tterm\tcalls\tprofiled total [ms]\t% profiled total\tavg [us]\n";
  for (const auto& entry : entries) {
    stream << '\t' << entry.scope << '\t' << entry.category << '\t' << entry.name << '\t' << entry.calls << '\t'
           << entry.totalMilliseconds << '\t' << entry.totalPercent << '\t' << entry.averageMicroseconds << '\n';
  }
  stream << std::endl;
}

class ActiveRecordGuard {
 public:
  explicit ActiveRecordGuard(std::atomic_size_t& counter) : counterPtr_(&counter) {}
  ~ActiveRecordGuard() { counterPtr_->fetch_sub(1, std::memory_order_release); }

 private:
  std::atomic_size_t* counterPtr_;
};

}  // namespace

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
Profiler& Profiler::instance() {
  static Profiler* profiler = new Profiler;
  return *profiler;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void Profiler::setEnabled(bool enabled) {
  enabled_.store(enabled, std::memory_order_relaxed);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void Profiler::reset() {
  // Reset is only used around profiler enablement; block new records until both epoch buffers are idle.
  resetInProgress_.store(true, std::memory_order_release);
  std::lock_guard<std::mutex> lock(mutex_);
  waitForEpoch(0);
  waitForEpoch(1);
  retiredAccumulators_.clear();
  for (auto* threadLocalData : threadLocalDataPtrs_) {
    threadLocalData->accumulators[0].clear();
    threadLocalData->accumulators[1].clear();
  }
  resetInProgress_.store(false, std::memory_order_release);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void Profiler::record(std::string_view scope, std::string_view category, std::string_view name,
                      std::chrono::steady_clock::duration duration) {
  auto& threadLocalData = getThreadLocalData();
  if (!threadLocalData.registered) {
    registerThreadLocalData(threadLocalData);
  }

  const auto durationNs = std::chrono::duration_cast<std::chrono::nanoseconds>(duration);
  size_t epoch;
  size_t epochIndex;
  while (true) {
    while (resetInProgress_.load(std::memory_order_acquire)) {
      std::this_thread::yield();
    }
    epoch = activeEpoch_.load(std::memory_order_acquire);
    epochIndex = epoch % 2;
    threadLocalData.activeRecords[epochIndex].fetch_add(1, std::memory_order_acq_rel);
    // If a reset or summary swap raced with this record, drop the reservation and retry on the new epoch.
    if (!resetInProgress_.load(std::memory_order_acquire) && epoch == activeEpoch_.load(std::memory_order_acquire)) {
      break;
    }
    threadLocalData.activeRecords[epochIndex].fetch_sub(1, std::memory_order_release);
  }

  ActiveRecordGuard guard(threadLocalData.activeRecords[epochIndex]);
  addToAccumulator(threadLocalData.accumulators[epochIndex][makeKey(scope, category, name)], durationNs);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::vector<Entry> Profiler::getEntries() const {
  rotateAndRetireInactiveEpoch();

  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<Entry> entries;
  entries.reserve(retiredAccumulators_.size());
  for (const auto& [key, accumulator] : retiredAccumulators_) {
    const auto firstSeparator = key.find('\0');
    const auto secondSeparator = key.find('\0', firstSeparator + 1);
    Entry entry;
    entry.scope = key.substr(0, firstSeparator);
    entry.category = key.substr(firstSeparator + 1, secondSeparator - firstSeparator - 1);
    entry.name = key.substr(secondSeparator + 1);
    entry.calls = accumulator.calls;
    entry.totalMilliseconds = toMilliseconds(accumulator.total);
    entry.averageMicroseconds = accumulator.calls > 0 ? toMicroseconds(accumulator.total) / static_cast<scalar_t>(accumulator.calls) : 0.0;
    entry.p90Microseconds = toMicroseconds(percentile90(accumulator.samples));
    entry.maxMicroseconds = toMicroseconds(accumulator.max);
    entries.push_back(std::move(entry));
  }
  addTotalPercent(entries);
  std::sort(entries.begin(), entries.end(), [](const Entry& lhs, const Entry& rhs) {
    return lhs.totalMilliseconds > rhs.totalMilliseconds;
  });
  return entries;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::string Profiler::makeKey(std::string_view scope, std::string_view category, std::string_view name) {
  std::string key;
  key.reserve(scope.size() + category.size() + name.size() + 2);
  if (!scope.empty()) {
    key.append(scope.data(), scope.size());
  }
  key += '\0';
  if (!category.empty()) {
    key.append(category.data(), category.size());
  }
  key += '\0';
  if (!name.empty()) {
    key.append(name.data(), name.size());
  }
  return key;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void Profiler::addToAccumulator(Accumulator& accumulator, std::chrono::nanoseconds duration) {
  accumulator.calls++;
  accumulator.total += duration;
  accumulator.max = std::max(accumulator.max, duration);
  accumulator.samples.push_back(duration);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void Profiler::mergeAccumulators(const std::unordered_map<std::string, Accumulator>& source,
                                 std::unordered_map<std::string, Accumulator>& destination) {
  for (const auto& [key, sourceAccumulator] : source) {
    auto& destinationAccumulator = destination[key];
    destinationAccumulator.calls += sourceAccumulator.calls;
    destinationAccumulator.total += sourceAccumulator.total;
    destinationAccumulator.max = std::max(destinationAccumulator.max, sourceAccumulator.max);
    destinationAccumulator.samples.insert(destinationAccumulator.samples.end(), sourceAccumulator.samples.begin(),
                                          sourceAccumulator.samples.end());
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
Profiler::ThreadLocalData& Profiler::getThreadLocalData() {
  thread_local ThreadLocalData threadLocalData;
  return threadLocalData;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void Profiler::registerThreadLocalData(ThreadLocalData& threadLocalData) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!threadLocalData.registered) {
    threadLocalDataPtrs_.push_back(&threadLocalData);
    threadLocalData.registered = true;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void Profiler::mergeAndUnregisterThreadLocalData(ThreadLocalData& threadLocalData) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (threadLocalData.registered) {
    // Preserve samples from solver worker threads that exit before the final summary is printed.
    waitForEpoch(0);
    waitForEpoch(1);
    mergeAccumulators(threadLocalData.accumulators[0], retiredAccumulators_);
    mergeAccumulators(threadLocalData.accumulators[1], retiredAccumulators_);
    threadLocalDataPtrs_.erase(std::remove(threadLocalDataPtrs_.begin(), threadLocalDataPtrs_.end(), &threadLocalData),
                               threadLocalDataPtrs_.end());
    threadLocalData.registered = false;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void Profiler::rotateAndRetireInactiveEpoch() const {
  std::lock_guard<std::mutex> lock(mutex_);
  const size_t inactiveEpochIndex = activeEpoch_.fetch_add(1, std::memory_order_acq_rel) % 2;
  waitForEpoch(inactiveEpochIndex);

  // New records are now using the other epoch, so this buffer can be merged without blocking the hot path.
  for (auto* threadLocalData : threadLocalDataPtrs_) {
    mergeAccumulators(threadLocalData->accumulators[inactiveEpochIndex], retiredAccumulators_);
    threadLocalData->accumulators[inactiveEpochIndex].clear();
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void Profiler::waitForEpoch(size_t epochIndex) const {
  for (const auto* threadLocalData : threadLocalDataPtrs_) {
    while (threadLocalData->activeRecords[epochIndex].load(std::memory_order_acquire) != 0) {
      std::this_thread::yield();
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
Profiler::ThreadLocalData::~ThreadLocalData() {
  if (registered) {
    Profiler::instance().mergeAndUnregisterThreadLocalData(*this);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScopedTimer::ScopedTimer(std::string_view scope, std::string_view category, std::string_view name)
    : enabled_(Profiler::instance().isEnabled()),
      scope_(enabled_ ? std::string(scope) : std::string()),
      category_(enabled_ ? std::string(category) : std::string()),
      name_(enabled_ ? std::string(name) : std::string()),
      startTime_(enabled_ ? std::chrono::steady_clock::now() : std::chrono::steady_clock::time_point()) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScopedTimer::~ScopedTimer() {
  if (enabled_) {
    Profiler::instance().record(scope_, category_, name_, std::chrono::steady_clock::now() - startTime_);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SummaryGuard::SummaryGuard(bool enabled) : enabled_(enabled) {
  if (enabled_) {
    Profiler::instance().reset();
    Profiler::instance().setEnabled(true);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SummaryGuard::~SummaryGuard() {
  if (enabled_) {
    printSummary(std::cerr);
    Profiler::instance().setEnabled(false);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
MpcRunProfiler::MpcRunProfiler(bool enabled) : enabled_(enabled) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MpcRunProfiler::start(scalar_t currentTime) {
  if (!enabled_) {
    return;
  }

  currentTime_ = currentTime;
  runIndex_++;
  beforeRunEntries_ = Profiler::instance().getEntries();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MpcRunProfiler::finish(bool controllerIsUpdated) {
  if (!enabled_) {
    return;
  }

  const auto afterRunEntries = Profiler::instance().getEntries();
  const auto runEntries = diffEntries(beforeRunEntries_, afterRunEntries);
  printMpcRunSummary(std::cerr, runIndex_, currentTime_, controllerIsUpdated, runEntries);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void printSummary(std::ostream& stream) {
  const auto entries = Profiler::instance().getEntries();
  if (entries.empty()) {
    return;
  }

  stream << "\nMobile Manipulator Cost/Constraint Term Profiling Summary:\n";
  stream << "\tscope\tcategory\tterm\tcalls\ttotal [ms]\t% profiled total\tavg [us]\tp90 [us]\tmax [us]\n";
  for (const auto& entry : entries) {
    stream << '\t' << entry.scope << '\t' << entry.category << '\t' << entry.name << '\t' << entry.calls << '\t'
           << entry.totalMilliseconds << '\t' << entry.totalPercent << '\t' << entry.averageMicroseconds << '\t'
           << entry.p90Microseconds << '\t' << entry.maxMicroseconds << '\n';
  }
  stream << std::endl;
}

}  // namespace profiling
}  // namespace mobile_manipulator
}  // namespace ocs2
