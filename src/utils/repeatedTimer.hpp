#pragma once

#include <algorithm>
#include <chrono>

namespace benchmark {

class RepeatedTimer {
 public:
  class ScopedTimer {
   public:
    ScopedTimer(RepeatedTimer& timer) : timer_(timer) { timer_.startTimer(); }
    ~ScopedTimer() { timer_.endTimer(); }

   private:
    RepeatedTimer& timer_;
  };

  RepeatedTimer()
      : numTimedIntervals_(0),
        totalTime_(std::chrono::nanoseconds::zero()),
        maxIntervalTime_(std::chrono::nanoseconds::zero()),
        lastIntervalTime_(std::chrono::nanoseconds::zero()),
        startTime_(std::chrono::steady_clock::now()) {}

  void reset() {
    numTimedIntervals_ = 0;
    totalTime_ = std::chrono::nanoseconds::zero();
    maxIntervalTime_ = std::chrono::nanoseconds::zero();
    lastIntervalTime_ = std::chrono::nanoseconds::zero();
  }

  void startTimer() { startTime_ = std::chrono::steady_clock::now(); }

  void endTimer() {
    auto endTime = std::chrono::steady_clock::now();
    lastIntervalTime_ = std::chrono::duration_cast<std::chrono::nanoseconds>(endTime - startTime_);
    maxIntervalTime_ = std::max(maxIntervalTime_, lastIntervalTime_);
    totalTime_ += lastIntervalTime_;
    numTimedIntervals_++;
  };

  int getNumTimedIntervals() const { return numTimedIntervals_; }

  double getTotalInMilliseconds() const { return std::chrono::duration<double, std::milli>(totalTime_).count(); }

  double getMaxIntervalInMilliseconds() const {
    return std::chrono::duration<double, std::milli>(maxIntervalTime_).count();
  }

  double getLastIntervalInMilliseconds() const {
    return std::chrono::duration<double, std::milli>(lastIntervalTime_).count();
  }

  double getAverageInMilliseconds() const { return getTotalInMilliseconds() / numTimedIntervals_; }

 private:
  int numTimedIntervals_;
  std::chrono::nanoseconds totalTime_;
  std::chrono::nanoseconds maxIntervalTime_;
  std::chrono::nanoseconds lastIntervalTime_;
  std::chrono::steady_clock::time_point startTime_;
};

}  // namespace benchmark