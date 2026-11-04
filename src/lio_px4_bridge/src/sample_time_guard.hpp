#pragma once
#include <cstdint>
#include <cstdlib>

namespace lio_px4 {
enum class TimeResult { Accept, Invalid, Stale, Future, Duplicate, Backward, ClockJump };
inline const char* TimeReason(TimeResult r) {
  switch (r) {
    case TimeResult::Accept: return "accepted";
    case TimeResult::Invalid: return "invalid timestamp";
    case TimeResult::Stale: return "stale odometry";
    case TimeResult::Future: return "future timestamp / clock mismatch";
    case TimeResult::Duplicate: return "duplicate timestamp";
    case TimeResult::Backward: return "sample time moved backward";
    case TimeResult::ClockJump: return "ROS clock jumped";
  }
  return "unknown";
}
class SampleTimeGuard {
 public:
  TimeResult Check(int64_t sample_ns, int64_t now_ns, int64_t steady_ns, int64_t max_age_ns) {
    if (now_ns <= 0 || sample_ns <= 0) return TimeResult::Invalid;
    if (last_now_ != 0 && std::llabs((now_ns - last_now_) - (steady_ns - last_steady_)) > 50000000LL) {
      last_now_ = now_ns; last_steady_ = steady_ns; last_sample_ = 0;
      return TimeResult::ClockJump;
    }
    last_now_ = now_ns; last_steady_ = steady_ns;
    if (sample_ns > now_ns) return TimeResult::Future;
    if (now_ns - sample_ns > max_age_ns) return TimeResult::Stale;
    if (sample_ns == last_sample_) return TimeResult::Duplicate;
    if (sample_ns < last_sample_) {
      last_sample_ = 0;
      return TimeResult::Backward;
    }
    last_sample_ = sample_ns;
    return TimeResult::Accept;
  }
 private:
  int64_t last_now_ = 0, last_steady_ = 0, last_sample_ = 0;
};
}
