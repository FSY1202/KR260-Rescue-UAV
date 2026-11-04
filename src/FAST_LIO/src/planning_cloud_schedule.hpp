#pragma once
#include <cmath>

namespace kr260_planning {

// Admission pacing only: sample timestamps and all freshness limits are intact.
// Use each real 10 Hz scan for the optional planning map. With 110--150 ms
// measured map-completion age, a 200 ms publication period repeatedly violates
// the unchanged 300 ms map-source-age limit even with a healthy sensor.
// Fixed-phase pacing tolerates scan jitter without manufacturing/repeating data.
// A 10 ms scheduling allowance does NOT accept older sensor data.
class CloudSchedule {
 public:
  bool due(double sample) const {
    if (!std::isfinite(sample) || sample < 0.) return false;
    if (!committed_) return true;
    return sample > last_sent_ && sample-last_sent_ >= minimum_spacing_ &&
           sample+phase_allowance_ >= next_due_;
  }
  void commit(double sample) {
    if (!due(sample)) return;
    if (!committed_) {
      next_due_=sample+period_;
      committed_=true;
    } else {
      const double steps=std::floor((sample+phase_allowance_-next_due_)/period_)+1.;
      next_due_+=steps*period_;
    }
    last_sent_=sample;
  }
 private:
  static constexpr double period_=.1;
  static constexpr double phase_allowance_=.01;
  static constexpr double minimum_spacing_=.075; // Never burst to catch up.
  bool committed_{};
  double next_due_{},last_sent_{};
};

} // namespace kr260_planning
