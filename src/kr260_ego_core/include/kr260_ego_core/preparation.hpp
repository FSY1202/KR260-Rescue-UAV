#pragma once

#include <cmath>
#include <string>

namespace kr260_ego {

// One-shot ground preparation, with no ROS, coordinate, or control side effects.
// Call from the controller's serialized callback group. The caller supplies all
// normal freshness/health checks; the gate never changes their thresholds.
class PreparationGate {
 public:
  std::string state{"BOOTSTRAP"};
  std::string reason{"waiting for fresh Position ground telemetry, localization, alignment and recording"};

  bool bootstrap() const { return state == "BOOTSTRAP"; }
  bool committed() const { return state == "COMMITTED"; }
  bool blocked() const { return state == "BLOCKED"; }
  unsigned reset_count() const { return reset_count_; }

  // Returns true once committed. After commitment, the original runtime guards
  // own all failures; this gate never rearms/restarts its preparation window.
  bool step(double now, bool healthy, bool ground, bool armed, bool fatal,
            const std::string& localization_run, double monitor_stamp,
            const std::string& detail = "") {
    if (!bootstrap()) return committed();
    if (!observe_time(now)) return false;
    if (armed) return block("armed observed during bootstrap; explicit ground reprepare required");
    if (fatal) return block("fatal preparation fault" + suffix(detail));
    // Missing startup/temporarily unavailable metadata is not a time regression.
    // Retain the last valid monitor timestamp, so recovery still detects an
    // actual regression. No preparation progress is possible without metadata.
    if (localization_run.empty() && !healthy) {
      stable_since_ = -1.;
      if (now - started_ >= 120.) return block("bootstrap timed out after 120 seconds" + suffix(detail));
      reason = "waiting for available localization monitor metadata" + suffix(detail);
      return false;
    }
    if (!std::isfinite(monitor_stamp) || monitor_stamp < 0. || monitor_stamp > now)
      return block("invalid/future localization monitor timestamp");
    if (previous_monitor_ >= 0. && monitor_stamp < previous_monitor_)
      return block("localization monitor timestamp moved backwards");
    previous_monitor_ = monitor_stamp;
    if (!localization_run.empty()) {
      if (run_.empty()) run_ = localization_run;
      else if (run_ != localization_run)
        return block("localization run changed during bootstrap; no automatic restart/reprepare");
    }
    if (now - started_ >= 120.) return block("bootstrap timed out after 120 seconds" + suffix(detail));
    if (!healthy || !ground || localization_run.empty()) {
      stable_since_ = -1.;
      reason = !ground ? "waiting for fresh landed/disarmed Position telemetry"
                        : "waiting for fresh same-run localization, alignment and recording";
      reason += suffix(detail);
      return false;
    }
    if (stable_since_ < 0.) {
      stable_since_ = now;
      first_monitor_ = monitor_stamp;
    }
    if (now - stable_since_ >= 5. && monitor_stamp > first_monitor_) {
      state = "COMMITTED";
      reason = "five-second ground baseline committed; original runtime guards remain active";
      return true;
    }
    reason = now - stable_since_ >= 5.
        ? "waiting for advancing localization monitor update before baseline commitment"
        : "stabilizing fresh ground baseline for five continuous seconds";
    return false;
  }

  // Absorb only an uncommitted PX4 estimator-counter reset. This method must
  // never be used for message-time regression, clock jumps, or interface faults.
  // false means the caller must apply its unchanged post-bootstrap reset guard.
  bool note_reset(double now) {
    if (!bootstrap() || !observe_time(now)) return false;
    if (now - started_ >= 120.) return block("bootstrap timed out after 120 seconds");
    stable_since_ = -1.;
    ++reset_count_;
    reason = "uncommitted PX4 reset; collecting a new five-second stable baseline";
    return true;
  }

 private:
  double started_{-1.}, previous_{-1.}, stable_since_{-1.};
  double previous_monitor_{-1.}, first_monitor_{-1.};
  unsigned reset_count_{};
  std::string run_;

  static std::string suffix(const std::string& detail) {
    return detail.empty() ? "" : ": " + detail;
  }
  bool block(const std::string& why) {
    state = "BLOCKED";
    reason = why;
    stable_since_ = -1.;
    return false;
  }
  bool observe_time(double now) {
    if (!std::isfinite(now) || now < 0.) return block("invalid monotonic preparation time");
    if (previous_ >= 0. && now < previous_) return block("monotonic preparation time moved backwards");
    // A paused observer cannot establish continuous health just by seeing two
    // healthy endpoints. Existing node ticks are 50 ms; tolerate scheduling up
    // to the existing 350 ms fresh-output interval before restarting this timer.
    if (previous_ >= 0. && now - previous_ > .35) stable_since_ = -1.;
    if (started_ < 0.) started_ = now;
    previous_ = now;
    return true;
  }
};

}  // namespace kr260_ego
