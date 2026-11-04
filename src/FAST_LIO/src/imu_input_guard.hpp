#pragma once
// Input validation only: never edits a sample, clears required IMU history,
// resets the estimator, or restarts a process. Owned by the single ROS executor.
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>

namespace lio_input {
constexpr int64_t kMs = 1000000LL;
constexpr int64_t kMaxClockSamplingSpan = kMs;
enum class Result { Accept, Duplicate, Fault };
enum class Fault { None, InvalidImu, ImuBackward, ImuGap, QueueFull,
                   ClockJump, InvalidClock, FutureImu, LidarBackward,
                   InvalidScan, ScanBackward, ImuCoverage };
inline const char* reason(Fault f) {
    switch (f) {
        case Fault::None: return "none";
        case Fault::InvalidImu: return "invalid IMU timestamp or nonfinite motion";
        case Fault::ImuBackward: return "IMU sample time moved backward";
        case Fault::ImuGap: return "IMU sample gap exceeds configured limit";
        case Fault::QueueFull: return "IMU application queue reached configured bound";
        case Fault::ClockJump: return "ROS clock changed relative to steady clock";
        case Fault::InvalidClock: return "invalid ROS or steady clock";
        case Fault::FutureImu: return "IMU timestamp is in the future";
        case Fault::LidarBackward: return "lidar sample time moved backward";
        case Fault::InvalidScan: return "invalid scan sampling interval";
        case Fault::ScanBackward: return "scan end did not advance";
        case Fault::ImuCoverage: return "IMU does not cover scan end within gap limit";
    }
    return "unknown";
}
struct Config {
    bool enabled = false;  // KR260 explicitly opts in; other sensor setups unchanged.
    int64_t max_gap_ns = 50*kMs;
    int64_t max_odom_age_ns = 200*kMs;
    int64_t future_tolerance_ns = 5*kMs;
    int64_t clock_jump_ns = 50*kMs;
    size_t max_queue_samples = 1000;
};
class ImuInputGuard {
public:
    void configure(const Config& config) {
        if (started_) throw std::logic_error("input guard cannot be reconfigured live");
        if (config.max_gap_ns < 10*kMs || config.max_gap_ns > 200*kMs ||
            config.max_odom_age_ns <= 0 || config.max_odom_age_ns > 200*kMs ||
            config.future_tolerance_ns < 0 || config.future_tolerance_ns > 5*kMs ||
            config.clock_jump_ns <= 0 || config.clock_jump_ns > 50*kMs ||
            config.max_queue_samples < 100 || config.max_queue_samples > 10000)
            throw std::invalid_argument("invalid IMU input guard configuration");
        config_ = config;
    }
    bool enabled() const { return config_.enabled; }
    bool faulted() const { return fault_ != Fault::None; }
    Fault fault() const { return fault_; }
    const Config& config() const { return config_; }
    // A fault is latched. Recovery requires an explicit, on-ground process restart.
    bool fail(Fault fault) {
        if (config_.enabled && !faulted()) fault_ = fault;
        return false;
    }
    bool clock(int64_t ros_ns, int64_t steady_ns, int64_t sampling_span_ns=0) {
        if (!enabled()) return true;
        started_ = true;
        if (faulted()) return false;
        if (ros_ns <= 0 || steady_ns < 0 || sampling_span_ns < 0) return fail(Fault::InvalidClock);
        // A scheduling pause between the two clock reads is NOT proof that ROS
        // time jumped. Keep the last coherent baseline; callers suppress live
        // output on this tick, but IMU input must not be discarded for this.
        if (sampling_span_ns > kMaxClockSamplingSpan) { ++uncertain_clock_pairs; return true; }
        if (have_clock_ && accepted != 0) {
            const long double delta = (static_cast<long double>(ros_ns)-last_ros_) -
                                      (static_cast<long double>(steady_ns)-last_steady_);
            if (steady_ns < last_steady_ || std::fabs(delta) > config_.clock_jump_ns)
                return fail(Fault::ClockJump);
        }
        last_ros_ = ros_ns; last_steady_ = steady_ns; have_clock_ = true;
        return true;
    }
    Result imu(int64_t sample_ns, int64_t ros_ns, int64_t steady_ns,
               bool finite_motion, size_t pending_samples, int64_t sampling_span_ns=0) {
        if (!enabled()) return Result::Accept;
        if (!clock(ros_ns, steady_ns, sampling_span_ns)) return Result::Fault;
        if (sample_ns <= 0 || !finite_motion) {
            fail(Fault::InvalidImu); return Result::Fault;
        }
        const long double age = static_cast<long double>(ros_ns)-sample_ns;
        if (age < -config_.future_tolerance_ns) {
            fail(Fault::FutureImu); return Result::Fault;
        }
        // Preserve EVERY continuous IMU, even delayed startup replay. Dropping
        // it here would create an integration gap. Output freshness is separate.
        if (accepted != 0) {
            if (sample_ns == last_imu_) { ++duplicates; return Result::Duplicate; }
            if (sample_ns < last_imu_) { fail(Fault::ImuBackward); return Result::Fault; }
            const int64_t gap = sample_ns-last_imu_;
            max_sample_gap_ns = std::max(max_sample_gap_ns, gap);
            if (gap > config_.max_gap_ns) { fail(Fault::ImuGap); return Result::Fault; }
        }
        if (pending_samples >= config_.max_queue_samples) {
            fail(Fault::QueueFull); return Result::Fault;
        }
        if (age > config_.max_odom_age_ns) ++aged_inputs;
        last_imu_ = sample_ns; ++accepted;
        return Result::Accept;
    }
    bool lidar(double stamp) {
        if (!enabled()) return true;
        started_ = true;
        if (faulted()) return false;
        if (!std::isfinite(stamp) || stamp <= 0) return fail(Fault::InvalidScan);
        if (last_lidar_ > 0) {
            if (stamp == last_lidar_) { ++lidar_duplicates; return false; }
            if (stamp < last_lidar_) return fail(Fault::LidarBackward);
        }
        last_lidar_ = stamp; return true;
    }
    bool scan(double begin, double end, double last_imu, double newest_imu) {
        if (!enabled()) return true;
        if (faulted()) return false;
        if (!std::isfinite(begin) || !std::isfinite(end) || !std::isfinite(last_imu) ||
            !std::isfinite(newest_imu) || begin <= 0 || end <= begin ||
            last_imu <= 0 || newest_imu <= 0)
            return fail(Fault::InvalidScan);
        if (last_scan_end_ > 0 && end <= last_scan_end_) return fail(Fault::ScanBackward);
        const double gap_s = config_.max_gap_ns/1e9;
        // Existing sync retains the first sample AFTER scan end in imu_buffer;
        // last_imu is the consumed tail, not that retained boundary sample.
        if (newest_imu < end || last_imu > end+1e-6 || end-last_imu > gap_s+1e-6)
            return fail(Fault::ImuCoverage);
        last_scan_end_ = end; return true;
    }
    bool output_fresh(double sample_s, int64_t ros_ns) {
        if (!enabled()) return true;
        if (faulted()) return false;
        const long double age = static_cast<long double>(ros_ns)-sample_s*1e9L;
        if (!std::isfinite(sample_s) || sample_s <= 0 || ros_ns <= 0 ||
            age < -config_.future_tolerance_ns || age > config_.max_odom_age_ns) {
            ++stale_outputs; return false;
        }
        return true;
    }
    uint64_t accepted=0, aged_inputs=0, duplicates=0, lidar_duplicates=0,
             stale_outputs=0, empty_scans=0, uncertain_clock_pairs=0;
    int64_t max_sample_gap_ns=0;
private:
    Config config_;
    Fault fault_=Fault::None;
    bool started_=false, have_clock_=false;
    int64_t last_ros_=0, last_steady_=0, last_imu_=0;
    double last_lidar_=0, last_scan_end_=0;
};
}  // namespace lio_input
