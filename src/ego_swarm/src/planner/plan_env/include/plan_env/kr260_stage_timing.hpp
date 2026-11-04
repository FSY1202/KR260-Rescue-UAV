#ifndef PLAN_ENV_KR260_STAGE_TIMING_HPP
#define PLAN_ENV_KR260_STAGE_TIMING_HPP

#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>

namespace plan_env {

// Diagnostic only: no ROS dependency, worker, timer, lock, or algorithm change.
// The caller must serialize access just as it serializes the mapping callbacks.
// Keep a pending Frame with its cloud, then move it back when the pose arrives.
class Kr260StageTiming {
public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;
  enum class Stage : std::size_t {
    PoseWait, PclDecode, Integrate, BoundaryFullGrid, BufferCopy,
    SnapshotPreparePublish, Count
  };
  enum class Outcome { Published, Rejected, Superseded };
  static constexpr std::size_t stage_count = static_cast<std::size_t>(Stage::Count);
  static constexpr std::size_t sample_capacity = 100;

  struct Stats {
    std::uint64_t count = 0;
    double sum_ms = 0;
    double max_ms = 0;
  };
  struct Sample {
    std::uint64_t seq = 0;
    std::int64_t source_stamp_sec = 0;
    std::uint32_t source_stamp_nanosec = 0;
    std::uint64_t revision = 0;
    std::int64_t callback_steady_ns = 0;
    // These are supplied ROS-clock ages, never inputs to steady durations.
    double ros_source_age_ms_at_callback = std::numeric_limits<double>::quiet_NaN();
    double ros_source_age_ms_at_finish = std::numeric_limits<double>::quiet_NaN();
    double callback_total_ms = 0;
    std::array<double, stage_count> stage_ms{};
    std::uint32_t recorded_stages = 0;
    Outcome outcome = Outcome::Published;
  };
  class Frame {
  public:
    Frame() = default;
    Frame(const Frame&) = delete;
    Frame& operator=(const Frame&) = delete;
    Frame(Frame&& other) noexcept { *this = std::move(other); }
    // Finish a superseded pending frame before overwriting it with another one.
    Frame& operator=(Frame&& other) noexcept {
      if (this != &other) {
        owner_ = std::exchange(other.owner_, nullptr);
        generation_ = other.generation_;
        sample_ = other.sample_;
        callback_ = other.callback_;
        last_mark_ = other.last_mark_;
        next_stage_ = other.next_stage_;
      }
      return *this;
    }
    bool active() const noexcept { return owner_ != nullptr; }
    std::uint64_t sequence() const noexcept { return sample_.seq; }

  private:
    friend class Kr260StageTiming;
    Kr260StageTiming* owner_ = nullptr;
    std::uint64_t generation_ = 0;
    Sample sample_{};
    TimePoint callback_{};
    TimePoint last_mark_{};
    std::size_t next_stage_ = 0;
  };

  Kr260StageTiming() = default;
  Kr260StageTiming(const Kr260StageTiming&) = delete;
  Kr260StageTiming& operator=(const Kr260StageTiming&) = delete;

  // Call once during initialization. Default-off operations do not read a clock,
  // format text, or allocate. Configuration invalidates outstanding frame tokens.
  // Preserve the sequence across reconfiguration, including disable/enable cycles.
  void configure(bool enabled, std::string_view run_id = {}) {
    enabled_ = enabled;
    ++generation_;
    if (enabled_) {
      if (run_id.empty()) run_id_.clear();
      else run_id_.assign(run_id.data(), run_id.size());
    }
    window_started_ = false;
    resetWindow();
  }
  bool enabled() const noexcept { return enabled_; }

  Frame begin(std::int64_t sec, std::uint32_t nanosec, double ros_age_ms) {
    if (!enabled_) return {};
    return beginAt(sec, nanosec, ros_age_ms, Clock::now());
  }
  Frame beginAt(std::int64_t sec, std::uint32_t nanosec, double ros_age_ms,
                TimePoint now) {
    Frame frame;
    if (!enabled_) return frame;
    frame.owner_ = this;
    frame.generation_ = generation_;
    frame.callback_ = frame.last_mark_ = now;
    frame.sample_.seq = ++next_sequence_;
    frame.sample_.source_stamp_sec = sec;
    frame.sample_.source_stamp_nanosec = nanosec;
    frame.sample_.ros_source_age_ms_at_callback = ros_age_ms;
    frame.sample_.callback_steady_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
    if (!window_started_) {
      window_started_ = true;
      window_start_ = now;
    }
    return frame;
  }

  // Mark all six stages, in order. Each is the steady interval since the last
  // mark; PoseWait is callback entry through pose readiness, including admission
  // checks. It also includes pending-cloud residence if a matching pose is late.
  // SnapshotPreparePublish ends only after publish() returns (not DDS delivery).
  bool mark(Frame& frame, Stage stage) {
    if (!owns(frame)) return false;
    return markAt(frame, stage, Clock::now());
  }
  bool markAt(Frame& frame, Stage stage, TimePoint now) {
    const auto index = static_cast<std::size_t>(stage);
    if (!owns(frame) || index != frame.next_stage_ || index >= stage_count ||
        now < frame.last_mark_) return false;
    frame.sample_.stage_ms[index] = milliseconds(now - frame.last_mark_);
    frame.sample_.recorded_stages |= std::uint32_t(1) << index;
    frame.last_mark_ = now;
    ++frame.next_stage_;
    return true;
  }

  bool finish(Frame& frame, std::uint64_t revision,
              Outcome outcome = Outcome::Published,
              double ros_age_ms = std::numeric_limits<double>::quiet_NaN()) {
    if (!owns(frame)) return false;
    return finishAt(frame, revision, outcome, ros_age_ms, Clock::now());
  }
  bool finishAt(Frame& frame, std::uint64_t revision, Outcome outcome,
                double ros_age_ms, TimePoint now) {
    if (!owns(frame) || now < frame.last_mark_) return false;
    auto& sample = frame.sample_;
    sample.revision = revision;
    sample.outcome = outcome;
    sample.ros_source_age_ms_at_finish = ros_age_ms;
    sample.callback_total_ms = milliseconds(now - frame.callback_);
    add(total_, sample.callback_total_ms);
    for (std::size_t i = 0; i < stage_count; ++i)
      if (sample.recorded_stages & (std::uint32_t(1) << i))
        add(stages_[i], sample.stage_ms[i]);
    if (sample_count_ == sample_capacity) {
      samples_[sample_head_] = sample;
      sample_head_ = (sample_head_ + 1) % sample_capacity;
      ++sample_dropped_;
    } else {
      samples_[(sample_head_ + sample_count_) % sample_capacity] = sample;
      ++sample_count_;
    }
    frame.owner_ = nullptr;  // A token cannot be committed twice.
    return true;
  }

  // At most one structured line per steady second. Call after completion and/or
  // from an existing serialized 1 Hz timer to drain the final idle window.
  // All completed-frame aggregates survive sample overflow; sample_dropped > 0
  // means per-frame percentiles for that window are incomplete.
  std::string flushDue() {
    if (!enabled_ || !window_started_) return {};
    return flushDueAt(Clock::now());
  }
  std::string flushDueAt(TimePoint now) {
    if (!enabled_ || !window_started_ || now < window_start_ ||
        now - window_start_ < std::chrono::seconds(1)) return {};
    if (!total_.count) {
      window_start_ = now;
      return {};
    }
    std::ostringstream out;
    out.imbue(std::locale::classic());
    out << std::fixed << std::setprecision(6)
        << "KR260_STAGE_TIMING {\"schema\":1,\"run_id\":";
    quoted(out, run_id_);
    out << ",\"duration_clock\":\"steady\",\"window_ms\":"
        << milliseconds(now - window_start_) << ",\"callback_total_ms\":";
    writeStats(out, total_);
    out << ",\"stage_stats\":{";
    for (std::size_t i = 0; i < stage_count; ++i) {
      if (i) out << ',';
      quoted(out, stageName(i)); out << ':'; writeStats(out, stages_[i]);
    }
    out << "},\"stage_order\":[";
    for (std::size_t i = 0; i < stage_count; ++i) {
      if (i) out << ',';
      quoted(out, stageName(i));
    }
    out << "],\"sample_capacity\":" << sample_capacity
        << ",\"sample_dropped\":" << sample_dropped_ << ",\"frames\":[";
    for (std::size_t i = 0; i < sample_count_; ++i) {
      if (i) out << ',';
      const auto& sample = *bufferedFrame(i);
      out << "{\"seq\":" << sample.seq << ",\"revision\":" << sample.revision
          << ",\"source_stamp_sec\":" << sample.source_stamp_sec
          << ",\"source_stamp_nanosec\":" << sample.source_stamp_nanosec
          << ",\"callback_steady_ns\":" << sample.callback_steady_ns
          << ",\"outcome\":";
      quoted(out, outcomeName(sample.outcome));
      out << ",\"ros_source_age_ms_at_callback\":";
      number(out, sample.ros_source_age_ms_at_callback);
      out << ",\"ros_source_age_ms_at_finish\":";
      number(out, sample.ros_source_age_ms_at_finish);
      out << ",\"callback_total_ms\":" << sample.callback_total_ms << ",\"stage_ms\":[";
      for (std::size_t j = 0; j < stage_count; ++j) {
        if (j) out << ',';
        if (sample.recorded_stages & (std::uint32_t(1) << j)) out << sample.stage_ms[j];
        else out << "null";
      }
      out << "]}";
    }
    out << "]}";
    auto line = out.str();
    resetWindow();
    window_start_ = now;
    return line;
  }

  const Stats& totalStats() const noexcept { return total_; }
  const Stats& stageStats(Stage stage) const { return stages_.at(static_cast<std::size_t>(stage)); }
  std::size_t bufferedFrames() const noexcept { return sample_count_; }
  std::uint64_t droppedSamples() const noexcept { return sample_dropped_; }
  const Sample* bufferedFrame(std::size_t oldest_first_index) const noexcept {
    if (oldest_first_index >= sample_count_) return nullptr;
    return &samples_[(sample_head_ + oldest_first_index) % sample_capacity];
  }

private:
  bool enabled_ = false;
  bool window_started_ = false;
  std::uint64_t generation_ = 0;
  std::uint64_t next_sequence_ = 0;
  std::string run_id_;
  TimePoint window_start_{};
  Stats total_{};
  std::array<Stats, stage_count> stages_{};
  std::array<Sample, sample_capacity> samples_{};
  std::size_t sample_head_ = 0;
  std::size_t sample_count_ = 0;
  std::uint64_t sample_dropped_ = 0;

  bool owns(const Frame& frame) const noexcept {
    return enabled_ && frame.owner_ == this && frame.generation_ == generation_;
  }
  static double milliseconds(Clock::duration duration) {
    return std::chrono::duration<double, std::milli>(duration).count();
  }
  static void add(Stats& stats, double value) noexcept {
    ++stats.count;
    stats.sum_ms += value;
    if (value > stats.max_ms) stats.max_ms = value;
  }
  void resetWindow() noexcept {
    total_ = {};
    stages_ = {};
    sample_head_ = sample_count_ = 0;
    sample_dropped_ = 0;
  }
  static const char* stageName(std::size_t index) noexcept {
    constexpr const char* names[] = {"pose_wait", "pcl_decode", "integrate",
        "boundary_full_grid", "buffer_copy", "snapshot_prepare_publish"};
    return names[index];
  }
  static const char* outcomeName(Outcome outcome) noexcept {
    switch (outcome) {
      case Outcome::Published: return "published";
      case Outcome::Rejected: return "rejected";
      case Outcome::Superseded: return "superseded";
    }
    return "unknown";
  }
  static void number(std::ostream& out, double value) {
    if (std::isfinite(value)) out << value;
    else out << "null";
  }
  static void writeStats(std::ostream& out, const Stats& stats) {
    out << "{\"count\":" << stats.count << ",\"sum_ms\":" << stats.sum_ms
        << ",\"max_ms\":" << stats.max_ms << '}';
  }
  static void quoted(std::ostream& out, std::string_view value) {
    constexpr char hex[] = "0123456789abcdef";
    out << '"';
    for (unsigned char c : value) {
      if (c == '"' || c == '\\') out << '\\' << char(c);
      else if (c < 0x20) out << "\\u00" << hex[c >> 4] << hex[c & 15];
      else out << char(c);
    }
    out << '"';
  }
};

}  // namespace plan_env

#endif  // PLAN_ENV_KR260_STAGE_TIMING_HPP
