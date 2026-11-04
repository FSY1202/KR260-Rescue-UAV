#pragma once
#include <chrono>
#include <cstdint>
#include <deque>
#include <stdexcept>
#include <utility>

namespace livox_ros {
// Caller owns the mutex. Residence time is independent of sensor/wall clocks.
template<class T> class FreshPacketQueue {
 public:
  using Clock = std::chrono::steady_clock;
  using Time = Clock::time_point;
  void Configure(double age_ms, size_t capacity) {
    if (!(age_ms > 0) || age_ms > 60000 || capacity == 0)
      throw std::invalid_argument("invalid raw packet queue limits");
    max_age_ = std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double, std::milli>(age_ms));
    capacity_ = capacity;
  }
  void Push(T value, Time now) {
    Trim(now);
    if (entries_.size() >= capacity_) { entries_.pop_front(); ++dropped_; }
    entries_.push_back({std::move(value), now});
    ++received_;
  }
  bool Pop(T& value, Time now, double& residence_ms) {
    Trim(now);
    if (entries_.empty()) return false;
    residence_ms = std::chrono::duration<double, std::milli>(now - entries_.front().time).count();
    value = std::move(entries_.front().value);
    entries_.pop_front();
    return true;
  }
  size_t Size() const { return entries_.size(); }
  uint64_t Dropped() const { return dropped_; }
  uint64_t Received() const { return received_; }
 private:
  void Trim(Time now) {
    while (!entries_.empty() && now - entries_.front().time > max_age_) {
      entries_.pop_front(); ++dropped_;
    }
  }
  struct Entry { T value; Time time; };
  std::deque<Entry> entries_;
  Clock::duration max_age_ = std::chrono::milliseconds(200);
  size_t capacity_ = 4096;
  uint64_t dropped_ = 0, received_ = 0;
};
}
