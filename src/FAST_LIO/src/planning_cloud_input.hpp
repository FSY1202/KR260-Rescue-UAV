#pragma once

// EGO-only ray selection. A six-metre integration horizon is not a valid reason
// to delete a return beyond six metres: the near part of that measured ray still
// provides free-space evidence. Do not publish projected/averaged endpoints.
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <unordered_map>
#include <vector>

namespace lio_planning {
struct RayEndpoint { double x{}, y{}, z{}; };

class RayEndpointSelection {
 public:
  explicit RayEndpointSelection(std::size_t capacity=50000,
                                double voxel_m=.1, double horizon_m=6.)
      : capacity_(capacity), voxel_(voxel_m), horizon_(horizon_m) {
    if (!capacity || capacity>50000 || !std::isfinite(voxel_m) || voxel_m<=0 ||
        !std::isfinite(horizon_m) || horizon_m<=0 || horizon_m/voxel_m>1000000.)
      throw std::invalid_argument("invalid bounded ray selection configuration");
    points_.reserve(capacity_);
    ranges_.reserve(capacity_);
    indices_.reserve(capacity_);
  }

  // Returns false for an unusable point or a latched capacity failure. Once the
  // capacity fails the WHOLE frame is unusable, even if later rays share keys.
  bool add(double x, double y, double z) {
    if (overflow_) return false;
    if (!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(z)) return false;
    // FAST-LIO is built as C++14. Nested hypot is overflow-safe and does not
    // require the three-argument C++17 overload.
    const double range=std::hypot(std::hypot(x,y),z);
    if (!std::isfinite(range)||range<.1) return false;
    // Divide first so collinear axis-aligned ranges (including huge finite
    // coordinates) share an exact horizon key without x*(horizon/range) drift.
    const bool far=range>horizon_;
    const Key key{coordinate(far?(x/range)*horizon_:x),
                  coordinate(far?(y/range)*horizon_:y),
                  coordinate(far?(z/range)*horizon_:z)};
    const auto existing=indices_.find(key);
    if (existing!=indices_.end()) {
      const auto i=existing->second;
      if (range<ranges_[i]) {
        points_[i]={x,y,z};ranges_[i]=range;
      }
      return true;
    }
    if (points_.size()==capacity_) { overflow_=true;return false; }
    const auto i=points_.size();
    indices_.emplace(key,i);points_.push_back({x,y,z});ranges_.push_back(range);
    return true;
  }

  bool overflowed() const noexcept { return overflow_; }
  bool empty() const noexcept { return points_.empty(); }
  std::size_t size() const noexcept { return points_.size(); }
  // Prevent accidental publication of a silently truncated frame.
  const std::vector<RayEndpoint>& points() const {
    if (overflow_) throw std::logic_error("overflowed ray frame must be rejected");
    return points_;
  }

 private:
  struct Key {
    std::int64_t x,y,z;
    bool operator==(const Key& other) const noexcept {
      return x==other.x&&y==other.y&&z==other.z;
    }
  };
  struct Hash {
    std::size_t operator()(const Key& key) const noexcept {
      std::size_t h=std::hash<std::int64_t>{}(key.x);
      h^=std::hash<std::int64_t>{}(key.y)+std::size_t(0x9e3779b9)+(h<<6)+(h>>2);
      h^=std::hash<std::int64_t>{}(key.z)+std::size_t(0x9e3779b9)+(h<<6)+(h>>2);
      return h;
    }
  };
  std::int64_t coordinate(double value) const {
    return static_cast<std::int64_t>(std::floor(value/voxel_));
  }
  std::size_t capacity_;
  double voxel_,horizon_;
  bool overflow_{};
  std::vector<RayEndpoint> points_;
  std::vector<double> ranges_;
  std::unordered_map<Key,std::size_t,Hash> indices_;
};
} // namespace lio_planning
