#pragma once
#include <cmath>
#include <cstdint>
#include <map>
#include <set>
#include <string>
#include <utility>

namespace kr260_ego {
// Source time is CLOCK_MONOTONIC on the same KR260, never HTTP receipt time.
struct PersonObservation {
  std::string detector_session,camera_session;
  uint64_t result{},frame{},source_ns{};
  bool person{};
  double confidence{};
};
class PersonConfirmation {
 public:
  unsigned count{};
  bool observe(const PersonObservation& o,uint64_t now_ns){
    const bool ids=id(o.detector_session)&&id(o.camera_session);
    if(!ids||!o.result||!o.source_ns||o.source_ns>now_ns||now_ns-o.source_ns>2000000000ULL||
       !std::isfinite(o.confidence)||o.confidence<0||o.confidence>1){count=0;return false;}
    const auto pair=std::make_pair(o.detector_session,o.camera_session);
    if(pair!=current_){
      if(retired_.count(pair)||retired_.size()>=32){count=0;return false;}
      if(!current_.first.empty())retired_.insert(current_);
      current_=pair;last_result_=0;count=0;
    }
    auto& watermark=camera_[o.camera_session];
    if(o.result<last_result_||o.frame<watermark.first||o.source_ns<watermark.second){count=0;return false;}
    // An empty frame must clear confirmation even if the producer repeats it.
    if(!o.person||o.confidence<.25){count=0;advance(o,watermark);return false;}
    if(o.result==last_result_||(watermark.second&&o.frame==watermark.first))return false;
    if(o.source_ns<=watermark.second){count=0;return false;}
    if(last_result_&&(o.result!=last_result_+1||o.source_ns-watermark.second>2000000000ULL))count=0;
    advance(o,watermark);if(count<2)++count;
    return count>=2;
  }
 private:
  std::pair<std::string,std::string> current_;
  std::set<std::pair<std::string,std::string>> retired_;
  std::map<std::string,std::pair<uint64_t,uint64_t>> camera_;
  uint64_t last_result_{};
  static bool id(const std::string& s){
    if(s.size()!=32)return false;
    for(char c:s)if(!((c>='0'&&c<='9')||(c>='a'&&c<='f')))return false;
    return true;
  }
  void advance(const PersonObservation& o,std::pair<uint64_t,uint64_t>& w){
    if(o.result>last_result_)last_result_=o.result;
    if(o.frame>w.first&&o.source_ns>w.second)w={o.frame,o.source_ns};
  }
};
enum class DropResult { NONE, COMPLETE, FAILED };
} // namespace kr260_ego
