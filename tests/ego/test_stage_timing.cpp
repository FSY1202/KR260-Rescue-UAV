/* Standalone fixture: c++ -std=c++17 -O2 -Wall -Wextra -Werror -pedantic
   -Isrc/ego_swarm/src/planner/plan_env/include tests/ego/test_stage_timing.cpp
   -o /tmp/test_stage_timing && /tmp/test_stage_timing */
#include "plan_env/kr260_stage_timing.hpp"
#include <cstdlib>
#include <iostream>
#include <new>
#include <type_traits>

static std::size_t allocation_count;
void* operator new(std::size_t size) {
  ++allocation_count;
  if (void* memory = std::malloc(size ? size : 1)) return memory;
  throw std::bad_alloc();
}
void* operator new[](std::size_t size) { return ::operator new(size); }
void operator delete(void* memory) noexcept { std::free(memory); }
void operator delete[](void* memory) noexcept { std::free(memory); }
void operator delete(void* memory, std::size_t) noexcept { std::free(memory); }
void operator delete[](void* memory, std::size_t) noexcept { std::free(memory); }

using Timing = plan_env::Kr260StageTiming;
using Stage = Timing::Stage;
using Outcome = Timing::Outcome;
static int checks;
#define CHECK(x) do { ++checks; if (!(x)) { \
  std::cerr << "FAIL " << __LINE__ << ": " << #x << '\n'; std::abort(); \
} } while (0)

static Timing::TimePoint at(long long milliseconds) {
  return Timing::TimePoint(std::chrono::milliseconds(milliseconds));
}
static void completed(Timing& timing, int start_ms, int width_ms, std::uint64_t revision) {
  auto frame = timing.beginAt(1770000000, 123456789, -9.5, at(start_ms));
  for (std::size_t i = 0; i < Timing::stage_count; ++i)
    CHECK(timing.markAt(frame, static_cast<Stage>(i), at(start_ms + int(i + 1) * width_ms)));
  CHECK(timing.finishAt(frame, revision, Outcome::Published, 975.0,
                        at(start_ms + int(Timing::stage_count) * width_ms)));
  CHECK(!frame.active());
  CHECK(!timing.finishAt(frame, revision, Outcome::Published, 0, at(start_ms + 100)));
}

int main(int argc, char* argv[]) {
  if (argc == 2 && std::string(argv[1]) == "--sample-json") {
    Timing timing;
    timing.configure(true, "run\"\\\n01");
    completed(timing, 0, 2, 41);
    completed(timing, 200, 3, 42);
    std::cout << timing.flushDueAt(at(1000)) << '\n';
    return 0;
  }
  static_assert(!std::is_copy_constructible<Timing::Frame>::value, "one owner per token");
  static_assert(std::is_nothrow_move_constructible<Timing::Frame>::value, "pending move is cheap");
  {
    const auto before = allocation_count;
    Timing off;
    CHECK(!off.enabled());
    off.configure(false, "a run id long enough to require allocation if it were copied");
    for (int i = 0; i < 1000; ++i) {
      auto frame = off.begin(1, 2, 3);
      CHECK(!frame.active());
      CHECK(!off.mark(frame, Stage::PoseWait));
      CHECK(!off.finish(frame, 1));
      CHECK(off.flushDue().empty());
    }
    CHECK(allocation_count == before);
    CHECK(off.totalStats().count == 0);
  }
  {
    Timing timing;
    timing.configure(true, "run\"\\\n01");
    completed(timing, 0, 2, 41);
    completed(timing, 200, 3, 42);
    CHECK(timing.totalStats().count == 2);
    CHECK(timing.totalStats().sum_ms == 30);
    CHECK(timing.totalStats().max_ms == 18);
    for (std::size_t i = 0; i < Timing::stage_count; ++i) {
      const auto& stats = timing.stageStats(static_cast<Stage>(i));
      CHECK(stats.count == 2 && stats.sum_ms == 5 && stats.max_ms == 3);
    }
    const auto* sample = timing.bufferedFrame(0);
    CHECK(sample && sample->seq == 1 && sample->revision == 41);
    CHECK(sample->source_stamp_sec == 1770000000);
    CHECK(sample->source_stamp_nanosec == 123456789);
    CHECK(sample->ros_source_age_ms_at_callback == -9.5);
    CHECK(sample->ros_source_age_ms_at_finish == 975.0);
    CHECK(sample->callback_total_ms == 12);  // independent of either ROS age
    CHECK(timing.bufferedFrame(1)->seq == 2);
    CHECK(timing.bufferedFrame(2) == nullptr);
    CHECK(timing.flushDueAt(at(999)).empty());
    const auto line = timing.flushDueAt(at(1000));
    CHECK(line.find("KR260_STAGE_TIMING {\"schema\":1") == 0);
    CHECK(line.find("\"run_id\":\"run\\\"\\\\\\u000a01\"") != std::string::npos);
    CHECK(line.find("\"source_stamp_sec\":1770000000,\"source_stamp_nanosec\":123456789") != std::string::npos);
    CHECK(line.find("\"stage_ms\":[2.000000,2.000000,2.000000,2.000000,2.000000,2.000000]") != std::string::npos);
    CHECK(line.find("\"sample_dropped\":0") != std::string::npos);
    CHECK(timing.bufferedFrames() == 0 && timing.totalStats().count == 0);
    completed(timing, 1100, 1, 43);
    CHECK(timing.bufferedFrame(0)->seq == 3);
    CHECK(timing.flushDueAt(at(1999)).empty());
    CHECK(!timing.flushDueAt(at(2000)).empty());
    CHECK(timing.flushDueAt(at(3000)).empty());
  }
  {
    Timing timing;
    timing.configure(true, "pose-resume");
    auto incoming = timing.beginAt(10, 20, 1, at(0));
    const auto sequence = incoming.sequence();
    Timing::Frame pending = std::move(incoming);
    CHECK(!incoming.active() && pending.active() && pending.sequence() == sequence);
    auto resumed = std::move(pending);
    CHECK(!pending.active());
    CHECK(!timing.markAt(resumed, Stage::PclDecode, at(50)));  // no skipped stages
    CHECK(!timing.markAt(resumed, Stage::PoseWait, at(-1)));  // monotonic only
    CHECK(timing.markAt(resumed, Stage::PoseWait, at(50)));
    CHECK(!timing.markAt(resumed, Stage::PoseWait, at(51)));  // no duplicate stages
    CHECK(timing.finishAt(resumed, 0, Outcome::Superseded,
                         std::numeric_limits<double>::infinity(), at(60)));
    CHECK(timing.stageStats(Stage::PoseWait).sum_ms == 50);
    CHECK(timing.stageStats(Stage::PclDecode).count == 0);
    const auto line = timing.flushDueAt(at(1000));
    CHECK(line.find("\"outcome\":\"superseded\"") != std::string::npos);
    CHECK(line.find("\"ros_source_age_ms_at_finish\":null") != std::string::npos);
    CHECK(line.find("\"stage_ms\":[50.000000,null,null,null,null,null]") != std::string::npos);
  }
  {
    Timing timing;
    timing.configure(true, "bounded");
    const auto before = allocation_count;
    for (int i = 0; i < 105; ++i) completed(timing, i, 0, std::uint64_t(i + 1));
    CHECK(allocation_count == before);  // enabled recording is also allocation-free
    CHECK(timing.bufferedFrames() == 100 && timing.droppedSamples() == 5);
    CHECK(timing.totalStats().count == 105);
    CHECK(timing.bufferedFrame(0)->seq == 6 && timing.bufferedFrame(99)->seq == 105);
    const auto line = timing.flushDueAt(at(1000));
    CHECK(line.find("\"sample_dropped\":5") != std::string::npos);
    CHECK(line.find("\"seq\":5,") == std::string::npos);
    CHECK(line.find("\"seq\":6,") != std::string::npos);
    CHECK(line.find("\"seq\":105,") != std::string::npos);
    auto stale = timing.beginAt(1, 2, 3, at(1100));
    timing.configure(false);
    CHECK(!timing.markAt(stale, Stage::PoseWait, at(1200)));
    timing.configure(true, "bounded");
    CHECK(!timing.finishAt(stale, 106, Outcome::Rejected, 0, at(1300)));
    auto next = timing.beginAt(1, 2, 3, at(1400));
    CHECK(next.sequence() == 107);  // sequence is never reused on reconfiguration
    Timing foreign;
    foreign.configure(true, "other-owner");
    CHECK(!foreign.finishAt(next, 1, Outcome::Published, 0, at(1500)));
  }
  std::cout << "PASS " << checks << " stage timing checks\n";
}
