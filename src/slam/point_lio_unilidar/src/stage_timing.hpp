#pragma once
#include <algorithm>
#include <cstdint>
#include <ostream>
#include <time.h>

namespace point_lio {
// Single executor thread only. Constant storage; no per-point instrumentation.
struct StageTiming {
  std::uint64_t count = 0;
  double wall_sum_ms = 0, cpu_sum_ms = 0, wall_max_ms = 0, cpu_max_ms = 0;
  void Add(double wall, double cpu) {
    ++count;
    wall_sum_ms += wall; cpu_sum_ms += cpu;
    wall_max_ms = std::max(wall_max_ms, wall);
    cpu_max_ms = std::max(cpu_max_ms, cpu);
  }
  void Json(std::ostream& out) const {
    out << "{\"count\":" << count << ",\"wall_sum_ms\":" << wall_sum_ms
        << ",\"cpu_sum_ms\":" << cpu_sum_ms << ",\"wall_max_ms\":" << wall_max_ms
        << ",\"cpu_max_ms\":" << cpu_max_ms << "}";
  }
};
class StageTimer {
 public:
  explicit StageTimer(StageTiming& stage) : stage_(&stage) { Start(); }
  ~StageTimer() { Stop(); }
  StageTimer(const StageTimer&) = delete;
  StageTimer& operator=(const StageTimer&) = delete;
  void Next(StageTiming& stage) { Stop(); stage_ = &stage; Start(); }
  void Stop() {
    if (!stage_) return;
    const auto wall = Read(CLOCK_MONOTONIC), cpu = Read(CLOCK_THREAD_CPUTIME_ID);
    stage_->Add((wall - wall_) / 1.e6, (cpu - cpu_) / 1.e6);
    stage_ = nullptr;
  }
 private:
  static std::int64_t Read(clockid_t clock) {
    timespec value{};
    clock_gettime(clock, &value);
    return value.tv_sec * 1000000000LL + value.tv_nsec;
  }
  void Start() { wall_ = Read(CLOCK_MONOTONIC); cpu_ = Read(CLOCK_THREAD_CPUTIME_ID); }
  StageTiming* stage_;
  std::int64_t wall_, cpu_;
};
}
