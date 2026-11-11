#include "stage_timing.hpp"
#include <cassert>
#include <chrono>
#include <sstream>
#include <thread>

int main() {
  point_lio::StageTiming first, second;
  {
    point_lio::StageTimer timer(first);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    timer.Next(second);
    timer.Stop();
    timer.Stop();  // Explicit stop plus destructor must not double count.
  }
  assert(first.count == 1 && second.count == 1);
  assert(first.wall_sum_ms >= 4);
  assert(first.cpu_sum_ms >= 0 && first.cpu_sum_ms < first.wall_sum_ms);
  const auto early_return = [&] { point_lio::StageTimer timer(second); return; };
  early_return();
  assert(second.count == 2);
  std::ostringstream out; first.Json(out);
  assert(out.str().find("\"count\":1") != std::string::npos);
}
