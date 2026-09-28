#include "far_planner/queued_goal_lifecycle.h"
#include <cassert>
#include <iostream>

int main() {
  QueuedGoalLifecycle lifecycle;
  lifecycle.Queue();
  assert(lifecycle.pending());
  assert(!lifecycle.ConsumeIfReady(false, false, false));
  assert(!lifecycle.ConsumeIfReady(true, false, true));
  assert(!lifecycle.ConsumeIfReady(true, true, false));
  assert(lifecycle.ConsumeIfReady(true, true, true));
  assert(!lifecycle.pending() && lifecycle.consumed_revision() == 1);
  assert(!lifecycle.ConsumeIfReady(true, true, true));
  lifecycle.Queue(); lifecycle.Queue();
  assert(lifecycle.queued_revision() == 3);
  assert(lifecycle.ConsumeIfReady(true, true, true));
  lifecycle.Queue(); lifecycle.Clear();
  assert(!lifecycle.ConsumeIfReady(true, true, true));
  std::cout << "far_queued_goal_lifecycle_test: PASS\n";
  return 0;
}
