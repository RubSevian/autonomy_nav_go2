#ifndef FAR_PLANNER_QUEUED_GOAL_LIFECYCLE_H
#define FAR_PLANNER_QUEUED_GOAL_LIFECYCLE_H

#include <cstdint>

class QueuedGoalLifecycle {
 public:
  void Queue() { pending_ = true; ++queued_revision_; }
  bool ConsumeIfReady(bool vgraph_ready, bool graph_nonempty, bool start_valid) {
    if (!pending_ || !vgraph_ready || !graph_nonempty || !start_valid) return false;
    pending_ = false; ++consumed_revision_; return true;
  }
  void Clear() { pending_ = false; }
  bool pending() const { return pending_; }
  std::uint64_t queued_revision() const { return queued_revision_; }
  std::uint64_t consumed_revision() const { return consumed_revision_; }
 private:
  bool pending_ = false;
  std::uint64_t queued_revision_ = 0;
  std::uint64_t consumed_revision_ = 0;
};

#endif
