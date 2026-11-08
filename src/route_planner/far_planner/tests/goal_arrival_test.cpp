#include "far_planner/graph_planner.h"
#include <cassert>
#include <limits>

int main(int argc, char** argv) {
  using FARGoalArrival::Evaluate;
  using FARGoalArrival::State;
  // Recorded 21:39 failure: graph inside, current vehicle outside.
  assert(Evaluate(.275276, -.219998, .342720, -.464083,
                  .342720, -.464083, .25) == State::APPROACHING);
  assert(Evaluate(.252182, -.233095, .342720, -.464083,
                  .342720, -.464083, .25) == State::REQUESTED_REACHED);
  assert(Evaluate(.25, 0, 0, 0, 0, 0, .25) == State::REQUESTED_REACHED);
  assert(Evaluate(.250001, 0, 0, 0, 0, 0, .25) == State::APPROACHING);
  assert(Evaluate(.6, 0, 0, 0, .6, 0, .25) == State::ADJUSTED_REACHED);
  assert(Evaluate(.2, 0, 0, 0, .4, 0, .25) == State::REQUESTED_REACHED);
  assert(Evaluate(NAN, 0, 0, 0, 0, 0, .25) == State::APPROACHING);

  rclcpp::init(argc, argv);
  {
    auto node = std::make_shared<rclcpp::Node>("far_goal_arrival_test");
    FARUtil::kLeafSize = .1f; FARUtil::kNearDist = .1f;
    FARUtil::IsMultiLayer = true;
    GraphPlanner planner;
    GraphPlannerParams params{};
    params.adjust_radius = .8f; params.converge_dist = .25f;
    planner.Init(node, params);
    auto start = std::make_shared<NavNode>();
    auto goal = std::make_shared<NavNode>();
    start->id = 1; goal->id = 2;
    start->position = Point3D(.2481f, 0, 0);
    goal->position = Point3D(0, 0, 0); goal->is_navpoint = true;
    planner.UpdaetVGraph({start, goal});
    planner.UpdateGoal(goal->position);
    planner.UpdateGraphTraverability(start, goal);
    goal->parent = start; goal->free_parent = start;
    NodePtrStack path; NavNodePtr waypoint; Point3D target;
    bool failed, reached, free_nav;
    // Repeated boundary excursions outside must retain the same active goal.
    for (double x : {.253231, .25001, .26, .250001}) {
      const auto arrival = Evaluate(x, 0, 0, 0, 0, 0, .25);
      assert(planner.PathToGoal(goal, arrival, path, waypoint, target, failed, reached, free_nav));
      assert(!failed && !reached && planner.GetGoalNodePtr() == goal);
      assert(path.size() == 2 && waypoint == goal);
    }
    assert(planner.PathToGoal(goal, Evaluate(.2441666, 0, 0, 0, 0, 0, .25),
                             path, waypoint, target, failed, reached, free_nav));
    assert(!failed && reached && !planner.GetGoalNodePtr());
    // Reaching B alone must retain A and the graph goal for continued planning.
    planner.UpdateGoal(goal->position);
    goal->position.x = .6f;
    assert(planner.PathToGoal(goal, Evaluate(.6, 0, 0, 0, .6, 0, .25),
                             path, waypoint, target, failed, reached, free_nav));
    assert(!failed && !reached && planner.GetGoalNodePtr() == goal);
  }
  rclcpp::shutdown();
}
