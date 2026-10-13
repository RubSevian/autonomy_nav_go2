// Reproduce early-goal correction before any traversability callback. This
// exercises the actual GraphPlanner/ContourGraph code, not a copied algorithm.
#include "far_planner/graph_planner.h"
#include <cassert>
#include <limits>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  {
    auto node = std::make_shared<rclcpp::Node>("far_start_dependency_test");
    FARUtil::kLeafSize = .1f;
    FARUtil::kSensorRange = 10.f;
    FARUtil::kTolerZ = 1.f;
    FARUtil::odom_pos = Point3D(0,0,0);
    GraphPlanner planner;
    GraphPlannerParams parameters{};
    parameters.adjust_radius = .8f;
    planner.Init(node, parameters);
    auto goal = std::make_shared<NavNode>();
    auto start = std::make_shared<NavNode>();
    goal->position = Point3D(1,0,.34f);
    start->position = Point3D(0,0,.34f);
    assert(!planner.GetOdomNodePtr());
    assert(!planner.ReEvaluateGoalPosition(goal, nullptr, false));
    assert(!planner.ReEvaluateGoalPosition(nullptr, start, false));
    assert(planner.ReEvaluateGoalPosition(goal, start, false));
    assert(!planner.GetOdomNodePtr());  // No implicit traversability prerequisite.
    assert(std::isfinite(goal->position.x));
    start->position.x = std::numeric_limits<float>::quiet_NaN();
    const auto previous = goal->position;
    assert(!planner.ReEvaluateGoalPosition(goal, start, false));
    assert(goal->position.x == previous.x && goal->position.z == previous.z);
  }
  rclcpp::shutdown();
}
