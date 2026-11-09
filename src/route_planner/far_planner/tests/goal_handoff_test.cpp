#include "far_planner/far_planner.h"
#include <cassert>

struct FARGoalContractTest {
  static void Run() {
    auto a=std::make_shared<NavNode>(); auto b=std::make_shared<NavNode>();
    a->position=Point3D(0,0,0); b->position=Point3D(1,2,0);
    const auto snapshot=FARMaster::RouteGeometry({a,b});
    assert(snapshot=="[[0,0],[1,2]]");
    b->position.x=7;
    assert(snapshot=="[[0,0],[1,2]]"); // immutable even if graph node moves
    assert(FARMaster::RouteGeometry({a})=="[]");
    assert(FARMaster::RouteGeometry({a,nullptr})=="[]");
    FARMaster planner;
    planner.goal_received_=true;
    planner.has_goal_original_=true;
    planner.has_committed_route_=true;
    planner.navigation_active_=true;
    planner.PublishNavigationActive(false);
    assert(!planner.navigation_active_);
    assert(!planner.has_committed_route_);
    assert(planner.goal_received_ && planner.has_goal_original_);
    // Heartbeats must not accidentally restore authorization or discard A.
    planner.PublishNavigationActive(false);
    assert(!planner.has_committed_route_ && planner.goal_received_);
    planner.PublishNavigationActive(true);
    assert(planner.navigation_active_ && !planner.has_committed_route_);
  }
};
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  FARGoalContractTest::Run();
  rclcpp::shutdown();
}
