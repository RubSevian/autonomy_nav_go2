#include "far_planner/far_planner.h"

int main(int argc, char** argv){
  rclcpp::init(argc, argv);

  try {
    auto far_planner_node = std::make_shared<FARMaster>();
    far_planner_node->Init();
    if (rclcpp::ok()) rclcpp::spin(far_planner_node->GetNodeHandle());
  } catch (const rclcpp::exceptions::RCLError&) {
    // SIGINT may arrive while Init allocates the graph/map and ROS publishers.
    // Suppress only shutdown-context errors; real initialization errors propagate.
    if (rclcpp::ok()) throw;
  }
  if (rclcpp::ok()) rclcpp::shutdown();

  return 0;
}
