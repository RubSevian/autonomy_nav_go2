/*
 * FAR Planner
 * Copyright (C) 2021 Fan Yang - All rights reserved
 * fanyang2@andrew.cmu.edu,   
 */



#include "far_planner/far_planner.h"
#include <cmath>
#include <cstdint>
#include <sstream>

/***************************************************************************************/
FARMaster::FARMaster()
{
   /* initialize node */
  nh_ = rclcpp::Node::make_shared("far_planner_node");
  
  /* initialize transform buffer and listener */
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(nh_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  RCLCPP_INFO(nh_->get_logger(), "FAR Planner Node Initiated");
}

void FARMaster::PublishPlannerStatus(const std::string& state, const std::string& code,
                                     const std::string& text, bool force) {
  if (!planner_status_pub_) return;
  const std::string signature = state + "|" + code + "|" + text;
  if (!force && signature == last_status_signature_) return;
  last_status_signature_ = signature;

  const NavNodePtr goal_ptr = graph_planner_.GetGoalNodePtr();
  const NavNodePtr graph_odom_ptr = graph_planner_.GetOdomNodePtr();
  if (goal_ptr != NULL) {
    goal_current_ = goal_ptr->position;
    graph_goal_origin_ = graph_planner_.GetOriginNodePos(false);
    has_goal_current_ = true;
    has_graph_goal_origin_ = true;
    goal_has_parent_ = goal_ptr->parent != NULL;
    goal_has_free_parent_ = goal_ptr->free_parent != NULL;
    goal_is_free_traversable_ = goal_ptr->is_free_traversable;
    goal_is_in_freespace_ = graph_planner_.IsGoalInFreespace();
  }

  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = nh_->now();
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "far_planner";
  status.hardware_id = "stage4d";
  status.level = (code == "WAYPOINT_PUBLISHED" || code == "GOAL_REACHED" ||
                  code == "GOAL_ACCEPTED") ? diagnostic_msgs::msg::DiagnosticStatus::OK
                                             : diagnostic_msgs::msg::DiagnosticStatus::WARN;
  status.message = text;
  auto add = [&status](const std::string& key, const std::string& value) {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = key;
    kv.value = value;
    status.values.push_back(kv);
  };
  auto point_value = [](const Point3D& point, bool available, const char* axis) {
    if (!available) return std::string("N/A");
    if (axis[0] == 'x') return std::to_string(point.x);
    if (axis[0] == 'y') return std::to_string(point.y);
    return std::to_string(point.z);
  };

  add("state", state);
  add("reason_code", code);
  add("reason_text", text);
  add("navigation_active", navigation_active_ ? "true" : "false");
  add("goal_received", goal_received_ ? "true" : "false");
  add("goal_frame", goal_frame_);
  add("is_odom_init", is_odom_init_ ? "true" : "false");
  add("is_cloud_init", is_cloud_init_ ? "true" : "false");
  add("is_graph_init", is_graph_init_ ? "true" : "false");
  add("far_lifecycle_state", lifecycle_state_);
  add("far_lifecycle_transition", lifecycle_transition_);
  add("pending_goal_present", has_pending_goal_ ? "true" : "false");
  add("queued_goal_revision", std::to_string(queued_goal_lifecycle_.queued_revision()));
  add("consumed_goal_revision", std::to_string(queued_goal_lifecycle_.consumed_revision()));
  add("robot_x", is_odom_init_ ? std::to_string(robot_pos_.x) : "N/A");
  add("robot_y", is_odom_init_ ? std::to_string(robot_pos_.y) : "N/A");
  add("robot_z", is_odom_init_ ? std::to_string(robot_pos_.z) : "N/A");
  add("graph_odom_x", graph_odom_ptr ? std::to_string(graph_odom_ptr->position.x) : "N/A");
  add("graph_odom_y", graph_odom_ptr ? std::to_string(graph_odom_ptr->position.y) : "N/A");
  add("graph_odom_z", graph_odom_ptr ? std::to_string(graph_odom_ptr->position.z) : "N/A");
  add("goal_terrain_associated", goal_ptr ? (graph_planner_.IsTerrainAssociated() ? "true" : "false") : "N/A");
  add("goal_original_x", point_value(goal_original_, has_goal_original_, "x"));
  add("goal_original_y", point_value(goal_original_, has_goal_original_, "y"));
  add("goal_original_z", point_value(goal_original_, has_goal_original_, "z"));
  add("graph_goal_origin_x", point_value(graph_goal_origin_, has_graph_goal_origin_, "x"));
  add("graph_goal_origin_y", point_value(graph_goal_origin_, has_graph_goal_origin_, "y"));
  add("graph_goal_origin_z", point_value(graph_goal_origin_, has_graph_goal_origin_, "z"));
  add("goal_current_x", point_value(goal_current_, has_goal_current_, "x"));
  add("goal_current_y", point_value(goal_current_, has_goal_current_, "y"));
  add("goal_current_z", point_value(goal_current_, has_goal_current_, "z"));
  add("goal_adjustment_distance", (has_goal_original_ && has_goal_current_)
          ? std::to_string((goal_current_ - goal_original_).norm()) : "N/A");
  add("distance_to_goal", (is_odom_init_ && has_goal_current_)
          ? std::to_string((robot_pos_ - goal_current_).norm()) : "N/A");
  add("terrain_cloud_points", temp_cloud_ptr_ ? std::to_string(temp_cloud_ptr_->size()) : "N/A");
  add("terrain_local_points", local_terrain_ptr_ ? std::to_string(local_terrain_ptr_->size()) : "N/A");
  add("surround_free_count", FARUtil::surround_free_cloud_ ? std::to_string(FARUtil::surround_free_cloud_->size()) : "N/A");
  add("surround_obs_count", FARUtil::surround_obs_cloud_ ? std::to_string(FARUtil::surround_obs_cloud_->size()) : "N/A");
  add("goal_local_free_count", goal_received_ ? std::to_string(goal_local_free_count_) : "N/A");
  add("goal_local_obs_count", goal_received_ ? std::to_string(goal_local_obs_count_) : "N/A");
  add("nav_graph_size", std::to_string(nav_graph_.size()));
  std::size_t graph_edge_count = 0;
  for (const auto& node : nav_graph_) if (node) graph_edge_count += node->connect_nodes.size();
  add("nav_graph_edge_count", std::to_string(graph_edge_count / 2));
  add("start_vertex_valid", odom_node_ptr_ ? "true" : "false");
  add("start_vertex_id", odom_node_ptr_ ? std::to_string(odom_node_ptr_->id) : "N/A");
  add("goal_vertex_valid", goal_ptr ? "true" : "false");
  add("goal_vertex_id", goal_ptr ? std::to_string(goal_ptr->id) : "N/A");
  add("goal_has_parent", has_goal_current_ ? (goal_has_parent_ ? "true" : "false") : "N/A");
  add("goal_has_free_parent", has_goal_current_ ? (goal_has_free_parent_ ? "true" : "false") : "N/A");
  add("goal_is_free_traversable", has_goal_current_ ? (goal_is_free_traversable_ ? "true" : "false") : "N/A");
  add("goal_is_in_freespace", has_goal_current_ ? (goal_is_in_freespace_ ? "true" : "false") : "N/A");
  add("path_size", goal_received_ ? std::to_string(path_size_) : "N/A");
  add("planning_failed", planning_failed_ ? "true" : "false");
  add("goal_reached", goal_reached_ ? "true" : "false");
  add("goal_revision", std::to_string(goal_revision_));
  add("graph_revision", std::to_string(graph_revision_));
  add("plan_revision", std::to_string(plan_revision_));
  add("committed_route_revision", std::to_string(committed_route_revision_));
  add("waypoint_revision", std::to_string(waypoint_revision_));
  add("committed_route_hash", has_committed_route_ ? std::to_string(committed_route_hash_) : "N/A");
  add("committed_route_nodes", committed_route_nodes_);
  add("committed_waypoint_x", has_committed_waypoint_ ? std::to_string(committed_waypoint_.x) : "N/A");
  add("committed_waypoint_y", has_committed_waypoint_ ? std::to_string(committed_waypoint_.y) : "N/A");
  add("path_validation", FARPathValidation::ToString(graph_planner_.GetLastPathStatus()));
  add("far_terrain_free_z", std::to_string(FARUtil::kFreeZ));
  add("far_temp_free_count", temp_free_ptr_ ? std::to_string(temp_free_ptr_->size()) : "N/A");
  add("far_temp_obs_count", temp_obs_ptr_ ? std::to_string(temp_obs_ptr_->size()) : "N/A");
  add("local_terrain_free_count", FARUtil::local_terrain_free_ ? std::to_string(FARUtil::local_terrain_free_->size()) : "N/A");
  add("local_terrain_obs_count", FARUtil::local_terrain_obs_ ? std::to_string(FARUtil::local_terrain_obs_->size()) : "N/A");
  array.status.push_back(status);
  planner_status_pub_->publish(array);
}

void FARMaster::LogLifecycleTransition(const std::string& transition, const std::string& state) {
  lifecycle_transition_ = transition;
  lifecycle_state_ = state;
  std::size_t edge_count = 0;
  for (const auto& node : nav_graph_) if (node) edge_count += node->connect_nodes.size();
  const std::string start_id = odom_node_ptr_ ? std::to_string(odom_node_ptr_->id) : "N/A";
  RCLCPP_INFO(nh_->get_logger(), "[FAR][LIFECYCLE] transition=%s state=%s vgraph_ready=%s graph_nodes=%zu graph_edges=%zu pending=%s queued_rev=%llu consumed_rev=%llu goal=(%.3f,%.3f,%.3f) start_valid=%s start_id=%s planning_requested=%s",
      transition.c_str(), state.c_str(), is_graph_init_ ? "true" : "false", nav_graph_.size(), edge_count / 2, has_pending_goal_ ? "true" : "false",
      static_cast<unsigned long long>(queued_goal_lifecycle_.queued_revision()), static_cast<unsigned long long>(queued_goal_lifecycle_.consumed_revision()),
      has_pending_goal_ ? pending_goal_.point.x : 0.0, has_pending_goal_ ? pending_goal_.point.y : 0.0, has_pending_goal_ ? pending_goal_.point.z : 0.0,
      odom_node_ptr_ ? "true" : "false", start_id.c_str(), goal_received_ ? "true" : "false");
}

void FARMaster::Init() {
  /* initialize subscriber and publisher */
  reset_graph_sub_    = nh_->create_subscription<std_msgs::msg::Empty>("/reset_visibility_graph", 5, std::bind(&FARMaster::ResetGraphCallBack, this, std::placeholders::_1));
  navigation_cancel_sub_ = nh_->create_subscription<std_msgs::msg::Empty>(
      "/navigation_cancel", 5, std::bind(&FARMaster::NavigationCancelCallBack, this, std::placeholders::_1));
  odom_sub_           = nh_->create_subscription<nav_msgs::msg::Odometry>("/odom_world", 5, std::bind(&FARMaster::OdomCallBack, this, std::placeholders::_1));
  terrain_sub_        = nh_->create_subscription<sensor_msgs::msg::PointCloud2>("/terrain_cloud", 1, std::bind(&FARMaster::TerrainCallBack, this, std::placeholders::_1));
  scan_sub_           = nh_->create_subscription<sensor_msgs::msg::PointCloud2>("/scan_cloud", 5, std::bind(&FARMaster::ScanCallBack, this, std::placeholders::_1));
  waypoint_sub_       = nh_->create_subscription<geometry_msgs::msg::PointStamped>("/goal_point", 1, std::bind(&FARMaster::WaypointCallBack, this, std::placeholders::_1));
  terrain_local_sub_  = nh_->create_subscription<sensor_msgs::msg::PointCloud2>("/terrain_local_cloud", 1, std::bind(&FARMaster::TerrainLocalCallBack, this, std::placeholders::_1));
  joy_command_sub_    = nh_->create_subscription<sensor_msgs::msg::Joy>("/joy", 5, std::bind(&FARMaster::JoyCommandCallBack, this, std::placeholders::_1));
  update_command_sub_ = nh_->create_subscription<std_msgs::msg::Bool>("/update_visibility_graph", 5, std::bind(&FARMaster::UpdateCommandCallBack, this, std::placeholders::_1));
  goal_pub_           = nh_->create_publisher<geometry_msgs::msg::PointStamped>("/way_point",5);
  planner_status_pub_ = nh_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/far/planner_status", rclcpp::QoS(1).transient_local());
  boundary_pub_       = nh_->create_publisher<geometry_msgs::msg::PolygonStamped>("/navigation_boundary",5);

  // Timers
  runtime_pub_        = nh_->create_publisher<std_msgs::msg::Float32>("/runtime",1);
  planning_time_pub_  = nh_->create_publisher<std_msgs::msg::Float32>("/planning_time",1);
  traverse_time_pub_  = nh_->create_publisher<std_msgs::msg::Float32>("/far_traverse_time", 5);

  // planning status publisher
  reach_goal_pub_     = nh_->create_publisher<std_msgs::msg::Bool>("/far_reach_goal_status", 5);
  navigation_active_pub_ = nh_->create_publisher<std_msgs::msg::Bool>(
      "/navigation_active", rclcpp::QoS(1).transient_local());
  PublishNavigationActive(false);
  PublishPlannerStatus("STARTUP", "STARTUP", "FAR initialized; waiting for odometry and obstacle map");

  // Terminal formatting subscriber
  read_command_sub_   = nh_->create_subscription<std_msgs::msg::String>("/read_file_dir", 1, std::bind(&FARMaster::ReadFileCommand, this, std::placeholders::_1));
  save_command_sub_   = nh_->create_subscription<std_msgs::msg::String>("/save_file_dir", 1, std::bind(&FARMaster::SaveFileCommand, this, std::placeholders::_1));

  // DEBUG Publisher
  dynamic_obs_pub_     = nh_->create_publisher<sensor_msgs::msg::PointCloud2>("/FAR_dynamic_obs_debug",1);
  surround_free_debug_ = nh_->create_publisher<sensor_msgs::msg::PointCloud2>("/FAR_free_debug",1);
  surround_obs_debug_  = nh_->create_publisher<sensor_msgs::msg::PointCloud2>("/FAR_obs_debug",1);
  scan_grid_debug_     = nh_->create_publisher<sensor_msgs::msg::PointCloud2>("/FAR_scanGrid_debug",1);
  new_PCL_pub_         = nh_->create_publisher<sensor_msgs::msg::PointCloud2>("/FAR_new_debug",1);
  terrain_height_pub_  = nh_->create_publisher<sensor_msgs::msg::PointCloud2>("/FAR_terrain_height_debug",1);

  //print publisher and subscriber init complete
  // RCLCPP_INFO(nh_->get_logger(), "FAR Planner Subscriber and Publisher Initiated");

  this->LoadROSParams();

  //print ROS params load complete
  RCLCPP_INFO(nh_->get_logger(), "FAR Planner ROS Params Initiated");

  /*init path generation thred callback*/
  const float duration_time = 1.0f / master_params_.main_run_freq;
  main_event_     = nh_->create_wall_timer(std::chrono::milliseconds(int(duration_time * 1000)), std::bind(&FARMaster::MainLoopCallBack, this));
  planning_event_ = nh_->create_wall_timer(std::chrono::milliseconds(int(duration_time * 1000)), std::bind(&FARMaster::PlanningCallBack, this));

  //print callback init complete
  RCLCPP_INFO(nh_->get_logger(), "FAR Planner Callback Initiated");

  /* init Dynamic Planner Processing Objects */
  contour_detector_.Init(cdetect_params_);
  graph_manager_.Init(nh_, graph_params_);
  graph_planner_.Init(nh_, gp_params_);
  contour_graph_.Init(nh_, cg_params_);
  planner_viz_.Init(nh_);
  map_handler_.Init(map_params_);
  scan_handler_.Init(scan_params_);
  graph_msger_.Init(nh_, msger_parmas_);

  //print processing objects init complete
  RCLCPP_INFO(nh_->get_logger(), "FAR Planner Processing Objects Initiated");

  /* init internal params */
  odom_node_ptr_      = NULL;
  is_cloud_init_      = false;
  is_odom_init_       = false;
  is_scan_init_       = false;
  is_planner_running_ = false;
  is_graph_init_      = false;
  is_reset_env_       = false;
  is_stop_update_     = false;
  is_init_completed_  = false;

  // allocate memory to pointers
  new_vertices_ptr_     = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
  temp_obs_ptr_         = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
  temp_free_ptr_        = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
  temp_cloud_ptr_       = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
  scan_grid_ptr_        = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
  local_terrain_ptr_    = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
  terrain_height_ptr_   = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
  viewpoint_around_ptr_ = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
  kdtree_viewpoint_obs_cloud_ = PointKdTreePtr(new pcl::KdTreeFLANN<PCLPoint>());

  // set kdtree sorted value
  FARUtil::kdtree_new_cloud_->setSortedResults(false);
  FARUtil::kdtree_filter_cloud_->setSortedResults(false);
  kdtree_viewpoint_obs_cloud_->setSortedResults(false);

  // init global utility cloud
  FARUtil::stack_new_cloud_->clear();
  FARUtil::stack_dyobs_cloud_->clear();

  // clear temp vectors and memory
  this->ClearTempMemory();
  FARUtil::robot_pos = Point3D(0,0,0);
  FARUtil::free_odom_p = Point3D(0,0,0);

  robot_pos_   = Point3D(0,0,0);
  nav_heading_ = Point3D(0,0,0);
  goal_waypoint_stamped_.header.frame_id = master_params_.world_frame;

  // waiting for one second
  std::this_thread::sleep_for(std::chrono::seconds(1));

  // init complete
  is_init_completed_ = true;
  RCLCPP_INFO(nh_->get_logger(), "FAR Planner Initiated Complete");
}

void FARMaster::ResetEnvironmentAndGraph() {
  PublishNavigationActive(false);
  has_pending_goal_ = false;
  defer_pending_goal_until_next_main_loop_ = false;
  queued_goal_lifecycle_.Clear();
  goal_received_ = false;
  has_last_requested_goal_ = false;
  has_goal_original_ = false;
  has_goal_current_ = false;
  has_graph_goal_origin_ = false;
  path_size_ = 0;
  planning_failed_ = false;
  goal_reached_ = false;
  this->ResetInternalValues();
  RCLCPP_INFO(nh_->get_logger(), "FAR Planner graph reset");
  graph_manager_.ResetCurrentGraph();
  map_handler_.ResetGripMapCloud();
  graph_planner_.ResetPlannerInternalValues();
  contour_graph_.ResetCurrentContour();
  /* Reset clouds */
  FARUtil::surround_obs_cloud_->clear();
  FARUtil::surround_free_cloud_->clear();
  FARUtil::stack_new_cloud_->clear();
  FARUtil::stack_dyobs_cloud_->clear();
  FARUtil::cur_new_cloud_->clear();
  FARUtil::cur_dyobs_cloud_->clear();
  /* Stop the robot if it is moving */
  goal_waypoint_stamped_.header.stamp = nh_->now();
  goal_waypoint_stamped_.point = FARUtil::Point3DToGeoMsgPoint(robot_pos_);
  goal_pub_->publish(goal_waypoint_stamped_);
  NodePtrStack empty_path;
  planner_viz_.VizPath(empty_path);
  PublishPlannerStatus("GRAPH_RESET", "GRAPH_RESET", "FAR graph and terrain state reset");
}

void FARMaster::MainLoopCallBack() {
  if (!is_init_completed_) {
    return;
  }
  if (is_reset_env_) {
      this->ResetEnvironmentAndGraph();
      is_reset_env_ = false;
      if (FARUtil::IsDebug) RCLCPP_WARN(nh_->get_logger(), "****************** Graph and Env Reset ******************");
      return;
  }

  if (!this->PreconditionCheck()) {
      if (!is_odom_init_) PublishPlannerStatus("WAITING_FOR_ODOM", "WAITING_FOR_ODOM", "waiting for /odom_world");
      else if (!is_cloud_init_) PublishPlannerStatus("WAITING_FOR_OBSTACLE_MAP_INIT", "WAITING_FOR_OBSTACLE_MAP_INIT", "waiting for non-empty FAR obstacle map");
      return;
  }

  /* add main process after this line */
  graph_manager_.UpdateRobotPosition(robot_pos_);
  odom_node_ptr_ = graph_manager_.GetOdomNode();
  if (odom_node_ptr_ == NULL) {
    RCLCPP_WARN(nh_->get_logger(),"FAR: Waiting for Odometry...");
    PublishPlannerStatus("WAITING_FOR_ODOM", "ODOM_NODE_MISSING", "odom message received but FAR odom node is unavailable");
    return;
  }
  /* Extract Vertices and new nodes */
  FARUtil::Timer.start_time("Total V-Graph Update");
  contour_detector_.BuildTerrainImgAndExtractContour(odom_node_ptr_, FARUtil::surround_obs_cloud_, realworld_contour_);
  contour_graph_.UpdateContourGraph(odom_node_ptr_, realworld_contour_);
  /* Adjust heights with terrain */
  map_handler_.AdjustCTNodeHeight(ContourGraph::contour_graph_);
  map_handler_.AdjustNodesHeight(nav_graph_);
  // Truncate for local range nodes
  graph_manager_.UpdateGlobalNearNodes();
  near_nav_graph_ = graph_manager_.GetExtendLocalNode();
  // Match near nav nodes with contour
  contour_graph_.MatchContourWithNavGraph(nav_graph_, near_nav_graph_, new_ctnodes_);
  if (master_params_.is_visual_opencv) {
    FARUtil::ConvertCTNodeStackToPCL(new_ctnodes_, new_vertices_ptr_);
    cv::Mat cloud_img = contour_detector_.GetCloudImgMat();
    contour_detector_.ShowCornerImage(cloud_img, new_vertices_ptr_);
  }
  /* update planner graph */
  new_nodes_.clear();
  if (!is_stop_update_ && graph_manager_.ExtractGraphNodes(new_ctnodes_)) {
    new_nodes_ = graph_manager_.GetNewNodes();
  }
  /* Graph Updating */
  graph_manager_.UpdateNavGraph(new_nodes_, is_stop_update_, clear_nodes_);

  runtimer_.data = FARUtil::Timer.end_time("Total V-Graph Update", false) / 1000.f; // Unit: second
  // runtimer_.data = FARUtil::Timer.end_time("Total V-Graph Update", is_graph_init_); // Unit: ms
  runtime_pub_->publish(runtimer_);

  /* Update v-graph in other modules */
  nav_graph_ = graph_manager_.GetNavGraph();
  ++graph_revision_;
  contour_graph_.ExtractGlobalContours();      // Global Polygon Update
  graph_planner_.UpdaetVGraph(nav_graph_);     // Graph Planner Update
  graph_msger_.UpdateGlobalGraph(nav_graph_);  // Graph Messager Update

  /* Publish local boundary to lower level local planner */
  this->LocalBoundaryHandler(ContourGraph::local_boundary_);

  /* Viz Navigation Graph */
  const NavNodePtr last_internav_ptr = graph_manager_.GetLastInterNavNode();
  if (last_internav_ptr != NULL) {
    planner_viz_.VizPoint3D(last_internav_ptr->position, "last_nav_node", VizColor::MAGNA, 1.0);
  }
  planner_viz_.VizNodes(clear_nodes_, "clear_nodes", VizColor::ORANGE);
  planner_viz_.VizNodes(graph_manager_.GetOutContourNodes(), "out_contour", VizColor::YELLOW);
  planner_viz_.VizPoint3D(FARUtil::free_odom_p, "free_odom_position", VizColor::ORANGE, 1.0);
  planner_viz_.VizGraph(nav_graph_);
  planner_viz_.VizContourGraph(ContourGraph::contour_graph_);
  planner_viz_.VizGlobalPolygons(ContourGraph::global_contour_, ContourGraph::unmatched_contour_);

  // publish nodes visualization
  planner_viz_.PubNodesVisualization();

  if (!is_graph_init_ && !nav_graph_.empty()) {
    is_graph_init_ = true;
    RCLCPP_INFO(nh_->get_logger(), "FAR Planner V-Graph initialized");
    LogLifecycleTransition("VGRAPH_READY", "READY");
    PublishPlannerStatus("VGRAPH_READY", "PLANNING", "FAR V-Graph initialized");
    defer_pending_goal_until_next_main_loop_ = has_pending_goal_;
  }
  this->TryConsumePendingGoal();
}

void FARMaster::TryConsumePendingGoal() {
  if (!has_pending_goal_) return;
  if (defer_pending_goal_until_next_main_loop_) {
    defer_pending_goal_until_next_main_loop_ = false;
    PublishPlannerStatus("GOAL_PENDING", "GOAL_WAITING_FOR_POST_GRAPH_CYCLE",
                         "queued goal retained for one complete post-V-Graph update");
    return;
  }
  if (!is_graph_init_ || nav_graph_.empty() || odom_node_ptr_ == NULL) {
    PublishPlannerStatus("GOAL_PENDING", "GOAL_WAITING_FOR_START",
                         "queued goal retained until graph and start vertex are valid");
    return;
  }
  LogLifecycleTransition("QUEUED_GOAL_CONSUME_BEGIN", "GOAL_PENDING");
  if (!queued_goal_lifecycle_.ConsumeIfReady(true, true, true)) {
    RCLCPP_ERROR(nh_->get_logger(), "[FAR][LIFECYCLE] pending-goal ownership mismatch");
    return;
  }
  const auto queued_goal = pending_goal_;
  has_pending_goal_ = false;
  LogLifecycleTransition("START_VERTEX_RESOLVED", "PLANNING");
  SetGoal(queued_goal);
  RCLCPP_INFO(nh_->get_logger(), "FAR Planner accepted the goal queued before graph/start initialization");
}

void FARMaster::PlanningCallBack() {
  if (!is_init_completed_ || !is_graph_init_) return;
  const NavNodePtr goal_ptr = graph_planner_.GetGoalNodePtr();
  if (odom_node_ptr_ == NULL || nav_graph_.empty()) {
    PublishNavigationActive(false);
    PublishPlannerStatus("PLANNING_WAIT", "START_OR_GRAPH_UNAVAILABLE",
                         "planning deferred: start vertex or V-Graph is unavailable");
    return;
  }
  if (goal_ptr == NULL) {
    graph_planner_.UpdateGraphTraverability(odom_node_ptr_, NULL);
    return;
  }
  ++plan_revision_;
  LogLifecycleTransition("GOAL_VERTEX_RESOLVED", "PLANNING");
  const Point3D ori_p = graph_planner_.GetOriginNodePos(true);
  PointCloudPtr goal_obs(new pcl::PointCloud<PCLPoint>());
  PointCloudPtr goal_free(new pcl::PointCloud<PCLPoint>());
  map_handler_.GetCloudOfPoint(ori_p, goal_obs, CloudType::OBS_CLOUD, true);
  map_handler_.GetCloudOfPoint(ori_p, goal_free, CloudType::FREE_CLOUD, true);
  goal_local_obs_count_ = goal_obs->size();
  goal_local_free_count_ = goal_free->size();
  graph_planner_.UpdateFreeTerrainGrid(ori_p, goal_obs, goal_free);
  graph_planner_.ReEvaluateGoalPosition(goal_ptr, !master_params_.is_multi_layer);
  goal_current_ = goal_ptr->position;
  has_goal_current_ = true;
  goal_is_in_freespace_ = graph_planner_.IsGoalInFreespace();

  graph_planner_.UpdateGoalNavNodeConnects(goal_ptr);
  graph_planner_.UpdaetVGraph(graph_manager_.GetNavGraph());
  graph_planner_.UpdateGraphTraverability(odom_node_ptr_, goal_ptr);
  goal_has_parent_ = goal_ptr->parent != NULL;
  goal_has_free_parent_ = goal_ptr->free_parent != NULL;
  goal_is_free_traversable_ = goal_ptr->is_free_traversable;

  NodePtrStack candidate_path;
  Point3D current_free_goal;
  NavNodePtr candidate_nav;
  bool is_planning_fails = false;
  bool is_current_free_nav = false;
  bool is_reach_goal = false;
  LogLifecycleTransition("ROUTE_SEARCH_BEGIN", "PLANNING");
  const bool path_found = graph_planner_.PathToGoal(
      goal_ptr, candidate_path, candidate_nav, current_free_goal, is_planning_fails,
      is_reach_goal, is_current_free_nav);
  path_size_ = candidate_path.size();
  planning_failed_ = is_planning_fails;
  goal_reached_ = is_reach_goal;

  const auto route_validation = FARPathValidation::ValidateRoute(candidate_path);
  bool candidate_valid = path_found && route_validation.ok() &&
      FARPathValidation::IsFiniteNode(candidate_nav);
  Point3D candidate_waypoint;
  if (candidate_valid) {
    candidate_waypoint = candidate_nav->position;
    if (candidate_nav != goal_ptr) candidate_waypoint = ProjectNavWaypoint(candidate_nav, nav_node_ptr_);
    candidate_valid = std::isfinite(candidate_waypoint.x) && std::isfinite(candidate_waypoint.y) &&
        std::isfinite(candidate_waypoint.z);
  }

  auto route_hash = [](const NodePtrStack& path) {
    std::uint64_t value = 1469598103934665603ULL;
    for (const auto& node : path) {
      value ^= static_cast<std::uint64_t>(node->id);
      value *= 1099511628211ULL;
    }
    return value;
  };
  auto route_nodes = [](const NodePtrStack& path) {
    std::ostringstream stream;
    for (std::size_t i = 0; i < path.size(); ++i) {
      if (i) stream << ',';
      stream << path[i]->id;
    }
    return stream.str();
  };

  if (candidate_valid) {
    const Point3D old_waypoint = committed_waypoint_;
    const bool had_old_waypoint = has_committed_waypoint_;
    const std::uint64_t candidate_hash = route_hash(candidate_path);
    const bool waypoint_changed = !has_committed_waypoint_ ||
        (candidate_waypoint - committed_waypoint_).norm() > FARUtil::kEpsilon;
    const bool route_changed = !has_committed_route_ || candidate_hash != committed_route_hash_ || waypoint_changed;
    // Commit all route metadata before publishing the corresponding waypoint.
    // A failed candidate never mutates the committed route.
    LogLifecycleTransition("ROUTE_SEARCH_SUCCESS", "PLANNING");
    if (route_changed) {
      committed_route_ = candidate_path;
      committed_route_hash_ = candidate_hash;
      committed_route_nodes_ = route_nodes(candidate_path);
      committed_waypoint_ = candidate_waypoint;
      has_committed_waypoint_ = true;
      has_committed_route_ = true;
      ++committed_route_revision_;
      ++waypoint_revision_;
      nav_node_ptr_ = candidate_nav;
      goal_waypoint_stamped_.header.stamp = nh_->now();
      goal_waypoint_stamped_.header.frame_id = master_params_.world_frame;
      goal_waypoint_stamped_.point = FARUtil::Point3DToGeoMsgPoint(candidate_waypoint);
      PublishNavigationActive(true);
      goal_pub_->publish(goal_waypoint_stamped_);
      planner_viz_.VizPoint3D(candidate_waypoint, "waypoint", VizColor::MAGNA, 1.5);
      planner_viz_.VizPath(committed_route_, is_current_free_nav);
      LogLifecycleTransition("ROUTE_PUBLISHED", "ACTIVE");
      PublishPlannerStatus("WAYPOINT_PUBLISHED", "ROUTE_COMMITTED",
          "validated FAR route committed and waypoint handed off", true);
      RCLCPP_INFO(nh_->get_logger(),
          "[FAR][ROUTE_COMMIT] goal_rev=%llu graph_rev=%llu plan_rev=%llu route_rev=%llu waypoint_rev=%llu old_wp=(%.3f,%.3f) new_wp=(%.3f,%.3f) hash=%llu nodes=%s",
          static_cast<unsigned long long>(goal_revision_), static_cast<unsigned long long>(graph_revision_),
          static_cast<unsigned long long>(plan_revision_), static_cast<unsigned long long>(committed_route_revision_),
          static_cast<unsigned long long>(waypoint_revision_),
          had_old_waypoint ? old_waypoint.x : 0.0f, had_old_waypoint ? old_waypoint.y : 0.0f,
          candidate_waypoint.x, candidate_waypoint.y, static_cast<unsigned long long>(candidate_hash),
          committed_route_nodes_.c_str());
    }
    is_planner_running_ = true;
  } else if (!is_reach_goal) {
    // A parent-chain inconsistency or missing graph connection is transient:
    // retain the active goal and retry next tick, but remove the motion permit
    // so localPlanner/pathFollower cannot continue a stale path.
    const auto status = graph_planner_.GetLastPathStatus();
    const std::string reason = FARPathValidation::ToString(
        route_validation.ok() ? status : route_validation.status);
    PublishNavigationActive(false);
    PublishPlannerStatus("REPLANNING_STOP", reason,
        "candidate route rejected; active goal retained for next graph update");
    RCLCPP_WARN(nh_->get_logger(), "[FAR][REPLAN_STOP] goal_rev=%llu graph_rev=%llu plan_rev=%llu reason=%s path=%zu",
        static_cast<unsigned long long>(goal_revision_), static_cast<unsigned long long>(graph_revision_),
        static_cast<unsigned long long>(plan_revision_), reason.c_str(), candidate_path.size());
  }

  auto reach_goal_msg = std_msgs::msg::Bool();
  reach_goal_msg.data = is_reach_goal;
  reach_goal_pub_->publish(reach_goal_msg);
  if (is_reach_goal) {
    PublishNavigationActive(false);
    has_committed_route_ = false;
    committed_route_.clear();
    has_last_requested_goal_ = false;

    PublishPlannerStatus("NAVIGATION_STOPPED", "GOAL_REACHED", "FAR reports goal reached", true);
  }
  plan_timer_.data = 0.0F;
  planning_time_pub_->publish(plan_timer_);
}

void FARMaster::LocalBoundaryHandler(const std::vector<PointPair>& local_boundary) {
  if (!master_params_.is_pub_boundary || local_boundary.empty()) return;

  geometry_msgs::msg::PolygonStamped boundary_poly;
  boundary_poly.header.frame_id = master_params_.world_frame;
  boundary_poly.header.stamp = nh_->now(); // Using the ROS2 method to get the current time

  float index_z = robot_pos_.z;
  std::vector<PointPair> sorted_boundary;
  for (const auto& edge : local_boundary) {
    if (FARUtil::DistanceToLineSeg2D(robot_pos_, edge) > master_params_.local_planner_range) continue;
    sorted_boundary.push_back(edge);
  }
  FARUtil::SortEdgesClockWise(robot_pos_, sorted_boundary); /* For better rviz visualization purpose only! */
  for (const auto& edge : sorted_boundary) {
    geometry_msgs::msg::Point32 geo_p1, geo_p2;
    geo_p1.x = edge.first.x,  geo_p1.y = edge.first.y,  geo_p1.z = index_z;
    geo_p2.x = edge.second.x, geo_p2.y = edge.second.y, geo_p2.z = index_z;
    boundary_poly.polygon.points.push_back(geo_p1), boundary_poly.polygon.points.push_back(geo_p2);
    index_z += 0.001f; // separate polygon lines
  }
  boundary_pub_->publish(boundary_poly);
}


Point3D FARMaster::ProjectNavWaypoint(const NavNodePtr& nav_node_ptr, const NavNodePtr& last_point_ptr) {
  if (nav_node_ptr == NULL) {
    RCLCPP_ERROR(nh_->get_logger(), "[FAR][ROUTE] refusing waypoint projection: candidate vertex is null");
    return robot_pos_;
  }
  bool is_momentum = false;
  if (last_point_ptr == nav_node_ptr || (last_point_ptr != NULL &&
      (last_point_ptr->position - nav_node_ptr->position).norm() < FARUtil::kNearDist)) {
    is_momentum = true;
  }
  Point3D waypoint = nav_node_ptr->position;
  float free_dist = master_params_.local_planner_range;
  const Point3D extend_p = this->ExtendViewpointOnObsCloud(nav_node_ptr, FARUtil::surround_obs_cloud_, free_dist);
  free_dist = std::max(free_dist, master_params_.robot_dim * 2.5f);
  if (master_params_.is_viewpoint_extend) {
    waypoint = extend_p;
    planner_viz_.VizViewpointExtend(nav_node_ptr, waypoint);
  }
  const Point3D diff_p = waypoint - robot_pos_;
  Point3D new_heading;
  if (is_momentum && nav_heading_.norm() > FARUtil::kEpsilon) {
    const float hdist = free_dist / 2.0f;
    const float ratio = std::min(hdist, diff_p.norm()) / hdist;
    new_heading = diff_p.normalize() * ratio + nav_heading_ * (1.0f - ratio);
  } else {
    new_heading = diff_p.normalize();
  }
  if (nav_heading_.norm() > FARUtil::kEpsilon && new_heading.norm_dot(nav_heading_) < 0.0f) { // negative direction reproject
    Point3D temp_heading(nav_heading_.y, -nav_heading_.x, nav_heading_.z);
    if (temp_heading.norm_dot(new_heading) < 0.0f) {
      temp_heading.x = -temp_heading.x, temp_heading.y = -temp_heading.y;
    }
    new_heading = temp_heading;
  }
  nav_heading_ = new_heading.normalize();
  if (diff_p.norm() < free_dist) {
    waypoint = waypoint + nav_heading_ * (free_dist - diff_p.norm());
  }
  return waypoint;
}

Point3D FARMaster::ExtendViewpointOnObsCloud(const NavNodePtr& nav_node_ptr, const PointCloudPtr& obsCloudIn, float& free_dist) {
  if (nav_node_ptr->free_direct != NodeFreeDirect::CONVEX || obsCloudIn->empty()) return nav_node_ptr->position;
  FARUtil::CropPCLCloud(obsCloudIn, viewpoint_around_ptr_, nav_node_ptr->position, free_dist + FARUtil::kNearDist);
  float maxR = std::min((nav_node_ptr->position - robot_pos_).norm(), free_dist) - FARUtil::kNearDist;
  maxR = std::max(maxR, 0.0f);
  bool is_wall = false;
  const Point3D direct = -FARUtil::SurfTopoDirect(nav_node_ptr->surf_dirs, is_wall);
  if (!is_wall) {
    Point3D waypoint = nav_node_ptr->position;
    if (viewpoint_around_ptr_->empty()) {
      waypoint = waypoint + direct * maxR;
    } else {
      kdtree_viewpoint_obs_cloud_->setInputCloud(viewpoint_around_ptr_);
      const int N_Thred = (int)std::floor(FARUtil::kNearDist / FARUtil::kLeafSize);
      const float R = FARUtil::kNearDist / 2.0f + FARUtil::kLeafSize;
      // ray tracing
      Point3D start_p = waypoint + direct * FARUtil::kNearDist;
      float ray_dist  = FARUtil::kNearDist; 
      bool is_occupied = int(FARUtil::PointInXCounter(start_p, R, kdtree_viewpoint_obs_cloud_)) > N_Thred;
      waypoint = start_p;
      while (!is_occupied && ray_dist < free_dist) {
        start_p = start_p + direct * FARUtil::kNearDist;
        ray_dist += FARUtil::kNearDist;
        is_occupied = int(FARUtil::PointInXCounter(start_p, R, kdtree_viewpoint_obs_cloud_)) > N_Thred;
        if (ray_dist < maxR) {
          waypoint = start_p;
        }
      }
      if (is_occupied) {
        waypoint = (nav_node_ptr->position + waypoint - direct * FARUtil::kNearDist) / 2.0f;
        waypoint.z = nav_node_ptr->position.z;
        free_dist = ray_dist - FARUtil::kNearDist;
      }
      return waypoint;
    }
  }
  return nav_node_ptr->position;
}


void FARMaster::LoadROSParams() {
  const std::string map_prefix      = "map_handler";
  const std::string scan_prefix     = "scan_handler";
  const std::string cdetect_prefix  = "c_detector";
  const std::string graph_prefix    = "graph";
  const std::string viz_prefix      = "viz";
  const std::string utility_prefix  = "util";
  const std::string planner_prefix  = "g_planner";
  const std::string contour_prefix  = "contour_graph";
  const std::string msger_prefix    = "graph_msger";

   // master params
  nh_->declare_parameter<float>("main_run_freq", 5.0);
  nh_->declare_parameter<float>("voxel_dim", 0.2);
  nh_->declare_parameter<float>("robot_dim", 0.8);
  nh_->declare_parameter<float>("vehicle_height", 0.75);
  nh_->declare_parameter<float>("sensor_range", 30.0);
  nh_->declare_parameter<float>("terrain_range", 15.0);
  nh_->declare_parameter<float>("local_planner_range", 5.0);
  nh_->declare_parameter<float>("visualize_ratio", 1.0);
  nh_->declare_parameter<bool>("is_viewpoint_extend", true);
  nh_->declare_parameter<bool>("is_multi_layer", false);
  nh_->declare_parameter<bool>("is_opencv_visual", true);
  nh_->declare_parameter<bool>("is_static_env", true);
  nh_->declare_parameter<bool>("is_pub_boundary", true);
  nh_->declare_parameter<bool>("is_debug_output", false);
  nh_->declare_parameter<bool>("is_attempt_autoswitch", true);
  nh_->declare_parameter<std::string>("world_frame", "map");
  
  // Get parameters
  nh_->get_parameter("main_run_freq", master_params_.main_run_freq);
  nh_->get_parameter("voxel_dim", master_params_.voxel_dim);
  nh_->get_parameter("robot_dim", master_params_.robot_dim);
  nh_->get_parameter("vehicle_height", master_params_.vehicle_height);
  nh_->get_parameter("sensor_range", master_params_.sensor_range);
  nh_->get_parameter("terrain_range", master_params_.terrain_range);
  nh_->get_parameter("local_planner_range", master_params_.local_planner_range);
  nh_->get_parameter("visualize_ratio", master_params_.viz_ratio);
  nh_->get_parameter("is_viewpoint_extend", master_params_.is_viewpoint_extend);
  nh_->get_parameter("is_multi_layer", master_params_.is_multi_layer);
  nh_->get_parameter("is_opencv_visual", master_params_.is_visual_opencv);
  nh_->get_parameter("is_static_env", master_params_.is_static_env);
  nh_->get_parameter("is_pub_boundary", master_params_.is_pub_boundary);
  nh_->get_parameter("is_debug_output", master_params_.is_debug_output);
  nh_->get_parameter("is_attempt_autoswitch", master_params_.is_attempt_autoswitch);
  nh_->get_parameter<std::string>("world_frame", master_params_.world_frame);
  master_params_.terrain_range = std::min(master_params_.terrain_range, master_params_.sensor_range);

  // print voxel_dim paramter in ROS2
  RCLCPP_INFO(nh_->get_logger(), "voxel_dim: %f", master_params_.voxel_dim);

  // Declare map parameters
  nh_->declare_parameter<float>(map_prefix + "/floor_height", 2.0);
  nh_->declare_parameter<float>(map_prefix + "/cell_length", 5.0);
  nh_->declare_parameter<float>(map_prefix + "/map_grid_max_length", 1000.0);
  nh_->declare_parameter<float>(map_prefix + "/map_grad_max_height", 100.0);

  // Get map parameters
  nh_->get_parameter(map_prefix + "/floor_height", map_params_.floor_height);
  nh_->get_parameter(map_prefix + "/cell_length", map_params_.cell_length);
  nh_->get_parameter(map_prefix + "/map_grid_max_length", map_params_.grid_max_length);
  nh_->get_parameter(map_prefix + "/map_grad_max_height", map_params_.grid_max_height);

  // Compute dependent parameters
  map_params_.height_voxel_dim = master_params_.voxel_dim * 2.0f;
  map_params_.cell_height = map_params_.floor_height / 2.5f;
  map_params_.sensor_range = master_params_.sensor_range;

  // Declare utility parameters
  nh_->declare_parameter<float>(utility_prefix + "/angle_noise", 15.0);
  nh_->declare_parameter<float>(utility_prefix + "/accept_max_align_angle", 15.0);
  nh_->declare_parameter<float>(utility_prefix + "/new_intensity_thred", 2.0);
  nh_->declare_parameter<float>(utility_prefix + "/nav_clear_dist", 0.5);
  nh_->declare_parameter<float>(utility_prefix + "/terrain_free_Z", 0.1);
  nh_->declare_parameter<int>(utility_prefix   + "/dyosb_update_thred", 4);
  nh_->declare_parameter<int>(utility_prefix   + "/new_point_counter", 10);
  nh_->declare_parameter<float>(utility_prefix + "/dynamic_obs_dacay_time", 10.0);
  nh_->declare_parameter<float>(utility_prefix + "/new_points_decay_time", 2.0);
  nh_->declare_parameter<int>(utility_prefix   + "/obs_inflate_size", 2);

  // Get utility parameters
  nh_->get_parameter(utility_prefix + "/angle_noise", FARUtil::kAngleNoise);
  nh_->get_parameter(utility_prefix + "/accept_max_align_angle", FARUtil::kAcceptAlign);
  nh_->get_parameter(utility_prefix + "/new_intensity_thred", FARUtil::kNewPIThred);
  nh_->get_parameter(utility_prefix + "/nav_clear_dist", FARUtil::kNavClearDist);
  nh_->get_parameter(utility_prefix + "/terrain_free_Z", FARUtil::kFreeZ);
  nh_->get_parameter(utility_prefix + "/dyosb_update_thred", FARUtil::kDyObsThred);
  nh_->get_parameter(utility_prefix + "/new_point_counter", FARUtil::KNewPointC);
  nh_->get_parameter(utility_prefix + "/dynamic_obs_dacay_time", FARUtil::kObsDecayTime);
  nh_->get_parameter(utility_prefix + "/new_points_decay_time", FARUtil::kNewDecayTime);
  nh_->get_parameter(utility_prefix + "/obs_inflate_size", FARUtil::kObsInflate);
  FARUtil::kLeafSize       = master_params_.voxel_dim;
  FARUtil::kNearDist       = master_params_.robot_dim;
  FARUtil::kHeightVoxel    = map_params_.height_voxel_dim;
  FARUtil::kMatchDist      = master_params_.robot_dim * 2.0f + FARUtil::kLeafSize;
  FARUtil::kNavClearDist   = master_params_.robot_dim / 2.0f + FARUtil::kLeafSize;
  FARUtil::kProjectDist    = master_params_.voxel_dim;
  FARUtil::worldFrameId    = master_params_.world_frame;
  FARUtil::kVizRatio       = master_params_.viz_ratio;
  FARUtil::kTolerZ         = map_params_.floor_height - FARUtil::kHeightVoxel;
  FARUtil::kCellLength     = map_params_.cell_length;
  FARUtil::kCellHeight     = map_params_.cell_height;
  FARUtil::kAcceptAlign    = FARUtil::kAcceptAlign / 180.0f * M_PI;
  FARUtil::kAngleNoise     = FARUtil::kAngleNoise  / 180.0f * M_PI; 
  FARUtil::robot_dim       = master_params_.robot_dim;
  FARUtil::IsStaticEnv     = master_params_.is_static_env;
  FARUtil::IsDebug         = master_params_.is_debug_output;
  FARUtil::IsMultiLayer    = master_params_.is_multi_layer;
  FARUtil::vehicle_height  = master_params_.vehicle_height;
  FARUtil::kSensorRange    = master_params_.sensor_range;
  FARUtil::kMarginDist     = master_params_.sensor_range - FARUtil::kMatchDist;
  FARUtil::kMarginHeight   = FARUtil::kTolerZ - FARUtil::kCellHeight / 2.0f;
  FARUtil::kTerrainRange   = master_params_.terrain_range;
  FARUtil::kLocalPlanRange = master_params_.local_planner_range;

  // graph planner params
  nh_->declare_parameter<float>(planner_prefix + "/converge_distance", 1.0);
  nh_->declare_parameter<float>(planner_prefix + "/goal_adjust_radius", 10.0);
  nh_->declare_parameter<int>(planner_prefix   + "/free_counter_thred", 5);
  nh_->declare_parameter<int>(planner_prefix   + "/reach_goal_vote_size", 5);
  nh_->declare_parameter<int>(planner_prefix   + "/path_momentum_thred", 5);
  
  nh_->get_parameter(planner_prefix + "/converge_distance", gp_params_.converge_dist);
  nh_->get_parameter(planner_prefix + "/goal_adjust_radius", gp_params_.adjust_radius);
  nh_->get_parameter(planner_prefix + "/free_counter_thred", gp_params_.free_thred);
  nh_->get_parameter(planner_prefix + "/reach_goal_vote_size", gp_params_.votes_size);
  nh_->get_parameter(planner_prefix + "/path_momentum_thred", gp_params_.momentum_thred);

  gp_params_.momentum_dist = master_params_.robot_dim / 2.0f;
  gp_params_.is_autoswitch = master_params_.is_attempt_autoswitch;

  // contour graph params
  cg_params_.kPillarPerimeter = master_params_.robot_dim * 4.0f;

  // dynamic graph params
  nh_->declare_parameter<int>(graph_prefix   + "/connect_votes_size", 10);
  nh_->declare_parameter<int>(graph_prefix   + "/clear_dumper_thred", 3);
  nh_->declare_parameter<int>(graph_prefix   + "/node_finalize_thred", 3);
  nh_->declare_parameter<int>(graph_prefix   + "/filter_pool_size", 12);
  nh_->declare_parameter<float>(graph_prefix + "/connect_angle_thred", 10.0);
  nh_->declare_parameter<float>(graph_prefix + "/dirs_filter_margin", 10.0);

  nh_->get_parameter(graph_prefix + "/connect_votes_size", graph_params_.votes_size);
  nh_->get_parameter(graph_prefix + "/clear_dumper_thred", graph_params_.dumper_thred);
  nh_->get_parameter(graph_prefix + "/node_finalize_thred", graph_params_.finalize_thred);
  nh_->get_parameter(graph_prefix + "/filter_pool_size", graph_params_.pool_size);
  nh_->get_parameter(graph_prefix + "/connect_angle_thred", graph_params_.kConnectAngleThred);
  nh_->get_parameter(graph_prefix + "/dirs_filter_margin", graph_params_.filter_dirs_margin);

  graph_params_.filter_pos_margin        = FARUtil::kNavClearDist;
  graph_params_.filter_dirs_margin       = FARUtil::kAngleNoise;
  graph_params_.kConnectAngleThred       = FARUtil::kAcceptAlign;
  graph_params_.frontier_perimeter_thred = FARUtil::kMatchDist * 4.0f;

  // graph messager params
  nh_->declare_parameter<int>(msger_prefix + "/robot_id", 0);
  nh_->get_parameter(msger_prefix + "/robot_id", msger_parmas_.robot_id);

  msger_parmas_.frame_id    = master_params_.world_frame;
  msger_parmas_.votes_size  = graph_params_.votes_size;
  msger_parmas_.pool_size   = graph_params_.pool_size;
  msger_parmas_.dist_margin = graph_params_.filter_pos_margin;

  // scan handler params
  scan_params_.terrain_range = master_params_.terrain_range;
  scan_params_.voxel_size    = master_params_.voxel_dim;
  scan_params_.ceil_height   = map_params_.floor_height;

  // contour detector params
  nh_->declare_parameter<float>(cdetect_prefix + "/resize_ratio", 5.0);
  nh_->declare_parameter<int>(cdetect_prefix   + "/filter_count_value", 5);
  nh_->declare_parameter<bool>(cdetect_prefix  + "/is_save_img", false);
  nh_->declare_parameter<std::string>(cdetect_prefix + "/img_folder_path", "");

  nh_->get_parameter(cdetect_prefix + "/resize_ratio", cdetect_params_.kRatio);
  nh_->get_parameter(cdetect_prefix + "/filter_count_value", cdetect_params_.kThredValue);
  nh_->get_parameter(cdetect_prefix + "/is_save_img", cdetect_params_.is_save_img);
  nh_->get_parameter(cdetect_prefix + "/img_folder_path", cdetect_params_.img_path);

  cdetect_params_.kBlurSize    = (int)std::round(FARUtil::kNavClearDist / master_params_.voxel_dim);
  cdetect_params_.sensor_range = master_params_.sensor_range;
  cdetect_params_.voxel_dim    = master_params_.voxel_dim;
}

void FARMaster::OdomCallBack(const nav_msgs::msg::Odometry::SharedPtr msg) {
  // transform from odom frame to mapping frame
  std::string odom_frame = msg->header.frame_id;
  tf2::Transform tf_odom_pose;
  tf2::fromMsg(msg->pose.pose, tf_odom_pose);
  
  if (!FARUtil::IsSameFrameID(odom_frame, master_params_.world_frame)) {
    if (FARUtil::IsDebug) RCLCPP_WARN_ONCE(nh_->get_logger(), "FARMaster: odom frame does NOT match with world frame!");
    tf2::Transform odom_to_world_tf_stamp;
    try
    {
      tf_buffer_->canTransform(master_params_.world_frame, odom_frame, tf2::TimePointZero, std::chrono::seconds(1));
      auto transform_stamped = tf_buffer_->lookupTransform(master_params_.world_frame, odom_frame, tf2::TimePointZero);
      tf2::fromMsg(transform_stamped.transform, odom_to_world_tf_stamp);
      tf_odom_pose = odom_to_world_tf_stamp * tf_odom_pose;
    }
    catch (tf2::TransformException& ex) {
      RCLCPP_ERROR(nh_->get_logger(), "Tracking odom TF lookup: %s",ex.what());
      return;
    }
  }
  
  robot_pos_.x = tf_odom_pose.getOrigin().x(); 
  robot_pos_.y = tf_odom_pose.getOrigin().y();
  robot_pos_.z = tf_odom_pose.getOrigin().z();

  // extract robot heading
  FARUtil::robot_pos = robot_pos_;
  double roll, pitch, yaw;
  tf_odom_pose.getBasis().getRPY(roll, pitch, yaw);
  robot_heading_ = Point3D(cos(yaw), sin(yaw), 0);

  if (!is_odom_init_) {
    // system start time
    FARUtil::systemStartTime = nh_->now().seconds();
    FARUtil::map_origin = robot_pos_;
    map_handler_.UpdateRobotPosition(robot_pos_);
  }

  const bool was_odom_init = is_odom_init_;
  is_odom_init_ = true;
  if (!was_odom_init) {
    PublishPlannerStatus("WAITING_FOR_TERRAIN", "WAITING_FOR_TERRAIN", "received first /odom_world message");
  }
}


void FARMaster::ProcessCloud(const sensor_msgs::msg::PointCloud2::SharedPtr pc, const PointCloudPtr cloudOut) 
{
  pcl::PointCloud<PCLPoint> temp_cloud;
  pcl::fromROSMsg(*pc, temp_cloud);
  cloudOut->clear(), *cloudOut = temp_cloud;
  if (cloudOut->empty()) return;
  FARUtil::FilterCloud(cloudOut, master_params_.voxel_dim);
  // transform cloud frame
  std::string cloud_frame = pc->header.frame_id;
  FARUtil::RemoveNanInfPoints(cloudOut);
  if (!FARUtil::IsSameFrameID(cloud_frame, master_params_.world_frame)) {
    if (FARUtil::IsDebug) RCLCPP_WARN_ONCE(nh_->get_logger(),"FARMaster: cloud frame does NOT match with world frame!");
    try
    {
      FARUtil::TransformPCLFrame(cloud_frame, 
                                 master_params_.world_frame, 
                                 tf_buffer_,
                                 cloudOut);
    }
    catch(const tf2::TransformException& ex)
    {
      RCLCPP_ERROR_STREAM(nh_->get_logger(), "[FAR][CLOUD_TF_ERROR] source=" << cloud_frame
                          << " target=" << master_params_.world_frame
                          << " stamp=" << rclcpp::Time(pc->header.stamp).seconds()
                          << " error=" << ex.what());
      PublishPlannerStatus("CLOUD_TF_ERROR", "CLOUD_TF_ERROR",
          "cloud transform failed: " + cloud_frame + " -> " + master_params_.world_frame + "; " + ex.what(), true);
      return;
    }
  }
}


void FARMaster::ScanCallBack(const sensor_msgs::msg::PointCloud2::SharedPtr scan_pc) {
  if (master_params_.is_static_env || !is_odom_init_) return;
  this->ProcessCloud(scan_pc, FARUtil::cur_scan_cloud_);
  scan_handler_.UpdateRobotPosition(robot_pos_);
}

void FARMaster::TerrainLocalCallBack(const sensor_msgs::msg::PointCloud2::SharedPtr pc) {
  if (master_params_.is_static_env) return;
  this->ProcessCloud(pc, local_terrain_ptr_);
  FARUtil::ExtractFreeAndObsCloud(local_terrain_ptr_, FARUtil::local_terrain_free_, FARUtil::local_terrain_obs_);
}

void FARMaster::TerrainCallBack(const sensor_msgs::msg::PointCloud2::SharedPtr pc) {
  if (!is_odom_init_) return;
  // update map grid robot center
  map_handler_.UpdateRobotPosition(FARUtil::robot_pos);
  if (!is_stop_update_) {
    this->ProcessCloud(pc, temp_cloud_ptr_);
    FARUtil::CropBoxCloud(temp_cloud_ptr_, robot_pos_, Point3D(master_params_.terrain_range,
                                                               master_params_.terrain_range,
                                                               FARUtil::kTolerZ));
    FARUtil::ExtractFreeAndObsCloud(temp_cloud_ptr_, temp_free_ptr_, temp_obs_ptr_);
    if (!master_params_.is_static_env) {
      FARUtil::RemoveOverlapCloud(temp_obs_ptr_, FARUtil::stack_dyobs_cloud_, true);
    }
    map_handler_.UpdateObsCloudGrid(temp_obs_ptr_);
    map_handler_.UpdateFreeCloudGrid(temp_free_ptr_);
    // extract new points
    FARUtil::ExtractNewObsPointCloud(temp_obs_ptr_,
                                     FARUtil::surround_obs_cloud_,
                                     FARUtil::cur_new_cloud_);
  } else { // stop env update
    temp_cloud_ptr_->clear();
    FARUtil::cur_new_cloud_->clear();
  }
  // extract surround free cloud & update terrain height
  map_handler_.GetSurroundFreeCloud(FARUtil::surround_free_cloud_);
  map_handler_.UpdateTerrainHeightGrid(FARUtil::surround_free_cloud_, terrain_height_ptr_);
  // update surround obs cloud
  map_handler_.GetSurroundObsCloud(FARUtil::surround_obs_cloud_);
  // extract dynamic obstacles
  FARUtil::cur_dyobs_cloud_->clear();
  if (!master_params_.is_static_env && !is_stop_update_) {
    this->ExtractDynamicObsFromScan(FARUtil::cur_scan_cloud_, 
                                    FARUtil::surround_obs_cloud_, 
                                    FARUtil::surround_free_cloud_, 
                                    FARUtil::cur_dyobs_cloud_);
    if (int(FARUtil::cur_dyobs_cloud_->size()) > FARUtil::kDyObsThred) {
      if (FARUtil::IsDebug) RCLCPP_WARN(nh_->get_logger(), "FARMaster: dynamic obstacle detected, size: %ld", FARUtil::cur_dyobs_cloud_->size());
      FARUtil::InflateCloud(FARUtil::cur_dyobs_cloud_, master_params_.voxel_dim, 1, true);
      map_handler_.RemoveObsCloudFromGrid(FARUtil::cur_dyobs_cloud_);
      FARUtil::RemoveOverlapCloud(FARUtil::surround_obs_cloud_, FARUtil::cur_dyobs_cloud_);
      FARUtil::FilterCloud(FARUtil::cur_dyobs_cloud_, master_params_.voxel_dim);
      // update new cloud
      *FARUtil::cur_new_cloud_ += *FARUtil::cur_dyobs_cloud_;
      FARUtil::FilterCloud(FARUtil::cur_new_cloud_, master_params_.voxel_dim);
    }
    // update world dynamic obstacles
    FARUtil::StackCloudByTime(FARUtil::cur_dyobs_cloud_, FARUtil::stack_dyobs_cloud_, FARUtil::kObsDecayTime, nh_);
  }
  
  // create and update kdtrees
  FARUtil::StackCloudByTime(FARUtil::cur_new_cloud_, FARUtil::stack_new_cloud_, FARUtil::kNewDecayTime, nh_);
  FARUtil::UpdateKdTrees(FARUtil::stack_new_cloud_);

  const bool was_cloud_init = is_cloud_init_;
  if (!FARUtil::surround_obs_cloud_->empty()) is_cloud_init_ = true;
  if (was_cloud_init != is_cloud_init_) {
    PublishPlannerStatus("TERRAIN_READY", "PLANNING", "FAR obstacle-map precondition became true");
  }

  /* visualize clouds */
  planner_viz_.VizPointCloud(new_PCL_pub_, FARUtil::stack_new_cloud_);
  planner_viz_.VizPointCloud(dynamic_obs_pub_, FARUtil::cur_dyobs_cloud_);
  planner_viz_.VizPointCloud(surround_free_debug_, FARUtil::surround_free_cloud_);
  planner_viz_.VizPointCloud(surround_obs_debug_,  FARUtil::surround_obs_cloud_);
  planner_viz_.VizPointCloud(terrain_height_pub_, terrain_height_ptr_);
  // visualize map grid
  PointStack neighbor_centers, occupancy_centers;
  map_handler_.GetNeighborCeilsCenters(neighbor_centers);
  map_handler_.GetOccupancyCeilsCenters(occupancy_centers);
  planner_viz_.VizMapGrids(neighbor_centers, occupancy_centers, map_params_.cell_length, map_params_.cell_height);
  // DBBUG visual raycast grids
  if (!master_params_.is_static_env) {
    scan_handler_.GridVisualCloud(scan_grid_ptr_, GridStatus::RAY);
    planner_viz_.VizPointCloud(scan_grid_debug_, scan_grid_ptr_);
  }
}

void FARMaster::ExtractDynamicObsFromScan(const PointCloudPtr scanCloudIn, 
                                          const PointCloudPtr obsCloudIn,
                                          const PointCloudPtr freeCloudIn,
                                          const PointCloudPtr dyObsCloudOut)
{
  scan_handler_.ReInitGrids();
  scan_handler_.SetCurrentScanCloud(scanCloudIn, freeCloudIn);
  scan_handler_.ExtractDyObsCloud(obsCloudIn, dyObsCloudOut);
}

void FARMaster::SetGoal(const geometry_msgs::msg::PointStamped& route_goal) {
  if (!is_graph_init_ || nav_graph_.empty() || odom_node_ptr_ == NULL) {
    pending_goal_ = route_goal;
    has_pending_goal_ = true;
    queued_goal_lifecycle_.Queue();
    LogLifecycleTransition("GOAL_QUEUED", "GOAL_PENDING");
    PublishNavigationActive(false);
    PublishPlannerStatus("GOAL_QUEUED", "GOAL_WAITING_FOR_START",
                         "goal retained until V-Graph and start vertex are valid", true);
    return;
  }
  goal_received_ = true;
  goal_frame_ = route_goal.header.frame_id.empty() ? "N/A" : route_goal.header.frame_id;
  if (!std::isfinite(route_goal.point.x) || !std::isfinite(route_goal.point.y) ||
      !std::isfinite(route_goal.point.z) || route_goal.header.frame_id.empty()) {
    RCLCPP_ERROR(nh_->get_logger(), "FAR Planner rejected goal with empty frame or non-finite coordinates");
    PublishPlannerStatus("GOAL_INVALID", "GOAL_INVALID", "goal has empty frame or non-finite coordinates", true);
    PublishNavigationActive(false);
    return;
  }
  Point3D goal_p(route_goal.point.x, route_goal.point.y, route_goal.point.z);
  const std::string goal_frame = route_goal.header.frame_id;
  if (!FARUtil::IsSameFrameID(goal_frame, master_params_.world_frame)) {
    if (!FARUtil::TransformPoint3DFrame(goal_frame, master_params_.world_frame, tf_buffer_, goal_p)) {
      RCLCPP_ERROR(nh_->get_logger(), "FAR Planner rejected goal: TF %s -> %s is unavailable",
                   goal_frame.c_str(), master_params_.world_frame.c_str());
      PublishPlannerStatus("GOAL_INVALID", "GOAL_TF_ERROR", "goal transform is unavailable", true);
      PublishNavigationActive(false);
      return;
    }
  }
  goal_original_ = goal_p;
  goal_current_ = goal_p;
  has_goal_original_ = true;
  has_goal_current_ = true;
  goal_local_free_count_ = 0;
  goal_local_obs_count_ = 0;
  path_size_ = 0;
  planning_failed_ = false;
  goal_reached_ = false;
  ++goal_revision_;
  has_committed_route_ = false;
  has_committed_waypoint_ = false;
  committed_route_.clear();
  committed_route_nodes_ = "N/A";
  // No old path may remain authorized while the new goal is being planned.
  PublishNavigationActive(false);
  graph_planner_.UpdateGoal(goal_p);
  LogLifecycleTransition("GOAL_VERTEX_RESOLVED", "PLANNING");
  PublishPlannerStatus("GOAL_ACCEPTED", "GOAL_ACCEPTED", "goal accepted; waiting for validated FAR route", true);
  FARUtil::Timer.start_time("Overall_executing", true);
  planner_viz_.VizPoint3D(goal_p, "original_goal", VizColor::RED, 1.5);
}

void FARMaster::PublishNavigationActive(bool active) {
  if (navigation_status_published_ && navigation_active_ == active && navigation_active_pub_) return;
  navigation_active_ = active;
  if (navigation_active_pub_) {
    std_msgs::msg::Bool msg;
    msg.data = active;
    navigation_active_pub_->publish(msg);
    navigation_status_published_ = true;
  }
}

void FARMaster::NavigationCancelCallBack(const std_msgs::msg::Empty::SharedPtr) {
  // This is deliberately narrower than the full environment reset: Point-LIO,
  // terrain, and the existing visibility graph must survive a user cancel.
  const bool had_goal = has_pending_goal_ || goal_received_ || navigation_active_ ||
                        graph_planner_.GetGoalNodePtr() != NULL;
  if (!had_goal) return;

  PublishNavigationActive(false);
  has_pending_goal_ = false;
  queued_goal_lifecycle_.Clear();
  goal_received_ = false;
  has_last_requested_goal_ = false;
  has_goal_original_ = false;
  has_goal_current_ = false;
  has_graph_goal_origin_ = false;
  goal_frame_ = "N/A";
  goal_local_free_count_ = 0;
  goal_local_obs_count_ = 0;
  path_size_ = 0;
  planning_failed_ = false;
  goal_reached_ = false;
  is_planner_running_ = false;
  nav_heading_ = Point3D(0,0,0);
  graph_planner_.CancelGoal();
  committed_route_.clear();
  has_committed_route_ = false;
  has_committed_waypoint_ = false;
  committed_route_nodes_ = "N/A";
  planner_viz_.ClearNavigationVisuals();

  // A navigation-active false transition is the authoritative stop signal for
  // localPlanner.  Re-publishing the current position also supersedes any old
  // waypoint for consumers which were already holding one.
  goal_waypoint_stamped_.header.stamp = nh_->now();
  goal_waypoint_stamped_.header.frame_id = master_params_.world_frame;
  goal_waypoint_stamped_.point = FARUtil::Point3DToGeoMsgPoint(robot_pos_);
  goal_pub_->publish(goal_waypoint_stamped_);
  PublishPlannerStatus("NAVIGATION_CANCELLED", "CANCELLED",
                       "goal cancelled; FAR graph and terrain preserved", true);
  RCLCPP_INFO(nh_->get_logger(), "[FAR][CANCEL] navigation goal cleared without graph reset");
}

void FARMaster::WaypointCallBack(const geometry_msgs::msg::PointStamped::SharedPtr route_goal) {
  if (!std::isfinite(route_goal->point.x) || !std::isfinite(route_goal->point.y) ||
      !std::isfinite(route_goal->point.z) || route_goal->header.frame_id.empty()) {
    RCLCPP_ERROR(nh_->get_logger(), "FAR Planner rejected goal with empty frame or non-finite coordinates");
    PublishPlannerStatus("GOAL_INVALID", "GOAL_INVALID", "goal has empty frame or non-finite coordinates", true);
    return;
  }
  // Goal publishers may retransmit the same click.  A duplicate must not reset
  // parents/route memory while PlanningCallBack is working on it.
  if (has_last_requested_goal_ && route_goal->header.frame_id == last_requested_goal_.header.frame_id &&
      std::fabs(route_goal->point.x - last_requested_goal_.point.x) < 1.0e-4 &&
      std::fabs(route_goal->point.y - last_requested_goal_.point.y) < 1.0e-4 &&
      std::fabs(route_goal->point.z - last_requested_goal_.point.z) < 1.0e-4) {
    return;
  }
  last_requested_goal_ = *route_goal;
  has_last_requested_goal_ = true;
  LogLifecycleTransition("GOAL_RECEIVED", is_graph_init_ ? "READY" : "INITIALIZING");
  if (!is_init_completed_ || !is_graph_init_) {
    pending_goal_ = *route_goal;
    has_pending_goal_ = true;
    queued_goal_lifecycle_.Queue();
    goal_received_ = true;
    goal_frame_ = route_goal->header.frame_id;
    LogLifecycleTransition("GOAL_QUEUED", "GOAL_PENDING");
    if (is_init_completed_) {
      PublishPlannerStatus("GOAL_QUEUED", "GOAL_QUEUED", "goal queued until V-Graph initialization", true);
    }
    return;
  }
  SetGoal(*route_goal);
}

/* allocate static utility PointCloud pointer memory */
PointCloudPtr  FARUtil::surround_obs_cloud_  = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
PointCloudPtr  FARUtil::surround_free_cloud_ = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
PointCloudPtr  FARUtil::stack_new_cloud_     = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
PointCloudPtr  FARUtil::cur_new_cloud_       = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
PointCloudPtr  FARUtil::cur_dyobs_cloud_     = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
PointCloudPtr  FARUtil::stack_dyobs_cloud_   = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
PointCloudPtr  FARUtil::cur_scan_cloud_      = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
PointCloudPtr  FARUtil::local_terrain_obs_   = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
PointCloudPtr  FARUtil::local_terrain_free_  = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
PointKdTreePtr FARUtil::kdtree_new_cloud_    = PointKdTreePtr(new pcl::KdTreeFLANN<PCLPoint>());
PointKdTreePtr FARUtil::kdtree_filter_cloud_ = PointKdTreePtr(new pcl::KdTreeFLANN<PCLPoint>());
/* init static utility values */
const float FARUtil::kEpsilon = 1e-7;
const float FARUtil::kINF     = std::numeric_limits<float>::max();
std::string FARUtil::worldFrameId;
float   FARUtil::kAngleNoise; 
Point3D FARUtil::robot_pos;
Point3D FARUtil::odom_pos;
Point3D FARUtil::map_origin;
Point3D FARUtil::free_odom_p;
float   FARUtil::robot_dim;
float   FARUtil::vehicle_height;
float   FARUtil::kLeafSize;
float   FARUtil::kHeightVoxel;
float   FARUtil::kNavClearDist;
float   FARUtil::kCellLength;
float   FARUtil::kCellHeight;
float   FARUtil::kNewPIThred;
float   FARUtil::kSensorRange;
float   FARUtil::kMarginDist;
float   FARUtil::kMarginHeight;
float   FARUtil::kTerrainRange;
float   FARUtil::kLocalPlanRange;
float   FARUtil::kFreeZ;
float   FARUtil::kVizRatio;
double  FARUtil::systemStartTime;
float   FARUtil::kObsDecayTime;
float   FARUtil::kNewDecayTime;
float   FARUtil::kNearDist;
float   FARUtil::kMatchDist;
float   FARUtil::kProjectDist;
int     FARUtil::kDyObsThred;
int     FARUtil::KNewPointC;
int     FARUtil::kObsInflate;
float   FARUtil::kTolerZ;
float   FARUtil::kAcceptAlign;
bool    FARUtil::IsStaticEnv;
bool    FARUtil::IsDebug;
bool    FARUtil::IsMultiLayer;
TimeMeasure FARUtil::Timer;

/* Global Graph */
DynamicGraphParams DynamicGraph::dg_params_;
NodePtrStack DynamicGraph::globalGraphNodes_;
std::size_t  DynamicGraph::id_tracker_;
std::unordered_map<std::size_t, NavNodePtr> DynamicGraph::idx_node_map_;
std::unordered_map<NavNodePtr, std::pair<int, std::unordered_set<NavNodePtr>>> DynamicGraph::out_contour_nodes_map_;

/* init static contour graph values */
CTNodeStack ContourGraph::polys_ctnodes_;
CTNodeStack ContourGraph::contour_graph_;
PolygonStack ContourGraph::contour_polygons_;
std::vector<PointPair> ContourGraph::global_contour_;
std::vector<PointPair> ContourGraph::unmatched_contour_;
std::vector<PointPair> ContourGraph::inactive_contour_;
std::vector<PointPair> ContourGraph::boundary_contour_;
std::vector<PointPair> ContourGraph::local_boundary_;
std::unordered_set<NavEdge, navedge_hash> ContourGraph::global_contour_set_;
std::unordered_set<NavEdge, navedge_hash> ContourGraph::boundary_contour_set_;

/* init terrain map values */
PointKdTreePtr MapHandler::kdtree_terrain_clould_;
std::vector<int> MapHandler::terrain_grid_occupy_list_;
std::vector<int> MapHandler::terrain_grid_traverse_list_;
std::unordered_set<int> MapHandler::neighbor_obs_indices_;
std::unordered_set<int> MapHandler::extend_obs_indices_;
std::unique_ptr<grid_ns::Grid<PointCloudPtr>> MapHandler::world_free_cloud_grid_;
std::unique_ptr<grid_ns::Grid<PointCloudPtr>> MapHandler::world_obs_cloud_grid_;
std::unique_ptr<grid_ns::Grid<std::vector<float>>> MapHandler::terrain_height_grid_;


int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  
  auto far_planner_node = std::make_shared<FARMaster>();
  far_planner_node->Init();
  rclcpp::spin(far_planner_node->GetNodeHandle());

  rclcpp::shutdown();

  return 0;
}
