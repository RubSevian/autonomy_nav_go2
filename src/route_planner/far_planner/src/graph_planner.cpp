/*
 * FAR Planner
 * Copyright (C) 2021 Fan Yang - All rights reserved
 * fanyang2@andrew.cmu.edu,   
 */



#include "far_planner/graph_planner.h"

/***************************************************************************************/

const char INIT_BIT = char(0); // 0000
const char OBS_BIT  = char(1); // 0001
const char FREE_BIT = char(2); // 0010


void GraphPlanner::Init(const rclcpp::Node::SharedPtr nh, const GraphPlannerParams& params) {
    nh_ = nh;
    gp_params_ = params;
    is_goal_init_ = false;
    current_graph_.clear();
    // attemptable planning listener
    attemptable_sub_ = nh_->create_subscription<std_msgs::msg::Bool>("/planning_attemptable", 5, std::bind(&GraphPlanner::AttemptStatusCallBack, this, std::placeholders::_1));
    // initialize terrain grid
    const int col_num = std::ceil(gp_params_.adjust_radius * 2.0f / FARUtil::kLeafSize);
    Eigen::Vector3i grid_size(col_num, col_num, 1);
    Eigen::Vector3d grid_origin(0,0,0);
    Eigen::Vector3d grid_resolution(FARUtil::kLeafSize, FARUtil::kLeafSize, FARUtil::kLeafSize);
    free_terrain_grid_ = std::make_unique<grid_ns::Grid<char>>(grid_size, INIT_BIT, grid_origin, grid_resolution, 3);
}

void GraphPlanner::UpdateGraphTraverability(const NavNodePtr& odom_node_ptr, const NavNodePtr& goal_ptr) 
{
    if (odom_node_ptr == NULL || current_graph_.empty()) {
        RCLCPP_ERROR(nh_->get_logger(), "GP: Update global graph traversablity fails.");
        return;
    }
    odom_node_ptr_ = odom_node_ptr;
    this->InitNodesStates(current_graph_);
    // start expand the whole current_graph_
    odom_node_ptr_->gscore = 0.0;
    IdxSet open_set;
    std::priority_queue<NavNodePtr, NodePtrStack, nodeptr_gcomp> open_queue;
    IdxSet close_set;
    // Expansion from odom node to all reachable navigation node
    open_queue.push(odom_node_ptr_);
    open_set.insert(odom_node_ptr_->id);
    while (!open_set.empty()) {
        const NavNodePtr current = open_queue.top();
        open_queue.pop();
        open_set.erase(current->id);
        close_set.insert(current->id);
        current->is_traversable = true; // reachable from current position
        for (const auto& neighbor : current->connect_nodes) {
            if (close_set.count(neighbor->id) || this->IsInvalidBoundary(current, neighbor)) continue;
            float edist = this->EulerCost(current, neighbor);
            if (neighbor == goal_ptr && edist > FARUtil::kEpsilon && !FARUtil::IsAtSameLayer(neighbor, current)) { // check for multi layer traverse cost
                const Point3D diff_p = neighbor->position - current->position;
                float factor = std::hypotf(diff_p.x, diff_p.y) / edist;
                if (factor > FARUtil::kEpsilon) {
                    edist /= factor;
                } else {
                    continue;
                }
            }
            const float temp_gscore = current->gscore + edist;
            if (temp_gscore < neighbor->gscore) {
                neighbor->parent = current;
                neighbor->gscore = temp_gscore;
                if (!open_set.count(neighbor->id)) {
                    open_queue.push(neighbor);
                    open_set.insert(neighbor->id);
                }
            }
        }
    }
    std::priority_queue<NavNodePtr, NodePtrStack, nodeptr_fgcomp> fopen_queue;
    IdxSet fopen_set;
    close_set.clear();
    // Expansion from odom node to all covered navigation node
    odom_node_ptr_->fgscore = 0.0;
    fopen_queue.push(odom_node_ptr_);
    fopen_set.insert(odom_node_ptr_->id);
    while (!fopen_set.empty()) {
        const NavNodePtr current = fopen_queue.top();
        fopen_queue.pop();
        fopen_set.erase(current->id);
        close_set.insert(current->id);
        current->is_free_traversable = true; // reachable from current position
        for (const auto& neighbor : current->connect_nodes) {
            if (!neighbor->is_covered || close_set.count(neighbor->id) || this->IsInvalidBoundary(current, neighbor)) continue;
            const float e_dist = this->EulerCost(current, neighbor);
            if (neighbor == goal_ptr && (!is_goal_in_freespace_ || e_dist > FARUtil::kTerrainRange)) continue;
            const float temp_fgscore = current->fgscore + e_dist;
            if (temp_fgscore < neighbor->fgscore) {
                neighbor->free_parent = current;
                neighbor->fgscore = temp_fgscore;
                if (!fopen_set.count(neighbor->id)) {
                    fopen_queue.push(neighbor);
                    fopen_set.insert(neighbor->id);
                } 
            }
        }
    }
}

void GraphPlanner::UpdateGoalNavNodeConnects(const NavNodePtr& goal_ptr)
{
    if (goal_ptr == NULL || is_use_internav_goal_) return;
    for (const auto& node_ptr : current_graph_) {
        if (node_ptr == goal_ptr) continue;
        if (this->IsValidConnectToGoal(node_ptr, goal_ptr)) {
            const bool is_directly_connect = node_ptr->is_odom ? true : false;
            DynamicGraph::RecordPolygonVote(node_ptr, goal_ptr, gp_params_.votes_size, is_directly_connect);
        } else {
            DynamicGraph::DeletePolygonVote(node_ptr, goal_ptr, gp_params_.votes_size);
        }
        const auto it = goal_ptr->edge_votes.find(node_ptr->id);
        if (it != goal_ptr->edge_votes.end() && FARUtil::IsVoteTrue(it->second, false)) {
            DynamicGraph::AddPolyEdge(node_ptr, goal_ptr), DynamicGraph::AddEdge(node_ptr, goal_ptr);
            node_ptr->is_block_to_goal = false;
        } else {
            DynamicGraph::ErasePolyEdge(node_ptr, goal_ptr), DynamicGraph::EraseEdge(node_ptr, goal_ptr);
            node_ptr->is_block_to_goal = true;
        }
    }
}

bool GraphPlanner::IsValidConnectToGoal(const NavNodePtr& node_ptr, const NavNodePtr& goal_node_ptr) {
    if (node_ptr->is_traversable && (!node_ptr->is_block_to_goal || IsResetBlockStatus(node_ptr, goal_node_ptr))) {
        const Point3D diff_p = goal_node_ptr->position - node_ptr->position;
        if (DynamicGraph::IsConvexConnect(node_ptr, goal_node_ptr) && (!FARUtil::IsAtSameLayer(node_ptr, goal_node_ptr) || FARUtil::IsOutReducedDirs(diff_p, node_ptr)) && 
            ContourGraph::IsNavToGoalConnectFreePolygon(node_ptr, goal_node_ptr)) 
        {
            return true;
        }
    }
    return false;
}

bool GraphPlanner::PathToGoal(const NavNodePtr& goal_ptr,
                              NodePtrStack& global_path,
                              NavNodePtr& _nav_node_ptr,
                              Point3D& _goal_p,
                              bool& _is_fail,
                              bool& _is_succeed,
                              bool& _is_free_nav)
{
    _is_fail = false;
    _is_succeed = false;
    _is_free_nav = false;
    global_path.clear();
    last_path_status_ = FARPathValidation::Status::NO_VALID_PATH;
    if (!is_goal_init_ || odom_node_ptr_ == NULL || goal_ptr == NULL || current_graph_.empty()) {
        last_path_status_ = FARPathValidation::Status::INVALID_PARENT_CHAIN;
        RCLCPP_WARN(nh_->get_logger(), "GP: path request is not initialized; retaining active goal for retry.");
        _is_fail = true;
        return false;
    }
    if (!FARPathValidation::IsFiniteNode(odom_node_ptr_) || !FARPathValidation::IsFiniteNode(goal_ptr)) {
        last_path_status_ = FARPathValidation::Status::NONFINITE_NODE;
        _is_fail = true;
        return false;
    }

    _goal_p = goal_ptr->position;
    if ((odom_node_ptr_->position - _goal_p).norm() < gp_params_.converge_dist ||
        (odom_node_ptr_->position - origin_goal_pos_).norm() < gp_params_.converge_dist) {
        global_path = {odom_node_ptr_, goal_ptr};
        const auto validation = FARPathValidation::ValidateRoute(global_path);
        last_path_status_ = validation.status;
        if (!validation.ok()) { _is_fail = true; global_path.clear(); return false; }
        _is_succeed = true;
        _is_free_nav = is_free_nav_goal_;
        _nav_node_ptr = goal_ptr;
        GoalReset();
        is_goal_init_ = false;
        return true;
    }

    if (!command_is_free_nav_) is_free_nav_goal_ = false;
    else if (goal_ptr->is_free_traversable) is_free_nav_goal_ = true;
    if (gp_params_.is_autoswitch && command_is_free_nav_ && !goal_ptr->is_free_traversable) {
        // Retry the ordinary graph in the next timer cycle.  Do not revive a
        // previously recorded path: graph topology may already have changed.
        is_free_nav_goal_ = false;
    }
    _is_free_nav = is_free_nav_goal_;

    const auto reconstruction = FARPathValidation::ReconstructAndValidate(
        goal_ptr, is_free_nav_goal_, odom_node_ptr_);
    last_path_status_ = reconstruction.status;
    if (!reconstruction.ok()) {
        _is_fail = true;
        RCLCPP_WARN(nh_->get_logger(), "GP: rejecting parent chain (%s): %s; will retry without clearing goal",
                    FARPathValidation::ToString(reconstruction.status), reconstruction.detail.c_str());
        return false;
    }
    global_path.assign(reconstruction.path.begin(), reconstruction.path.end());
    _nav_node_ptr = NextNavWaypointFromPath(global_path, goal_ptr);
    if (!FARPathValidation::IsFiniteNode(_nav_node_ptr)) {
        last_path_status_ = FARPathValidation::Status::NO_VALID_PATH;
        global_path.clear();
        _is_fail = true;
        return false;
    }
    RecordPathInfo(global_path);
    return true;
}

bool GraphPlanner::ReconstructPath(const NavNodePtr& goal_node_ptr,
                                   const bool& is_free_nav,
                                   NodePtrStack& global_path)
{
    const auto result = FARPathValidation::ReconstructAndValidate(
        goal_node_ptr, is_free_nav, odom_node_ptr_);
    last_path_status_ = result.status;
    global_path.clear();
    if (!result.ok()) {
        RCLCPP_WARN(nh_->get_logger(), "GP: reconstruct path rejected (%s): %s",
                    FARPathValidation::ToString(result.status), result.detail.c_str());
        return false;
    }
    global_path.assign(result.path.begin(), result.path.end());
    return true;
}

NavNodePtr GraphPlanner::NextNavWaypointFromPath(const NodePtrStack& global_path, const NavNodePtr) {
    const auto validation = FARPathValidation::ValidateRoute(global_path);
    if (!validation.ok() || odom_node_ptr_ == NULL) {
        last_path_status_ = validation.ok() ? FARPathValidation::Status::INVALID_PARENT_CHAIN : validation.status;
        RCLCPP_WARN(nh_->get_logger(), "GP: refusing invalid route for waypoint (%s)",
                    FARPathValidation::ToString(last_path_status_));
        return NULL;
    }
    std::size_t nav_idx = 1;
    NavNodePtr nav_point_ptr = global_path[nav_idx];
    float dist = (nav_point_ptr->position - odom_node_ptr_->position).norm();
    while (dist < gp_params_.converge_dist && ++nav_idx < global_path.size()) {
        nav_point_ptr = global_path[nav_idx];
        dist = (nav_point_ptr->position - odom_node_ptr_->position).norm();
    }
    return nav_point_ptr;
}

void GraphPlanner::UpdateGoal(const Point3D& goal) {
    this->GoalReset();
    is_use_internav_goal_ = false;
    float min_dist = FARUtil::kNearDist;
    for (const auto& node_ptr : current_graph_) {
        node_ptr->is_block_to_goal = false;
        if (node_ptr->is_navpoint) { // check if goal is near internav node
            const float cur_dist = (node_ptr->position - goal).norm();
            if (cur_dist < min_dist) {
                is_use_internav_goal_   = true;
                goal_node_ptr_          = node_ptr;
                min_dist                = cur_dist;
                goal_node_ptr_->is_goal = true;
            }
        }
    }
    if (!is_use_internav_goal_) {
        DynamicGraph::CreateNavNodeFromPoint(goal, goal_node_ptr_, false, false, true);
        DynamicGraph::AddNodeToGraph(goal_node_ptr_);
    }
    if (FARUtil::IsDebug) RCLCPP_INFO(nh_->get_logger(), "GP: *********** new goal updated ***********");
    is_goal_init_          = true;
    is_global_path_init_   = false;
    is_terrain_associated_ = false;
    origin_goal_pos_       = goal_node_ptr_->position;
    is_free_nav_goal_      = command_is_free_nav_;
    next_waypoint_         = Point3D(0,0,0);
    last_waypoint_dist_    = 0.0f;
    last_planning_odom_    = Point3D(0,0,0);
    path_momentum_counter_ = 0;
    recorded_path_.clear();
    if (!FARUtil::IsMultiLayer) {
        goal_node_ptr_->position.z = MapHandler::NearestTerrainHeightofNavPoint(origin_goal_pos_, is_terrain_associated_) + FARUtil::vehicle_height;
    }
    this->ResetFreeTerrainGridOrigin(goal_node_ptr_->position);
}

void GraphPlanner::ReEvaluateGoalPosition(const NavNodePtr& goal_ptr, const bool& is_adjust_height)
{
    if (is_use_internav_goal_) return; // return if using an exsiting internav node as goal
    if (is_adjust_height && is_global_path_init_ && recorded_path_.size() > 1) { // use path to adjust goal height
        const auto it = recorded_path_.end() - 2;
        if (!is_terrain_associated_) {
            goal_ptr->position.z = (*it)->position.z;
        } else if ((*it)->is_odom) {
            goal_ptr->position.z = (*it)->position.z;
        }
        
    }
    const Eigen::Vector3i ori_sub = free_terrain_grid_->Pos2Sub(origin_goal_pos_.x, origin_goal_pos_.y, grid_center_.z);
    const Point3D ori_pos_height(origin_goal_pos_.x, origin_goal_pos_.y, goal_ptr->position.z);
    const bool is_origin_free = ContourGraph::IsPoint3DConnectFreePolygon(ori_pos_height, odom_node_ptr_->position) ? true : false;
    if (is_origin_free) {
        goal_ptr->position = ori_pos_height;
    } else { // reproject to nearby free space
        std::array<int, 4> dx = {-1, 0, 1, 0};
        std::array<int, 4> dy = { 0, 1, 0,-1};
        std::deque<int> q;
        std::unordered_set<int> visited_set;
        q.push_back(free_terrain_grid_->Sub2Ind(ori_sub));
        visited_set.insert(free_terrain_grid_->Sub2Ind(ori_sub));
        int valid_idx = -1;
        while (!q.empty()) {
            const int cur_id = q.front();
            q.pop_front();
            if (free_terrain_grid_->GetCell(cur_id) == FREE_BIT) {
                valid_idx = cur_id;
                break;
            }
            for (int i=0; i<4; i++) {
                Eigen::Vector3i csub = free_terrain_grid_->Ind2Sub(cur_id);
                csub.x() += dx[i], csub.y() += dy[i], csub.z() = 0;
                if (!free_terrain_grid_->InRange(csub)) continue;
                const int cidx = free_terrain_grid_->Sub2Ind(csub);
                if (!visited_set.count(cidx)) {
                    q.push_back(cidx);
                    visited_set.insert(cidx);
                }
            }
        }
        if (valid_idx != -1 && free_terrain_grid_->InRange(valid_idx)) {
            Point3D new_p = Point3D(free_terrain_grid_->Ind2Pos(valid_idx));
            new_p.z = goal_ptr->position.z;
            const float pred = (goal_ptr->position - ori_pos_height).norm();
            const float curd = (new_p - ori_pos_height).norm();
            if (abs(curd - pred) > FARUtil::kLeafSize && !ContourGraph::IsEdgeCollideBoundary(goal_ptr->position, new_p)) {
                if (FARUtil::IsDebug) RCLCPP_INFO(nh_->get_logger(), "GP: adjusting goal into free space.");
                goal_ptr->position = new_p;
            }
        } 
        if (FARUtil::IsNodeInLocalRange(goal_ptr)) {
            Point3D current_goal_pos = goal_ptr->position;
            if (ContourGraph::ReprojectPointOutsidePolygons(current_goal_pos, FARUtil::kNearDist)) {
                if (FARUtil::IsDebug) {
                    const float reproject_dist = (current_goal_pos - origin_goal_pos_).norm_flat();
                    RCLCPP_INFO(nh_->get_logger(),"GP: current goal is inside polygon, reproject goal position distance to origin goal: %f.", reproject_dist);
                }
                goal_ptr->position = current_goal_pos;
            }
        }
    }
}

void GraphPlanner::AttemptStatusCallBack(const std_msgs::msg::Bool::SharedPtr msg) {
    if (command_is_free_nav_ && msg->data) { // current goal is not attemptable
        if (FARUtil::IsDebug) RCLCPP_WARN(nh_->get_logger(), "GP: switch to attemptable planning mode.");
        command_is_free_nav_ = false;
        path_momentum_counter_ = gp_params_.momentum_thred;
    } 
    if (!command_is_free_nav_ && !msg->data) { // current attemptable planning
        if (FARUtil::IsDebug) RCLCPP_WARN(nh_->get_logger(), "GP: planning without attempting.");
        command_is_free_nav_ = true; 
        path_momentum_counter_ = gp_params_.momentum_thred;
    }
}

void GraphPlanner::UpdateFreeTerrainGrid(const Point3D& center,
                                         const PointCloudPtr& obsCloudIn, 
                                         const PointCloudPtr& freeCloudIn) 
{
    // reset grid
    const Point3D origin_center(origin_goal_pos_.x, origin_goal_pos_.y, center.z);
    this->ResetFreeTerrainGridOrigin(origin_center);
    free_terrain_grid_->ReInitGrid(INIT_BIT);
    // evaluate goal freespace status
    is_goal_in_freespace_ = false;
    if (!freeCloudIn->empty() || !obsCloudIn->empty()) { // process terrain cloud
        const int C_IF = FARUtil::kObsInflate;
        for (const auto& point : obsCloudIn->points) { // Set obstacle cloud in free terrain grid
            Eigen::Vector3i c_sub = free_terrain_grid_->Pos2Sub(point.x, point.y, grid_center_.z);
            for (int i = -C_IF; i <= C_IF; i++) {
                for (int j = -C_IF; j <= C_IF; j++) {
                    Eigen::Vector3i sub = c_sub;
                    sub.x() += i, sub.y() += j, sub.z() = 0;
                    if (free_terrain_grid_->InRange(sub)) {
                        const int ind = free_terrain_grid_->Sub2Ind(sub);
                        free_terrain_grid_->GetCell(ind) = free_terrain_grid_->GetCell(ind) | OBS_BIT;
                    }
                }
            }
        }
        for (const auto& point : freeCloudIn->points) { // Set free cloud in free terrain grid
            Eigen::Vector3i c_sub = free_terrain_grid_->Pos2Sub(point.x, point.y, grid_center_.z);
            for (int i = -1; i <= 1; i++) {
                for (int j = -1; j <= 1; j++) {
                    Eigen::Vector3i sub = c_sub;
                    sub.x() += i, sub.y() += j, sub.z() = 0;
                    if (free_terrain_grid_->InRange(sub)) {
                        const int ind = free_terrain_grid_->Sub2Ind(sub);
                        free_terrain_grid_->GetCell(ind) = free_terrain_grid_->GetCell(ind) | FREE_BIT;
                        is_goal_in_freespace_ = true;
                    }
                }
            }
        }
    }
    if (!is_goal_in_freespace_) { // check for freespace status if no terrain clouds around
        float min_dist = FARUtil::kINF;
        for (const auto& node_ptr : current_graph_) {
            if (!node_ptr->is_navpoint) continue;
            const float cur_dist = (node_ptr->position - center).norm_flat();
            if (cur_dist < FARUtil::kLocalPlanRange && FARUtil::IsAtSameLayer(node_ptr, goal_node_ptr_)) {
                if (cur_dist < min_dist) {
                    goal_node_ptr_->position.z = node_ptr->position.z;
                    min_dist = cur_dist;
                }
                is_goal_in_freespace_ = true;
            }
        }
    }
}
