#ifndef FAR_PLANNER_H
#define FAR_PLANNER_H

#include "utility.h"
#include "dynamic_graph.h"
#include "contour_detector.h"
#include "contour_graph.h"
#include "graph_planner.h"
#include "map_handler.h"
#include "planner_visualizer.h"
#include "scan_handler.h"
#include "graph_msger.h"
#include "queued_goal_lifecycle.h"
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <cstdint>


struct FARMasterParams {
    FARMasterParams() = default;
    float robot_dim; 
    float vehicle_height;
    float voxel_dim;
    float sensor_range;
    float terrain_range;
    float local_planner_range;
    float main_run_freq;
    float viz_ratio;
    bool  is_multi_layer;
    bool  is_viewpoint_extend;
    bool  is_visual_opencv;
    bool  is_static_env;
    bool  is_pub_boundary;
    bool  is_debug_output;
    bool  is_attempt_autoswitch;
    std::string world_frame;
};

class FARMaster
{
public:
    FARMaster();
    ~FARMaster() = default;
    
    void Init(); // ROS initialization
    rclcpp::Node::SharedPtr GetNodeHandle() { return nh_; }

private:
    rclcpp::Node::SharedPtr nh_;

    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reset_graph_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr navigation_cancel_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_command_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr update_command_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr terrain_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr terrain_local_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr waypoint_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr read_command_sub_, save_command_sub_;

    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr goal_pub_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr planner_status_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr boundary_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr runtime_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr planning_time_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr traverse_time_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reach_goal_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr navigation_active_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr dynamic_obs_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr surround_free_debug_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr surround_obs_debug_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr scan_grid_debug_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr new_PCL_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr terrain_height_pub_;


    rclcpp::TimerBase::SharedPtr planning_event_;
    rclcpp::TimerBase::SharedPtr main_event_;

    std_msgs::msg::Float32 runtimer_, plan_timer_;

    Point3D robot_pos_, robot_heading_, nav_heading_;

    bool is_reset_env_, is_stop_update_, is_init_completed_;

    geometry_msgs::msg::PointStamped goal_waypoint_stamped_;
    geometry_msgs::msg::PointStamped pending_goal_;
    bool has_pending_goal_ = false;
    QueuedGoalLifecycle queued_goal_lifecycle_;
    std::string lifecycle_state_ = "INITIALIZING";
    std::string lifecycle_transition_ = "STARTUP";

    bool is_cloud_init_ = false, is_scan_init_ = false, is_odom_init_ = false, is_planner_running_ = false;
    bool is_graph_init_ = false;
    bool navigation_active_ = false;
    bool navigation_status_published_ = false;

    // Retained facts for diagnostic snapshots. They are populated only from
    // data FAR has actually processed; "N/A" is reported otherwise.
    bool goal_received_ = false;
    bool has_goal_original_ = false;
    std::string goal_frame_ = "N/A";
    Point3D goal_original_;
    Point3D goal_current_;
    Point3D graph_goal_origin_;
    bool has_goal_current_ = false;
    bool has_graph_goal_origin_ = false;
    bool goal_has_parent_ = false;
    bool goal_has_free_parent_ = false;
    bool goal_is_free_traversable_ = false;
    bool goal_is_in_freespace_ = false;
    std::size_t goal_local_free_count_ = 0;
    std::size_t goal_local_obs_count_ = 0;
    std::size_t path_size_ = 0;
    bool planning_failed_ = false;
    bool goal_reached_ = false;
    std::string last_status_signature_;
    // Route commits are an atomic handoff boundary: no partially validated
    // candidate may replace these values or be sent to localPlanner.
    NodePtrStack committed_route_;
    bool has_committed_route_ = false;
    std::uint64_t goal_revision_ = 0;
    std::uint64_t graph_revision_ = 0;
    std::uint64_t plan_revision_ = 0;
    std::uint64_t committed_route_revision_ = 0;
    std::uint64_t waypoint_revision_ = 0;
    std::uint64_t committed_route_hash_ = 0;
    std::string committed_route_nodes_ = "N/A";
    Point3D committed_waypoint_;
    bool has_committed_waypoint_ = false;
    geometry_msgs::msg::PointStamped last_requested_goal_;
    bool has_last_requested_goal_ = false;

    void PublishNavigationActive(bool active);
    void PublishPlannerStatus(const std::string& state, const std::string& code, const std::string& text, bool force = false);
    void LogLifecycleTransition(const std::string& transition, const std::string& state);

    PointCloudPtr new_vertices_ptr_;
    PointCloudPtr temp_obs_ptr_;
    PointCloudPtr temp_free_ptr_;
    PointCloudPtr temp_cloud_ptr_;
    PointCloudPtr scan_grid_ptr_;
    PointCloudPtr local_terrain_ptr_;
    PointCloudPtr terrain_height_ptr_;

    /* veiwpoint extension clouds */
    PointCloudPtr  viewpoint_around_ptr_;
    PointKdTreePtr kdtree_viewpoint_obs_cloud_;

    NavNodePtr odom_node_ptr_ = NULL;
    NavNodePtr nav_node_ptr_  = NULL;
    NodePtrStack new_nodes_;
    NodePtrStack nav_graph_;
    NodePtrStack near_nav_graph_;
    NodePtrStack clear_nodes_;

    CTNodeStack new_ctnodes_;
    std::vector<PointStack> realworld_contour_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    /* module objects */
    ContourDetector contour_detector_;
    DynamicGraph graph_manager_;
    DPVisualizer planner_viz_;
    GraphPlanner graph_planner_;
    ContourGraph contour_graph_;
    MapHandler map_handler_;
    ScanHandler scan_handler_;
    GraphMsger graph_msger_;

    /* ROS Params */
    FARMasterParams     master_params_;
    ContourDetectParams cdetect_params_;
    DynamicGraphParams  graph_params_;
    GraphPlannerParams  gp_params_;
    ContourGraphParams  cg_params_;
    MapHandlerParams    map_params_;
    ScanHandlerParams   scan_params_;
    GraphMsgerParams    msger_parmas_;

    void LoadROSParams();

    void ResetEnvironmentAndGraph();

    void LocalBoundaryHandler(const std::vector<PointPair>& local_boundary);

    void MainLoopCallBack();

    void PlanningCallBack();
    
    void ProcessCloud(const sensor_msgs::msg::PointCloud2::SharedPtr pc, const PointCloudPtr cloudOut);

    void SetGoal(const geometry_msgs::msg::PointStamped& route_goal);

    
    Point3D ExtendViewpointOnObsCloud(const NavNodePtr& nav_node_ptr, const PointCloudPtr& obsCloudIn, float& free_dist);

    Point3D ProjectNavWaypoint(const NavNodePtr& nav_node_ptr, const NavNodePtr& last_point_ptr);

        // Callback Functions
    void OdomCallBack(const nav_msgs::msg::Odometry::SharedPtr msg);
    void TerrainCallBack(const sensor_msgs::msg::PointCloud2::SharedPtr pc);
    void TerrainLocalCallBack(const sensor_msgs::msg::PointCloud2::SharedPtr pc);

    // Rest of your callback functions...

    inline void ResetGraphCallBack(const std_msgs::msg::Empty::SharedPtr msg) {
        is_reset_env_ = true;
    }

    inline void JoyCommandCallBack(const sensor_msgs::msg::Joy::SharedPtr msg) {
        if (msg->buttons[4] > 0.5) {
            is_reset_env_ = true;
        }
    } 

    inline void UpdateCommandCallBack(const std_msgs::msg::Bool::SharedPtr msg) {
        if (is_stop_update_ && msg->data) {
            if (FARUtil::IsDebug) RCLCPP_WARN(nh_->get_logger(), "FARMaster: Resume visibility graph update.");
            is_stop_update_ = !msg->data;
        }
        if (!is_stop_update_ && !msg->data) {
            if (FARUtil::IsDebug) RCLCPP_WARN(nh_->get_logger(), "FARMaster: Stop visibility graph update.");
            is_stop_update_ = !msg->data;
        }   
    }

    inline void FakeTerminalInit() {
        std::cout<<std::endl;
        if (master_params_.is_static_env) {
            std::cout<<"\033[1;33m **************** STATIC ENV PLANNING **************** \033[0m\n"<<std::endl;
        } else {
            std::cout<< "\033[1;33m **************** DYNAMIC ENV PLANNING **************** \033[0m\n" << std::endl;
        }
        std::cout<<"\n"<<std::endl;
        if (!PreconditionCheck()) return;
        printf("\033[A"), printf("\033[A"), printf("\033[2K");
        if (is_graph_init_) {
            std::cout<< "\033[1;32m V-Graph Initialized \033[0m\n" << std::endl;
            std::cout<<std::endl<<std::endl;
        } else {
            std::cout<< "\033[1;31m V-Graph Resetting...\033[0m\n" << std::endl;
        }
    }

    inline void ReadFileCommand(const std_msgs::msg::String::SharedPtr msg) {
        if (!FARUtil::IsDebug) { // Terminal Output
            printf("\033[2J"), printf("\033[0;0H"); // cleanup screen
            FakeTerminalInit();
        }
    }

    inline void SaveFileCommand(const std_msgs::msg::String::SharedPtr msg) {
        if (!FARUtil::IsDebug) { // Terminal Output
            printf("\033[2J"), printf("\033[0;0H"); // cleanup screen
            FakeTerminalInit();
        }
    }

    void ScanCallBack(const sensor_msgs::msg::PointCloud2::SharedPtr scan_pc);
    void WaypointCallBack(const geometry_msgs::msg::PointStamped::SharedPtr route_goal);
    void NavigationCancelCallBack(const std_msgs::msg::Empty::SharedPtr msg);

    void ExtractDynamicObsFromScan(const PointCloudPtr scanCloudIn, 
                                   const PointCloudPtr obsCloudIn,
                                   const PointCloudPtr freeCloudIn, 
                                   const PointCloudPtr dyObsCloudOut);

    /* define inline functions */
    inline bool PreconditionCheck() {
        if (is_cloud_init_ && is_odom_init_) {
            return true;
        }
        return false;
    }
    inline void ClearTempMemory() {
        new_vertices_ptr_->clear();
        new_nodes_.clear();
        nav_graph_.clear();
        clear_nodes_.clear();
        new_ctnodes_.clear();
        near_nav_graph_.clear();
        realworld_contour_.clear();
    }

    inline void ResetInternalValues() {
        odom_node_ptr_ = NULL;
        is_planner_running_ = false;  
        is_graph_init_      = false; 
        ClearTempMemory();
    }
};

#endif
