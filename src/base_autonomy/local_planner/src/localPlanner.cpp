#include <math.h>
#include <cmath>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <chrono>
#include <iostream>
#include <algorithm>
#include <array>
#include <limits>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/clock.hpp"
#include "builtin_interfaces/msg/time.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <nav_msgs/msg/path.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <sensor_msgs/msg/imu.h>

#include "visibility_graph_msg/msg/local_path_constraint.hpp"
#include "local_planner/narrow_passage.hpp"

#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "rmw/types.h"
#include "rmw/qos_profiles.h"

using namespace std;

const double PI = 3.1415926;

#define PLOTPATHSET 1

string pathFolder;
double vehicleLength = 0.62;
double vehicleWidth = 0.40;
double sensorOffsetX = 0;
double sensorOffsetY = 0;
bool twoWayDrive = true;
double laserVoxelSize = 0.05;
double terrainVoxelSize = 0.2;
bool useTerrainAnalysis = false;
bool checkObstacle = true;
bool checkRotObstacle = false;
double adjacentRange = 3.5;
double obstacleHeightThre = 0.2;
double groundHeightThre = 0.1;
double costHeightThre = 0.1;
double costScore = 0.02;
bool useCost = false;
const int laserCloudStackNum = 1;
int laserCloudCount = 0;
int pointPerPathThre = 2;
double minRelZ = -0.5;
double maxRelZ = 0.25;
double maxSpeed = 1.0;
double dirWeight = 0.02;
double dirThre = 90.0;
bool dirToVehicle = false;
double pathScale = 1.0;
double minPathScale = 0.75;
double pathScaleStep = 0.25;
bool enableNarrowPassageMode = false;  // legacy follower compatibility
bool enableOrientationAwareCheck = false;
bool enableNarrowRecoveredSelection = false;
double narrowFootprintLength = 0.62;
double narrowFootprintWidth = 0.40;
double narrowLongitudinalMargin = 0.02;
double narrowLateralMargin = 0.02;
double narrowEnterYawToleranceDeg = 6.0;
double narrowContinueYawToleranceDeg = 9.0;
double narrowStopYawToleranceDeg = 12.0;
double narrowSpeedScale = 0.4;
double narrowMaxTangentDeltaDeg = 12.0;
double narrowMaxCurvatureRadPerM = 1.0;
double narrowRecoveryYawLimitDeg = 20.0;
bool pathScaleBySpeed = true;
double minPathRange = 1.0;
double pathRangeStep = 0.5;
bool pathRangeBySpeed = true;
bool pathCropByGoal = true;
bool autonomyMode = false;
double autonomySpeed = 1.0;
double joyToSpeedDelay = 2.0;
double joyToCheckObstacleDelay = 5.0;
double goalCloseDis = 1.0;
double goalClearRange = 0.5;
double goalX = 0;
double goalY = 0;

// Temporal candidate stabilization: safety may always override these gates.
bool enableTemporalPathStabilization = true;
double pathSwitchScoreMargin = 0.15;
double safePathSwitchMinIntervalSec = 0.50;
bool activeCandidateValid = false;
int activeCandidateGroupID = -1;
int activeCandidateRotationID = -1;
int activeCandidateDirection = 0;
bool activeCandidateNarrowMode = false;
double activeCandidateScore = 0.0;
std::chrono::steady_clock::time_point lastSafeCandidateSwitch;
std::uint64_t pathSwitchCount = 0;
std::uint64_t pathRefreshCount = 0;
std::string pathSwitchReason = "INITIAL";

float joySpeed = 0;
float joySpeedRaw = 0;
float joyDir = 0;

const int pathNum = 343;
const int groupNum = 7;
float gridVoxelSize = 0.02;
float searchRadius = 0.55;
float gridVoxelOffsetX = 3.2;
float gridVoxelOffsetY = 4.5;
const int gridVoxelNumX = 161;
const int gridVoxelNumY = 451;
const int gridVoxelNum = gridVoxelNumX * gridVoxelNumY;

pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCrop(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudDwz(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudCrop(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudDwz(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudStack[laserCloudStackNum];
pcl::PointCloud<pcl::PointXYZI>::Ptr plannerCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr plannerCloudCrop(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr boundaryCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr addedObstacles(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZ>::Ptr startPaths[groupNum];
// Candidate geometry is needed for collision checking independently of RViz.
#if PLOTPATHSET == 1
pcl::PointCloud<pcl::PointXYZI>::Ptr paths[pathNum];
pcl::PointCloud<pcl::PointXYZI>::Ptr freePaths(new pcl::PointCloud<pcl::PointXYZI>());
#endif

int pathList[pathNum] = {0};
float endDirPathList[pathNum] = {0};
int clearPathList[36 * pathNum] = {0};
float pathPenaltyList[36 * pathNum] = {0};
float clearPathPerGroupScore[36 * groupNum] = {0};
std::vector<int> correspondences[gridVoxelNum];

bool newLaserCloud = false;
bool newTerrainCloud = false;
// FAR owns this state: a /way_point alone must never arm motion because it is
// an internal, repeatedly published intermediate result.
bool navigationActive = false;
bool hasOdometry = false;
std::uint64_t localPathRevision = 0;
bool hasGoal = false;
bool waypointPendingImmediatePlan = false;
bool hasCachedPlannerInput = false;
std::uint64_t waypointRevision = 0;
std::chrono::steady_clock::time_point lastPlannerInputReceive;
std::chrono::steady_clock::time_point waypointReceive;
size_t lastLaserInputPoints = 0;
size_t lastTerrainInputPoints = 0;

struct LocalPlannerStatusData {
  size_t plannerCloudPoints = 0;
  size_t plannerCloudCropPoints = 0;
  float relativeGoalX = 0;
  float relativeGoalY = 0;
  float relativeGoalDistance = 0;
  float activePathScale = 0;
  float activePathRange = 0;
  int candidateTotal = 0;
  int candidateBlocked = 0;
  int candidateScored = 0;
  int candidateEligible = 0;
  int narrowChecked = 0;
  int narrowRecovered = 0;
  int narrowRejected = 0;
  // Index order is geometry unavailable, curvature, alignment zone, footprint
  // collision. Values describe candidates in the final path-scale attempt.
  std::array<int, 4> narrowReasonCounts{};
  std::array<std::array<int, 4>, groupNum> narrowReasonCountsByGroup{};
  std::array<int, groupNum> narrowChecksByGroup{};
  std::array<int, groupNum> narrowRecoveredByGroup{};
  std::array<int, groupNum> narrowOtherRejectedByGroup{};
  bool bestStraightAvailable = false;
  int bestStraightPathID = -1;
  int bestStraightGroupID = -1;
  int bestStraightRotationID = -1;
  double bestStraightDirectionErrorDeg = 0.0;
  std::string bestStraightReason = "NOT_CHECKED";
  bool bestStraightClearanceAvailable = false;
  double bestStraightMinClearance = -1.0;
  bool bestStraightHasBlockingPoint = false;
  double bestStraightBlockingX = 0.0;
  double bestStraightBlockingY = 0.0;
  bool selectedRequiresAlignment = false;
  double selectedMinClearance = 0.0;
  double selectedTurnRadius = 0.0;
  std::string selectedRejectionReason = "NONE";
  double planningCycleMs = 0.0;
  double narrowCheckMs = 0.0;
  int broadPhasePassCount = 0;
  int recoveredAfterCurvatureFilter = 0;
  int recoveredAfterDirectionFilter = 0;
  int recoveredAfterOtherFilters = 0;
  int recoveredEnteredSelection = 0;
  int recoveredSelected = 0;
  int finalSelectableCount = 0;
  int bestCandidateGroupID = -1;
  int bestCandidateRotationID = -1;
  int bestRecoveredGroupID = -1;
  int bestRecoveredRotationID = -1;
  double bestRecoveredScore = 0.0;
  std::string selectionFailureReason = "NOT_EVALUATED";
  int selectedGroupID = -1;
  int selectedPathLength = 0;
  bool pathFound = false;
  size_t publishedPathSize = 0;
  double waypointToPathDelaySec = -1.0;
  int selectedRotationID = -1;
  int selectedDirection = 0;
  bool selectedNarrowMode = false;
  double currentScore = 0.0;
  double bestScore = 0.0;
  std::string switchReason = "INITIAL";
  bool hasPlanningData = false;
};
LocalPlannerStatusData localStatus;
rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr localStatusPub;
rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr collisionEnvelopePub;
rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr narrowMarkerPub;
// Fixed after the immutable baseline path library is read.  This topic is a
// diagnostic view only; it never feeds path selection.
int diagnosticStraightPathID = -1;
std::string lastLocalStatusSignature;

double odomTime = 0;
double joyTime = 0;

float vehicleRoll = 0, vehiclePitch = 0, vehicleYaw = 0;
float vehicleX = 0, vehicleY = 0, vehicleZ = 0;

pcl::VoxelGrid<pcl::PointXYZI> laserDwzFilter, terrainDwzFilter;
rclcpp::Node::SharedPtr nh;

void publishLocalStatus(const std::string& state, const std::string& code,
                        const std::string& text, bool force = false)
{
  if (!localStatusPub) return;
  const std::string signature = state + "|" + code + "|" + text + "|" +
      std::to_string(localStatus.narrowChecked) + "|" +
      std::to_string(localStatus.narrowRecovered) + "|" +
      std::to_string(localStatus.narrowRejected) + "|" +
      std::to_string(localStatus.candidateScored) + "|" +
      std::to_string(localStatus.pathFound) + "|" +
      localStatus.selectedRejectionReason + "|" +
      localStatus.bestStraightReason + "|" +
      std::to_string(localStatus.bestStraightBlockingX) + "|" +
      std::to_string(localStatus.bestStraightBlockingY) + "|" +
      std::to_string(localStatus.selectedGroupID) + ":" +
      std::to_string(localStatus.selectedRotationID) + ":" +
      localStatus.switchReason + "|" +
      std::to_string(pathSwitchCount) + "|" + std::to_string(pathRefreshCount) + "|" +
      std::to_string(localStatus.finalSelectableCount) + "|" +
      std::to_string(localStatus.bestCandidateGroupID) + ":" +
      std::to_string(localStatus.bestCandidateRotationID) + "|" +
      localStatus.selectionFailureReason;
  if (!force && signature == lastLocalStatusSignature) return;
  lastLocalStatusSignature = signature;

  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = nh->now();
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "local_planner";
  status.hardware_id = "stage4d";
  status.level = (code == "PATH_FOUND" || code == "PATH_PUBLISHED")
      ? diagnostic_msgs::msg::DiagnosticStatus::OK
      : diagnostic_msgs::msg::DiagnosticStatus::WARN;
  status.message = text;
  auto add = [&status](const std::string& key, const std::string& value) {
    diagnostic_msgs::msg::KeyValue kv; kv.key = key; kv.value = value; status.values.push_back(kv);
  };
  auto number = [](double value, bool available) { return available ? std::to_string(value) : std::string("N/A"); };
  const bool hasInput = useTerrainAnalysis ? lastTerrainInputPoints > 0 : lastLaserInputPoints > 0;
  add("state", state); add("reason_code", code); add("reason_text", text);
  add("navigation_active", navigationActive ? "true" : "false");
  add("useTerrainAnalysis", useTerrainAnalysis ? "true" : "false");
  add("checkObstacle", checkObstacle ? "true" : "false");
  add("checkRotObstacle", checkRotObstacle ? "true" : "false");
  // Visualization/diagnostic parameters only.  They document the candidate
  // correspondence model; they do not alter selection or collision rejection.
  add("vehicle_length", std::to_string(vehicleLength));
  add("vehicle_width", std::to_string(vehicleWidth));
  add("adjacent_range", std::to_string(adjacentRange));
  add("obstacle_height_threshold", std::to_string(obstacleHeightThre));
  add("ground_height_threshold", std::to_string(groundHeightThre));
  add("point_per_path_threshold", std::to_string(pointPerPathThre));
  add("dir_weight", std::to_string(dirWeight));
  add("dir_threshold", std::to_string(dirThre));
  add("base_path_scale", std::to_string(pathScale));
  add("min_path_scale", std::to_string(minPathScale));
  add("path_scale_step", std::to_string(pathScaleStep));
  add("min_path_range", std::to_string(minPathRange));
  add("path_range_step", std::to_string(pathRangeStep));
  add("correspondence_search_radius", std::to_string(searchRadius));
  add("correspondence_voxel_size", std::to_string(gridVoxelSize));
  add("vehicle_x", number(vehicleX, hasOdometry)); add("vehicle_y", number(vehicleY, hasOdometry));
  add("vehicle_yaw", number(vehicleYaw, hasOdometry));
  add("goal_x", number(goalX, hasGoal)); add("goal_y", number(goalY, hasGoal));
  add("relative_goal_x", number(localStatus.relativeGoalX, localStatus.hasPlanningData));
  add("relative_goal_y", number(localStatus.relativeGoalY, localStatus.hasPlanningData));
  add("relative_goal_distance", number(localStatus.relativeGoalDistance, localStatus.hasPlanningData));
  add("joy_dir", number(joyDir, localStatus.hasPlanningData));
  add("input_cloud_points", hasInput ? std::to_string(useTerrainAnalysis ? lastTerrainInputPoints : lastLaserInputPoints) : "N/A");
  add("planner_cloud_points", localStatus.hasPlanningData ? std::to_string(localStatus.plannerCloudPoints) : "N/A");
  add("planner_cloud_crop_points", localStatus.hasPlanningData ? std::to_string(localStatus.plannerCloudCropPoints) : "N/A");
  add("path_scale", number(localStatus.activePathScale, localStatus.hasPlanningData));
  add("path_range", number(localStatus.activePathRange, localStatus.hasPlanningData));
  add("candidate_paths_total", localStatus.hasPlanningData ? std::to_string(localStatus.candidateTotal) : "N/A");
  add("candidate_paths_blocked", localStatus.hasPlanningData ? std::to_string(localStatus.candidateBlocked) : "N/A");
  add("candidate_paths_scored", localStatus.hasPlanningData ? std::to_string(localStatus.candidateScored) : "N/A");
  add("candidate_paths_eligible", localStatus.hasPlanningData ? std::to_string(localStatus.candidateEligible) : "N/A");
  add("broad_phase_blocked_candidates", std::to_string(localStatus.candidateBlocked));
  add("narrow_phase_checked_candidates", std::to_string(localStatus.narrowChecked));
  add("narrow_phase_recovered_candidates", std::to_string(localStatus.narrowRecovered));
  add("narrow_phase_rejected_candidates", std::to_string(localStatus.narrowRejected));
  add("broad_phase_pass_count", std::to_string(localStatus.broadPhasePassCount));
  add("recovered_after_curvature_filter", std::to_string(localStatus.recoveredAfterCurvatureFilter));
  add("recovered_after_direction_filter", std::to_string(localStatus.recoveredAfterDirectionFilter));
  add("recovered_after_other_filters", std::to_string(localStatus.recoveredAfterOtherFilters));
  add("recovered_entered_selection", std::to_string(localStatus.recoveredEnteredSelection));
  add("recovered_selected", std::to_string(localStatus.recoveredSelected));
  add("final_selectable_count", std::to_string(localStatus.finalSelectableCount));
  add("best_candidate_id", std::to_string(localStatus.bestCandidateGroupID) + ":" + std::to_string(localStatus.bestCandidateRotationID));
  add("best_recovered_candidate_id", std::to_string(localStatus.bestRecoveredGroupID) + ":" + std::to_string(localStatus.bestRecoveredRotationID));
  add("best_recovered_score", std::to_string(localStatus.bestRecoveredScore));
  add("selection_failure_reason", localStatus.selectionFailureReason);
  static const std::array<const char*, 4> narrowReasonNames = {
      "geometry_unavailable", "curvature_too_high", "alignment_zone_blocked",
      "footprint_collision"};
  for (size_t reason = 0; reason < narrowReasonNames.size(); ++reason) {
    add(std::string("narrow_") + narrowReasonNames[reason] + "_candidates",
        std::to_string(localStatus.narrowReasonCounts[reason]));
  }
  for (int group = 0; group < groupNum; ++group) {
    const std::string prefix = "narrow_group_" + std::to_string(group) + "_";
    add(prefix + "checked_candidates", std::to_string(localStatus.narrowChecksByGroup[group]));
    add(prefix + "recovered_candidates", std::to_string(localStatus.narrowRecoveredByGroup[group]));
    for (size_t reason = 0; reason < narrowReasonNames.size(); ++reason) {
      add(prefix + narrowReasonNames[reason] + "_candidates",
          std::to_string(localStatus.narrowReasonCountsByGroup[group][reason]));
    }
    add(prefix + "other_rejected_candidates",
        std::to_string(localStatus.narrowOtherRejectedByGroup[group]));
  }
  add("narrow_straight_path_id", std::to_string(localStatus.bestStraightPathID));
  add("narrow_straight_group_id", std::to_string(localStatus.bestStraightGroupID));
  add("narrow_straight_rotation_id", std::to_string(localStatus.bestStraightRotationID));
  add("narrow_straight_direction_error_deg",
      localStatus.bestStraightAvailable ? std::to_string(localStatus.bestStraightDirectionErrorDeg) : "N/A");
  add("narrow_straight_reason", localStatus.bestStraightReason);
  add("candidate_id", localStatus.hasPlanningData ?
      std::to_string(localStatus.selectedGroupID) + ":" +
      std::to_string(localStatus.selectedRotationID) + ":" +
      std::to_string(localStatus.selectedDirection) + ":" +
      (localStatus.selectedNarrowMode ? "NARROW" : "NORMAL") : "N/A");
  add("current_score", number(localStatus.currentScore, localStatus.hasPlanningData));
  add("best_score", number(localStatus.bestScore, localStatus.hasPlanningData));
  add("switch_reason", localStatus.switchReason);
  add("path_switch_count", std::to_string(pathSwitchCount));
  add("path_refresh_count", std::to_string(pathRefreshCount));
  add("narrow_straight_min_clearance",
      localStatus.bestStraightClearanceAvailable ? std::to_string(localStatus.bestStraightMinClearance) : "N/A");
  add("narrow_straight_blocking_point_x_vehicle",
      localStatus.bestStraightHasBlockingPoint ? std::to_string(localStatus.bestStraightBlockingX) : "N/A");
  add("narrow_straight_blocking_point_y_vehicle",
      localStatus.bestStraightHasBlockingPoint ? std::to_string(localStatus.bestStraightBlockingY) : "N/A");
  add("selected_path_requires_alignment", localStatus.selectedRequiresAlignment ? "true" : "false");
  add("selected_path_revision", std::to_string(localPathRevision));
  add("selected_path_min_clearance", std::to_string(localStatus.selectedMinClearance));
  add("selected_path_turn_radius", std::to_string(localStatus.selectedTurnRadius));
  add("selected_path_rejection_reason", localStatus.selectedRejectionReason);
  add("planning_cycle_ms", std::to_string(localStatus.planningCycleMs));
  add("narrow_check_ms", std::to_string(localStatus.narrowCheckMs));
  add("enable_narrow_passage_mode", enableNarrowRecoveredSelection ? "true" : "false");
  add("enable_orientation_aware_check", enableOrientationAwareCheck ? "true" : "false");
  add("enable_narrow_recovered_selection", enableNarrowRecoveredSelection ? "true" : "false");
  add("narrow_mode", !enableOrientationAwareCheck ? "A_OFF_OFF" : (enableNarrowRecoveredSelection ? "C_ON_ON" : "B_ON_OFF"));
  add("selected_group_id", localStatus.hasPlanningData ? std::to_string(localStatus.selectedGroupID) : "N/A");
  add("selected_path_length", localStatus.hasPlanningData ? std::to_string(localStatus.selectedPathLength) : "N/A");
  add("path_found", localStatus.hasPlanningData ? (localStatus.pathFound ? "true" : "false") : "N/A");
  add("published_path_size", localStatus.hasPlanningData ? std::to_string(localStatus.publishedPathSize) : "N/A");
  add("waypoint_revision", std::to_string(waypointRevision));
  add("waypoint_to_path_delay_sec", localStatus.waypointToPathDelaySec >= 0.0 ? std::to_string(localStatus.waypointToPathDelaySec) : "N/A");
  array.status.push_back(status);
  localStatusPub->publish(array);
}

void odometryHandler(const nav_msgs::msg::Odometry::ConstSharedPtr odom)
{
  odomTime = rclcpp::Time(odom->header.stamp).seconds();
  double roll, pitch, yaw;
  geometry_msgs::msg::Quaternion geoQuat = odom->pose.pose.orientation;
  tf2::Matrix3x3(tf2::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);

  vehicleRoll = roll;
  vehiclePitch = pitch;
  vehicleYaw = yaw;
  vehicleX = odom->pose.pose.position.x - cos(yaw) * sensorOffsetX + sin(yaw) * sensorOffsetY;
  vehicleY = odom->pose.pose.position.y - sin(yaw) * sensorOffsetX - cos(yaw) * sensorOffsetY;
  vehicleZ = odom->pose.pose.position.z;
  hasOdometry = true;
}

void laserCloudHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr laserCloud2)
{
  if (!useTerrainAnalysis) {
    laserCloud->clear();
    pcl::fromROSMsg(*laserCloud2, *laserCloud);
    lastLaserInputPoints = laserCloud->points.size();

    pcl::PointXYZI point;
    laserCloudCrop->clear();
    int laserCloudSize = laserCloud->points.size();
    for (int i = 0; i < laserCloudSize; i++) {
      point = laserCloud->points[i];

      float pointX = point.x;
      float pointY = point.y;
      float pointZ = point.z;

      float dis = sqrt((pointX - vehicleX) * (pointX - vehicleX) + (pointY - vehicleY) * (pointY - vehicleY));
      if (dis < adjacentRange) {
        point.x = pointX;
        point.y = pointY;
        point.z = pointZ;
        laserCloudCrop->push_back(point);
      }
    }

    laserCloudDwz->clear();
    laserDwzFilter.setInputCloud(laserCloudCrop);
    laserDwzFilter.filter(*laserCloudDwz);

    newLaserCloud = true;
    hasCachedPlannerInput = true;
    lastPlannerInputReceive = std::chrono::steady_clock::now();
  }
}

void terrainCloudHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr terrainCloud2)
{
  if (useTerrainAnalysis) {
    terrainCloud->clear();
    pcl::fromROSMsg(*terrainCloud2, *terrainCloud);
    lastTerrainInputPoints = terrainCloud->points.size();

    pcl::PointXYZI point;
    terrainCloudCrop->clear();
    int terrainCloudSize = terrainCloud->points.size();
    for (int i = 0; i < terrainCloudSize; i++) {
      point = terrainCloud->points[i];

      float pointX = point.x;
      float pointY = point.y;
      float pointZ = point.z;

      float dis = sqrt((pointX - vehicleX) * (pointX - vehicleX) + (pointY - vehicleY) * (pointY - vehicleY));
      if (dis < adjacentRange && (point.intensity > obstacleHeightThre || useCost)) {
        point.x = pointX;
        point.y = pointY;
        point.z = pointZ;
        terrainCloudCrop->push_back(point);
      }
    }

    terrainCloudDwz->clear();
    terrainDwzFilter.setInputCloud(terrainCloudCrop);
    terrainDwzFilter.filter(*terrainCloudDwz);

    newTerrainCloud = true;
    hasCachedPlannerInput = true;
    lastPlannerInputReceive = std::chrono::steady_clock::now();
  }
}

void joystickHandler(const sensor_msgs::msg::Joy::ConstSharedPtr joy)
{
  joyTime = nh->now().seconds();
  joySpeedRaw = sqrt(joy->axes[3] * joy->axes[3] + joy->axes[4] * joy->axes[4]);
  joySpeed = joySpeedRaw;
  if (joySpeed > 1.0) joySpeed = 1.0;
  if (joy->axes[4] == 0) joySpeed = 0;

  if (joySpeed > 0) {
    joyDir = atan2(joy->axes[3], joy->axes[4]) * 180 / PI;
    if (joy->axes[4] < 0) joyDir *= -1;
  }

  if (joy->axes[4] < 0 && !twoWayDrive) joySpeed = 0;

  if (joy->axes[2] > -0.1) {
    autonomyMode = false;
  } else {
    autonomyMode = true;
  }

  if (joy->axes[5] > -0.1) {
    checkObstacle = true;
  } else {
    checkObstacle = false;
  }
}

void goalHandler(const geometry_msgs::msg::PointStamped::ConstSharedPtr goal)
{
  if (!std::isfinite(goal->point.x) || !std::isfinite(goal->point.y)) {
    RCLCPP_WARN(nh->get_logger(), "Ignoring non-finite navigation waypoint");
    return;
  }
  goalX = goal->point.x;
  goalY = goal->point.y;
  hasGoal = true;
  waypointPendingImmediatePlan = true;
  ++waypointRevision;
  waypointReceive = std::chrono::steady_clock::now();
}

void navigationActiveHandler(const std_msgs::msg::Bool::ConstSharedPtr active)
{
  const bool wasNavigationActive = navigationActive;
  navigationActive = active->data;
  if (wasNavigationActive && !navigationActive) {
    activeCandidateValid = false;
    activeCandidateGroupID = -1;
    activeCandidateRotationID = -1;
    activeCandidateDirection = 0;
    activeCandidateNarrowMode = false;
    activeCandidateScore = 0.0;
    pathSwitchReason = "NAVIGATION_RESET";
  }
}

void speedHandler(const std_msgs::msg::Float32::ConstSharedPtr speed)
{
  double speedTime = nh->now().seconds();
  if (autonomyMode && speedTime - joyTime > joyToSpeedDelay && joySpeedRaw == 0) {
    joySpeed = speed->data / maxSpeed;

    if (joySpeed < 0) joySpeed = 0;
    else if (joySpeed > 1.0) joySpeed = 1.0;
  }
}

void boundaryHandler(const geometry_msgs::msg::PolygonStamped::ConstSharedPtr boundary)
{
  boundaryCloud->clear();
  pcl::PointXYZI point, point1, point2;
  int boundarySize = boundary->polygon.points.size();

  if (boundarySize >= 1) {
    point2.x = boundary->polygon.points[0].x;
    point2.y = boundary->polygon.points[0].y;
    point2.z = boundary->polygon.points[0].z;
  }

  for (int i = 0; i < boundarySize; i++) {
    point1 = point2;

    point2.x = boundary->polygon.points[i].x;
    point2.y = boundary->polygon.points[i].y;
    point2.z = boundary->polygon.points[i].z;

    if (point1.z == point2.z) {
      float disX = point1.x - point2.x;
      float disY = point1.y - point2.y;
      float dis = sqrt(disX * disX + disY * disY);

      int pointNum = int(dis / terrainVoxelSize) + 1;
      for (int pointID = 0; pointID < pointNum; pointID++) {
        point.x = float(pointID) / float(pointNum) * point1.x + (1.0 - float(pointID) / float(pointNum)) * point2.x;
        point.y = float(pointID) / float(pointNum) * point1.y + (1.0 - float(pointID) / float(pointNum)) * point2.y;
        point.z = 0;
        point.intensity = 100.0;

        for (int j = 0; j < pointPerPathThre; j++) {
          boundaryCloud->push_back(point);
        }
      }
    }
  }
}

void addedObstaclesHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr addedObstacles2)
{
  addedObstacles->clear();
  pcl::fromROSMsg(*addedObstacles2, *addedObstacles);

  int addedObstaclesSize = addedObstacles->points.size();
  for (int i = 0; i < addedObstaclesSize; i++) {
    addedObstacles->points[i].intensity = 200.0;
  }
}

void checkObstacleHandler(const std_msgs::msg::Bool::ConstSharedPtr checkObs)
{
  double checkObsTime = nh->now().seconds();
  if (autonomyMode && checkObsTime - joyTime > joyToCheckObstacleDelay) {
    checkObstacle = checkObs->data;
  }
}

int readPlyHeader(FILE *filePtr)
{
  char str[50];
  int val, pointNum;
  string strCur, strLast;
  while (strCur != "end_header") {
    val = fscanf(filePtr, "%s", str);
    if (val != 1) {
      RCLCPP_INFO(nh->get_logger(), "Error reading input files, exit.");
      exit(1);
    }

    strLast = strCur;
    strCur = string(str);

    if (strCur == "vertex" && strLast == "element") {
      val = fscanf(filePtr, "%d", &pointNum);
      if (val != 1) {
        RCLCPP_INFO(nh->get_logger(), "Error reading input files, exit.");
        exit(1);
      }
    }
  }

  return pointNum;
}

void readStartPaths()
{
  string fileName = pathFolder + "/startPaths.ply";

  FILE *filePtr = fopen(fileName.c_str(), "r");
  if (filePtr == NULL) {
    RCLCPP_INFO(nh->get_logger(), "Cannot read input files, exit.");
    exit(1);
  }

  int pointNum = readPlyHeader(filePtr);

  pcl::PointXYZ point;
  int val1, val2, val3, val4, groupID;
  for (int i = 0; i < pointNum; i++) {
    val1 = fscanf(filePtr, "%f", &point.x);
    val2 = fscanf(filePtr, "%f", &point.y);
    val3 = fscanf(filePtr, "%f", &point.z);
    val4 = fscanf(filePtr, "%d", &groupID);

    if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1) {
      RCLCPP_INFO(nh->get_logger(), "Error reading input files, exit.");
        exit(1);
    }

    if (groupID >= 0 && groupID < groupNum) {
      startPaths[groupID]->push_back(point);
    }
  }

  fclose(filePtr);
}

#if PLOTPATHSET == 1
void readPaths()
{
  string fileName = pathFolder + "/paths.ply";

  FILE *filePtr = fopen(fileName.c_str(), "r");
  if (filePtr == NULL) {
    RCLCPP_INFO(nh->get_logger(), "Cannot read input files, exit.");
    exit(1);
  }

  int pointNum = readPlyHeader(filePtr);

  pcl::PointXYZI point;
  int pointSkipNum = 30;
  int pointSkipCount = 0;
  int val1, val2, val3, val4, val5, pathID;
  for (int i = 0; i < pointNum; i++) {
    val1 = fscanf(filePtr, "%f", &point.x);
    val2 = fscanf(filePtr, "%f", &point.y);
    val3 = fscanf(filePtr, "%f", &point.z);
    val4 = fscanf(filePtr, "%d", &pathID);
    val5 = fscanf(filePtr, "%f", &point.intensity);

    if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1 || val5 != 1) {
      RCLCPP_INFO(nh->get_logger(), "Error reading input files, exit.");
        exit(1);
    }

    if (pathID >= 0 && pathID < pathNum) {
      pointSkipCount++;
      if (pointSkipCount > pointSkipNum) {
        paths[pathID]->push_back(point);
        pointSkipCount = 0;
      }
    }
  }

  fclose(filePtr);
}
#endif

void readPathList()
{
  string fileName = pathFolder + "/pathList.ply";

  FILE *filePtr = fopen(fileName.c_str(), "r");
  if (filePtr == NULL) {
    RCLCPP_INFO(nh->get_logger(), "Cannot read input files, exit.");
    exit(1);
  }

  if (pathNum != readPlyHeader(filePtr)) {
    RCLCPP_INFO(nh->get_logger(), "Incorrect path number, exit.");
    exit(1);
  }

  int val1, val2, val3, val4, val5, pathID, groupID;
  float endX, endY, endZ;
  for (int i = 0; i < pathNum; i++) {
    val1 = fscanf(filePtr, "%f", &endX);
    val2 = fscanf(filePtr, "%f", &endY);
    val3 = fscanf(filePtr, "%f", &endZ);
    val4 = fscanf(filePtr, "%d", &pathID);
    val5 = fscanf(filePtr, "%d", &groupID);

    if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1 || val5 != 1) {
      RCLCPP_INFO(nh->get_logger(), "Error reading input files, exit.");
        exit(1);
    }

    if (pathID >= 0 && pathID < pathNum && groupID >= 0 && groupID < groupNum) {
      pathList[pathID] = groupID;
      endDirPathList[pathID] = 2.0 * atan2(endY, endX) * 180 / PI;
    }
  }

  fclose(filePtr);
}


void publishCollisionEnvelope(float activePathScale)
{
  // This marker grid is diagnostic only; do not recompute it at planner rate
  // when RViz is absent or when a recent visualization is still current.
  if (!collisionEnvelopePub || collisionEnvelopePub->get_subscription_count() == 0) return;
  static auto lastPublish = std::chrono::steady_clock::time_point::min();
  const auto nowSteady = std::chrono::steady_clock::now();
  if (lastPublish != std::chrono::steady_clock::time_point::min() &&
      nowSteady - lastPublish < std::chrono::milliseconds(200)) return;
  lastPublish = nowSteady;
  if (!collisionEnvelopePub || diagnosticStraightPathID < 0 ||
      diagnosticStraightPathID >= pathNum || activePathScale <= 0.0f) {
    return;
  }

  visualization_msgs::msg::MarkerArray markers;
  const auto stamp = nh->now();
  auto baseMarker = [&](int id, int type, const std::string& ns) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "vehicle";
    marker.header.stamp = stamp;
    marker.ns = ns;
    marker.id = id;
    marker.type = type;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    return marker;
  };

  // The exact stored candidate centreline, scaled by the same runtime scale
  // used for collision lookup.  This deliberately does not select or score it.
  auto centreline = baseMarker(0, visualization_msgs::msg::Marker::LINE_STRIP,
                               "localplanner_collision_envelope_centerline");
  centreline.scale.x = 0.018;
  centreline.color.r = 0.05f;
  centreline.color.g = 0.95f;
  centreline.color.b = 0.25f;
  centreline.color.a = 0.95f;
  for (const auto& point : paths[diagnosticStraightPathID]->points) {
    if (point.x < 0.0f || point.x > adjacentRange / activePathScale) continue;
    geometry_msgs::msg::Point p;
    p.x = activePathScale * point.x;
    p.y = activePathScale * point.y;
    p.z = 0.04;
    centreline.points.push_back(p);
  }
  markers.markers.push_back(centreline);

  // Each point here is the centre of a generated correspondence voxel that
  // contains this candidate path ID.  These are the cells that increment the
  // path's blocking counter (once obstacle-height and point-count gates pass).
  auto cells = baseMarker(1, visualization_msgs::msg::Marker::CUBE_LIST,
                          "localplanner_collision_envelope_correspondence_cells");
  cells.scale.x = gridVoxelSize * activePathScale;
  cells.scale.y = gridVoxelSize * activePathScale;
  cells.scale.z = 0.012;
  cells.color.r = 1.0f;
  cells.color.g = 0.64f;
  cells.color.b = 0.05f;
  cells.color.a = 0.22f;
  for (int indX = 0; indX < gridVoxelNumX; ++indX) {
    const float x = gridVoxelOffsetX - gridVoxelSize * indX;
    if (x < 0.0f || x > adjacentRange / activePathScale) continue;
    const float scaleY = x / gridVoxelOffsetX + searchRadius / gridVoxelOffsetY *
        (gridVoxelOffsetX - x) / gridVoxelOffsetX;
    for (int indY = 0; indY < gridVoxelNumY; ++indY) {
      const int ind = gridVoxelNumY * indX + indY;
      const auto& candidates = correspondences[ind];
      if (std::find(candidates.begin(), candidates.end(), diagnosticStraightPathID) == candidates.end()) continue;
      const float y = scaleY * (gridVoxelOffsetY - gridVoxelSize * indY);
      geometry_msgs::msg::Point p;
      p.x = activePathScale * x;
      p.y = activePathScale * y;
      p.z = 0.012;
      cells.points.push_back(p);
    }
  }
  markers.markers.push_back(cells);

  // A simple, legible straight-path bound: source library membership is shown
  // above; these two lines show the canonical radial search bound scaled into
  // vehicle metres.  They are annotations, not a rectangle collision model.
  auto bounds = baseMarker(2, visualization_msgs::msg::Marker::LINE_LIST,
                           "localplanner_collision_envelope_radial_bound");
  bounds.scale.x = 0.012;
  bounds.color.r = 0.95f;
  bounds.color.g = 0.20f;
  bounds.color.b = 0.95f;
  bounds.color.a = 0.90f;
  const double yBound = searchRadius * activePathScale;
  const double xBound = adjacentRange;
  for (double y : {-yBound, yBound}) {
    geometry_msgs::msg::Point a, b;
    a.x = 0.0; a.y = y; a.z = 0.05;
    b.x = xBound; b.y = y; b.z = 0.05;
    bounds.points.push_back(a);
    bounds.points.push_back(b);
  }
  markers.markers.push_back(bounds);
  collisionEnvelopePub->publish(markers);
}

void readCorrespondences()
{
  string fileName = pathFolder + "/correspondences.txt";

  FILE *filePtr = fopen(fileName.c_str(), "r");
  if (filePtr == NULL) {
    RCLCPP_INFO(nh->get_logger(), "Cannot read input files, exit.");
    exit(1);
  }

  int val1, gridVoxelID, pathID;
  for (int i = 0; i < gridVoxelNum; i++) {
    val1 = fscanf(filePtr, "%d", &gridVoxelID);
    if (val1 != 1) {
      RCLCPP_INFO(nh->get_logger(), "Error reading input files, exit.");
        exit(1);
    }

    while (1) {
      val1 = fscanf(filePtr, "%d", &pathID);
      if (val1 != 1) {
        RCLCPP_INFO(nh->get_logger(), "Error reading input files, exit.");
          exit(1);
      }

      if (pathID != -1) {
        if (gridVoxelID >= 0 && gridVoxelID < gridVoxelNum && pathID >= 0 && pathID < pathNum) {
          correspondences[gridVoxelID].push_back(pathID);
        }
      } else {
        break;
      }
    }
  }

  fclose(filePtr);
}
// Exactly the same group centreline is used here and in the published /path.
std::vector<local_planner::Point2D> groupPathSamples(
    int group_id, double rotation, double active_scale,
    double active_range, double relative_goal_distance) {
  std::vector<local_planner::Point2D> samples;
  if (group_id < 0 || group_id >= groupNum || active_scale <= 0.0) return samples;
  const double c = std::cos(rotation), s = std::sin(rotation);
  for (const auto& point : startPaths[group_id]->points) {
    const double distance = std::hypot(point.x, point.y);
    if (distance > active_range / active_scale ||
        distance > relative_goal_distance / active_scale) break;
    samples.push_back({active_scale * (c * point.x - s * point.y),
                       active_scale * (s * point.x + c * point.y)});
  }
  return samples;
}

struct NarrowCheckResult {
  bool free = false;
  double min_clearance = 0.0;
  double turn_radius = 0.0;
  std::string reason = "NARROW_DISABLED";
  bool has_blocking_point = false;
  local_planner::Point2D blocking_point;
};

int narrowReasonIndex(const std::string& reason) {
  if (reason == "NARROW_GEOMETRY_UNAVAILABLE") return 0;
  if (reason == "NARROW_CURVATURE_TOO_HIGH") return 1;
  if (reason == "ALIGNMENT_ZONE_BLOCKED") return 2;
  if (reason == "NARROW_FOOTPRINT_COLLISION") return 3;
  return -1;
}

NarrowCheckResult checkNarrowGroup(int group_id, double rotation,
                                   double active_scale, double active_range,
                                   double relative_goal_distance,
                                   const pcl::PointCloud<pcl::PointXYZI>& obstacles) {
  NarrowCheckResult result;
  const auto samples = groupPathSamples(group_id, rotation, active_scale,
                                         active_range, relative_goal_distance);
  if (samples.size() < 2) {
    result.reason = "NARROW_GEOMETRY_UNAVAILABLE";
    return result;
  }
  std::vector<double> headings;
  if (!local_planner::computeTangents(samples, &headings)) {
    result.reason = "NARROW_TANGENT_INVALID";
    return result;
  }
  if (!local_planner::curvatureWithinLimit(headings,
          narrowMaxTangentDeltaDeg * PI / 180.0) ||
      !local_planner::curvatureRateWithinLimit(
          samples, headings, narrowMaxCurvatureRadPerM)) {
    result.reason = "NARROW_CURVATURE_TOO_HIGH";
    return result;
  }
  local_planner::NarrowFootprint physical;
  physical.length = narrowFootprintLength;
  physical.width = narrowFootprintWidth;
  physical.longitudinal_margin = narrowLongitudinalMargin;
  physical.lateral_margin = narrowLateralMargin;
  result.turn_radius = local_planner::turnRadius(physical);
  const bool needs_turn = std::abs(local_planner::wrapAngle(headings.front())) >
      narrowEnterYawToleranceDeg * PI / 180.0;
  std::vector<local_planner::Point2D> collision_points;
  collision_points.reserve(obstacles.points.size());
  double nearest_alignment_distance = std::numeric_limits<double>::infinity();
  bool alignment_zone_blocked = false;
  for (const auto& obstacle : obstacles.points) {
    if (!std::isfinite(obstacle.x) || !std::isfinite(obstacle.y) ||
        !std::isfinite(obstacle.intensity)) {
      result.reason = "NARROW_INVALID_OBSTACLE";
      return result;
    }
    if (useTerrainAnalysis && obstacle.intensity <= obstacleHeightThre) continue;
    collision_points.push_back({obstacle.x, obstacle.y});
    if (needs_turn && std::hypot(obstacle.x, obstacle.y) <= result.turn_radius) {
      alignment_zone_blocked = true;
      const double distance = std::hypot(obstacle.x, obstacle.y);
      if (distance < nearest_alignment_distance) {
        nearest_alignment_distance = distance;
        result.has_blocking_point = true;
        result.blocking_point = {obstacle.x, obstacle.y};
      }
    }
  }
  if (alignment_zone_blocked) {
    result.reason = "ALIGNMENT_ZONE_BLOCKED";
    return result;
  }
  const auto swept = local_planner::checkSweptPath(
      samples, headings, collision_points, physical,
      narrowRecoveryYawLimitDeg * PI / 180.0);
  if (!swept.free) {
    result.reason = "NARROW_FOOTPRINT_COLLISION";
    result.min_clearance = swept.min_lateral_clearance;
    result.has_blocking_point = swept.has_blocking_point;
    result.blocking_point = swept.blocking_point;
    return result;
  }
  result.free = true;
  result.reason = "NARROW_RECOVERED";
  result.min_clearance = swept.min_lateral_clearance;
  return result;
}

void publishNarrowMarkers(const nav_msgs::msg::Path& path, bool critical,
                          double turn_radius) {
  if (!enableNarrowRecoveredSelection || !narrowMarkerPub) return;
  visualization_msgs::msg::MarkerArray array;
  visualization_msgs::msg::Marker clear;
  clear.header = path.header;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  array.markers.push_back(clear);
  if (!critical || path.poses.size() < 2) {
    narrowMarkerPub->publish(array);
    return;
  }
  auto makeMarker = [&](int id, int type, const std::string& name,
                        float red, float green, float blue) {
    visualization_msgs::msg::Marker marker;
    marker.header = path.header;
    marker.ns = name;
    marker.id = id;
    marker.type = type;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.012;
    marker.color.r = red;
    marker.color.g = green;
    marker.color.b = blue;
    marker.color.a = 0.85;
    return marker;
  };
  auto point = [](double x, double y) {
    geometry_msgs::msg::Point value;
    value.x = x; value.y = y; value.z = 0.025;
    return value;
  };
  auto center = makeMarker(0, visualization_msgs::msg::Marker::LINE_STRIP,
                           "narrow_centerline", 0.0, 0.6, 1.0);
  std::vector<local_planner::Point2D> samples;
  for (const auto& pose : path.poses) {
    center.points.push_back(point(pose.pose.position.x, pose.pose.position.y));
    samples.push_back({pose.pose.position.x, pose.pose.position.y});
  }
  array.markers.push_back(center);
  std::vector<double> headings;
  if (local_planner::computeTangents(samples, &headings)) {
    local_planner::NarrowFootprint physical;
    physical.length = narrowFootprintLength;
    physical.width = narrowFootprintWidth;
    physical.longitudinal_margin = narrowLongitudinalMargin;
    physical.lateral_margin = narrowLateralMargin;
    const auto envelope = local_planner::yawInflatedFootprint(
        physical, narrowRecoveryYawLimitDeg * PI / 180.0);
    const double half_x = envelope.length * 0.5 + envelope.longitudinal_margin;
    const double half_y = envelope.width * 0.5 + envelope.lateral_margin;
    auto footprint = makeMarker(1, visualization_msgs::msg::Marker::LINE_LIST,
                                "narrow_swept_footprint", 0.1, 1.0, 0.7);
    const size_t stride = std::max<size_t>(1, samples.size() / 16);
    for (size_t index = 0; index < samples.size(); index += stride) {
      const double c = std::cos(headings[index]), s = std::sin(headings[index]);
      geometry_msgs::msg::Point corners[4];
      for (int corner = 0; corner < 4; ++corner) {
        const double x = corner < 2 ? half_x : -half_x;
        const double y = (corner == 0 || corner == 3) ? half_y : -half_y;
        corners[corner] = point(samples[index].x + c * x - s * y,
                                samples[index].y + s * x + c * y);
      }
      for (int edge = 0; edge < 4; ++edge) {
        footprint.points.push_back(corners[edge]);
        footprint.points.push_back(corners[(edge + 1) % 4]);
      }
    }
    array.markers.push_back(footprint);
  }
  auto turn = makeMarker(2, visualization_msgs::msg::Marker::LINE_STRIP,
                         "narrow_start_turn_envelope", 1.0, 0.5, 0.0);
  for (int index = 0; index <= 48; ++index) {
    const double angle = 2.0 * PI * index / 48.0;
    turn.points.push_back(point(turn_radius * std::cos(angle),
                                turn_radius * std::sin(angle)));
  }
  array.markers.push_back(turn);
  narrowMarkerPub->publish(array);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  nh = rclcpp::Node::make_shared("localPlanner");

  nh->declare_parameter<std::string>("pathFolder", pathFolder);
  nh->declare_parameter<double>("vehicleLength", vehicleLength);
  nh->declare_parameter<double>("vehicleWidth", vehicleWidth);
  nh->declare_parameter<double>("sensorOffsetX", sensorOffsetX);
  nh->declare_parameter<double>("sensorOffsetY", sensorOffsetY);
  nh->declare_parameter<bool>("twoWayDrive", twoWayDrive);
  nh->declare_parameter<double>("laserVoxelSize", laserVoxelSize);
  nh->declare_parameter<double>("terrainVoxelSize", terrainVoxelSize);
  nh->declare_parameter<bool>("useTerrainAnalysis", useTerrainAnalysis);
  nh->declare_parameter<bool>("checkObstacle", checkObstacle);
  nh->declare_parameter<bool>("checkRotObstacle", checkRotObstacle);
  nh->declare_parameter<double>("adjacentRange", adjacentRange);
  nh->declare_parameter<double>("obstacleHeightThre", obstacleHeightThre);
  nh->declare_parameter<double>("groundHeightThre", groundHeightThre);
  nh->declare_parameter<double>("costHeightThre", costHeightThre);
  nh->declare_parameter<double>("costScore", costScore);
  nh->declare_parameter<bool>("useCost", useCost);
  nh->declare_parameter<int>("pointPerPathThre", pointPerPathThre);
  nh->declare_parameter<double>("minRelZ", minRelZ);
  nh->declare_parameter<double>("maxRelZ", maxRelZ);
  nh->declare_parameter<double>("maxSpeed", maxSpeed);
  nh->declare_parameter<double>("dirWeight", dirWeight);
  nh->declare_parameter<double>("dirThre", dirThre);
  nh->declare_parameter<bool>("enableNarrowPassageMode", enableNarrowPassageMode);
  nh->declare_parameter<bool>("enableOrientationAwareCheck", enableOrientationAwareCheck);
  nh->declare_parameter<bool>("enableNarrowRecoveredSelection", enableNarrowRecoveredSelection);
  nh->declare_parameter<double>("narrowFootprintLength", narrowFootprintLength);
  nh->declare_parameter<double>("narrowFootprintWidth", narrowFootprintWidth);
  nh->declare_parameter<double>("narrowLongitudinalMargin", narrowLongitudinalMargin);
  nh->declare_parameter<double>("narrowLateralMargin", narrowLateralMargin);
  nh->declare_parameter<double>("narrowEnterYawToleranceDeg", narrowEnterYawToleranceDeg);
  nh->declare_parameter<double>("narrowContinueYawToleranceDeg", narrowContinueYawToleranceDeg);
  nh->declare_parameter<double>("narrowStopYawToleranceDeg", narrowStopYawToleranceDeg);
  nh->declare_parameter<double>("narrowSpeedScale", narrowSpeedScale);
  nh->declare_parameter<double>("narrowMaxTangentDeltaDeg", narrowMaxTangentDeltaDeg);
  nh->declare_parameter<double>("narrowMaxCurvatureRadPerM", narrowMaxCurvatureRadPerM);
  nh->declare_parameter<double>("narrowRecoveryYawLimitDeg", narrowRecoveryYawLimitDeg);
  nh->declare_parameter<bool>("dirToVehicle", dirToVehicle);
  nh->declare_parameter<double>("pathScale", pathScale);
  nh->declare_parameter<double>("minPathScale", minPathScale);
  nh->declare_parameter<double>("pathScaleStep", pathScaleStep);
  nh->declare_parameter<bool>("pathScaleBySpeed", pathScaleBySpeed);
  nh->declare_parameter<double>("minPathRange", minPathRange);
  nh->declare_parameter<double>("pathRangeStep", pathRangeStep);
  nh->declare_parameter<bool>("pathRangeBySpeed", pathRangeBySpeed);
  nh->declare_parameter<bool>("pathCropByGoal", pathCropByGoal);
  nh->declare_parameter<bool>("enableTemporalPathStabilization", enableTemporalPathStabilization);
  nh->declare_parameter<double>("pathSwitchScoreMargin", pathSwitchScoreMargin);
  nh->declare_parameter<double>("safePathSwitchMinIntervalSec", safePathSwitchMinIntervalSec);
  nh->declare_parameter<bool>("autonomyMode", autonomyMode);
  nh->declare_parameter<double>("autonomySpeed", autonomySpeed);
  nh->declare_parameter<double>("joyToSpeedDelay", joyToSpeedDelay);
  nh->declare_parameter<double>("joyToCheckObstacleDelay", joyToCheckObstacleDelay);
  nh->declare_parameter<double>("goalClearRange", goalClearRange);
  nh->declare_parameter<double>("searchRadius", searchRadius);
  nh->declare_parameter<double>("goalX", goalX);
  nh->declare_parameter<double>("goalY", goalY);

  nh->get_parameter("pathFolder", pathFolder);
  nh->get_parameter("vehicleLength", vehicleLength);
  nh->get_parameter("vehicleWidth", vehicleWidth);
  nh->get_parameter("sensorOffsetX", sensorOffsetX);
  nh->get_parameter("sensorOffsetY", sensorOffsetY);
  nh->get_parameter("twoWayDrive", twoWayDrive);
  nh->get_parameter("laserVoxelSize", laserVoxelSize);
  nh->get_parameter("terrainVoxelSize", terrainVoxelSize);
  nh->get_parameter("useTerrainAnalysis", useTerrainAnalysis);
  nh->get_parameter("checkObstacle", checkObstacle);
  nh->get_parameter("checkRotObstacle", checkRotObstacle);
  nh->get_parameter("adjacentRange", adjacentRange);
  nh->get_parameter("obstacleHeightThre", obstacleHeightThre);
  nh->get_parameter("groundHeightThre", groundHeightThre);
  nh->get_parameter("costHeightThre", costHeightThre);
  nh->get_parameter("costScore", costScore);
  nh->get_parameter("useCost", useCost);
  nh->get_parameter("pointPerPathThre", pointPerPathThre);
  nh->get_parameter("minRelZ", minRelZ);
  nh->get_parameter("maxRelZ", maxRelZ);
  nh->get_parameter("maxSpeed", maxSpeed);
  nh->get_parameter("dirWeight", dirWeight);
  nh->get_parameter("dirThre", dirThre);
  nh->get_parameter("enableNarrowPassageMode", enableNarrowPassageMode);
  nh->get_parameter("enableOrientationAwareCheck", enableOrientationAwareCheck);
  nh->get_parameter("enableNarrowRecoveredSelection", enableNarrowRecoveredSelection);
  nh->get_parameter("narrowFootprintLength", narrowFootprintLength);
  nh->get_parameter("narrowFootprintWidth", narrowFootprintWidth);
  nh->get_parameter("narrowLongitudinalMargin", narrowLongitudinalMargin);
  nh->get_parameter("narrowLateralMargin", narrowLateralMargin);
  nh->get_parameter("narrowEnterYawToleranceDeg", narrowEnterYawToleranceDeg);
  nh->get_parameter("narrowContinueYawToleranceDeg", narrowContinueYawToleranceDeg);
  nh->get_parameter("narrowStopYawToleranceDeg", narrowStopYawToleranceDeg);
  nh->get_parameter("narrowSpeedScale", narrowSpeedScale);
  nh->get_parameter("narrowMaxTangentDeltaDeg", narrowMaxTangentDeltaDeg);
  nh->get_parameter("narrowMaxCurvatureRadPerM", narrowMaxCurvatureRadPerM);
  nh->get_parameter("narrowRecoveryYawLimitDeg", narrowRecoveryYawLimitDeg);
  nh->get_parameter("dirToVehicle", dirToVehicle);
  nh->get_parameter("pathScale", pathScale);
  nh->get_parameter("minPathScale", minPathScale);
  nh->get_parameter("pathScaleStep", pathScaleStep);
  nh->get_parameter("pathScaleBySpeed", pathScaleBySpeed);
  nh->get_parameter("minPathRange", minPathRange);
  nh->get_parameter("pathRangeStep", pathRangeStep);
  nh->get_parameter("pathRangeBySpeed", pathRangeBySpeed);
  nh->get_parameter("pathCropByGoal", pathCropByGoal);
  nh->get_parameter("enableTemporalPathStabilization", enableTemporalPathStabilization);
  nh->get_parameter("pathSwitchScoreMargin", pathSwitchScoreMargin);
  nh->get_parameter("safePathSwitchMinIntervalSec", safePathSwitchMinIntervalSec);
  nh->get_parameter("autonomyMode", autonomyMode);
  nh->get_parameter("autonomySpeed", autonomySpeed);
  nh->get_parameter("joyToSpeedDelay", joyToSpeedDelay);
  nh->get_parameter("joyToCheckObstacleDelay", joyToCheckObstacleDelay);
  nh->get_parameter("goalCloseDis", goalCloseDis);
  nh->get_parameter("goalClearRange", goalClearRange);
  nh->get_parameter("searchRadius", searchRadius);
  nh->get_parameter("goalX", goalX);
  nh->get_parameter("goalY", goalY);
  if (!std::isfinite(pathSwitchScoreMargin) || pathSwitchScoreMargin < 0.0 ||
      !std::isfinite(safePathSwitchMinIntervalSec) || safePathSwitchMinIntervalSec < 0.0) {
    RCLCPP_FATAL(nh->get_logger(), "Invalid temporal path stabilization configuration");
    rclcpp::shutdown();
    return 1;
  }

  if (enableNarrowRecoveredSelection &&
      (!(narrowFootprintLength > 0.0 && narrowFootprintWidth > 0.0 &&
         narrowLongitudinalMargin >= 0.0 && narrowLateralMargin >= 0.0 &&
         narrowEnterYawToleranceDeg > 0.0 &&
         narrowEnterYawToleranceDeg < narrowContinueYawToleranceDeg &&
         narrowContinueYawToleranceDeg < narrowStopYawToleranceDeg &&
         narrowStopYawToleranceDeg < narrowRecoveryYawLimitDeg &&
         narrowRecoveryYawLimitDeg <= 45.0 && narrowMaxTangentDeltaDeg > 0.0 &&
         narrowMaxCurvatureRadPerM > 0.0 &&
         narrowSpeedScale > 0.0 && narrowSpeedScale <= 1.0) ||
       !std::isfinite(narrowFootprintLength + narrowFootprintWidth +
                      narrowLongitudinalMargin + narrowLateralMargin +
                      narrowRecoveryYawLimitDeg + narrowSpeedScale))) {
    RCLCPP_FATAL(nh->get_logger(), "Invalid narrow-passage configuration");
    rclcpp::shutdown();
    return 1;
  }

  auto subOdometry = nh->create_subscription<nav_msgs::msg::Odometry>("/state_estimation", 5, odometryHandler);

  auto subLaserCloud = nh->create_subscription<sensor_msgs::msg::PointCloud2>("/registered_scan", 5, laserCloudHandler);

  auto subTerrainCloud = nh->create_subscription<sensor_msgs::msg::PointCloud2>("/terrain_map", 5, terrainCloudHandler);

  auto subJoystick = nh->create_subscription<sensor_msgs::msg::Joy>("/joy", 5, joystickHandler);

  auto subGoal = nh->create_subscription<geometry_msgs::msg::PointStamped> ("/way_point", 5, goalHandler);

  auto subNavigationActive = nh->create_subscription<std_msgs::msg::Bool>(
      "/navigation_active", rclcpp::QoS(1).transient_local(), navigationActiveHandler);

  auto subSpeed = nh->create_subscription<std_msgs::msg::Float32>("/speed", 5, speedHandler);

  auto subBoundary = nh->create_subscription<geometry_msgs::msg::PolygonStamped>("/navigation_boundary", 5, boundaryHandler);

  auto subAddedObstacles = nh->create_subscription<sensor_msgs::msg::PointCloud2>("/added_obstacles", 5, addedObstaclesHandler);

  auto subCheckObstacle = nh->create_subscription<std_msgs::msg::Bool>("/check_obstacle", 5, checkObstacleHandler);

  auto pubPath = nh->create_publisher<nav_msgs::msg::Path>("/path", 5);
  auto pubPathConstraint = nh->create_publisher<visibility_graph_msg::msg::LocalPathConstraint>(
      "/local_path_constraint", rclcpp::QoS(5).transient_local());
  localStatusPub = nh->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/local_planner/status", rclcpp::QoS(1).transient_local());
  collisionEnvelopePub = nh->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/stage4d/localplanner_collision_envelope", rclcpp::QoS(1).transient_local());
  narrowMarkerPub = nh->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/stage4d/narrow_passage_markers", rclcpp::QoS(1).transient_local());
  publishLocalStatus("STARTUP", "STARTUP", "localPlanner initialized; waiting for FAR navigation");
  nav_msgs::msg::Path path;

  #if PLOTPATHSET == 1
  auto pubFreePaths = nh->create_publisher<sensor_msgs::msg::PointCloud2>("/free_paths", 2);
  #endif

  //auto pubLaserCloud = nh->create_publisher<sensor_msgs::msg::PointCloud2> ("/stacked_scans", 2);

  RCLCPP_INFO(nh->get_logger(), "Reading path files.");

  if (autonomyMode) {
    joySpeed = autonomySpeed / maxSpeed;

    if (joySpeed < 0) joySpeed = 0;
    else if (joySpeed > 1.0) joySpeed = 1.0;
  }

  for (int i = 0; i < laserCloudStackNum; i++) {
    laserCloudStack[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
  }
  for (int i = 0; i < groupNum; i++) {
    startPaths[i].reset(new pcl::PointCloud<pcl::PointXYZ>());
  }
  #if PLOTPATHSET == 1
  for (int i = 0; i < pathNum; i++) {
    paths[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
  }
  #endif
  for (int i = 0; i < gridVoxelNum; i++) {
    correspondences[i].resize(0);
  }

  laserDwzFilter.setLeafSize(laserVoxelSize, laserVoxelSize, laserVoxelSize);
  terrainDwzFilter.setLeafSize(terrainVoxelSize, terrainVoxelSize, terrainVoxelSize);

  readStartPaths();
  #if PLOTPATHSET == 1
  readPaths();
  #endif
  readPathList();
  readCorrespondences();
  // Identify one straight baseline path once.  It is never used in planning;
  // closest terminal lateral displacement makes the choice deterministic.
  float bestStraightScore = std::numeric_limits<float>::infinity();
  for (int pathID = 0; pathID < pathNum; ++pathID) {
    if (paths[pathID]->points.empty()) continue;
    const auto& end = paths[pathID]->points.back();
    const float score = std::fabs(end.y) + 0.001f * std::fabs(end.x - adjacentRange);
    if (score < bestStraightScore) {
      bestStraightScore = score;
      diagnosticStraightPathID = pathID;
    }
  }
  RCLCPP_INFO(nh->get_logger(), "Collision-envelope diagnostic path ID: %d", diagnosticStraightPathID);

  RCLCPP_INFO(nh->get_logger(), "Initialization complete.");

  rclcpp::Rate rate(100);
  bool status = rclcpp::ok();
  while (status) {
    rclcpp::spin_some(nh);

    // Initial (0,0) launch parameters are configuration defaults, not a goal.
    // Do not turn point clouds into /path until FAR has accepted /goal_point.
    if (!navigationActive) {
      publishLocalStatus("NAVIGATION_INACTIVE", "NAVIGATION_INACTIVE", "waiting for FAR /navigation_active=true");
      rate.sleep();
      status = rclcpp::ok();
      continue;
    }
    const auto plannerNow = std::chrono::steady_clock::now();
    const bool cachedInputFresh = hasCachedPlannerInput &&
        std::chrono::duration<double>(plannerNow - lastPlannerInputReceive).count() <= 0.5;
    const bool immediateWaypointPlan = waypointPendingImmediatePlan && hasOdometry && cachedInputFresh;
    if (!hasGoal) {
      publishLocalStatus("WAITING_FOR_GOAL", "WAITING_FOR_GOAL", "navigation is active but no FAR waypoint has been received");
    } else if (waypointPendingImmediatePlan && !immediateWaypointPlan) {
      publishLocalStatus("WAITING_FOR_FRESH_INPUT_AFTER_WAYPOINT", "WAITING_FOR_FRESH_INPUT_AFTER_WAYPOINT",
          "new FAR waypoint is waiting for fresh cached odometry/perception");
    } else if (!(newLaserCloud || newTerrainCloud)) {
      publishLocalStatus("WAITING_FOR_POINTCLOUD", "WAITING_FOR_POINTCLOUD", "waiting for the next planner input cloud");
    }

    if (newLaserCloud || newTerrainCloud || immediateWaypointPlan) {
      const auto planningStart = std::chrono::steady_clock::now();
      if (newLaserCloud) {
        newLaserCloud = false;

        laserCloudStack[laserCloudCount]->clear();
        *laserCloudStack[laserCloudCount] = *laserCloudDwz;
        laserCloudCount = (laserCloudCount + 1) % laserCloudStackNum;

        plannerCloud->clear();
        for (int i = 0; i < laserCloudStackNum; i++) {
          *plannerCloud += *laserCloudStack[i];
        }
      }

      if (newTerrainCloud) {
        newTerrainCloud = false;

        plannerCloud->clear();
        *plannerCloud = *terrainCloudDwz;
      }

      float sinVehicleRoll = sin(vehicleRoll);
      float cosVehicleRoll = cos(vehicleRoll);
      float sinVehiclePitch = sin(vehiclePitch);
      float cosVehiclePitch = cos(vehiclePitch);
      float sinVehicleYaw = sin(vehicleYaw);
      float cosVehicleYaw = cos(vehicleYaw);

      pcl::PointXYZI point;
      plannerCloudCrop->clear();
      int plannerCloudSize = plannerCloud->points.size();
      for (int i = 0; i < plannerCloudSize; i++) {
        float pointX1 = plannerCloud->points[i].x - vehicleX;
        float pointY1 = plannerCloud->points[i].y - vehicleY;
        float pointZ1 = plannerCloud->points[i].z - vehicleZ;

        point.x = pointX1 * cosVehicleYaw + pointY1 * sinVehicleYaw;
        point.y = -pointX1 * sinVehicleYaw + pointY1 * cosVehicleYaw;
        point.z = pointZ1;
        point.intensity = plannerCloud->points[i].intensity;

        float dis = sqrt(point.x * point.x + point.y * point.y);
        if (dis < adjacentRange && ((point.z > minRelZ && point.z < maxRelZ) || useTerrainAnalysis)) {
          plannerCloudCrop->push_back(point);
        }
      }

      int boundaryCloudSize = boundaryCloud->points.size();
      for (int i = 0; i < boundaryCloudSize; i++) {
        point.x = ((boundaryCloud->points[i].x - vehicleX) * cosVehicleYaw 
                + (boundaryCloud->points[i].y - vehicleY) * sinVehicleYaw);
        point.y = (-(boundaryCloud->points[i].x - vehicleX) * sinVehicleYaw 
                + (boundaryCloud->points[i].y - vehicleY) * cosVehicleYaw);
        point.z = boundaryCloud->points[i].z;
        point.intensity = boundaryCloud->points[i].intensity;

        float dis = sqrt(point.x * point.x + point.y * point.y);
        if (dis < adjacentRange) {
          plannerCloudCrop->push_back(point);
        }
      }

      int addedObstaclesSize = addedObstacles->points.size();
      for (int i = 0; i < addedObstaclesSize; i++) {
        point.x = ((addedObstacles->points[i].x - vehicleX) * cosVehicleYaw 
                + (addedObstacles->points[i].y - vehicleY) * sinVehicleYaw);
        point.y = (-(addedObstacles->points[i].x - vehicleX) * sinVehicleYaw 
                + (addedObstacles->points[i].y - vehicleY) * cosVehicleYaw);
        point.z = addedObstacles->points[i].z;
        point.intensity = addedObstacles->points[i].intensity;

        float dis = sqrt(point.x * point.x + point.y * point.y);
        if (dis < adjacentRange) {
          plannerCloudCrop->push_back(point);
        }
      }

      float pathRange = adjacentRange;
      if (pathRangeBySpeed) pathRange = adjacentRange * joySpeed;
      if (pathRange < minPathRange) pathRange = minPathRange;
      float relativeGoalDis = adjacentRange;

      float relativeGoalX = 0;
      float relativeGoalY = 0;
      if (autonomyMode) {
        relativeGoalX = ((goalX - vehicleX) * cosVehicleYaw + (goalY - vehicleY) * sinVehicleYaw);
        relativeGoalY = (-(goalX - vehicleX) * sinVehicleYaw
                         + (goalY - vehicleY) * cosVehicleYaw);

        relativeGoalDis = sqrt(relativeGoalX * relativeGoalX + relativeGoalY * relativeGoalY);
        joyDir = atan2(relativeGoalY, relativeGoalX) * 180 / PI;

        if (!twoWayDrive) {
          if (joyDir > 90.0) joyDir = 90.0;
          else if (joyDir < -90.0) joyDir = -90.0;
        }
      }

      localStatus.hasPlanningData = true;
      localStatus.plannerCloudPoints = plannerCloud->points.size();
      localStatus.plannerCloudCropPoints = plannerCloudCrop->points.size();
      localStatus.relativeGoalX = relativeGoalX;
      localStatus.relativeGoalY = relativeGoalY;
      localStatus.relativeGoalDistance = relativeGoalDis;
      localStatus.pathFound = false;
      localStatus.publishedPathSize = 0;
      localStatus.selectedGroupID = -1;
      localStatus.selectedPathLength = 0;
      // Status is published only after this cycle has a final selection result.
      // Publishing here would expose stale values from the previous cycle.

      bool pathFound = false;
      float defPathScale = pathScale;
      if (pathScaleBySpeed) pathScale = defPathScale * joySpeed;
      if (pathScale < minPathScale) pathScale = minPathScale;

      while (pathScale >= minPathScale && pathRange >= minPathRange) {
        localStatus.activePathScale = pathScale;
        localStatus.activePathRange = pathRange;
        publishCollisionEnvelope(pathScale);
        localStatus.candidateTotal = 36 * pathNum;
        localStatus.candidateBlocked = 0;
        localStatus.candidateScored = 0;
        localStatus.candidateEligible = 0;
        localStatus.narrowChecked = 0;
        localStatus.narrowRecovered = 0;
        localStatus.narrowRejected = 0;
        localStatus.narrowReasonCounts.fill(0);
        for (auto& counts : localStatus.narrowReasonCountsByGroup) counts.fill(0);
        localStatus.narrowChecksByGroup.fill(0);
        localStatus.narrowRecoveredByGroup.fill(0);
        localStatus.narrowOtherRejectedByGroup.fill(0);
        localStatus.bestStraightAvailable = false;
        localStatus.bestStraightPathID = -1;
        localStatus.bestStraightGroupID = -1;
        localStatus.bestStraightRotationID = -1;
        localStatus.bestStraightDirectionErrorDeg = 0.0;
        localStatus.bestStraightReason = "NOT_CHECKED";
        localStatus.bestStraightClearanceAvailable = false;
        localStatus.bestStraightMinClearance = -1.0;
        localStatus.bestStraightHasBlockingPoint = false;
        localStatus.bestStraightBlockingX = 0.0;
        localStatus.bestStraightBlockingY = 0.0;
        localStatus.selectedRequiresAlignment = false;
        localStatus.selectedMinClearance = 0.0;
        localStatus.selectedTurnRadius = 0.0;
        localStatus.selectedRejectionReason = "NONE";
        localStatus.narrowCheckMs = 0.0;
        localStatus.broadPhasePassCount = 0;
        localStatus.recoveredAfterCurvatureFilter = 0;
        localStatus.recoveredAfterDirectionFilter = 0;
        localStatus.recoveredAfterOtherFilters = 0;
        localStatus.recoveredEnteredSelection = 0;
        localStatus.finalSelectableCount = 0;
        localStatus.bestCandidateGroupID = -1;
        localStatus.bestCandidateRotationID = -1;
        localStatus.bestRecoveredGroupID = -1;
        localStatus.bestRecoveredRotationID = -1;
        localStatus.bestRecoveredScore = 0.0;
        localStatus.selectionFailureReason = "NOT_SELECTED";
        NarrowCheckResult narrowGroupResults[36 * groupNum];
        bool narrowGroupChecked[36 * groupNum] = {false};
        bool narrowGroupRecovered[36 * groupNum] = {false};
        bool recoveredScoredGroup[36 * groupNum] = {false};
        for (int i = 0; i < 36 * pathNum; i++) {
          clearPathList[i] = 0;
          pathPenaltyList[i] = 0;
        }
        for (int i = 0; i < 36 * groupNum; i++) {
          clearPathPerGroupScore[i] = 0;
        }

        float minObsAngCW = -180.0;
        float minObsAngCCW = 180.0;
        float diameter = sqrt(vehicleLength / 2.0 * vehicleLength / 2.0 + vehicleWidth / 2.0 * vehicleWidth / 2.0);
        float angOffset = atan2(vehicleWidth, vehicleLength) * 180.0 / PI;
        int plannerCloudCropSize = plannerCloudCrop->points.size();
        for (int i = 0; i < plannerCloudCropSize; i++) {
          float x = plannerCloudCrop->points[i].x / pathScale;
          float y = plannerCloudCrop->points[i].y / pathScale;
          float h = plannerCloudCrop->points[i].intensity;
          float dis = sqrt(x * x + y * y);

          if (dis < pathRange / pathScale && (dis <= (relativeGoalDis + goalClearRange) / pathScale || !pathCropByGoal) && checkObstacle) {
            for (int rotDir = 0; rotDir < 36; rotDir++) {
              float rotAng = (10.0 * rotDir - 180.0) * PI / 180;
              float angDiff = fabs(joyDir - (10.0 * rotDir - 180.0));
              if (angDiff > 180.0) {
                angDiff = 360.0 - angDiff;
              }
              if ((angDiff > dirThre && !dirToVehicle) || (fabs(10.0 * rotDir - 180.0) > dirThre && fabs(joyDir) <= 90.0 && dirToVehicle) ||
                  ((10.0 * rotDir > dirThre && 360.0 - 10.0 * rotDir > dirThre) && fabs(joyDir) > 90.0 && dirToVehicle)) {
                continue;
              }

              float x2 = cos(rotAng) * x + sin(rotAng) * y;
              float y2 = -sin(rotAng) * x + cos(rotAng) * y;

              float scaleY = x2 / gridVoxelOffsetX + searchRadius / gridVoxelOffsetY 
                             * (gridVoxelOffsetX - x2) / gridVoxelOffsetX;

              int indX = int((gridVoxelOffsetX + gridVoxelSize / 2 - x2) / gridVoxelSize);
              int indY = int((gridVoxelOffsetY + gridVoxelSize / 2 - y2 / scaleY) / gridVoxelSize);
              if (indX >= 0 && indX < gridVoxelNumX && indY >= 0 && indY < gridVoxelNumY) {
                int ind = gridVoxelNumY * indX + indY;
                int blockedPathByVoxelNum = correspondences[ind].size();
                for (int j = 0; j < blockedPathByVoxelNum; j++) {
                  if (h > obstacleHeightThre || !useTerrainAnalysis) {
                    clearPathList[pathNum * rotDir + correspondences[ind][j]]++;
                  } else {
                    if (pathPenaltyList[pathNum * rotDir + correspondences[ind][j]] < h && h > groundHeightThre) {
                      pathPenaltyList[pathNum * rotDir + correspondences[ind][j]] = h;
                    }
                  }
                }
              }
            }
          }

          if (dis < diameter / pathScale && (fabs(x) > vehicleLength / pathScale / 2.0 || fabs(y) > vehicleWidth / pathScale / 2.0) && 
              (h > obstacleHeightThre || !useTerrainAnalysis) && checkRotObstacle) {
            float angObs = atan2(y, x) * 180.0 / PI;
            if (angObs > 0) {
              if (minObsAngCCW > angObs - angOffset) minObsAngCCW = angObs - angOffset;
              if (minObsAngCW < angObs + angOffset - 180.0) minObsAngCW = angObs + angOffset - 180.0;
            } else {
              if (minObsAngCW < angObs + angOffset) minObsAngCW = angObs + angOffset;
              if (minObsAngCCW > 180.0 + angObs - angOffset) minObsAngCCW = 180.0 + angObs - angOffset;
            }
          }
        }

        if (minObsAngCW > 0) minObsAngCW = 0;
        if (minObsAngCCW < 0) minObsAngCCW = 0;

        for (int i = 0; i < 36 * pathNum; i++) {
          int rotDir = int(i / pathNum);
          float angDiff = fabs(joyDir - (10.0 * rotDir - 180.0));
          if (angDiff > 180.0) {
            angDiff = 360.0 - angDiff;
          }
          if ((angDiff > dirThre && !dirToVehicle) || (fabs(10.0 * rotDir - 180.0) > dirThre && fabs(joyDir) <= 90.0 && dirToVehicle) ||
              ((10.0 * rotDir > dirThre && 360.0 - 10.0 * rotDir > dirThre) && fabs(joyDir) > 90.0 && dirToVehicle)) {
            continue;
          }

          ++localStatus.candidateEligible;
          const bool broadBlocked = clearPathList[i] >= pointPerPathThre;
          if (broadBlocked) ++localStatus.candidateBlocked;
          else ++localStatus.broadPhasePassCount;
          bool narrowRecovered = false;
          if (broadBlocked && enableOrientationAwareCheck && checkObstacle) {
            ++localStatus.narrowChecked;
            const int pathID = i % pathNum;
            const int groupID = pathList[pathID];
            const int groupIndex = groupNum * rotDir + groupID;
            ++localStatus.narrowChecksByGroup[groupID];
            if (!narrowGroupChecked[groupIndex]) {
              narrowGroupChecked[groupIndex] = true;
              const auto checkStart = std::chrono::steady_clock::now();
              narrowGroupResults[groupIndex] = checkNarrowGroup(
                  groupID, (10.0 * rotDir - 180.0) * PI / 180.0,
                  pathScale, pathRange, relativeGoalDis, *plannerCloudCrop);
              localStatus.narrowCheckMs += std::chrono::duration<double, std::milli>(
                  std::chrono::steady_clock::now() - checkStart).count();
            }
            narrowRecovered = narrowGroupResults[groupIndex].free;
            if (narrowRecovered) {
              ++localStatus.narrowRecovered;
              ++localStatus.narrowRecoveredByGroup[groupID];
              ++localStatus.recoveredAfterCurvatureFilter;
              ++localStatus.recoveredAfterDirectionFilter;
              narrowGroupRecovered[groupIndex] = true;
            } else {
              ++localStatus.narrowRejected;
              const int reasonIndex = narrowReasonIndex(narrowGroupResults[groupIndex].reason);
              if (reasonIndex >= 0) {
                ++localStatus.narrowReasonCounts[reasonIndex];
                ++localStatus.narrowReasonCountsByGroup[groupID][reasonIndex];
              } else {
                ++localStatus.narrowOtherRejectedByGroup[groupID];
              }
              // Kept for compatibility; detailed reason counters below are the
              // authoritative diagnostic instead of this last-loop value.
              localStatus.selectedRejectionReason = narrowGroupResults[groupIndex].reason;
            }
            // Group 3 is the uncurved centreline in the immutable path library.
            // Choose the candidate most aligned with the goal; ties preserve the
            // lower rotation index. This is a diagnostic observation only.
            if (groupID == 3) {
              const double directionErrorDeg = angDiff;
              if (!localStatus.bestStraightAvailable ||
                  directionErrorDeg < localStatus.bestStraightDirectionErrorDeg ||
                  (directionErrorDeg == localStatus.bestStraightDirectionErrorDeg &&
                   rotDir < localStatus.bestStraightRotationID)) {
                const auto& result = narrowGroupResults[groupIndex];
                localStatus.bestStraightAvailable = true;
                localStatus.bestStraightPathID = pathID;
                localStatus.bestStraightGroupID = groupID;
                localStatus.bestStraightRotationID = rotDir;
                localStatus.bestStraightDirectionErrorDeg = directionErrorDeg;
                localStatus.bestStraightReason = result.reason;
                localStatus.bestStraightClearanceAvailable =
                    result.reason == "NARROW_RECOVERED" ||
                    result.reason == "NARROW_FOOTPRINT_COLLISION";
                localStatus.bestStraightMinClearance = result.min_clearance;
                localStatus.bestStraightHasBlockingPoint = result.has_blocking_point;
                localStatus.bestStraightBlockingX = result.blocking_point.x;
                localStatus.bestStraightBlockingY = result.blocking_point.y;
              }
            }
          }
          if (!broadBlocked || (narrowRecovered && enableNarrowRecoveredSelection)) {
            float penaltyScore = 1.0 - pathPenaltyList[i] / costHeightThre;
            if (penaltyScore < costScore) penaltyScore = costScore;

            float dirDiff = fabs(joyDir - endDirPathList[i % pathNum] - (10.0 * rotDir - 180.0));
            if (dirDiff > 360.0) {
              dirDiff -= 360.0;
            }
            if (dirDiff > 180.0) {
              dirDiff = 360.0 - dirDiff;
            }

            float rotDirW;
            if (rotDir < 18) rotDirW = fabs(fabs(rotDir - 9) + 1);
            else rotDirW = fabs(fabs(rotDir - 27) + 1);
            float groupDirW = 4  - fabs(pathList[i % pathNum] - 3);
            float score = (1 - sqrt(sqrt(dirWeight * dirDiff))) * rotDirW * rotDirW * rotDirW * rotDirW * penaltyScore;
            if (relativeGoalDis < goalCloseDis) score = (1 - sqrt(sqrt(dirWeight * dirDiff))) * groupDirW * groupDirW * penaltyScore;
            if (std::isfinite(score) && score > 0) {
              if (narrowRecovered) {
                ++localStatus.recoveredAfterOtherFilters;
                recoveredScoredGroup[groupNum * rotDir + pathList[i % pathNum]] = true;
              }
              localStatus.candidateScored++;
              clearPathPerGroupScore[groupNum * rotDir + pathList[i % pathNum]] += score;
            }
          }
        }

        float bestScore = 0.0F;
        int bestCandidateIndex = -1;
        for (int i = 0; i < 36 * groupNum; i++) {
          const int candidateRotation = int(i / groupNum);
          const float rotAng = (10.0F * candidateRotation - 180.0F) * PI / 180.0F;
          float rotDeg = 10.0F * candidateRotation;
          if (rotDeg > 180.0F) rotDeg -= 360.0F;
          const bool rotationSafe =
              (rotAng * 180.0F / PI > minObsAngCW && rotAng * 180.0F / PI < minObsAngCCW) ||
              (rotDeg > minObsAngCW && rotDeg < minObsAngCCW && twoWayDrive) || !checkRotObstacle;
          if (clearPathPerGroupScore[i] > bestScore && rotationSafe) {
            bestScore = clearPathPerGroupScore[i];
            bestCandidateIndex = i;
          }
        }

        for (int i = 0; i < 36 * groupNum; ++i) {
          const double candidateScore = clearPathPerGroupScore[i];
          if (!std::isfinite(candidateScore) || candidateScore <= 0.0) continue;
          ++localStatus.finalSelectableCount;
          const int candidateGroup = i % groupNum;
          const int candidateRotation = i / groupNum;
          if (recoveredScoredGroup[i]) {
            ++localStatus.recoveredEnteredSelection;
            if (candidateScore > localStatus.bestRecoveredScore) {
              localStatus.bestRecoveredScore = candidateScore;
              localStatus.bestRecoveredGroupID = candidateGroup;
              localStatus.bestRecoveredRotationID = candidateRotation;
            }
          }
        }
        if (bestCandidateIndex >= 0) {
          localStatus.bestCandidateGroupID = bestCandidateIndex % groupNum;
          localStatus.bestCandidateRotationID = bestCandidateIndex / groupNum;
          localStatus.selectionFailureReason = "NONE";
        } else if (localStatus.narrowRecovered > 0) {
          localStatus.selectionFailureReason = "RECOVERED_NOT_IN_FINAL_POOL";
        } else if (localStatus.candidateScored == 0) {
          localStatus.selectionFailureReason = "ALL_CANDIDATES_FAILED_SCORING";
        } else {
          localStatus.selectionFailureReason = "NO_ELIGIBLE_CANDIDATES";
        }

        int selectedCandidateIndex = bestCandidateIndex;
        std::string switchReason = "INITIAL_CANDIDATE";
        double currentScore = 0.0;
        bool currentSafe = false;
        bool bestMatchesActive = false;
        if (activeCandidateValid) {
          const int activeIndex = groupNum * activeCandidateRotationID + activeCandidateGroupID;
          if (activeCandidateRotationID >= 0 && activeCandidateRotationID < 36 &&
              activeCandidateGroupID >= 0 && activeCandidateGroupID < groupNum) {
            const float activeRotAng = (10.0F * activeCandidateRotationID - 180.0F) * PI / 180.0F;
            float activeRotDeg = 10.0F * activeCandidateRotationID;
            if (activeRotDeg > 180.0F) activeRotDeg -= 360.0F;
            const bool activeRotationSafe =
                (activeRotAng * 180.0F / PI > minObsAngCW && activeRotAng * 180.0F / PI < minObsAngCCW) ||
                (activeRotDeg > minObsAngCW && activeRotDeg < minObsAngCCW && twoWayDrive) || !checkRotObstacle;
            currentScore = clearPathPerGroupScore[activeIndex];
            const int activeDirection = std::cos(activeRotAng) >= 0.0F ? 1 : -1;
            const bool activeNarrowMode = narrowGroupRecovered[activeIndex];
            currentSafe = activeRotationSafe && currentScore > 0.0 &&
                activeDirection == activeCandidateDirection &&
                activeNarrowMode == activeCandidateNarrowMode;
          }
          if (bestCandidateIndex >= 0) {
            const int bestRotation = bestCandidateIndex / groupNum;
            const int bestGroup = bestCandidateIndex % groupNum;
            const int bestDirection = std::cos((10.0F * bestRotation - 180.0F) * PI / 180.0F) >= 0.0F ? 1 : -1;
            const bool bestNarrowMode = narrowGroupRecovered[bestCandidateIndex];
            bestMatchesActive = bestGroup == activeCandidateGroupID &&
                bestRotation == activeCandidateRotationID &&
                bestDirection == activeCandidateDirection &&
                bestNarrowMode == activeCandidateNarrowMode;
          }
          if (!currentSafe) {
            switchReason = "SAFETY_CURRENT_BLOCKED";
          } else if (bestMatchesActive) {
            switchReason = "REFRESH_SAME_CANDIDATE";
          } else if (enableTemporalPathStabilization && bestCandidateIndex >= 0) {
            const double improvement = (bestScore - currentScore) / std::max(currentScore, 1.0e-6);
            const double sinceSwitch = std::chrono::duration<double>(
                planningStart - lastSafeCandidateSwitch).count();
            if (improvement < pathSwitchScoreMargin) {
              selectedCandidateIndex = activeIndex;
              switchReason = "HYSTERESIS_HOLD";
            } else if (sinceSwitch < safePathSwitchMinIntervalSec) {
              selectedCandidateIndex = activeIndex;
              switchReason = "RATE_LIMITED_SAFE_SWITCH";
            } else {
              switchReason = "SCORE_MARGIN_SWITCH";
            }
          } else if (bestCandidateIndex >= 0) {
            switchReason = "BEST_CANDIDATE";
          }
        }

        if (selectedCandidateIndex >= 0) {
          int rotDir = int(selectedCandidateIndex / groupNum);
          float rotAng = (10.0 * rotDir - 180.0) * PI / 180;
          const int selectedGroupID = selectedCandidateIndex % groupNum;
          const int selectedIndex = groupNum * rotDir + selectedGroupID;
          const int selectedDirection = std::cos(rotAng) >= 0.0F ? 1 : -1;
          const bool selectedRequiresAlignment = narrowGroupRecovered[selectedIndex];
          const double selectedClearance = selectedRequiresAlignment ?
              narrowGroupResults[selectedIndex].min_clearance : 0.0;
          localStatus.selectedRequiresAlignment = selectedRequiresAlignment;
          localStatus.recoveredSelected = selectedRequiresAlignment ? 1 : 0;
          localStatus.selectedMinClearance = selectedClearance;
          localStatus.selectedTurnRadius = selectedRequiresAlignment ?
              narrowGroupResults[selectedIndex].turn_radius : 0.0;
          if (selectedRequiresAlignment) localStatus.selectedRejectionReason = "NONE";
          int selectedPathLength = startPaths[selectedGroupID]->points.size();
          localStatus.selectedGroupID = selectedGroupID;
          localStatus.selectedPathLength = selectedPathLength;
          localStatus.selectedRotationID = rotDir;
          localStatus.selectedDirection = selectedDirection;
          localStatus.selectedNarrowMode = selectedRequiresAlignment;
          localStatus.currentScore = clearPathPerGroupScore[selectedIndex];
          localStatus.bestScore = bestScore;
          localStatus.switchReason = switchReason;
          const bool sameCandidate = activeCandidateValid &&
              selectedGroupID == activeCandidateGroupID &&
              rotDir == activeCandidateRotationID &&
              selectedDirection == activeCandidateDirection &&
              selectedRequiresAlignment == activeCandidateNarrowMode;
          if (sameCandidate) {
            ++pathRefreshCount;
          } else {
            if (activeCandidateValid) ++pathSwitchCount;
            lastSafeCandidateSwitch = planningStart;
          }
          activeCandidateValid = true;
          activeCandidateGroupID = selectedGroupID;
          activeCandidateRotationID = rotDir;
          activeCandidateDirection = selectedDirection;
          activeCandidateNarrowMode = selectedRequiresAlignment;
          activeCandidateScore = localStatus.currentScore;
          pathSwitchReason = switchReason;
          path.poses.resize(selectedPathLength);
          for (int i = 0; i < selectedPathLength; i++) {
            float x = startPaths[selectedGroupID]->points[i].x;
            float y = startPaths[selectedGroupID]->points[i].y;
            float z = startPaths[selectedGroupID]->points[i].z;
            float dis = sqrt(x * x + y * y);

            if (dis <= pathRange / pathScale && dis <= relativeGoalDis / pathScale) {
              path.poses[i].pose.position.x = pathScale * (cos(rotAng) * x - sin(rotAng) * y);
              path.poses[i].pose.position.y = pathScale * (sin(rotAng) * x + cos(rotAng) * y);
              path.poses[i].pose.position.z = pathScale * z;
            } else {
              path.poses.resize(i);
              break;
            }
          }

          path.header.stamp = enableNarrowRecoveredSelection ? nh->now() :
              rclcpp::Time(static_cast<uint64_t>(odomTime * 1e9));
          path.header.frame_id = "vehicle";
          localStatus.planningCycleMs = std::chrono::duration<double, std::milli>(
              std::chrono::steady_clock::now() - planningStart).count();
          localStatus.pathFound = true;
          localStatus.publishedPathSize = path.poses.size();
          visibility_graph_msg::msg::LocalPathConstraint constraint;
          constraint.header = path.header;
          constraint.path_revision = ++localPathRevision;
          constraint.requires_alignment = selectedRequiresAlignment;
          constraint.lateral_clearance = selectedClearance;
          constraint.footprint_length = narrowFootprintLength;
          constraint.footprint_width = narrowFootprintWidth;
          constraint.enter_yaw_tolerance = narrowEnterYawToleranceDeg * PI / 180.0;
          constraint.continue_yaw_tolerance = narrowContinueYawToleranceDeg * PI / 180.0;
          constraint.stop_yaw_tolerance = narrowStopYawToleranceDeg * PI / 180.0;
          constraint.recovery_yaw_limit = narrowRecoveryYawLimitDeg * PI / 180.0;
          constraint.speed_scale = narrowSpeedScale;
          constraint.narrow_approach_x = 0.0F;
          constraint.narrow_approach_y = 0.0F;
          constraint.narrow_approach_heading = 0.0F;
          if (selectedRequiresAlignment && path.poses.size() >= 2) {
            // This point lies ahead on the approved centreline, before the
            // narrow traverse. It is a fixed entry reference for the follower,
            // not its rolling lookahead target.
            constexpr double kApproachDistance = 0.25;
            size_t approachIndex = 1;
            double traversed = 0.0;
            for (size_t index = 1; index < path.poses.size(); ++index) {
              const auto& previous = path.poses[index - 1].pose.position;
              const auto& current = path.poses[index].pose.position;
              traversed += std::hypot(current.x - previous.x, current.y - previous.y);
              approachIndex = index;
              if (traversed >= kApproachDistance) break;
            }
            const size_t headingPrevious = approachIndex > 0 ? approachIndex - 1 : 0;
            const size_t headingNext = std::min(approachIndex + 1, path.poses.size() - 1);
            const auto& from = path.poses[headingPrevious].pose.position;
            const auto& to = path.poses[headingNext].pose.position;
            const double dx = to.x - from.x;
            const double dy = to.y - from.y;
            if (std::hypot(dx, dy) > 1.0e-6) {
              constraint.narrow_approach_x = path.poses[approachIndex].pose.position.x;
              constraint.narrow_approach_y = path.poses[approachIndex].pose.position.y;
              constraint.narrow_approach_heading = std::atan2(dy, dx);
            }
          }
          constraint.candidate_group_id = selectedGroupID;
          constraint.candidate_rotation_id = rotDir;
          constraint.candidate_direction = selectedDirection;
          constraint.candidate_narrow_mode = selectedRequiresAlignment;
          constraint.current_score = localStatus.currentScore;
          constraint.best_score = bestScore;
          constraint.switch_reason = switchReason;
          constraint.reason = selectedRequiresAlignment ? "ORIENTATION_CRITICAL" : "NORMAL";
          pubPathConstraint->publish(constraint);
          pubPath->publish(path);
          publishNarrowMarkers(path, selectedRequiresAlignment,
                               localStatus.selectedTurnRadius);
          publishLocalStatus("PATH_PUBLISHED", "PATH_PUBLISHED", "selected local path published", true);

          #if PLOTPATHSET == 1
          freePaths->clear();
          for (int i = 0; i < 36 * pathNum; i++) {
            int rotDir = int(i / pathNum);
            float rotAng = (10.0 * rotDir - 180.0) * PI / 180;
            float rotDeg = 10.0 * rotDir;
            if (rotDeg > 180.0) rotDeg -= 360.0;
            float angDiff = fabs(joyDir - (10.0 * rotDir - 180.0));
            if (angDiff > 180.0) angDiff = 360.0 - angDiff;
            if ((angDiff > dirThre && !dirToVehicle) ||
                (fabs(10.0 * rotDir - 180.0) > dirThre && fabs(joyDir) <= 90.0 && dirToVehicle) ||
                ((10.0 * rotDir > dirThre && 360.0 - 10.0 * rotDir > dirThre) && fabs(joyDir) > 90.0 && dirToVehicle) ||
                !((rotAng * 180.0 / PI > minObsAngCW && rotAng * 180.0 / PI < minObsAngCCW) ||
                  (rotDeg > minObsAngCW && rotDeg < minObsAngCCW && twoWayDrive) || !checkRotObstacle)) continue;
            if (clearPathList[i] < pointPerPathThre) {
              int freePathLength = paths[i % pathNum]->points.size();
              for (int j = 0; j < freePathLength; j++) {
                point = paths[i % pathNum]->points[j];
                float x = point.x;
                float y = point.y;
                float z = point.z;
                float dis = sqrt(x * x + y * y);
                if (dis <= pathRange / pathScale &&
                    (dis <= (relativeGoalDis + goalClearRange) / pathScale || !pathCropByGoal)) {
                  point.x = pathScale * (cos(rotAng) * x - sin(rotAng) * y);
                  point.y = pathScale * (sin(rotAng) * x + cos(rotAng) * y);
                  point.z = pathScale * z;
                  point.intensity = 1.0;
                  freePaths->push_back(point);
                }
              }
            }
          }
          sensor_msgs::msg::PointCloud2 freePaths2;
          pcl::toROSMsg(*freePaths, freePaths2);
          freePaths2.header.stamp = rclcpp::Time(static_cast<uint64_t>(odomTime * 1e9));
          freePaths2.header.frame_id = "vehicle";
          pubFreePaths->publish(freePaths2);
          #endif
        }
        if (selectedCandidateIndex < 0) {
          if (pathScale >= minPathScale + pathScaleStep) {
            pathScale -= pathScaleStep;
            pathRange = adjacentRange * pathScale / defPathScale;
          } else {
            pathRange -= pathRangeStep;
          }
        } else {
          pathFound = true;
          break;
        }
      }
      pathScale = defPathScale;

      if (!pathFound) {
        localStatus.pathFound = false;
        localStatus.selectedGroupID = -1;
        localStatus.selectedPathLength = 0;
        localStatus.publishedPathSize = 1;
        localStatus.planningCycleMs = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - planningStart).count();
        const std::string stopReason = localStatus.candidateEligible > 0 &&
            localStatus.candidateBlocked >= localStatus.candidateEligible
            ? "ALL_CANDIDATES_BLOCKED" : "NO_POSITIVE_SCORE";
        publishLocalStatus("STOP_PATH_PUBLISHED", stopReason,
            "no usable local candidate path exists", true);
        RCLCPP_WARN(nh->get_logger(),
            "[LOCAL][STOP] reason=%s cloud=%zu cropped=%zu blocked=%d scored=%d goal_dist=%.3f",
            stopReason.c_str(), localStatus.plannerCloudPoints, localStatus.plannerCloudCropPoints,
            localStatus.candidateBlocked, localStatus.candidateScored, localStatus.relativeGoalDistance);
        activeCandidateValid = false;
        pathSwitchReason = "NO_SAFE_CANDIDATE";
        localStatus.selectedRotationID = -1;
        localStatus.selectedDirection = 0;
        localStatus.selectedNarrowMode = false;
        localStatus.currentScore = 0.0;
        // Retain the final attempted planning-cycle best score in stop diagnostics.
        localStatus.switchReason = pathSwitchReason;
        path.poses.resize(1);
        path.poses[0].pose.position.x = 0;
        path.poses[0].pose.position.y = 0;
        path.poses[0].pose.position.z = 0;

        path.header.stamp = enableNarrowRecoveredSelection ? nh->now() :
            rclcpp::Time(static_cast<uint64_t>(odomTime * 1e9));
        path.header.frame_id = "vehicle";
        visibility_graph_msg::msg::LocalPathConstraint constraint;
        constraint.header = path.header;
        constraint.path_revision = ++localPathRevision;
        constraint.requires_alignment = false;
        constraint.candidate_group_id = -1;
        constraint.candidate_rotation_id = -1;
        constraint.candidate_direction = 0;
        constraint.candidate_narrow_mode = false;
        constraint.current_score = 0.0F;
        constraint.best_score = localStatus.bestScore;
        constraint.switch_reason = pathSwitchReason;
        constraint.reason = stopReason;
        pubPathConstraint->publish(constraint);
        pubPath->publish(path);
        publishNarrowMarkers(path, false, 0.0);

        #if PLOTPATHSET == 1
        freePaths->clear();
        sensor_msgs::msg::PointCloud2 freePaths2;
        pcl::toROSMsg(*freePaths, freePaths2);
        freePaths2.header.stamp = rclcpp::Time(static_cast<uint64_t>(odomTime * 1e9));
        freePaths2.header.frame_id = "vehicle";
        pubFreePaths->publish(freePaths2);
        #endif
      }

      if (waypointPendingImmediatePlan) {
        localStatus.waypointToPathDelaySec = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - waypointReceive).count();
        waypointPendingImmediatePlan = false;
      }

      /*sensor_msgs::msg::PointCloud2 plannerCloud2;
      pcl::toROSMsg(*plannerCloudCrop, plannerCloud2);
      plannerCloud2.header.stamp = rclcpp::Time(static_cast<uint64_t>(odomTime * 1e9));
      plannerCloud2.header.frame_id = "vehicle";
      pubLaserCloud->publish(plannerCloud2);*/
    }

    status = rclcpp::ok();
    rate.sleep();
  }

  return 0;
}
