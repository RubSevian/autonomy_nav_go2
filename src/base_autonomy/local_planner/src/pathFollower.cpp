#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <chrono>
#include <algorithm>
#include <cmath>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/clock.hpp"
#include "builtin_interfaces/msg/time.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/bool.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include "visibility_graph_msg/msg/local_path_constraint.hpp"
#include "local_planner/narrow_passage.hpp"
#include "local_planner/unicycle_follower.hpp"
#include <sensor_msgs/msg/imu.h>

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

// For real robot
#include "unitree_api/msg/request.hpp"
#include "common/ros2_sport_client.h"

using namespace std;

const double PI = 3.1415926;

double sensorOffsetX = 0;
double sensorOffsetY = 0;
int pubSkipNum = 1;
int pubSkipCount = 0;
bool twoWayDrive = true;
double lookAheadDis = 0.5;
double yawRateGain = 7.5;
double stopYawRateGain = 7.5;
double maxYawRate = 45.0;
double maxSpeed = 1.0;
double maxAccel = 1.0;
double switchTimeThre = 1.0;
double dirDiffThre = 0.1;
double omniDirDiffThre = 1.5;
double noRotSpeed = 10.0;
double stopDisThre = 0.2;
double slowDwnDisThre = 1.0;
bool useInclRateToSlow = false;
double inclRateThre = 120.0;
double slowRate1 = 0.25;
double slowRate2 = 0.5;
double slowTime1 = 2.0;
double slowTime2 = 2.0;
bool useInclToStop = false;
double inclThre = 45.0;
double stopTime = 5.0;
bool noRotAtStop = false;
bool noRotAtGoal = true;
bool manualMode = false;
bool autonomyMode = false;
double autonomySpeed = 1.0;
double joyToSpeedDelay = 2.0;
double goalCloseDis = 1.0;
double odomTimeoutSec = 0.5;
double pathTimeoutSec = 0.5;
// A real planner must continuously refresh its local route.  The MuJoCo
// smoke-test deliberately sends one static path in the vehicle frame.
bool allowStaticPath = false;
bool is_real_robot = false;
// RL locomotion owns the low-level interface.  Sport Mode must remain off in
// that configuration, otherwise both controllers command the same robot.
bool sendSportCommand = false;
bool enableNarrowPassageMode = false;
int narrowAlignmentConfirmCycles = 5;
double narrowRealignTimeoutSec = 2.0;
std::string followerMotionModel = "holonomic";
double unicycleRotateEnterDeg = 55.0;
double unicycleRotateExitDeg = 35.0;
double unicycleRotateGain = 1.5;
bool unicycleRotateInPlace = false;
float lookaheadXBody = 0.0F, lookaheadYBody = 0.0F, lookaheadDistance = 0.0F;
float purePursuitCurvatureValue = 0.0F, pathTangentHeading = 0.0F, pathHeadingError = 0.0F;
bool unicycleWzSaturated = false;
std::uint64_t unicycleRotateEvents = 0;
double unicycleRotateTimeSec = 0.0;

float joySpeed = 0;
float joySpeedRaw = 0;
float joyYaw = 0;
float joyManualFwd = 0;
float joyManualLeft = 0;
float joyManualYaw = 0;
int safetyStop = 0;

float vehicleX = 0;
float vehicleY = 0;
float vehicleZ = 0;
float vehicleRoll = 0;
float vehiclePitch = 0;
float vehicleYaw = 0;

float vehicleXRec = 0;
float vehicleYRec = 0;
float vehicleZRec = 0;
float vehicleRollRec = 0;
float vehiclePitchRec = 0;
float vehicleYawRec = 0;
float pathSourceYaw = 0;  // Pose associated with the most recently accepted vehicle-frame path.

float vehicleYawRate = 0;
float vehicleSpeed = 0;

double odomTime = 0;
double joyTime = 0;
double slowInitTime = 0;
double stopInitTime = false;
int pathPointID = 0;
bool pathInit = false;
bool navigationActive = false;
bool navFwd = true;
double switchTime = 0;
bool odomReceived = false;
bool pathReceived = false;
bool constraintReceived = false;
bool constraintValid = false;
bool constraintMatchesPath = false;
std::uint64_t activePathRevision = 0;
visibility_graph_msg::msg::LocalPathConstraint latestConstraint;
float narrowYawError = 0.0F;
bool narrowHeadingLocked = false;
double desiredNarrowHeading = 0.0;
double lockedNarrowHeading = 0.0;
float narrowApproachX = 0.0F;
float narrowApproachY = 0.0F;
int selectedNarrowDirection = 0;  // +1 forward through lane, -1 backward.
std::string lastNarrowStateTransition = "NONE";
std::uint64_t narrowStateTransitions = 0;
bool trackedCandidateValid = false;
int trackedCandidateGroupID = -1;
int trackedCandidateRotationID = -1;
int trackedCandidateDirection = 0;
bool trackedCandidateNarrowMode = false;
std::uint64_t followerPathSwitchCount = 0;
std::uint64_t followerPathRefreshCount = 0;
std::string followerPathUpdateReason = "INITIAL";
std::uint64_t alignmentEntries = 0;
std::uint64_t alignmentSuccesses = 0;
std::uint64_t realignEvents = 0;
std::uint64_t alignmentTimeouts = 0;
float maxNarrowYawError = 0.0F;
std::chrono::steady_clock::time_point lastOdomReceive;
std::chrono::steady_clock::time_point lastPathReceive;

nav_msgs::msg::Path path;
rclcpp::Node::SharedPtr nh;

void resetNarrowHeadingLock() {
  narrowHeadingLocked = false;
  desiredNarrowHeading = 0.0;
  lockedNarrowHeading = 0.0;
  narrowApproachX = 0.0F;
  narrowApproachY = 0.0F;
  selectedNarrowDirection = 0;
}

local_planner::StampKey stampKey(const builtin_interfaces::msg::Time& stamp) {
  return {stamp.sec, stamp.nanosec};
}

void refreshConstraintMatch() {
  constraintMatchesPath = false;
  if (!pathReceived || !constraintReceived || !constraintValid) return;
  const auto pathStamp = stampKey(path.header.stamp);
  const auto constraintStamp = stampKey(latestConstraint.header.stamp);
  if (local_planner::matchingConstraint(pathStamp, constraintStamp,
                                        latestConstraint.path_revision,
                                        activePathRevision)) {
    activePathRevision = latestConstraint.path_revision;
    constraintMatchesPath = true;
  } else if (pathStamp.sec == constraintStamp.sec &&
             pathStamp.nanosec == constraintStamp.nanosec &&
             activePathRevision == latestConstraint.path_revision) {
    constraintMatchesPath = true;
  }
}

void applyConstraint(
    const visibility_graph_msg::msg::LocalPathConstraint::ConstSharedPtr message) {
  if (constraintReceived && message->path_revision < latestConstraint.path_revision) return;
  latestConstraint = *message;
  constraintReceived = true;
  constraintValid = !message->requires_alignment ||
      (std::isfinite(message->enter_yaw_tolerance) &&
       std::isfinite(message->continue_yaw_tolerance) &&
       std::isfinite(message->stop_yaw_tolerance) &&
       std::isfinite(message->recovery_yaw_limit) &&
       std::isfinite(message->speed_scale) &&
       std::isfinite(message->narrow_approach_x) &&
       std::isfinite(message->narrow_approach_y) &&
       std::isfinite(message->narrow_approach_heading) &&
       message->enter_yaw_tolerance > 0.0F &&
       message->enter_yaw_tolerance < message->continue_yaw_tolerance &&
       message->continue_yaw_tolerance < message->stop_yaw_tolerance &&
       message->stop_yaw_tolerance < message->recovery_yaw_limit &&
       message->recovery_yaw_limit <= PI / 4.0 &&
       message->speed_scale > 0.0F && message->speed_scale <= 1.0F);
  refreshConstraintMatch();
}

unitree_api::msg::Request req;
SportClient sport_req;

void odomHandler(const nav_msgs::msg::Odometry::ConstSharedPtr odomIn)
{
  const auto& position = odomIn->pose.pose.position;
  const auto& orientation = odomIn->pose.pose.orientation;
  const double orientationNorm = orientation.x * orientation.x + orientation.y * orientation.y +
                                 orientation.z * orientation.z + orientation.w * orientation.w;
  if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
      !std::isfinite(position.z) || !std::isfinite(orientation.x) ||
      !std::isfinite(orientation.y) || !std::isfinite(orientation.z) ||
      !std::isfinite(orientation.w) || orientationNorm < 1.0e-12) {
    RCLCPP_WARN(nh->get_logger(), "Ignoring non-finite odometry pose");
    return;
  }
  lastOdomReceive = std::chrono::steady_clock::now();
  odomReceived = true;
  odomTime = rclcpp::Time(odomIn->header.stamp).seconds();
  double roll, pitch, yaw;
  geometry_msgs::msg::Quaternion geoQuat = odomIn->pose.pose.orientation;
  tf2::Matrix3x3(tf2::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);

  vehicleRoll = roll;
  vehiclePitch = pitch;
  vehicleYaw = yaw;
  vehicleX = odomIn->pose.pose.position.x - cos(yaw) * sensorOffsetX + sin(yaw) * sensorOffsetY;
  vehicleY = odomIn->pose.pose.position.y - sin(yaw) * sensorOffsetX - cos(yaw) * sensorOffsetY;
  vehicleZ = odomIn->pose.pose.position.z;

  if ((fabs(roll) > inclThre * PI / 180.0 || fabs(pitch) > inclThre * PI / 180.0) && useInclToStop) {
    stopInitTime = rclcpp::Time(odomIn->header.stamp).seconds();
  }

  if ((fabs(odomIn->twist.twist.angular.x) > inclRateThre * PI / 180.0 || fabs(odomIn->twist.twist.angular.y) > inclRateThre * PI / 180.0) && useInclRateToSlow) {
    slowInitTime = rclcpp::Time(odomIn->header.stamp).seconds();
  }
}

void applyPath(const nav_msgs::msg::Path::ConstSharedPtr pathIn)
{
  if (!navigationActive) return;
  if (pathIn->header.frame_id != "vehicle" && pathIn->header.frame_id != "/vehicle") {
    RCLCPP_WARN(nh->get_logger(), "Rejecting path with unexpected frame '%s'",
                pathIn->header.frame_id.c_str());
    path.poses.clear(); pathReceived = false; pathInit = false;
    vehicleSpeed = 0.0F; vehicleYawRate = 0.0F;
    return;
  }
  const int pathSize = pathIn->poses.size();
  lastPathReceive = std::chrono::steady_clock::now();
  pathReceived = pathSize > 0;
  if (!pathReceived) { path.poses.clear(); pathInit = false; return; }
  for (const auto& pose : pathIn->poses) {
    if (!std::isfinite(pose.pose.position.x) || !std::isfinite(pose.pose.position.y) ||
        !std::isfinite(pose.pose.position.z)) {
      RCLCPP_WARN(nh->get_logger(), "Rejecting path with non-finite position");
      path.poses.clear(); pathReceived = false; pathInit = false;
      vehicleSpeed = 0.0F; vehicleYawRate = 0.0F;
      return;
    }
  }

  pathSourceYaw = vehicleYaw;
  const auto incomingStamp = stampKey(pathIn->header.stamp);
  const auto constraintStamp = stampKey(latestConstraint.header.stamp);
  const bool incomingConstraintMatches = constraintReceived && constraintValid &&
      incomingStamp.sec == constraintStamp.sec && incomingStamp.nanosec == constraintStamp.nanosec;
  const bool equivalentRefresh = incomingConstraintMatches && pathInit && trackedCandidateValid &&
      latestConstraint.candidate_group_id == trackedCandidateGroupID &&
      latestConstraint.candidate_rotation_id == trackedCandidateRotationID &&
      latestConstraint.candidate_direction == trackedCandidateDirection &&
      latestConstraint.candidate_narrow_mode == trackedCandidateNarrowMode;

  path.header = pathIn->header;
  path.poses.resize(pathSize);
  for (int i = 0; i < pathSize; ++i) {
    path.poses[i] = pathIn->poses[i];
    if (equivalentRefresh) {
      // Refresh coordinates are expressed in the current vehicle frame.
      const double worldX = vehicleX + std::cos(vehicleYaw) * pathIn->poses[i].pose.position.x -
          std::sin(vehicleYaw) * pathIn->poses[i].pose.position.y;
      const double worldY = vehicleY + std::sin(vehicleYaw) * pathIn->poses[i].pose.position.x +
          std::cos(vehicleYaw) * pathIn->poses[i].pose.position.y;
      const double dx = worldX - vehicleXRec;
      const double dy = worldY - vehicleYRec;
      path.poses[i].pose.position.x = std::cos(vehicleYawRec) * dx + std::sin(vehicleYawRec) * dy;
      path.poses[i].pose.position.y = -std::sin(vehicleYawRec) * dx + std::cos(vehicleYawRec) * dy;
    }
  }

  if (incomingConstraintMatches) {
    if (equivalentRefresh) {
      ++followerPathRefreshCount;
      followerPathUpdateReason = "REFRESH_SAME_CANDIDATE";
      // Fresh samples start at the robot; an old index skips the checked near segment.
      pathPointID = 0;
      double closestDistance = std::numeric_limits<double>::infinity();
      const double robotX = std::cos(vehicleYawRec) * (vehicleX - vehicleXRec) +
          std::sin(vehicleYawRec) * (vehicleY - vehicleYRec);
      const double robotY = -std::sin(vehicleYawRec) * (vehicleX - vehicleXRec) +
          std::cos(vehicleYawRec) * (vehicleY - vehicleYRec);
      for (int i = 0; i < pathSize; ++i) {
        const auto& point = path.poses[i].pose.position;
        const double distance = std::hypot(point.x - robotX, point.y - robotY);
        if (distance < closestDistance) {
          closestDistance = distance;
          pathPointID = i;
        }
      }
    } else {
      if (trackedCandidateValid) ++followerPathSwitchCount;
      followerPathUpdateReason = latestConstraint.switch_reason;
      vehicleXRec = vehicleX; vehicleYRec = vehicleY; vehicleZRec = vehicleZ;
      vehicleRollRec = vehicleRoll; vehiclePitchRec = vehiclePitch; vehicleYawRec = vehicleYaw;
      pathPointID = 0;
    }
    trackedCandidateValid = latestConstraint.candidate_group_id >= 0;
    trackedCandidateGroupID = latestConstraint.candidate_group_id;
    trackedCandidateRotationID = latestConstraint.candidate_rotation_id;
    trackedCandidateDirection = latestConstraint.candidate_direction;
    trackedCandidateNarrowMode = latestConstraint.candidate_narrow_mode;
  } else {
    // A new candidate is never followed with an old reference. The constraint
    // is published first by localPlanner, so normal refreshes take the branch above.
    if (trackedCandidateValid) ++followerPathSwitchCount;
    followerPathUpdateReason = "WAITING_FOR_PAIRED_CONSTRAINT";
    vehicleXRec = vehicleX; vehicleYRec = vehicleY; vehicleZRec = vehicleZ;
    vehicleRollRec = vehicleRoll; vehiclePitchRec = vehiclePitch; vehicleYawRec = vehicleYaw;
    pathPointID = 0;
  }
  pathInit = true;
  if (enableNarrowPassageMode) refreshConstraintMatch();
}

visibility_graph_msg::msg::LocalPathConstraint::ConstSharedPtr pendingConstraint;
nav_msgs::msg::Path::ConstSharedPtr pendingPath;

void applyPendingRoute() {
  if (!pendingPath || !pendingConstraint) return;
  const auto& a = pendingPath->header.stamp;
  const auto& b = pendingConstraint->header.stamp;
  if (a.sec != b.sec || a.nanosec != b.nanosec) return;
  applyConstraint(pendingConstraint);
  applyPath(pendingPath);
  pendingPath.reset();
  pendingConstraint.reset();
}

void constraintHandler(
    const visibility_graph_msg::msg::LocalPathConstraint::ConstSharedPtr message) {
  if (constraintReceived && message->path_revision < latestConstraint.path_revision) return;
  pendingConstraint = message;
  applyPendingRoute();
}

void pathHandler(const nav_msgs::msg::Path::ConstSharedPtr message) {
  if (!navigationActive) return;
  // Revocations and invalid frames take effect without waiting for metadata.
  if (message->poses.size() <= 1 ||
      (message->header.frame_id != "vehicle" && message->header.frame_id != "/vehicle")) {
    pendingPath.reset();
    applyPath(message);
    return;
  }
  if (!enableNarrowPassageMode && !constraintReceived && !pendingConstraint) {
    applyPath(message);  // Legacy producers may not publish constraints.
    return;
  }
  pendingPath = message;
  applyPendingRoute();
}

void navigationActiveHandler(const std_msgs::msg::Bool::ConstSharedPtr active)
{
  navigationActive = active->data;
  if (!navigationActive) {
    pendingPath.reset();
    pendingConstraint.reset();
    path.poses.clear();
    pathInit = false;
    pathReceived = false;
    constraintMatchesPath = false;
    vehicleSpeed = 0.0F;
    vehicleYawRate = 0.0F;
    resetNarrowHeadingLock();
    trackedCandidateValid = false;
    trackedCandidateGroupID = -1;
    trackedCandidateRotationID = -1;
    trackedCandidateDirection = 0;
    trackedCandidateNarrowMode = false;
  }
}

void joystickHandler(const sensor_msgs::msg::Joy::ConstSharedPtr joy)
{
  joyTime = nh->now().seconds(); 
  joySpeedRaw = sqrt(joy->axes[3] * joy->axes[3] + joy->axes[4] * joy->axes[4]);
  joySpeed = joySpeedRaw;
  if (joySpeed > 1.0) joySpeed = 1.0;
  if (joy->axes[4] == 0) joySpeed = 0;
  joyYaw = joy->axes[3];
  if (joySpeed == 0 && noRotAtStop) joyYaw = 0;

  if (joy->axes[4] < 0 && !twoWayDrive) {
    joySpeed = 0;
    joyYaw = 0;
  }

  joyManualFwd = joy->axes[4];
  joyManualLeft = joy->axes[3];
  joyManualYaw = joy->axes[0];

  if (joy->axes[2] > -0.1) {
    autonomyMode = false;
  } else {
    autonomyMode = true;
  }

  if (joy->axes[5] > -0.1) {
    manualMode = false;
  } else {
    manualMode = true;
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

void stopHandler(const std_msgs::msg::Int8::ConstSharedPtr stop)
{
  safetyStop = stop->data;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  nh = rclcpp::Node::make_shared("pathFollower");

  nh->declare_parameter<double>("sensorOffsetX", sensorOffsetX);
  nh->declare_parameter<double>("sensorOffsetY", sensorOffsetY);
  nh->declare_parameter<int>("pubSkipNum", pubSkipNum);
  nh->declare_parameter<bool>("twoWayDrive", twoWayDrive);
  nh->declare_parameter<double>("lookAheadDis", lookAheadDis);
  nh->declare_parameter<double>("yawRateGain", yawRateGain);
  nh->declare_parameter<double>("stopYawRateGain", stopYawRateGain);
  nh->declare_parameter<double>("maxYawRate", maxYawRate);
  nh->declare_parameter<double>("maxSpeed", maxSpeed);
  nh->declare_parameter<double>("maxAccel", maxAccel);
  nh->declare_parameter<double>("switchTimeThre", switchTimeThre);
  nh->declare_parameter<double>("dirDiffThre", dirDiffThre);
  nh->declare_parameter<double>("omniDirDiffThre", omniDirDiffThre);
  nh->declare_parameter<double>("noRotSpeed", noRotSpeed);
  nh->declare_parameter<double>("stopDisThre", stopDisThre);
  nh->declare_parameter<double>("slowDwnDisThre", slowDwnDisThre);
  nh->declare_parameter<bool>("useInclRateToSlow", useInclRateToSlow);
  nh->declare_parameter<double>("inclRateThre", inclRateThre);
  nh->declare_parameter<double>("slowRate1", slowRate1);
  nh->declare_parameter<double>("slowRate2", slowRate2);
  nh->declare_parameter<double>("slowTime1", slowTime1);
  nh->declare_parameter<double>("slowTime2", slowTime2);
  nh->declare_parameter<bool>("useInclToStop", useInclToStop);
  nh->declare_parameter<double>("inclThre", inclThre);
  nh->declare_parameter<double>("stopTime", stopTime);
  nh->declare_parameter<bool>("noRotAtStop", noRotAtStop);
  nh->declare_parameter<bool>("noRotAtGoal", noRotAtGoal);
  nh->declare_parameter<bool>("autonomyMode", autonomyMode);
  nh->declare_parameter<double>("autonomySpeed", autonomySpeed);
  nh->declare_parameter<double>("joyToSpeedDelay", joyToSpeedDelay);
  nh->declare_parameter<double>("goalCloseDis", goalCloseDis);
  nh->declare_parameter<double>("odomTimeoutSec", odomTimeoutSec);
  nh->declare_parameter<double>("pathTimeoutSec", pathTimeoutSec);
  nh->declare_parameter<bool>("allowStaticPath", allowStaticPath);
  nh->declare_parameter<bool>("is_real_robot", is_real_robot);
  nh->declare_parameter<bool>("sendSportCommand", sendSportCommand);
  nh->declare_parameter<bool>("enableNarrowPassageMode", enableNarrowPassageMode);
  nh->declare_parameter<int>("narrowAlignmentConfirmCycles", narrowAlignmentConfirmCycles);
  nh->declare_parameter<double>("narrowRealignTimeoutSec", narrowRealignTimeoutSec);
  nh->declare_parameter<std::string>("followerMotionModel", followerMotionModel);
  nh->declare_parameter<double>("unicycleRotateEnterDeg", unicycleRotateEnterDeg);
  nh->declare_parameter<double>("unicycleRotateExitDeg", unicycleRotateExitDeg);
  nh->declare_parameter<double>("unicycleRotateGain", unicycleRotateGain);

  nh->get_parameter("sensorOffsetX", sensorOffsetX);
  nh->get_parameter("sensorOffsetY", sensorOffsetY);
  nh->get_parameter("pubSkipNum", pubSkipNum);
  nh->get_parameter("twoWayDrive", twoWayDrive);
  nh->get_parameter("lookAheadDis", lookAheadDis);
  nh->get_parameter("yawRateGain", yawRateGain);
  nh->get_parameter("stopYawRateGain", stopYawRateGain);
  nh->get_parameter("maxYawRate", maxYawRate);
  nh->get_parameter("maxSpeed", maxSpeed);
  nh->get_parameter("maxAccel", maxAccel);
  nh->get_parameter("switchTimeThre", switchTimeThre);
  nh->get_parameter("dirDiffThre", dirDiffThre);
  nh->get_parameter("omniDirDiffThre", omniDirDiffThre);
  nh->get_parameter("noRotSpeed", noRotSpeed);
  nh->get_parameter("stopDisThre", stopDisThre);
  nh->get_parameter("slowDwnDisThre", slowDwnDisThre);
  nh->get_parameter("useInclRateToSlow", useInclRateToSlow);
  nh->get_parameter("inclRateThre", inclRateThre);
  nh->get_parameter("slowRate1", slowRate1);
  nh->get_parameter("slowRate2", slowRate2);
  nh->get_parameter("slowTime1", slowTime1);
  nh->get_parameter("slowTime2", slowTime2);
  nh->get_parameter("useInclToStop", useInclToStop);
  nh->get_parameter("inclThre", inclThre);
  nh->get_parameter("stopTime", stopTime);
  nh->get_parameter("noRotAtStop", noRotAtStop);
  nh->get_parameter("noRotAtGoal", noRotAtGoal);
  nh->get_parameter("autonomyMode", autonomyMode);
  nh->get_parameter("autonomySpeed", autonomySpeed);
  nh->get_parameter("joyToSpeedDelay", joyToSpeedDelay);
  nh->get_parameter("goalCloseDis", goalCloseDis);
  nh->get_parameter("odomTimeoutSec", odomTimeoutSec);
  nh->get_parameter("pathTimeoutSec", pathTimeoutSec);
  nh->get_parameter("allowStaticPath", allowStaticPath);
  nh->get_parameter("is_real_robot", is_real_robot);
  nh->get_parameter("sendSportCommand", sendSportCommand);
  nh->get_parameter("enableNarrowPassageMode", enableNarrowPassageMode);
  nh->get_parameter("narrowAlignmentConfirmCycles", narrowAlignmentConfirmCycles);
  nh->get_parameter("narrowRealignTimeoutSec", narrowRealignTimeoutSec);
  nh->get_parameter("followerMotionModel", followerMotionModel);
  nh->get_parameter("unicycleRotateEnterDeg", unicycleRotateEnterDeg);
  nh->get_parameter("unicycleRotateExitDeg", unicycleRotateExitDeg);
  nh->get_parameter("unicycleRotateGain", unicycleRotateGain);
  if (!local_planner::validMotionModel(followerMotionModel) ||
      !std::isfinite(unicycleRotateEnterDeg) || !std::isfinite(unicycleRotateExitDeg) ||
      !std::isfinite(unicycleRotateGain) || unicycleRotateExitDeg <= 0.0 ||
      unicycleRotateEnterDeg <= unicycleRotateExitDeg || unicycleRotateGain <= 0.0) {
    RCLCPP_FATAL(nh->get_logger(), "Invalid unicycle follower configuration");
    rclcpp::shutdown();
    return 1;
  }

  if (enableNarrowPassageMode &&
      (narrowAlignmentConfirmCycles < 1 || narrowRealignTimeoutSec <= 0.0 ||
       !std::isfinite(narrowRealignTimeoutSec))) {
    RCLCPP_FATAL(nh->get_logger(), "Invalid narrow-passage follower configuration");
    rclcpp::shutdown();
    return 1;
  }

  auto subOdom = nh->create_subscription<nav_msgs::msg::Odometry>("/state_estimation", 5, odomHandler);

  auto subPath = nh->create_subscription<nav_msgs::msg::Path>("/path", 5, pathHandler);
  auto subPathConstraint = nh->create_subscription<visibility_graph_msg::msg::LocalPathConstraint>(
      "/local_path_constraint", rclcpp::QoS(5).transient_local(), constraintHandler);

  auto subNavigationActive = nh->create_subscription<std_msgs::msg::Bool>(
      "/navigation_active", rclcpp::QoS(1).transient_local(), navigationActiveHandler);

  auto subJoystick = nh->create_subscription<sensor_msgs::msg::Joy>("/joy", 5, joystickHandler);

  auto subSpeed = nh->create_subscription<std_msgs::msg::Float32>("/speed", 5, speedHandler);

  auto subStop = nh->create_subscription<std_msgs::msg::Int8>("/stop", 5, stopHandler);

  auto pubSpeed = nh->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 5);
  auto pathFollowerStatusPub = nh->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/path_follower/status", rclcpp::QoS(1).transient_local());
  std::string lastPathFollowerStatus;
  local_planner::NarrowStateMachine narrowMachine(
      6.0 * PI / 180.0, 9.0 * PI / 180.0, 12.0 * PI / 180.0,
      narrowAlignmentConfirmCycles, narrowRealignTimeoutSec);
  local_planner::RotateInPlaceHysteresis unicycleRotateState(
      unicycleRotateEnterDeg * PI / 180.0, unicycleRotateExitDeg * PI / 180.0);

  // Only the feature gate is dynamic. Numerical tuning remains a launch-time
  // configuration so one A/B run has fixed controller parameters.
  auto motionModelCallback = nh->add_on_set_parameters_callback(
      [&](const std::vector<rclcpp::Parameter>& parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto& parameter : parameters) {
          if (parameter.get_name() == "followerMotionModel") {
            const auto requested = parameter.as_string();
            if (!local_planner::validMotionModel(requested)) {
              result.successful = false;
              result.reason = "followerMotionModel must be holonomic or unicycle";
              return result;
            }
            followerMotionModel = requested;
            unicycleRotateInPlace = false;
          }
        }
        return result;
      });
  auto pubGo2Request = nh->create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);

  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.frame_id = "vehicle";

  const auto publishStop = [&]() {
    cmd_vel.header.stamp = nh->now();
    cmd_vel.twist.linear.x = 0.0;
    cmd_vel.twist.linear.y = 0.0;
    cmd_vel.twist.angular.z = 0.0;
    pubSpeed->publish(cmd_vel);
    if (enableNarrowPassageMode && is_real_robot && sendSportCommand) {
      sport_req.StopMove(req);
      pubGo2Request->publish(req);
    }
  };
  const auto publishPathFollowerStatus = [&](const std::string& reason, bool odomFresh, bool pathFresh) {
    const std::string signature = reason + "|" + (navigationActive ? "1" : "0") + "|" +
        (odomFresh ? "1" : "0") + "|" + (pathFresh ? "1" : "0") + "|" +
        (pathInit ? "1" : "0") + "|" + std::to_string(path.poses.size()) +
        (enableNarrowPassageMode ?
         "|" + std::string(local_planner::narrowStateName(narrowMachine.state())) +
         "|" + std::to_string(activePathRevision) +
         "|" + (constraintMatchesPath ? "1" : "0") +
         "|" + std::to_string(static_cast<int>(std::round(narrowYawError * 100.0F))) +
         "|" + std::to_string(selectedNarrowDirection) +
         "|" + std::to_string(narrowStateTransitions) : "") +
        "|" + followerMotionModel + "|" +
        std::to_string(static_cast<int>(std::round(purePursuitCurvatureValue * 100.0F))) + "|" +
        (unicycleRotateInPlace ? "1" : "0") + "|" +
        std::to_string(followerPathSwitchCount) + "|" + std::to_string(followerPathRefreshCount);
    if (signature == lastPathFollowerStatus) return;
    lastPathFollowerStatus = signature;
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = nh->now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "path_follower"; status.hardware_id = "stage4d";
    status.level = reason == "TRACKING" ? diagnostic_msgs::msg::DiagnosticStatus::OK
                                          : diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = reason;
    auto add = [&status](const std::string& key, const std::string& value) {
      diagnostic_msgs::msg::KeyValue kv; kv.key = key; kv.value = value; status.values.push_back(kv);
    };
    add("motion_model", followerMotionModel);
    add("tracking_point_index", std::to_string(pathPointID));
    add("controller_state", followerMotionModel == "unicycle" ?
        (unicycleRotateInPlace ? "UNICYCLE_ROTATE_IN_PLACE" : "UNICYCLE_PURE_PURSUIT") : "HOLONOMIC");
    add("lookahead_x_body", std::to_string(lookaheadXBody));
    add("lookahead_y_body", std::to_string(lookaheadYBody));
    add("lookahead_distance", std::to_string(lookaheadDistance));
    add("pure_pursuit_curvature", std::to_string(purePursuitCurvatureValue));
    add("path_tangent_heading", std::to_string(pathTangentHeading));
    add("heading_error", std::to_string(pathHeadingError));
    add("command_v", std::to_string(vehicleSpeed));
    add("command_vy", followerMotionModel == "unicycle" ? "0.000000" : "N/A");
    add("command_wz", std::to_string(vehicleYawRate));
    add("rotate_in_place_active", unicycleRotateInPlace ? "true" : "false");
    add("wz_saturated", unicycleWzSaturated ? "true" : "false");
    add("rotate_in_place_events", std::to_string(unicycleRotateEvents));
    add("rotate_in_place_time_sec", std::to_string(unicycleRotateTimeSec));
    const auto ageNow = std::chrono::steady_clock::now();
    const double odomAge = odomReceived ? std::chrono::duration<double>(ageNow - lastOdomReceive).count() : -1.0;
    add("candidate_id", constraintReceived ?
        std::to_string(latestConstraint.candidate_group_id) + ":" +
        std::to_string(latestConstraint.candidate_rotation_id) + ":" +
        std::to_string(latestConstraint.candidate_direction) + ":" +
        (latestConstraint.candidate_narrow_mode ? "NARROW" : "NORMAL") : "N/A");
    add("current_score", constraintReceived ? std::to_string(latestConstraint.current_score) : "N/A");
    add("best_score", constraintReceived ? std::to_string(latestConstraint.best_score) : "N/A");
    add("switch_reason", constraintReceived ? latestConstraint.switch_reason : "N/A");
    add("path_switch_count", std::to_string(followerPathSwitchCount));
    add("path_refresh_count", std::to_string(followerPathRefreshCount));
    add("path_update_reason", followerPathUpdateReason);
    const double pathAge = pathReceived ? std::chrono::duration<double>(ageNow - lastPathReceive).count() : -1.0;
    add("state", reason); add("last_stop_reason", reason); add("navigation_active", navigationActive ? "true" : "false");
    add("odom_age_sec", odomAge >= 0.0 ? std::to_string(odomAge) : "N/A");
    add("path_age_sec", pathAge >= 0.0 ? std::to_string(pathAge) : "N/A");
    add("odom_fresh", odomFresh ? "true" : "false"); add("path_fresh", pathFresh ? "true" : "false");
    add("path_init", pathInit ? "true" : "false"); add("path_size", std::to_string(path.poses.size()));
    add("recovery", "new valid /path automatically re-arms tracking; node restart is not required");
    add("narrow_state", local_planner::narrowStateName(narrowMachine.state()));
    add("path_revision", std::to_string(activePathRevision));
    add("constraint_revision", constraintReceived ?
        std::to_string(latestConstraint.path_revision) : "N/A");
    add("narrow_heading_locked", narrowHeadingLocked ? "true" : "false");
    add("narrow_approach_x_vehicle", narrowHeadingLocked ? std::to_string(narrowApproachX) : "N/A");
    add("narrow_approach_y_vehicle", narrowHeadingLocked ? std::to_string(narrowApproachY) : "N/A");
    add("desired_narrow_heading", narrowHeadingLocked ? std::to_string(desiredNarrowHeading) : "N/A");
    add("locked_narrow_heading", narrowHeadingLocked ? std::to_string(lockedNarrowHeading) : "N/A");
    add("locked_narrow_yaw_error", narrowHeadingLocked ? std::to_string(narrowYawError) : "N/A");
    add("selected_narrow_direction", selectedNarrowDirection > 0 ? "FORWARD" :
        (selectedNarrowDirection < 0 ? "BACKWARD" : "UNLOCKED"));
    add("narrow_state_transition", lastNarrowStateTransition);
    add("narrow_state_transition_count", std::to_string(narrowStateTransitions));
    add("constraint_matches_path", constraintMatchesPath ? "true" : "false");
    add("yaw_error", std::to_string(narrowYawError));
    add("enter_tolerance", constraintReceived ?
        std::to_string(latestConstraint.enter_yaw_tolerance) : "N/A");
    add("continue_tolerance", constraintReceived ?
        std::to_string(latestConstraint.continue_yaw_tolerance) : "N/A");
    add("stop_tolerance", constraintReceived ?
        std::to_string(latestConstraint.stop_yaw_tolerance) : "N/A");
    add("speed_scale", constraintReceived ?
        std::to_string(latestConstraint.speed_scale) : "N/A");
    add("alignment_entries", std::to_string(alignmentEntries));
    add("alignment_successes", std::to_string(alignmentSuccesses));
    add("realign_events", std::to_string(realignEvents));
    add("alignment_timeouts", std::to_string(alignmentTimeouts));
    add("max_narrow_yaw_error", std::to_string(maxNarrowYawError));
    array.status.push_back(status); pathFollowerStatusPub->publish(array);
  };

  if (autonomyMode) {
    joySpeed = autonomySpeed / maxSpeed;

    if (joySpeed < 0) joySpeed = 0;
    else if (joySpeed > 1.0) joySpeed = 1.0;
  }

  rclcpp::Rate rate(100);
  auto lastControlTick = std::chrono::steady_clock::now();
  while (rclcpp::ok()) {
    const auto controlTick = std::chrono::steady_clock::now();
    const double elapsed = std::chrono::duration<double>(controlTick - lastControlTick).count();
    const double controlDt = std::max(0.001, std::min(elapsed, 0.05));
    lastControlTick = controlTick;
    rclcpp::spin_some(nh);

    if (!navigationActive) {
      narrowMachine.reset();
      resetNarrowHeadingLock();
      vehicleSpeed = 0.0F;
      vehicleYawRate = 0.0F;
      publishStop();
      publishPathFollowerStatus("NAVIGATION_INACTIVE", false, false);
      rate.sleep();
      continue;
    }

    const auto now = std::chrono::steady_clock::now();
    const bool odomFresh = odomReceived &&
      std::chrono::duration<double>(now - lastOdomReceive).count() <= odomTimeoutSec;
    const bool pathFresh = pathReceived && (allowStaticPath ||
      std::chrono::duration<double>(now - lastPathReceive).count() <= pathTimeoutSec);
    if (!odomFresh || !pathFresh || !pathInit || path.poses.empty()) {
      // A route must be produced again after a stale input.  This prevents
      // resuming an old trajectory when mapping or planning returns.
      pathInit = false;
      narrowMachine.reset();
      resetNarrowHeadingLock();
      vehicleSpeed = 0.0;
      vehicleYawRate = 0.0;
      publishStop();
      publishPathFollowerStatus(!odomFresh ? "WAITING_FOR_FRESH_ODOMETRY" : "WAITING_FOR_FRESH_PATH", odomFresh, pathFresh);
      RCLCPP_WARN_THROTTLE(nh->get_logger(), *nh->get_clock(), 2000,
        "Navigation stopped: waiting for fresh odometry and path");
      rate.sleep();
      continue;
    }

    // A singleton revokes the route: stop immediately, without a speed ramp.
    if (path.poses.size() <= 1) {
      vehicleSpeed = 0.0F;
      vehicleYawRate = 0.0F;
      narrowMachine.reset();
      resetNarrowHeadingLock();
      publishStop();
      publishPathFollowerStatus("PLANNER_STOP_PATH", odomFresh, pathFresh);
      rate.sleep();
      continue;
    }

    if (enableNarrowPassageMode && !constraintMatchesPath) {
      narrowMachine.reset();
      resetNarrowHeadingLock();
      vehicleSpeed = 0.0F;
      vehicleYawRate = 0.0F;
      publishStop();
      publishPathFollowerStatus("WAITING_FOR_MATCHING_CONSTRAINT", odomFresh, pathFresh);
      rate.sleep();
      continue;
    }

    if (pathInit) {
      if (!enableNarrowPassageMode) publishPathFollowerStatus("TRACKING", odomFresh, pathFresh);
      const bool narrowRequired = enableNarrowPassageMode && latestConstraint.requires_alignment;
      if (narrowRequired) {
        narrowMachine.configure(latestConstraint.enter_yaw_tolerance,
            latestConstraint.continue_yaw_tolerance,
            latestConstraint.stop_yaw_tolerance,
            narrowAlignmentConfirmCycles, narrowRealignTimeoutSec);
      }
      float vehicleXRel = cos(vehicleYawRec) * (vehicleX - vehicleXRec) 
                        + sin(vehicleYawRec) * (vehicleY - vehicleYRec);
      float vehicleYRel = -sin(vehicleYawRec) * (vehicleX - vehicleXRec) 
                        + cos(vehicleYawRec) * (vehicleY - vehicleYRec);

      int pathSize = path.poses.size();
      float endDisX = path.poses[pathSize - 1].pose.position.x - vehicleXRel;
      float endDisY = path.poses[pathSize - 1].pose.position.y - vehicleYRel;
      float endDis = sqrt(endDisX * endDisX + endDisY * endDisY);

      float disX, disY, dis;
      const float activeLookAhead = narrowRequired ? 0.15F : lookAheadDis;
      while (pathPointID < pathSize - 1) {
        disX = path.poses[pathPointID].pose.position.x - vehicleXRel;
        disY = path.poses[pathPointID].pose.position.y - vehicleYRel;
        dis = sqrt(disX * disX + disY * disY);
        if (dis < activeLookAhead) {
          pathPointID++;
        } else {
          break;
        }
      }

      disX = path.poses[pathPointID].pose.position.x - vehicleXRel;
      disY = path.poses[pathPointID].pose.position.y - vehicleYRel;
      dis = sqrt(disX * disX + disY * disY);
      float pathDir = atan2(disY, disX);

      float dirDiff = vehicleYaw - vehicleYawRec - pathDir;
      if (dirDiff > PI) dirDiff -= 2 * PI;
      else if (dirDiff < -PI) dirDiff += 2 * PI;
      if (dirDiff > PI) dirDiff -= 2 * PI;
      else if (dirDiff < -PI) dirDiff += 2 * PI;

      if (twoWayDrive && !narrowRequired && followerMotionModel == "holonomic") {
        double time = nh->now().seconds();
        if (fabs(dirDiff) > PI / 2 && navFwd && time - switchTime > switchTimeThre) {
          navFwd = false;
          switchTime = time;
        } else if (fabs(dirDiff) < PI / 2 && !navFwd && time - switchTime > switchTimeThre) {
          navFwd = true;
          switchTime = time;
        }
      }

      float joySpeed2 = maxSpeed * joySpeed;
      if (!navFwd && !narrowRequired && followerMotionModel == "holonomic") {
        dirDiff += PI;
        if (dirDiff > PI) dirDiff -= 2 * PI;
        joySpeed2 *= -1;
      }

      const float maxSpeedStep = static_cast<float>(maxAccel * controlDt);
      if (fabs(vehicleSpeed) < 2.0F * maxSpeedStep) vehicleYawRate = -stopYawRateGain * dirDiff;
      else vehicleYawRate = -yawRateGain * dirDiff;

      if (vehicleYawRate > maxYawRate * PI / 180.0) vehicleYawRate = maxYawRate * PI / 180.0;
      else if (vehicleYawRate < -maxYawRate * PI / 180.0) vehicleYawRate = -maxYawRate * PI / 180.0;

      if (joySpeed2 == 0 && !autonomyMode) {
        vehicleYawRate = maxYawRate * joyYaw * PI / 180.0;
      } else if (pathSize <= 1 || (dis < stopDisThre && noRotAtGoal)) {
        vehicleYawRate = 0;
      }

      if (pathSize <= 1) {
        joySpeed2 = 0;
      } else if (endDis / slowDwnDisThre < joySpeed) {
        joySpeed2 *= endDis / slowDwnDisThre;
      }

      float joySpeed3 = joySpeed2;
      if (odomTime < slowInitTime + slowTime1 && slowInitTime > 0) joySpeed3 *= slowRate1;
      else if (odomTime < slowInitTime + slowTime1 + slowTime2 && slowInitTime > 0) joySpeed3 *= slowRate2;

      const float targetSpeed =
        ((fabs(dirDiff) < dirDiffThre ||
          (dis < goalCloseDis && fabs(dirDiff) < omniDirDiffThre)) &&
         dis > stopDisThre) ? joySpeed3 : 0.0F;
      const float speedDelta = targetSpeed - vehicleSpeed;
      if (!narrowRequired && followerMotionModel == "holonomic") {
        vehicleSpeed += std::max(-maxSpeedStep, std::min(speedDelta, maxSpeedStep));
      }

      if (fabs(vehicleSpeed) > noRotSpeed) vehicleYawRate = 0;
      if (followerMotionModel == "unicycle" && !narrowRequired) {
        const double theta = vehicleYaw - vehicleYawRec;
        const auto bodyTarget = local_planner::targetInCurrentBody(disX, disY, theta);
        lookaheadXBody = static_cast<float>(bodyTarget.x);
        lookaheadYBody = static_cast<float>(bodyTarget.y);
        lookaheadDistance = static_cast<float>(std::hypot(bodyTarget.x, bodyTarget.y));
        const int tangentBefore = std::max(0, pathPointID - 1);
        const int tangentAfter = std::min(pathSize - 1, pathPointID + 1);
        const auto& tangentStart = path.poses[tangentBefore].pose.position;
        const auto& tangentEnd = path.poses[tangentAfter].pose.position;
        pathTangentHeading = static_cast<float>(std::atan2(
            tangentEnd.y - tangentStart.y, tangentEnd.x - tangentStart.x));
        pathHeadingError = static_cast<float>(local_planner::wrapAngle(pathTangentHeading - theta));
        const bool wasRotating = unicycleRotateState.active();
        unicycleRotateInPlace = unicycleRotateState.update(pathHeadingError) || bodyTarget.x <= 0.0;
        if (!wasRotating && unicycleRotateInPlace) ++unicycleRotateEvents;
        if (unicycleRotateInPlace) unicycleRotateTimeSec += controlDt;
        const float unicycleTarget = (!unicycleRotateInPlace && pathSize > 1 &&
            endDis > stopDisThre) ? std::max(0.0F, joySpeed3) : 0.0F;
        vehicleSpeed += std::max(-maxSpeedStep, std::min(unicycleTarget - vehicleSpeed, maxSpeedStep));
        vehicleSpeed = std::max(0.0F, vehicleSpeed);
        const auto unicycleCommand = local_planner::makeUnicycleCommand(
            vehicleSpeed, bodyTarget, pathHeadingError, unicycleRotateInPlace,
            unicycleRotateGain, maxYawRate * PI / 180.0);
        purePursuitCurvatureValue = static_cast<float>(local_planner::purePursuitCurvature(bodyTarget));
        vehicleYawRate = static_cast<float>(unicycleCommand.wz);
        unicycleWzSaturated = unicycleCommand.saturated;
      }


      if (narrowRequired) {
        // The planner supplies a fixed pre-entry tangent. At narrow-section
        // entry choose psi or psi+pi once; no rolling lookahead may replace
        // that target while this section is active.
        if (!narrowHeadingLocked) {
          desiredNarrowHeading = local_planner::wrapAngle(
              pathSourceYaw + latestConstraint.narrow_approach_heading);
          const auto lock = local_planner::chooseNarrowHeading(
              desiredNarrowHeading, vehicleYaw);
          lockedNarrowHeading = lock.heading;
          selectedNarrowDirection = lock.direction;
          narrowApproachX = latestConstraint.narrow_approach_x;
          narrowApproachY = latestConstraint.narrow_approach_y;
          narrowHeadingLocked = true;
          lastNarrowStateTransition = "LOCKED_HEADING";
        }
        narrowYawError = static_cast<float>(local_planner::wrapAngle(
            vehicleYaw - lockedNarrowHeading));
        const auto before = narrowMachine.state();
        const auto after = narrowMachine.step(true, true, narrowYawError, controlDt);
        if (after != before) {
          lastNarrowStateTransition = std::string(local_planner::narrowStateName(before)) +
              "->" + local_planner::narrowStateName(after);
          ++narrowStateTransitions;
          if (after == local_planner::NarrowState::NARROW_APPROACH) {
            ++alignmentEntries;
            if (before == local_planner::NarrowState::NARROW_TRAVERSE) ++realignEvents;
          }
          if (after == local_planner::NarrowState::NARROW_TRAVERSE) ++alignmentSuccesses;
          if (after == local_planner::NarrowState::REALIGN_TIMEOUT) ++alignmentTimeouts;
        }
        if (after == local_planner::NarrowState::NARROW_TRAVERSE) {
          maxNarrowYawError = std::max(maxNarrowYawError, std::abs(narrowYawError));
        }
        // No crab motion. twoWayDrive is bypassed and the selected direction
        // remains fixed for the entire narrow section.
        const float narrowTarget = (pathSize > 1 && endDis > stopDisThre &&
                                    narrowMachine.permitsForwardMotion(narrowYawError)) ?
            std::max(0.0F, static_cast<float>(maxSpeed * joySpeed *
                                             latestConstraint.speed_scale)) : 0.0F;
        if (narrowTarget <= 0.0F) {
          vehicleSpeed = 0.0F;
        } else {
          vehicleSpeed += std::max(-maxSpeedStep,
              std::min(narrowTarget - vehicleSpeed, maxSpeedStep));
          vehicleSpeed = std::max(0.0F, vehicleSpeed);
        }
        const bool mayRotate = after != local_planner::NarrowState::REALIGN_TIMEOUT &&
            std::abs(narrowYawError) <= latestConstraint.recovery_yaw_limit &&
            pathSize > 1 && endDis > stopDisThre;
        vehicleYawRate = mayRotate ? static_cast<float>(std::max(
            -maxYawRate * PI / 180.0,
            std::min(-stopYawRateGain * narrowYawError, maxYawRate * PI / 180.0))) : 0.0F;
        publishPathFollowerStatus(local_planner::narrowStateName(after), odomFresh, pathFresh);
      } else if (enableNarrowPassageMode) {
        narrowMachine.step(false, true, 0.0, controlDt);
        resetNarrowHeadingLock();
        publishPathFollowerStatus("TRACKING", odomFresh, pathFresh);
      }

      if (odomTime < stopInitTime + stopTime && stopInitTime > 0) {
        vehicleSpeed = 0;
        vehicleYawRate = 0;
      }

      if ((safetyStop & 1) > 0 && vehicleSpeed > 0) vehicleSpeed = 0;
      if ((safetyStop & 2) > 0 && vehicleSpeed < 0) vehicleSpeed = 0;
      if ((safetyStop & 4) > 0 && vehicleYawRate > 0) vehicleYawRate = 0;
      if ((safetyStop & 8) > 0 && vehicleYawRate < 0) vehicleYawRate = 0;
      //if ((safetyStop & 1) > 0 || (safetyStop & 2) > 0) vehicleYawRate = 0; //No rotation at forward/backward stop

      pubSkipCount--;
      if (pubSkipCount < 0) {
        cmd_vel.header.stamp = nh->now();
        if (fabs(vehicleSpeed) <= std::max(maxSpeedStep, 1.0e-4F)) {
          cmd_vel.twist.linear.x = 0;
          cmd_vel.twist.linear.y = 0;
        } else {
          cmd_vel.twist.linear.x = cos(dirDiff) * vehicleSpeed;
          cmd_vel.twist.linear.y = -sin(dirDiff) * vehicleSpeed;
        }
        cmd_vel.twist.angular.z = vehicleYawRate;
        if (followerMotionModel == "unicycle" && !narrowRequired) {
          cmd_vel.twist.linear.x = vehicleSpeed;
          cmd_vel.twist.linear.y = 0.0;
        }
        if (narrowRequired) {
          cmd_vel.twist.linear.x = narrowMachine.permitsForwardMotion(narrowYawError) ?
              static_cast<double>(selectedNarrowDirection) * std::max(0.0F, vehicleSpeed) : 0.0F;
          cmd_vel.twist.linear.y = 0.0;
        }
        if (manualMode && !narrowRequired && followerMotionModel == "holonomic") {
          cmd_vel.twist.linear.x = maxSpeed * joyManualFwd;
          cmd_vel.twist.linear.y = maxSpeed / 2.0 * joyManualLeft;
          cmd_vel.twist.angular.z = maxYawRate * PI / 180.0 * joyManualYaw;
        }

        pubSpeed->publish(cmd_vel);

        pubSkipCount = pubSkipNum;

        if (is_real_robot && sendSportCommand)
        {
          if (cmd_vel.twist.linear.x == 0 && cmd_vel.twist.linear.y == 0 && cmd_vel.twist.angular.z == 0){
          	sport_req.StopMove(req);
          }
          else{
               sport_req.Move(req, cmd_vel.twist.linear.x, cmd_vel.twist.linear.y, cmd_vel.twist.angular.z);
          }
          pubGo2Request->publish(req);
        }
      }
    }

    rate.sleep();
  }

  return 0;
}
