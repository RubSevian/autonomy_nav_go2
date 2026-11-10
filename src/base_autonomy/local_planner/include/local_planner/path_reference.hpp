#pragma once
#include <array>
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace local_planner {
struct PlanarPose { double x, y, yaw; };

// Express a source-frame point in the fixed reference frame of the follower.
inline std::array<double, 2> reframePoint(double x, double y,
                                         const PlanarPose& source,
                                         const PlanarPose& reference) {
  const double dx = source.x + std::cos(source.yaw)*x - std::sin(source.yaw)*y - reference.x;
  const double dy = source.y + std::sin(source.yaw)*x + std::cos(source.yaw)*y - reference.y;
  return {std::cos(reference.yaw)*dx + std::sin(reference.yaw)*dy,
          -std::sin(reference.yaw)*dx + std::cos(reference.yaw)*dy};
}

inline bool measurementFresh(int64_t stamp, int64_t now, double timeout) {
  return stamp > 0 && now >= stamp && double(now-stamp)*1e-9 <= timeout;
}

inline double applySpeedDeadband(double speed, double deadband) {
  return std::abs(speed) <= deadband ? 0. : speed;
}

// Floor applies only to an existing approach request, before safety slowdown.
inline double approachSpeed(double requested, double end_distance, double slow_distance, double minimum) {
  if (requested == 0.) return 0.;
  const double magnitude=std::abs(requested);
  const double slowed=magnitude*std::min(1.,std::max(0.,end_distance/slow_distance));
  return std::copysign(std::min(magnitude,std::max(minimum,slowed)),requested);
}

// Full normalized quaternion rotation of sensor-to-vehicle translation.
inline std::array<double, 3> vehicleOffset(double qx, double qy, double qz, double qw,
                                          double offset_x, double offset_y) {
  const double n = qx*qx+qy*qy+qz*qz+qw*qw;
  return {-(1-2*(qy*qy+qz*qz)/n)*offset_x - 2*(qx*qy-qz*qw)/n*offset_y,
          -2*(qx*qy+qz*qw)/n*offset_x - (1-2*(qx*qx+qz*qz)/n)*offset_y,
          -2*(qx*qz-qy*qw)/n*offset_x - 2*(qy*qz+qx*qw)/n*offset_y};
}
}  // namespace local_planner
