#pragma once

#include <algorithm>
#include <cmath>
#include <string>

#include "local_planner/narrow_passage.hpp"

namespace local_planner {

struct BodyTarget { double x = 0.0; double y = 0.0; };

inline BodyTarget targetInCurrentBody(double dx_ref, double dy_ref, double theta) {
  return {std::cos(theta) * dx_ref + std::sin(theta) * dy_ref,
          -std::sin(theta) * dx_ref + std::cos(theta) * dy_ref};
}

inline double purePursuitCurvature(const BodyTarget& target, double epsilon = 1.0e-6) {
  const double distance_sq = target.x * target.x + target.y * target.y;
  return distance_sq <= epsilon ? 0.0 : 2.0 * target.y / distance_sq;
}

inline double clampYawRate(double yaw_rate, double max_abs_yaw_rate) {
  return std::max(-max_abs_yaw_rate, std::min(yaw_rate, max_abs_yaw_rate));
}

inline double localPathTangent(const BodyTarget& previous, const BodyTarget& next) {
  return std::atan2(next.y - previous.y, next.x - previous.x);
}

class RotateInPlaceHysteresis {
 public:
  RotateInPlaceHysteresis(double enter, double exit) : enter_(enter), exit_(exit) {}
  bool update(double heading_error) {
    const double magnitude = std::abs(heading_error);
    if (!active_ && magnitude > enter_) active_ = true;
    else if (active_ && magnitude < exit_) active_ = false;
    return active_;
  }
  bool active() const { return active_; }
 private:
  double enter_;
  double exit_;
  bool active_ = false;
};

struct UnicycleCommand {
  double vx = 0.0;
  double vy = 0.0;
  double wz = 0.0;
  bool rotate_in_place = false;
  bool saturated = false;
};

inline UnicycleCommand makeUnicycleCommand(double target_speed, const BodyTarget& target,
                                            double heading_error, bool rotate_in_place,
                                            double rotate_gain, double max_yaw_rate) {
  UnicycleCommand command;
  command.rotate_in_place = rotate_in_place || target.x <= 0.0;
  const double requested = command.rotate_in_place ? rotate_gain * heading_error
      : target_speed * purePursuitCurvature(target);
  command.wz = clampYawRate(requested, max_yaw_rate);
  command.saturated = std::abs(command.wz - requested) > 1.0e-9;
  command.vx = command.rotate_in_place ? 0.0 : std::max(0.0, target_speed);
  command.vy = 0.0;
  return command;
}

inline bool validMotionModel(const std::string& model) {
  return model == "holonomic" || model == "unicycle";
}

}  // namespace local_planner
