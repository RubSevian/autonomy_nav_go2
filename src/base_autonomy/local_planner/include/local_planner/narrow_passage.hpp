#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

namespace local_planner {

constexpr double kPi = 3.14159265358979323846;

struct Point2D {
  double x = 0.0;
  double y = 0.0;
};

struct NarrowFootprint {
  double length = 0.62;
  double width = 0.40;
  double longitudinal_margin = 0.0;
  double lateral_margin = 0.0;
};

enum class NarrowState {
  NORMAL,
  NARROW_APPROACH,
  NARROW_TRAVERSE,
  REALIGN_TIMEOUT,
};

inline double wrapAngle(double angle) {
  while (angle > kPi) angle -= 2.0 * kPi;
  while (angle < -kPi) angle += 2.0 * kPi;
  return angle;
}

// The lane has two physically equivalent body headings.  Select once at
// narrow-section entry; callers retain the returned value until the section
// ends, rather than reselecting from a moving lookahead point every cycle.
struct NarrowHeadingLock {
  double heading = 0.0;
  int direction = 1;  // +1: body forward along lane, -1: body backward.
};

inline NarrowHeadingLock chooseNarrowHeading(double lane_heading,
                                             double current_heading) {
  const double forward = wrapAngle(lane_heading);
  const double backward = wrapAngle(lane_heading + kPi);
  if (std::abs(wrapAngle(current_heading - backward)) <
      std::abs(wrapAngle(current_heading - forward))) {
    return {backward, -1};
  }
  return {forward, 1};
}

inline bool footprintCollides(const Point2D& pose, double heading,
                              const Point2D& obstacle,
                              const NarrowFootprint& footprint) {
  const double dx_world = obstacle.x - pose.x;
  const double dy_world = obstacle.y - pose.y;
  const double c = std::cos(heading);
  const double s = std::sin(heading);
  const double dx = c * dx_world + s * dy_world;
  const double dy = -s * dx_world + c * dy_world;
  return std::abs(dx) <= footprint.length * 0.5 + footprint.longitudinal_margin &&
         std::abs(dy) <= footprint.width * 0.5 + footprint.lateral_margin;
}

inline double lateralClearance(const Point2D& pose, double heading,
                               const Point2D& obstacle,
                               const NarrowFootprint& footprint) {
  const double dx_world = obstacle.x - pose.x;
  const double dy_world = obstacle.y - pose.y;
  const double c = std::cos(heading);
  const double s = std::sin(heading);
  const double dy = -s * dx_world + c * dy_world;
  return std::abs(dy) - (footprint.width * 0.5 + footprint.lateral_margin);
}

inline double turnRadius(const NarrowFootprint& footprint) {
  const double length = footprint.length + 2.0 * footprint.longitudinal_margin;
  const double width = footprint.width + 2.0 * footprint.lateral_margin;
  return std::sqrt((length * 0.5) * (length * 0.5) +
                   (width * 0.5) * (width * 0.5));
}

// Conservative axis-aligned envelope of a rectangle rotated within +/- yaw_limit.
// All dimensions stay in physical metres, independent of pathScale.
inline NarrowFootprint yawInflatedFootprint(const NarrowFootprint& physical,
                                            double yaw_limit) {
  const double limit = std::max(0.0, std::min(std::abs(yaw_limit), kPi / 4.0));
  const double half_x = physical.length * 0.5 + physical.longitudinal_margin;
  const double half_y = physical.width * 0.5 + physical.lateral_margin;
  const auto maximum_projection = [limit](double forward, double side) {
    const double peak = std::atan2(side, forward);
    return limit >= peak ? std::hypot(forward, side) :
           forward * std::cos(limit) + side * std::sin(limit);
  };
  NarrowFootprint envelope = physical;
  envelope.longitudinal_margin =
      maximum_projection(half_x, half_y) - physical.length * 0.5;
  envelope.lateral_margin =
      maximum_projection(half_y, half_x) - physical.width * 0.5;
  return envelope;
}

struct StampKey {
  std::int32_t sec = 0;
  std::uint32_t nanosec = 0;
};

inline bool matchingConstraint(const StampKey& path_stamp,
                               const StampKey& constraint_stamp,
                               std::uint64_t revision,
                               std::uint64_t previous_revision) {
  return revision > previous_revision &&
         path_stamp.sec == constraint_stamp.sec &&
         path_stamp.nanosec == constraint_stamp.nanosec;
}

inline bool computeTangents(const std::vector<Point2D>& points,
                            std::vector<double>* headings) {
  headings->assign(points.size(), 0.0);
  if (points.size() < 2) return false;
  bool any_valid = false;
  for (size_t index = 0; index < points.size(); ++index) {
    Point2D delta;
    if (index == 0) {
      delta = {points[1].x - points[0].x, points[1].y - points[0].y};
    } else if (index + 1 == points.size()) {
      delta = {points[index].x - points[index - 1].x,
               points[index].y - points[index - 1].y};
    } else {
      delta = {points[index + 1].x - points[index - 1].x,
               points[index + 1].y - points[index - 1].y};
    }
    if (std::hypot(delta.x, delta.y) <= 1.0e-6) return false;
    (*headings)[index] = std::atan2(delta.y, delta.x);
    any_valid = true;
  }
  return any_valid;
}

inline bool curvatureWithinLimit(const std::vector<double>& headings,
                                 double max_delta_rad) {
  for (size_t index = 1; index < headings.size(); ++index) {
    if (std::abs(wrapAngle(headings[index] - headings[index - 1])) >
        max_delta_rad) {
      return false;
    }
  }
  return true;
}

inline const char* narrowStateName(NarrowState state) {
  switch (state) {
    case NarrowState::NORMAL: return "NORMAL";
    case NarrowState::NARROW_APPROACH: return "NARROW_APPROACH";
    case NarrowState::NARROW_TRAVERSE: return "NARROW_TRAVERSE";
    case NarrowState::REALIGN_TIMEOUT: return "REALIGN_TIMEOUT";
  }
  return "UNKNOWN";
}

inline bool curvatureRateWithinLimit(const std::vector<Point2D>& points,
                                    const std::vector<double>& headings,
                                    double max_rad_per_metre) {
  if (points.size() != headings.size() || points.size() < 2 ||
      max_rad_per_metre <= 0.0) return false;
  for (size_t index = 1; index < points.size(); ++index) {
    const double distance = std::hypot(points[index].x - points[index - 1].x,
                                       points[index].y - points[index - 1].y);
    if (distance <= 1.0e-6 ||
        std::abs(wrapAngle(headings[index] - headings[index - 1])) /
            distance > max_rad_per_metre) return false;
  }
  return true;
}

struct SweptCheckResult {
  bool free = false;
  double min_lateral_clearance = -1.0;
  bool has_blocking_point = false;
  Point2D blocking_point;
};

inline SweptCheckResult checkSweptPath(
    const std::vector<Point2D>& samples,
    const std::vector<double>& headings,
    const std::vector<Point2D>& obstacles,
    const NarrowFootprint& physical,
    double recovery_yaw_limit) {
  SweptCheckResult result;
  if (samples.size() < 2 || samples.size() != headings.size()) return result;
  const auto envelope = yawInflatedFootprint(physical, recovery_yaw_limit);
  double minimum = std::numeric_limits<double>::infinity();
  double nearest_blocking_distance = std::numeric_limits<double>::infinity();
  bool collision_found = false;
  for (size_t index = 0; index < samples.size(); ++index) {
    const double c = std::cos(headings[index]), s = std::sin(headings[index]);
    for (const auto& obstacle : obstacles) {
      const double dx = c * (obstacle.x - samples[index].x) +
                        s * (obstacle.y - samples[index].y);
      if (std::abs(dx) <= envelope.length * 0.5 + envelope.longitudinal_margin) {
        const double clearance = lateralClearance(
            samples[index], headings[index], obstacle, envelope);
        minimum = std::min(minimum, clearance);
        if (footprintCollides(samples[index], headings[index], obstacle, envelope)) {
          collision_found = true;
          const double distance = std::hypot(
              obstacle.x - samples[index].x, obstacle.y - samples[index].y);
          if (distance < nearest_blocking_distance) {
            nearest_blocking_distance = distance;
            result.has_blocking_point = true;
            result.blocking_point = obstacle;
          }
        }
      }
    }
  }
  result.free = !collision_found;
  result.min_lateral_clearance = std::isfinite(minimum) ? minimum : -1.0;
  return result;
}

class NarrowStateMachine {
 public:
  NarrowStateMachine(double enter_tolerance, double continue_tolerance,
                     double stop_tolerance, int confirmation_cycles,
                     double realign_timeout)
      : enter_tolerance_(enter_tolerance),
        continue_tolerance_(continue_tolerance),
        stop_tolerance_(stop_tolerance),
        confirmation_cycles_(std::max(1, confirmation_cycles)),
        realign_timeout_(realign_timeout) {}

  void configure(double enter_tolerance, double continue_tolerance,
                 double stop_tolerance, int confirmation_cycles,
                 double realign_timeout) {
    enter_tolerance_ = enter_tolerance;
    continue_tolerance_ = continue_tolerance;
    stop_tolerance_ = stop_tolerance;
    confirmation_cycles_ = std::max(1, confirmation_cycles);
    realign_timeout_ = realign_timeout;
  }

  NarrowState step(bool requires_alignment, bool constraint_matches,
                   double yaw_error, double dt) {
    if (!requires_alignment || !constraint_matches) {
      state_ = NarrowState::NORMAL;
      aligned_cycles_ = 0;
      realign_elapsed_ = 0.0;
      return state_;
    }
    const double error = std::abs(yaw_error);
    if (state_ == NarrowState::NORMAL) state_ = NarrowState::NARROW_APPROACH;
    if (state_ == NarrowState::NARROW_TRAVERSE && error > stop_tolerance_) {
      state_ = NarrowState::NARROW_APPROACH;
      aligned_cycles_ = 0;
      realign_elapsed_ = 0.0;
    }
    if (state_ == NarrowState::REALIGN_TIMEOUT && error <= enter_tolerance_) {
      state_ = NarrowState::NARROW_APPROACH;
      aligned_cycles_ = 0;
      realign_elapsed_ = 0.0;
    }
    if (state_ == NarrowState::NARROW_APPROACH) {
      if (error <= enter_tolerance_) {
        ++aligned_cycles_;
        if (aligned_cycles_ >= confirmation_cycles_) {
          state_ = NarrowState::NARROW_TRAVERSE;
          realign_elapsed_ = 0.0;
        }
      } else {
        aligned_cycles_ = 0;
        realign_elapsed_ += std::max(0.0, dt);
        if (realign_elapsed_ >= realign_timeout_) {
          state_ = NarrowState::REALIGN_TIMEOUT;
        }
      }
    }
    return state_;
  }

  bool permitsForwardMotion(double yaw_error) const {
    return state_ == NarrowState::NARROW_TRAVERSE &&
           std::abs(yaw_error) <= continue_tolerance_;
  }

  NarrowState state() const { return state_; }

  void reset() {
    state_ = NarrowState::NORMAL;
    aligned_cycles_ = 0;
    realign_elapsed_ = 0.0;
  }

 private:
  double enter_tolerance_;
  double continue_tolerance_;
  double stop_tolerance_;
  int confirmation_cycles_;
  double realign_timeout_;
  int aligned_cycles_ = 0;
  double realign_elapsed_ = 0.0;
  NarrowState state_ = NarrowState::NORMAL;
};

}  // namespace local_planner
