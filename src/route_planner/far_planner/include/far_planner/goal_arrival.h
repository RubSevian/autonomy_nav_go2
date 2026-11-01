#pragma once
#include <cmath>

namespace FARGoalArrival {
enum class State { APPROACHING, REQUESTED_REACHED, ADJUSTED_REACHED };

// One current vehicle pose and one XY tolerance for both planning and status.
// Graph-node positions and terrain height are not arrival measurements.
inline State Evaluate(double x, double y, double requested_x, double requested_y,
                      double adjusted_x, double adjusted_y, double tolerance) {
  if (!std::isfinite(x) || !std::isfinite(y) ||
      !std::isfinite(requested_x) || !std::isfinite(requested_y) ||
      !std::isfinite(adjusted_x) || !std::isfinite(adjusted_y) ||
      !std::isfinite(tolerance) || tolerance <= 0.) return State::APPROACHING;
  if (std::hypot(x-requested_x, y-requested_y) <= tolerance)
    return State::REQUESTED_REACHED;
  // Numerical identity threshold, not an additional arrival allowance.
  if (std::hypot(adjusted_x-requested_x, adjusted_y-requested_y) > 1.e-4 &&
      std::hypot(x-adjusted_x, y-adjusted_y) <= tolerance)
    return State::ADJUSTED_REACHED;
  return State::APPROACHING;
}
}  // namespace FARGoalArrival
