#pragma once
#include <cmath>
#include <limits>

// Pure decision layer: never grants collision clearance. ROUTE still requires
// a newly validated graph path and local-planner obstacle checks.
class GoalApproach {
 public:
  enum class Decision { ROUTE, CORRECTION_WAIT, CORRECTION_REJECTED,
                        ADJUSTED_WAIT, CONTINUE_ROUTE, ARRIVAL_HOLD };
  double max_adjustment = .8;
  double stable_s = .4;
  double stable_distance = .05;
  double follower_stop_distance = .2;
  double continuation_margin = .01;

  void Reset() { valid_ = false; near_seen_ = false; ready_at_ = 0.; }
  Decision Update(double now, double x, double y, double ax, double ay,
                  double bx, double by, double tolerance) {
    if (!std::isfinite(now) || !std::isfinite(x) || !std::isfinite(y) ||
        !std::isfinite(ax) || !std::isfinite(ay) || !std::isfinite(bx) ||
        !std::isfinite(by) || !std::isfinite(tolerance) || tolerance <= 0.) {
      Reset(); return Decision::CORRECTION_REJECTED;
    }
    if (std::hypot(x-ax,y-ay) <= tolerance) return Decision::ARRIVAL_HOLD;
    if (std::hypot(bx-ax,by-ay) > max_adjustment) {
      Reset(); return Decision::CORRECTION_REJECTED;
    }
    if (!valid_ || now < last_time_ || std::hypot(bx-anchor_x_,by-anchor_y_) > stable_distance) {
      valid_=true; anchor_x_=bx; anchor_y_=by; ready_at_=now+stable_s;
    }
    last_time_=now;
    if (now < ready_at_) return Decision::CORRECTION_WAIT;
    const double distance=std::hypot(x-bx,y-by);
    if (distance <= tolerance) {
      // Stop once before re-evaluating the remaining approach. Merely observing
      // the same point/revision cannot authorize a terminal success.
      if (!near_seen_) { near_seen_=true; ready_at_=now+stable_s; return Decision::ADJUSTED_WAIT; }
      // The follower stops at its own radius. Do not repeatedly authorize an
      // already exhausted segment; wait for a genuinely usable route instead.
      if (distance <= follower_stop_distance+continuation_margin)
        return Decision::ADJUSTED_WAIT;
      return Decision::CONTINUE_ROUTE;
    }
    return Decision::ROUTE;
  }
 private:
  bool valid_=false, near_seen_=false;
  double anchor_x_=0., anchor_y_=0., ready_at_=0., last_time_=0.;
};
