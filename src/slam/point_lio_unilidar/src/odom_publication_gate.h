#pragma once

#include <cmath>
#include <cstdint>
#include <stdexcept>

// Limits only outgoing odometry/TF traffic. The EKF still evaluates every
// input sample. Use steady wall time so a queued sensor burst cannot publish
// a burst of old poses merely because its source timestamps are spaced out.
class OdomPublicationGate {
 public:
  explicit OdomPublicationGate(double max_rate_hz) {
    if (!std::isfinite(max_rate_hz) || max_rate_hz < 0.0 || max_rate_hz > 1000.0) {
      throw std::invalid_argument("Invalid odometry publication rate");
    }
    period_ns_ = max_rate_hz == 0.0 ? 0 : static_cast<int64_t>(std::ceil(1e9 / max_rate_hz));
  }

  bool ShouldPublish(int64_t now_ns) {
    if (period_ns_ == 0) return true;
    if (last_publish_ns_ >= 0 && now_ns - last_publish_ns_ < period_ns_) return false;
    last_publish_ns_ = now_ns;
    return true;
  }

 private:
  int64_t period_ns_ = 0;
  int64_t last_publish_ns_ = -1;
};
