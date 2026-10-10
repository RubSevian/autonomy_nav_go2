#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

// Height of the odometry origin above locally observed ground, not robot size.
// Only terrain already classified as free may supply samples. The nominal
// height bounds adaptation so a tabletop cannot silently become the floor.
class VehicleHeightEstimator {
 public:
  explicit VehicleHeightEstimator(float nominal) : nominal_(nominal), height_(nominal) {}

  bool Update(std::vector<float> samples, double stamp) {
    samples.erase(std::remove_if(samples.begin(), samples.end(), [this](float h) {
      return !std::isfinite(h) || h < 0.10f || std::abs(h - nominal_) > 0.20f;
    }), samples.end());
    sample_count_ = samples.size();
    if (samples.size() < 16 || !std::isfinite(stamp)) return false;
    std::sort(samples.begin(), samples.end());
    // Lower ground-height quantile (upper odometry-to-ground distance quantile)
    // is less affected by isolated low obstacles labelled free by terrainAnalysis.
    const float candidate = samples[(samples.size() - 1) * 4 / 5];
    if (samples[(samples.size() - 1) * 4 / 5] - samples[(samples.size() - 1) / 5] > 0.15f)
      return false;
    if (initialized_ && stamp <= last_stamp_) return false;
    const float limit = initialized_ ? static_cast<float>(0.10 * std::min(stamp - last_stamp_, 1.0)) : 0.20f;
    height_ += std::max(-limit, std::min(candidate - height_, limit));
    last_stamp_ = stamp;
    initialized_ = true;
    return true;
  }

  float height() const { return height_; }
  bool initialized() const { return initialized_; }
  std::size_t sample_count() const { return sample_count_; }

 private:
  float nominal_, height_;
  double last_stamp_ = 0.0;
  bool initialized_ = false;
  std::size_t sample_count_ = 0;
};
