#include "far_planner/vehicle_height_estimator.h"
#include <iostream>
#include <limits>
#include <stdexcept>

static void Check(bool result) {
  if (!result) throw std::runtime_error("vehicle height regression");
}

int main() {
  VehicleHeightEstimator height(0.34f);
  Check(!height.initialized() && height.height() == 0.34f);
  Check(!height.Update(std::vector<float>(15, 0.33f), 1.0));
  Check(height.Update(std::vector<float>(30, 0.33f), 1.0));
  Check(std::abs(height.height() - 0.33f) < 1e-5f);
  Check(!height.Update({std::numeric_limits<float>::quiet_NaN(), 4.0f, -1.0f}, 2.0));
  Check(height.sample_count() == 0 && std::abs(height.height() - 0.33f) < 1e-5f);
  Check(!height.Update(std::vector<float>(30, 0.02f), 2.0)); // tabletop, not ground
  Check(!height.Update(std::vector<float>(30, 0.50f), 0.5)); // old epoch must not update
  Check(height.Update(std::vector<float>(30, 0.50f), 1.1));
  Check(std::abs(height.height() - 0.34f) < 1e-5f); // bounded 0.10 m/s
  Check(!height.Update(std::vector<float>(30, 0.34f), std::numeric_limits<double>::infinity()));
  std::vector<float> ground(80, 0.34f);
  ground.insert(ground.end(), 20, 0.20f);
  VehicleHeightEstimator robust(0.34f);
  Check(robust.Update(ground, 1.0) && std::abs(robust.height() - 0.34f) < 1e-5f);
  VehicleHeightEstimator fresh(0.34f);
  Check(!fresh.initialized());
  std::cout << "vehicle_height_estimator_test: PASS\n";
}
