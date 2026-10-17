#include "IMU_Processing.hpp"
#include <cassert>

namespace {
sensor_msgs::msg::Imu::ConstSharedPtr sample(int index, double x) {
  auto imu = std::make_shared<sensor_msgs::msg::Imu>();
  imu->header.stamp.sec = 10;
  imu->header.stamp.nanosec = index * 4000000;
  imu->linear_acceleration.x = x;
  imu->linear_acceleration.z = 9.81;
  return imu;
}
void near(double actual, double expected) { assert(std::abs(actual - expected) < 1e-10); }
}

int main() {
  // Exercise the real processing class without a ROS node or DDS context.
  ImuProcess process(100);
  MeasureGroup batch;
  batch.lidar->resize(1);
  batch.lidar->points[0].x = 7;
  PointCloudXYZI::Ptr output(new PointCloudXYZI());

  // Deliberately retain overlapping batches: even a repeated caller must not
  // turn 60 unique observations into the 120 counted by the previous code.
  for (int group = 0; group < 3; ++group) {
    for (int j = 1; j <= 20; ++j) batch.imu.push_back(sample(group * 20 + j, group));
    process.Process(batch, output);
    assert(process.imu_need_init_ && output->empty());
  }
  near(process.mean_acc.x(), 1.0);
  process.Process(batch, output);
  assert(process.imu_need_init_);
  near(process.mean_acc.x(), 1.0);

  batch.imu.clear();
  // An old sample with a deliberately different value must be ignored.
  batch.imu.push_back(sample(1, 1000));
  for (int i = 61; i <= 99; ++i) batch.imu.push_back(sample(i, 1));
  process.Process(batch, output);
  assert(process.imu_need_init_ && output->empty());
  batch.imu.clear();
  batch.imu.push_back(sample(100, 1));
  process.Process(batch, output);
  assert(!process.imu_need_init_ && !process.gravity_align_);
  assert(output->size() == 1);
  near(process.mean_acc.x(), 1.0);
  near(process.mean_acc.z(), 9.81);

  // Working IMU propagation consumes imu_deque, not the initialization batch.
  batch.imu.clear();
  batch.lidar->points[0].x = 9;
  process.Process(batch, output);
  assert(process.gravity_align_);
  near(output->points[0].x, 9);

  process.Reset();
  process.Process(batch, output);
  assert(process.imu_need_init_ && process.b_first_frame_ && !process.gravity_align_);
  assert(output->empty());
  // Reset clears the timestamp watermark too.
  for (int i = 1; i <= 100; ++i) batch.imu.push_back(sample(i, 2));
  process.Process(batch, output);
  assert(!process.imu_need_init_);
  near(process.mean_acc.x(), 2);

  // Batch boundaries must not change the result.
  ImuProcess separate(100);
  for (int i = 1; i <= 100; ++i) {
    batch.imu.clear();
    batch.imu.push_back(sample(i, i));
    separate.Process(batch, output);
    assert(separate.imu_need_init_ == (i < 100));
  }
  near(separate.mean_acc.x(), 50.5);
  bool rejected = false;
  try { ImuProcess invalid(0); } catch (const std::invalid_argument&) { rejected = true; }
  assert(rejected);
  std::cout << "PASS: unique samples, threshold, means, empty working batch, reset, batching\n";
}
