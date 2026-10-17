#include <cmath>
#include <math.h>
#include <deque>
#include <optional>
#include <stdexcept>
#include <mutex>
#include <thread>
#include <fstream>
#include <csignal>
#include <rclcpp/rclcpp.hpp>
#include <so3_math.h>
#include <Eigen/Eigen>
#include <common_lib.h>
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <condition_variable>
#include <nav_msgs/msg/odometry.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <tf2_ros/transform_broadcaster.h>
// #include <eigen_conversions/eigen_msg.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/vector3.hpp>


class ImuProcess
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit ImuProcess(int init_samples = 100);
  ~ImuProcess();
  
  void Reset();
  void Reset(double start_timestamp, const sensor_msgs::msg::Imu::ConstSharedPtr &lastimu);
  void Process(const MeasureGroup &meas, PointCloudXYZI::Ptr pcl_un_);
  void Set_init(Eigen::Vector3d &tmp_gravity, Eigen::Matrix3d &rot);

  ofstream fout_imu;
 
  int    lidar_type;
  bool   imu_en;
  V3D mean_acc, gravity_;
  bool   imu_need_init_ = true;
  bool   b_first_frame_ = true;
  bool   gravity_align_ = false;

 private:
  void IMU_init(const MeasureGroup &meas);
  V3D mean_gyr;
  const int init_samples_;
  int init_iter_num = 0;
  std::optional<int64_t> last_init_stamp_ns_;
};

ImuProcess::ImuProcess(int init_samples) : init_samples_(init_samples)
{
  if (init_samples < 1) throw std::invalid_argument("mapping.imu_init_samples must be positive");
  imu_en = true;
  // mean_acc      = V3D(0, 0, -1.0);
  mean_acc      = V3D(0, 0, 0.0);
  mean_gyr      = V3D(0, 0, 0);
}

ImuProcess::~ImuProcess() {}

void ImuProcess::Reset() 
{
  printf("Reset ImuProcess\n");
  // mean_acc      = V3D(0, 0, -1.0);
  mean_acc      = V3D(0, 0, 0.0);
  mean_gyr      = V3D(0, 0, 0);
  imu_need_init_    = true;
  init_iter_num     = 0;
  last_init_stamp_ns_.reset();
  b_first_frame_ = true;
  gravity_align_ = false;
}

void ImuProcess::IMU_init(const MeasureGroup &meas)
{
  /** 1. initializing the gravity, gyro bias, acc and gyro covariance
   ** 2. normalize the acceleration measurenments to unit gravity **/
  V3D cur_acc, cur_gyr;

  for (const auto &imu : meas.imu)
  {
    const int64_t stamp_ns = int64_t(imu->header.stamp.sec) * 1000000000LL + imu->header.stamp.nanosec;
    // A repeated/overlapping batch must not count an observation twice.
    if (last_init_stamp_ns_ && stamp_ns <= *last_init_stamp_ns_) continue;
    last_init_stamp_ns_ = stamp_ns;
    b_first_frame_ = false;
    const auto &imu_acc = imu->linear_acceleration;
    const auto &gyr_acc = imu->angular_velocity;
    cur_acc << imu_acc.x, imu_acc.y, imu_acc.z;
    cur_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;

    ++init_iter_num;
    mean_acc      += (cur_acc - mean_acc) / init_iter_num;
    mean_gyr      += (cur_gyr - mean_gyr) / init_iter_num;
  }
  std::cout << "IMU unique initialization samples: " << init_iter_num << "/" << init_samples_ << std::endl;
}

void ImuProcess::Process(const MeasureGroup &meas, PointCloudXYZI::Ptr cur_pcl_un_)
{  
  // if (imu_need_init_){
  //   std::cout << "imu_en: " << imu_en << ", imu_need_init:" << imu_need_init_ << std::endl;
  // }
  if (imu_en)
  {
   
    assert(meas.lidar != nullptr);

   
    if (imu_need_init_)
    {
      cur_pcl_un_->clear();
      if (meas.imu.empty()) return;
      IMU_init(meas);

      imu_need_init_ = true;

      if (init_iter_num >= init_samples_)
      {
        // printf("IMU Initializing: %.1f %%", 100.0);
        std::cout << "IMU Initializing: 100.0%" << std::endl;
        imu_need_init_ = false;
        *cur_pcl_un_ = *(meas.lidar);
      }
      return;
    }
    if (!gravity_align_) 
      gravity_align_ = true;
    *cur_pcl_un_ = *(meas.lidar);
    return;
  }
  else
  {
   
    if (!b_first_frame_) {
      if (!gravity_align_) 
        gravity_align_ = true;
    }
    else{
      b_first_frame_ = false;
      return;
    }

   
    *cur_pcl_un_ = *(meas.lidar);
    return;
  }
}

void ImuProcess::Set_init(Eigen::Vector3d &tmp_gravity, Eigen::Matrix3d &rot)
{
  /** 1. initializing the gravity, gyro bias, acc and gyro covariance
   ** 2. normalize the acceleration measurenments to unit gravity **/
 
  M3D hat_grav;
  hat_grav << 0.0, gravity_(2), -gravity_(1),
              -gravity_(2), 0.0, gravity_(0),
              gravity_(1), -gravity_(0), 0.0;
  double align_norm = (hat_grav * tmp_gravity).norm() / tmp_gravity.norm() / gravity_.norm();
  double align_cos = gravity_.transpose() * tmp_gravity;
  align_cos = align_cos / gravity_.norm() / tmp_gravity.norm();
  if (align_norm < 1e-6)
  {
    if (align_cos > 1e-6)
    {
      rot = Eye3d;
    }
    else
    {
      rot = -Eye3d;
    }
  }
  else
  {
    V3D align_angle = hat_grav * tmp_gravity / (hat_grav * tmp_gravity).norm() * acos(align_cos); 
    rot = Exp(align_angle(0), align_angle(1), align_angle(2));
  }
}
