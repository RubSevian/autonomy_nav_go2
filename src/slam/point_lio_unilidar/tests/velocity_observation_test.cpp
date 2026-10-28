#include "velocity_observation.hpp"
#include <Eigen/Geometry>
#include <cassert>
#include <limits>

int main() {
  using namespace point_lio_diagnostics;
  const Eigen::Matrix3d r = Eigen::AngleAxisd(.7, Eigen::Vector3d(.2, .5, .8).normalized()).toRotationMatrix();
  const Eigen::Vector3d v(.3, -.2, .1), omega(.1, .2, -.3);
  Eigen::Matrix<double, 30, 30> a = Eigen::Matrix<double, 30, 30>::Random();
  Eigen::Matrix<double, 30, 30> p = a * a.transpose() * 1.e-4;
  BodyVelocity out;
  assert(outputBodyVelocity(r, v, omega, p, out));
  assert((out.value.head<3>() - r.transpose()*v).norm() < 1.e-12);
  assert((out.value.tail<3>() - omega).norm() < 1.e-12);
  // Independently differentiate the nonlinear observation, including attitude.
  Eigen::Matrix<double, 6, 30> numerical;
  const double epsilon = 1.e-6;
  for (int i = 0; i < 30; ++i) {
    Eigen::Matrix<double, 6, 1> samples[2];
    for (int sign = 0; sign < 2; ++sign) {
      Eigen::Matrix<double, 30, 1> d = Eigen::Matrix<double, 30, 1>::Zero();
      d(i) = sign ? epsilon : -epsilon;
      const Eigen::Vector3d angle = d.segment<3>(3);
      Eigen::Matrix3d delta = Eigen::Matrix3d::Identity();
      if (angle.norm() > 0.) delta = Eigen::AngleAxisd(angle.norm(), angle.normalized()).toRotationMatrix();
      samples[sign].head<3>() = (r*delta).transpose() * (v+d.segment<3>(12));
      samples[sign].tail<3>() = omega+d.segment<3>(15);
    }
    numerical.col(i) = (samples[1]-samples[0])/(2.*epsilon);
  }
  assert((out.covariance - numerical*p*numerical.transpose()).norm() < 1.e-10);
  // Attitude uncertainty contributes at nonzero velocity, including off-diagonal terms.
  p.setZero(); p.block<3,3>(3,3).setIdentity();
  assert(outputBodyVelocity(r,v,omega,p,out));
  assert((out.covariance.topLeftCorner<3,3>().trace() > .1));
  assert(outputBodyVelocity(r,Eigen::Vector3d::Zero(),omega,p,out));
  assert(out.covariance.norm() < 1.e-12);
  p.setIdentity(); p(12,12)=-10.;
  assert(!outputBodyVelocity(Eigen::Matrix3d::Identity(),Eigen::Vector3d::Zero(),omega,p,out));
  p.setIdentity();p(0,0)=std::numeric_limits<double>::quiet_NaN();
  assert(!outputBodyVelocity(r,v,omega,p,out));
}
