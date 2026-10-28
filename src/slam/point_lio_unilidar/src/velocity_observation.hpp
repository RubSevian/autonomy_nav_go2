#pragma once

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

namespace point_lio_diagnostics {
inline Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
  Eigen::Matrix3d m;
  m << 0., -v.z(), v.y(), v.z(), 0., -v.x(), -v.y(), v.x(), 0.;
  return m;
}

struct BodyVelocity {
  Eigen::Matrix<double, 6, 1> value;
  Eigen::Matrix<double, 6, 6> covariance;
};

// state_output uses a RIGHT SO(3) perturbation: R_true = R * Exp(dtheta).
// Its ordering is pos, rot, extrinsic_R, extrinsic_T, vel, omg, acc, g, bg, ba.
// Keep rotation/velocity/angular-rate cross covariance. R^T Pvv R alone
// incorrectly treats the estimated attitude as an exact frame transform.
inline bool outputBodyVelocity(const Eigen::Matrix3d& world_from_body,
                               const Eigen::Vector3d& velocity_world,
                               const Eigen::Vector3d& omega_body,
                               const Eigen::Matrix<double, 30, 30>& p,
                               BodyVelocity& out) {
  if (!world_from_body.allFinite() || !velocity_world.allFinite() ||
      !omega_body.allFinite() || !p.allFinite()) return false;
  out.value.head<3>() = world_from_body.transpose() * velocity_world;
  out.value.tail<3>() = omega_body;
  Eigen::Matrix<double, 6, 30> j = Eigen::Matrix<double, 6, 30>::Zero();
  j.block<3, 3>(0, 3) = skew(out.value.head<3>());
  j.block<3, 3>(0, 12) = world_from_body.transpose();
  j.block<3, 3>(3, 15).setIdentity();
  const Eigen::Matrix<double, 6, 6> raw = j * p * j.transpose();
  out.covariance = (raw + raw.transpose()) * .5;
  if (!out.covariance.allFinite()) return false;
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> eigen(out.covariance);
  if (eigen.info() != Eigen::Success || eigen.eigenvalues().minCoeff() < -1.e-9)
    return false;
  // Remove roundoff only; a materially indefinite covariance is not published.
  if (eigen.eigenvalues().minCoeff() < 0.)
    out.covariance = eigen.eigenvectors() * eigen.eigenvalues().cwiseMax(0.).asDiagonal()
                   * eigen.eigenvectors().transpose();
  return true;
}
}  // namespace point_lio_diagnostics
