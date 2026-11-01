#include "../include/IKFoM/IKFoM_toolkit/esekfom/esekfom.hpp"
#include <cassert>
#include <Eigen/Geometry>

using Vec3 = MTK::vect<3, double>;
using SO3 = MTK::SO3<double>;
MTK_BUILD_MANIFOLD(RotationState, ((SO3, rot)) ((Vec3, omega)));
using Filter = esekfom::esekf<RotationState, 6>;
using Matrix6 = Eigen::Matrix<double, 6, 6>;
Eigen::Matrix<double, 6, 1> dynamics(RotationState& s, const RotationState&) {
  Eigen::Matrix<double, 6, 1> f = Eigen::Matrix<double, 6, 1>::Zero();
  f.head<3>() = s.omega;
  return f;
}
Matrix6 jacobian(RotationState&, const RotationState&) {
  Matrix6 f = Matrix6::Zero();
  f.block<3,3>(0,3).setIdentity();
  return f;
}
void measurement(RotationState&, esekfom::dyn_share_modified<double>&) {}

int main() {
  // Exercise actual predict(), not a separate quaternion helper. With exact
  // angular rate, an anisotropic attitude covariance rotates by Exp(-omega dt).
  for (double dt : {0., .004, .05, .2}) {
    RotationState state;
    state.omega << .4, -.7, 1.2;
    Matrix6 p = Matrix6::Zero();
    p.diagonal().head<3>() << .1, .2, .4;
    Filter filter(state,p);
    filter.init_dyn_share_modified(dynamics,jacobian,measurement);
    Matrix6 q = Matrix6::Zero();
    const Eigen::Vector3d angle = Eigen::Vector3d(state.omega) * dt;
    const Eigen::Matrix3d r = Eigen::AngleAxisd(angle.norm(),Eigen::Vector3d(state.omega).normalized()).toRotationMatrix();
    filter.predict(dt,q,state,true,true);
    assert((filter.get_x().rot.toRotationMatrix()-r).norm()<1.e-12);
    const Eigen::Matrix3d expected = r.transpose()*p.topLeftCorner<3,3>()*r;
    assert((filter.get_P().topLeftCorner<3,3>()-expected).norm()<1.e-12);
    assert((filter.get_P()-filter.get_P().transpose()).norm()<1.e-12);
  }
}
