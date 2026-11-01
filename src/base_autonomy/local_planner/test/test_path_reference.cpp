#include "local_planner/path_reference.hpp"
#include <gtest/gtest.h>

TEST(PathReference, DelayedTranslationDoesNotMoveWorldTarget) {
  for (double delay : {0., .1, .2}) {
    const auto p=local_planner::reframePoint(1.,0.,{0.,0.,0.},{.2*delay,0.,0.});
    EXPECT_NEAR(p[0],1.-.2*delay,1.e-12);
    EXPECT_NEAR(p[1],0.,1.e-12);
  }
}
TEST(PathReference, DelayedRotationAndEquivalentRefresh) {
  const double pi=std::acos(-1.);
  const auto p=local_planner::reframePoint(1.,0.,{1.,2.,pi/2},{0.,0.,0.});
  EXPECT_NEAR(p[0],1.,1.e-12); EXPECT_NEAR(p[1],3.,1.e-12);
  const auto inverse=local_planner::reframePoint(p[0],p[1],{0.,0.,0.},{1.,2.,pi/2});
  EXPECT_NEAR(inverse[0],1.,1.e-12); EXPECT_NEAR(inverse[1],0.,1.e-12);
}
TEST(PathReference, MeasurementTimeCannotBeRenewedByCallback) {
  EXPECT_TRUE(local_planner::measurementFresh(1000000000,1490000000,.5));
  EXPECT_FALSE(local_planner::measurementFresh(1000000000,1510000000,.5));
  EXPECT_FALSE(local_planner::measurementFresh(1600000000,1510000000,.5));
  EXPECT_FALSE(local_planner::measurementFresh(0,1510000000,.5));
}
TEST(PathReference, DeadbandDoesNotChangeWithCycleTime) {
  for (double dt : {.01,.02,.05}) {
    const double step=2.*dt;
    EXPECT_GE(step,.02);
    EXPECT_DOUBLE_EQ(local_planner::applySpeedDeadband(.03,.02),.03);
    EXPECT_DOUBLE_EQ(local_planner::applySpeedDeadband(-.03,.02),-.03);
    EXPECT_DOUBLE_EQ(local_planner::applySpeedDeadband(.019,.02),0.);
  }
}
TEST(PathReference, FullPitchLeverArmAndQuaternionNormalization) {
  const double angle=.4;
  for (double scale : {1.,2.}) {
    const auto p=local_planner::vehicleOffset(0.,scale*std::sin(angle/2),0.,scale*std::cos(angle/2),.3,0.);
    EXPECT_NEAR(p[0],-.3*std::cos(angle),1.e-12);
    EXPECT_NEAR(p[1],0.,1.e-12);
    EXPECT_NEAR(p[2],.3*std::sin(angle),1.e-12);
  }
}
