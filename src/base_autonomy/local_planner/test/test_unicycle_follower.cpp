#include <cmath>
#include <gtest/gtest.h>
#include "local_planner/unicycle_follower.hpp"
namespace lp = local_planner;
TEST(UnicycleFollower, CurrentBodyTransformCardinalAngles) {
  const auto zero = lp::targetInCurrentBody(1.0, 2.0, 0.0);
  EXPECT_NEAR(zero.x, 1.0, 1e-9); EXPECT_NEAR(zero.y, 2.0, 1e-9);
  const auto plus90 = lp::targetInCurrentBody(1.0, 0.0, lp::kPi / 2.0);
  EXPECT_NEAR(plus90.x, 0.0, 1e-9); EXPECT_NEAR(plus90.y, -1.0, 1e-9);
  const auto minus90 = lp::targetInCurrentBody(1.0, 0.0, -lp::kPi / 2.0);
  EXPECT_NEAR(minus90.x, 0.0, 1e-9); EXPECT_NEAR(minus90.y, 1.0, 1e-9);
  const auto pi = lp::targetInCurrentBody(1.0, 0.0, lp::kPi);
  EXPECT_NEAR(pi.x, -1.0, 1e-9); EXPECT_NEAR(pi.y, 0.0, 1e-9);
}
TEST(UnicycleFollower, CurvatureAndZeroLateralVelocity) {
  EXPECT_NEAR(lp::purePursuitCurvature({1.0, 0.0}), 0.0, 1e-9);
  EXPECT_GT(lp::purePursuitCurvature({1.0, 1.0}), 0.0);
  EXPECT_LT(lp::purePursuitCurvature({1.0, -1.0}), 0.0);
  EXPECT_NEAR(lp::purePursuitCurvature({0.0, 0.0}), 0.0, 1e-9);
  const auto command = lp::makeUnicycleCommand(1.0, {1.0, 1.0}, 0.0, false, 1.0, 2.0);
  EXPECT_DOUBLE_EQ(command.vy, 0.0);
}
TEST(UnicycleFollower, ClampRotateAndTargetBehind) {
  const auto saturated = lp::makeUnicycleCommand(1.0, {1.0, 1.0}, 3.0, true, 2.0, 0.5);
  EXPECT_NEAR(saturated.wz, 0.5, 1e-9); EXPECT_TRUE(saturated.saturated);
  const auto behind = lp::makeUnicycleCommand(1.0, {-1.0, 0.0}, 0.3, false, 1.0, 1.0);
  EXPECT_TRUE(behind.rotate_in_place); EXPECT_DOUBLE_EQ(behind.vx, 0.0); EXPECT_DOUBLE_EQ(behind.vy, 0.0);
}
TEST(UnicycleFollower, RotateHysteresis) {
  lp::RotateInPlaceHysteresis state(0.7, 0.4);
  EXPECT_FALSE(state.update(0.6)); EXPECT_TRUE(state.update(0.8));
  EXPECT_TRUE(state.update(0.5)); EXPECT_FALSE(state.update(0.3));
}
TEST(UnicycleFollower, BaselineMotionModelRemainsAvailable) {
  EXPECT_TRUE(lp::validMotionModel("holonomic")); EXPECT_TRUE(lp::validMotionModel("unicycle"));
  EXPECT_FALSE(lp::validMotionModel("ackermann"));
}
