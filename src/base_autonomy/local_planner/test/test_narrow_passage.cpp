#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "local_planner/narrow_passage.hpp"

namespace lp = local_planner;

TEST(NarrowPassage, AlignedAndRotatedRectangle) {
  const lp::NarrowFootprint physical{0.62, 0.40, 0.02, 0.02};
  EXPECT_TRUE(lp::footprintCollides({0.0, 0.0}, 0.0, {0.30, 0.0}, physical));
  EXPECT_FALSE(lp::footprintCollides({0.0, 0.0}, 0.0, {0.0, 0.30}, physical));
  EXPECT_TRUE(lp::footprintCollides({0.0, 0.0}, lp::kPi / 2.0,
                                    {0.0, 0.30}, physical));
  const auto envelope = lp::yawInflatedFootprint(physical, 20.0 * lp::kPi / 180.0);
  EXPECT_GT(envelope.lateral_margin, physical.lateral_margin);
  EXPECT_GT(envelope.longitudinal_margin, physical.longitudinal_margin);
}

TEST(NarrowPassage, TangentAndCurvature) {
  const std::vector<lp::Point2D> line{{0.0, 0.0}, {0.1, 0.0}, {0.2, 0.0}};
  std::vector<double> headings;
  ASSERT_TRUE(lp::computeTangents(line, &headings));
  for (double heading : headings) EXPECT_NEAR(heading, 0.0, 1.0e-9);
  EXPECT_TRUE(lp::curvatureWithinLimit(headings, 0.1));
  EXPECT_TRUE(lp::curvatureRateWithinLimit(line, headings, 1.0));
  EXPECT_FALSE(lp::computeTangents({{0.0, 0.0}, {0.0, 0.0}}, &headings));
  const std::vector<lp::Point2D> tight{{0.0, 0.0}, {0.01, 0.0}, {0.01, 0.01}};
  ASSERT_TRUE(lp::computeTangents(tight, &headings));
  EXPECT_FALSE(lp::curvatureRateWithinLimit(tight, headings, 1.0));
}

TEST(NarrowPassage, PhysicalWidthIndependentOfPathScale) {
  const lp::NarrowFootprint physical{0.62, 0.40, 0.02, 0.02};
  const std::vector<lp::Point2D> gap66{{0.5, -0.33}, {0.5, 0.33},
                                        {0.7, -0.33}, {0.7, 0.33}};
  const std::vector<lp::Point2D> gap60{{0.5, -0.30}, {0.5, 0.30},
                                        {0.7, -0.30}, {0.7, 0.30}};
  for (double scale : {0.75, 1.0}) {
    std::vector<lp::Point2D> samples;
    for (int index = 0; index <= 120; ++index)
      samples.push_back({scale * index * 0.01, 0.0});
    std::vector<double> headings;
    ASSERT_TRUE(lp::computeTangents(samples, &headings));
    const auto free = lp::checkSweptPath(samples, headings, gap66, physical,
                                         20.0 * lp::kPi / 180.0);
    EXPECT_TRUE(free.free) << "pathScale=" << scale;
    EXPECT_GT(free.min_lateral_clearance, 0.0);
    EXPECT_FALSE(lp::checkSweptPath(samples, headings, gap60, physical,
                                     20.0 * lp::kPi / 180.0).free);
  }
  EXPECT_NEAR(lp::turnRadius(physical), std::hypot(0.33, 0.22), 1.0e-9);
}

TEST(NarrowPassage, RevisionAndStampAssociation) {
  const lp::StampKey path{123, 456};
  const lp::StampKey same{123, 456};
  EXPECT_TRUE(lp::matchingConstraint(path, same, 8, 7));
  EXPECT_FALSE(lp::matchingConstraint(path, {123, 457}, 8, 7));
  EXPECT_FALSE(lp::matchingConstraint(path, same, 7, 7));
  EXPECT_FALSE(lp::matchingConstraint(path, same, 6, 7));
}

TEST(NarrowPassage, LocksClosestHeadingAndDirection) {
  const auto forward = lp::chooseNarrowHeading(0.0, 0.20);
  EXPECT_NEAR(forward.heading, 0.0, 1.0e-9);
  EXPECT_EQ(forward.direction, 1);
  const auto backward = lp::chooseNarrowHeading(0.0, lp::kPi - 0.20);
  EXPECT_NEAR(backward.heading, lp::kPi, 1.0e-9);
  EXPECT_EQ(backward.direction, -1);
}


TEST(NarrowPassage, StateTransitionsAndStaleConstraint) {
  lp::NarrowStateMachine machine(0.10, 0.15, 0.20, 2, 0.5);
  EXPECT_EQ(machine.step(false, true, 0.0, 0.01), lp::NarrowState::NORMAL);
  EXPECT_EQ(machine.step(true, true, 0.30, 0.01), lp::NarrowState::NARROW_APPROACH);
  EXPECT_EQ(machine.step(true, true, 0.05, 0.01), lp::NarrowState::NARROW_APPROACH);
  EXPECT_EQ(machine.step(true, true, 0.05, 0.01), lp::NarrowState::NARROW_TRAVERSE);
  EXPECT_TRUE(machine.permitsForwardMotion(0.10));
  EXPECT_FALSE(machine.permitsForwardMotion(0.16));
  EXPECT_EQ(machine.step(true, true, 0.25, 0.01), lp::NarrowState::NARROW_APPROACH);
  EXPECT_EQ(machine.step(true, true, 0.05, 0.01), lp::NarrowState::NARROW_APPROACH);
  EXPECT_EQ(machine.step(true, true, 0.05, 0.01), lp::NarrowState::NARROW_TRAVERSE);
  EXPECT_EQ(machine.step(true, false, 0.0, 0.01), lp::NarrowState::NORMAL);
  machine.step(true, true, 0.0, 0.01);
  machine.step(true, true, 0.0, 0.01);
  machine.step(true, true, 0.0, 0.01);
  ASSERT_EQ(machine.step(true, true, 0.25, 0.01), lp::NarrowState::NARROW_APPROACH);
  EXPECT_EQ(machine.step(true, true, 0.25, 0.51), lp::NarrowState::REALIGN_TIMEOUT);
  EXPECT_FALSE(machine.permitsForwardMotion(0.0));
  EXPECT_EQ(machine.step(false, true, 0.0, 0.01), lp::NarrowState::NORMAL);
}
