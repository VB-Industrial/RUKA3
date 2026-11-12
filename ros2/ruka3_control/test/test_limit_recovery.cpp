#include <gtest/gtest.h>
#include "ruka3_control/limit_recovery.hpp"
using namespace ruka3_control;
struct Joint { double position{}, velocity{}, soft_lower{-0.8}, soft_upper{0.8}; };

TEST(LimitRecovery, BothDirectionsAndNoAutomaticResume)
{
  for (const double sign : {-1., 1.}) {
    std::array<Joint,6> joints;
    joints[0].position=sign*.9;
    joints[1].position=.2;
    LimitRecovery recovery;
    EXPECT_TRUE(recovery.update(joints,.01));
    double last=joints[0].position;
    double previous_velocity = recovery.velocity[0];
    for (int tick=0;tick<400;++tick) {
      recovery.update(joints,.01);
      EXPECT_LE(std::fabs(recovery.target[0]-last),.00101);
      EXPECT_LE(std::fabs(recovery.velocity[0]-previous_velocity),.00201);
      previous_velocity = recovery.velocity[0];
      EXPECT_LE(sign*(recovery.target[0]-last),0.000001);
      EXPECT_DOUBLE_EQ(recovery.target[1],.2);
      last=joints[0].position=recovery.target[0];
      joints[0].velocity=recovery.velocity[0];
    }
    EXPECT_NEAR(joints[0].position,sign*.79,1e-6);
    EXPECT_TRUE(recovery.settled(joints));
    EXPECT_TRUE(recovery.latched);
  }
}

TEST(LimitRecovery, DoesNotLatchInsideAndWaitsForMeasuredReturn)
{
  std::array<Joint,6> joints;
  LimitRecovery recovery;
  EXPECT_FALSE(recovery.update(joints,.01));
  EXPECT_FALSE(recovery.latched);
  joints[5].position=-.9;
  recovery.update(joints,.01);
  for (int tick=0;tick<400;++tick) recovery.update(joints,.01);
  EXPECT_FALSE(recovery.settled(joints));
}
