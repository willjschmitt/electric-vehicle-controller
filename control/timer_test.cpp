#include "control/timer.h"

#include <chrono>

#include "gtest/gtest.h"

namespace electric_vehicle {
namespace control {
namespace {

using std::chrono::duration;
using std::chrono::system_clock;

TEST(AbsoluteTimer, ProvidesTime) {
  // This test can definitely be flaky, but there's nothing that can be done to
  // mock the underlying timer without defeating the intent of the test itself,
  // which is checking the timer is returning roughly the same function.
  const double want = 0.0;
  AbsoluteTimer timer;
  const double got = timer.Time();
  const double difference = got - want;
  EXPECT_LE(difference, 1E-6);
}

TEST(SettableTimer, ProvidesSetTime) {
  SettableTimer timer;
  const double want = 123.45;
  timer.SetTime(want);
  EXPECT_EQ(timer.Time(), want);
}

TEST(SettableTimer, ProvidesSetTimeMultipleTimes) {
  SettableTimer timer;

  const double want1 = 123.45;
  timer.SetTime(want1);
  EXPECT_EQ(timer.Time(), want1);

  const double want2 = 678.90;
  timer.SetTime(want2);
  EXPECT_EQ(timer.Time(), want2);
}

TEST(SamplingTimer, ProvidesTime) {
  // This test can definitely be flaky, but there's nothing that can be done to
  // mock the underlying timer without defeating the intent of the test itself,
  // which is checking the timer is returning roughly the same function.
  const double want = 0.0;
  SamplingTimer timer;
  timer.SampleTime();
  const double got = timer.Time();
  const double difference = got - want;
  EXPECT_LE(difference, 1E-6);
}

}  // namespace
}  // namespace control
}  // namespace electric_vehicle

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
