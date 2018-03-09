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
  const system_clock::time_point want = system_clock::now();
  AbsoluteTimer timer;
  const system_clock::time_point got = timer.Time();
  const duration<double> difference = got - want;
  EXPECT_LE(difference.count(), 1E-6);
}

TEST(SettableTimer, ProvidesSetTime) {
  SettableTimer timer;
  const system_clock::time_point want = system_clock::now();
  timer.SetTime(want);
  EXPECT_EQ(timer.Time(), want);
}

TEST(SettableTimer, ProvidesSetTimeMultipleTimes) {
  SettableTimer timer;

  const system_clock::time_point want1 = system_clock::now();
  timer.SetTime(want1);
  EXPECT_EQ(timer.Time(), want1);

  const system_clock::time_point want2 = system_clock::now();
  timer.SetTime(want2);
  EXPECT_EQ(timer.Time(), want2);
}

TEST(SamplingTimer, ProvidesTime) {
  // This test can definitely be flaky, but there's nothing that can be done to
  // mock the underlying timer without defeating the intent of the test itself,
  // which is checking the timer is returning roughly the same function.
  const system_clock::time_point want = system_clock::now();
  SamplingTimer timer;
  timer.SampleTime();
  const system_clock::time_point got = timer.Time();
  const duration<double> difference = got - want;
  EXPECT_LE(difference.count(), 1E-6);
}

}  // namespace
}  // namespace control
}  // namespace electric_vehicle

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
