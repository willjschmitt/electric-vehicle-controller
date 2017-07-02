#include "control/timer.h"

#include <ctime>

#include "gtest/gtest.h"

namespace electric_vehicle {
namespace control {
namespace {

TEST(AbsoluteTimer, ProvidesTime) {
  // This test can definitely be flaky, but there's nothing that can be done to
  // mock the underlying timer without defeating the intent of the test itself,
  // which is checking the timer is returning roughly the same function.
  const std::time_t want = std::time(NULL);
  AbsoluteTimer timer;
  const std::time_t got = timer.Time();
  EXPECT_LE(std::difftime(got, want), 1E-6);
}

TEST(SettableTimer, ProvidesSetTime) {
  SettableTimer timer;
  const std::time_t want = std::time(NULL);
  timer.SetTime(want);
  EXPECT_EQ(timer.Time(), want);
}

TEST(SettableTimer, ProvidesSetTimeMultipleTimes) {
  SettableTimer timer;

  const std::time_t want1 = std::time(NULL);
  timer.SetTime(want1);
  EXPECT_EQ(timer.Time(), want1);

  const std::time_t want2 = std::time(NULL);
  timer.SetTime(want2);
  EXPECT_EQ(timer.Time(), want2);
}

TEST(SamplingTimer, ProvidesTime) {
  // This test can definitely be flaky, but there's nothing that can be done to
  // mock the underlying timer without defeating the intent of the test itself,
  // which is checking the timer is returning roughly the same function.
  const std::time_t want = std::time(NULL);
  SamplingTimer timer;
  timer.SampleTime();
  const std::time_t got = timer.Time();
  EXPECT_LE(std::difftime(got, want), 1E-6);
}

}  // namespace
}  // namespace control
}  // namespace electric_vehicle

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
