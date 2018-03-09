#include "signal_processing/first_order_lag.h"

#include <chrono>

#include "control/timer.h"
#include "gtest/gtest.h"

namespace electric_vehicle {
namespace signal_processing {
namespace {

using ::electric_vehicle::control::SettableTimer;
using std::chrono::duration;
using std::chrono::milliseconds;
using std::chrono::system_clock;

TEST(FirstOrderLag, ReturnsInitalValue) {
  SettableTimer timer;
  system_clock::time_point current_time = system_clock::now();
  const double kTau = 10.0;
  FirstOrderLag first_order_lag(&timer, kTau);
  constexpr double kInputValue = 20.0;
  const double got = first_order_lag.Solve(kInputValue);
  EXPECT_DOUBLE_EQ(got, kInputValue);
}

TEST(FirstOrderLag, LagsAtTimeConstant) {
  SettableTimer timer;
  double current_time = 123.45;
  const double kTau = 1.0;
  FirstOrderLag first_order_lag(&timer, kTau);
  constexpr double kInitialValue = 0.0;
  constexpr double kInputValue = 1.0;
  timer.SetTime(current_time);
  double got;
  got = first_order_lag.Solve(kInitialValue);
  EXPECT_DOUBLE_EQ(got, kInitialValue);

  // Make 10 small steps to the first time constant.
  for (unsigned int i = 0; i < 1000; i++) {
    current_time += 0.001;
    timer.SetTime(current_time);
    got = first_order_lag.Solve(kInputValue);
  }
  constexpr double kOneTimeConstant = 0.632;
  EXPECT_NEAR(got, kOneTimeConstant, 0.001);

  // Make 10 small steps to the first time constant.
  for (unsigned int i = 0; i < 1000; i++) {
    current_time += 0.001;
    timer.SetTime(current_time);
    got = first_order_lag.Solve(kInputValue);
  }
  constexpr double kTwoTimeConstant = 0.865;
  EXPECT_NEAR(got, kTwoTimeConstant, 0.001);
}

}  // namespace
}  // namespace signal_processing
}  // namespace electric_vehicle

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
