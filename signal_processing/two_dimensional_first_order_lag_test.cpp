#include "signal_processing/two_dimensional_first_order_lag.h"

#include <chrono>

#include "control/timer.h"
#include "gtest/gtest.h"
#include "signal_processing/angle_math.h"
#include "angle_math.h"

namespace electric_vehicle {
namespace signal_processing {
namespace {

using ::electric_vehicle::control::SettableTimer;
using std::chrono::duration;
using std::chrono::milliseconds;
using std::chrono::system_clock;

TEST(TwoDimensionalFirstOrderLag, ReturnsInitalValue) {
SettableTimer timer;
const double kTau = 10.0;
TwoDimensionalFirstOrderLag first_order_lag(&timer, kTau);
constexpr TwoDimensionalVector kInputValue = {20.0, 40.0};
const TwoDimensionalVector got = first_order_lag.Solve(kInputValue);
EXPECT_DOUBLE_EQ(got.x, kInputValue.x);
EXPECT_DOUBLE_EQ(got.y, kInputValue.y);
}

TEST(TwoDimensionalFirstOrderLag, LagsAtTimeConstant) {
SettableTimer timer;
double current_time = 123.45;
const double kTau = 1.0;
TwoDimensionalFirstOrderLag first_order_lag(&timer, kTau);
constexpr TwoDimensionalVector kInitialValue = {0.0, 0.0};
constexpr TwoDimensionalVector kInputValue = {1.0, 0.5};
timer.SetTime(current_time);
TwoDimensionalVector got;
got = first_order_lag.Solve(kInitialValue);
EXPECT_DOUBLE_EQ(got.x, kInitialValue.x);
EXPECT_DOUBLE_EQ(got.y, kInitialValue.y);

// Make 10 small steps to the first time constant.
for (unsigned int i = 0; i < 1000; i++) {
current_time += 0.001;
timer.SetTime(current_time);
got = first_order_lag.Solve(kInputValue);
}
constexpr double kOneTimeConstant = 0.632;
EXPECT_NEAR(got.x, kOneTimeConstant, 0.001);
EXPECT_NEAR(got.y, kOneTimeConstant * 0.5, 0.001);

// Make 10 small steps to the first time constant.
for (unsigned int i = 0; i < 1000; i++) {
current_time += 0.001;
timer.SetTime(current_time);
got = first_order_lag.Solve(kInputValue);
}
constexpr double kTwoTimeConstant = 0.865;
EXPECT_NEAR(got.x, kTwoTimeConstant, 0.001);
EXPECT_NEAR(got.y, kTwoTimeConstant * 0.5, 0.001);
}

}  // namespace
}  // namespace signal_processing
}  // namespace electric_vehicle
