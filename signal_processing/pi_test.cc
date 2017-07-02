#include "signal_processing/pi.h"

#include <ctime>

#include "gtest/gtest.h"

namespace electric_vehicle {
namespace signal_processing {
namespace {

using electric_vehicle::control::SettableTimer;

TEST(ProportionalIntegralController, Integrates) {
  const double kGainProportional = 0.0;
  const double kGainIntegral = 0.1;
  SettableTimer timer;
  std::time_t start = std::time(NULL);
  ProportionalIntegralController pi_controller(&timer, kGainProportional, kGainIntegral);

  const double kInputActual = 0.0;
  const double kInputReference = 1.0;
  
  timer.SetTime(start);  
  const double integration1 = pi_controller.Solve(
      kInputActual, kInputReference);
  EXPECT_EQ(integration1, 0.0);
  
  const std::time_t kOneSecond = (std::time_t)1.0;
  timer.SetTime(start + kOneSecond);
  const double integration2 = pi_controller.Solve(
      kInputActual, kInputReference);
  EXPECT_EQ(integration2, 0.1);

  const std::time_t kTwoSeconds = (std::time_t)2.0;
  timer.SetTime(start + kTwoSeconds);
  const double integration3 = pi_controller.Solve(
      kInputActual, kInputReference);
  EXPECT_EQ(integration3, 0.2);
}

TEST(ProportionalIntegralController, ProportionalAction) {
  const double kGainProportional = 0.1;
  const double kGainIntegral = 0.0;
  SettableTimer timer;
  std::time_t start = std::time(NULL);
  ProportionalIntegralController pi_controller(&timer, kGainProportional, kGainIntegral);

  const double kInputActual = 0.0;
  const double kInputReference = 1.0;
  
  timer.SetTime(start);  
  const double calculation1 = pi_controller.Solve(
      kInputActual, kInputReference);
  EXPECT_EQ(calculation1, 0.0);
  
  const std::time_t kOneSecond = (std::time_t)1.0;
  timer.SetTime(start + kOneSecond);
  const double calculation2 = pi_controller.Solve(
      kInputActual, kInputReference);
  EXPECT_EQ(calculation2, 0.1);

  const std::time_t kTwoSeconds = (std::time_t)2.0;
  timer.SetTime(start + kTwoSeconds);
  const double calculation3 = pi_controller.Solve(
      kInputActual, kInputReference);
  EXPECT_EQ(calculation3, 0.1);
}

}  // namespace
}  // namespace signal_processing
}  // namespace electric_vehicle

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
