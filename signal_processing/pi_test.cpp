#include "signal_processing/pi.h"

#include <cfloat>

#include "gtest/gtest.h"

namespace electric_vehicle {
namespace signal_processing {
namespace {

using electric_vehicle::control::SettableTimer;

constexpr double kSmallError = 1E-9;

TEST(ProportionalIntegralController, Integrates) {
  const double kGainProportional = 0.0;
  const double kGainIntegral = 0.1;
  SettableTimer timer;
  double start = 123.45;
  ProportionalIntegralController pi_controller(&timer, kGainProportional, kGainIntegral);

  const double kInputActual = 0.0;
  const double kInputReference = 1.0;
  
  timer.SetTime(start);  
  const double integration1 = pi_controller.Solve(
      kInputActual, kInputReference);
  EXPECT_NEAR(integration1, 0.0, kSmallError);
  
  const double kOneSecond = 1.0;
  timer.SetTime(start + kOneSecond);
  const double integration2 = pi_controller.Solve(
      kInputActual, kInputReference);
  EXPECT_NEAR(integration2, 0.1, kSmallError);

  const double kTwoSeconds = 2.0;
  timer.SetTime(start + kTwoSeconds);
  const double integration3 = pi_controller.Solve(
      kInputActual, kInputReference);
  EXPECT_NEAR(integration3, 0.2, kSmallError);
}

TEST(ProportionalIntegralController, ProportionalAction) {
  const double kGainProportional = 0.1;
  const double kGainIntegral = 0.0;
  SettableTimer timer;
  double start = 123.45;
  ProportionalIntegralController pi_controller(&timer, kGainProportional, kGainIntegral);

  const double kInputActual = 0.0;
  const double kInputReference = 1.0;
  
  timer.SetTime(start);  
  const double calculation1 = pi_controller.Solve(
      kInputActual, kInputReference);
  EXPECT_NEAR(calculation1, 0.0, kSmallError);
  
  const double kOneSecond = 1.0;
  timer.SetTime(start + kOneSecond);
  const double calculation2 = pi_controller.Solve(
      kInputActual, kInputReference);
  EXPECT_NEAR(calculation2, 0.1, kSmallError);

  const double kTwoSeconds = 2.0;
  timer.SetTime(start + kTwoSeconds);
  const double calculation3 = pi_controller.Solve(
      kInputActual, kInputReference);
  EXPECT_NEAR(calculation3, 0.1, kSmallError);
}

TEST(ProportionalIntegralController, LimitsMinimum) {
  const double kGainProportional = 0.1;
  const double kGainIntegral = 0.0;
  const double kMinimum = 100.0;
  const double kMaximum = DBL_MAX;
  SettableTimer timer;
  const double start = 123.45;
  ProportionalIntegralController pi_controller(
      &timer, kGainProportional, kGainIntegral, kMinimum, kMaximum);

  const double kInputActual = 0.0;
  const double kInputReference = 1.0;
  
  timer.SetTime(start);  
  const double calculation1 = pi_controller.Solve(
      kInputActual, kInputReference);
  EXPECT_NEAR(calculation1, 0.0, kSmallError);
  
  const double kOneSecond = 1.0;
  timer.SetTime(start + kOneSecond);
  const double calculation2 = pi_controller.Solve(
      kInputActual, kInputReference);
  EXPECT_NEAR(calculation2, kMinimum, kSmallError);

  const double kTwoSeconds = 2.0;
  timer.SetTime(start + kTwoSeconds);
  const double calculation3 = pi_controller.Solve(
      kInputActual, kInputReference);
  EXPECT_NEAR(calculation3, kMinimum, kSmallError);
}

TEST(ProportionalIntegralController, LimitsMaximum) {
  const double kGainProportional = 0.1;
  const double kGainIntegral = 0.0;
  const double kMinimum = -DBL_MAX;
  const double kMaximum = -100.0;
  SettableTimer timer;
  const double start = 123.45;
  ProportionalIntegralController pi_controller(
      &timer, kGainProportional, kGainIntegral, kMinimum, kMaximum);

  const double kInputActual = 0.0;
  const double kInputReference = 1.0;
  
  timer.SetTime(start);  
  const double calculation1 = pi_controller.Solve(
      kInputActual, kInputReference);
  EXPECT_NEAR(calculation1, 0.0, kSmallError);
  
  const double kOneSecond = 1.0;
  timer.SetTime(start + kOneSecond);
  const double calculation2 = pi_controller.Solve(
      kInputActual, kInputReference);
  EXPECT_NEAR(calculation2, kMaximum, kSmallError);

  const double kTwoSeconds = 2.0;
  timer.SetTime(start + kTwoSeconds);
  const double calculation3 = pi_controller.Solve(
      kInputActual, kInputReference);
  EXPECT_NEAR(calculation3, kMaximum, kSmallError);
}

TEST(ProportionalIntegralController,
     BacksOffOfMinimumWithoutIntegratorWindup) {
  const double kGainProportional = 0.0;
  const double kGainIntegral = 0.1;
  const double kMinimum = -0.2;
  const double kMaximum = DBL_MAX;
  SettableTimer timer;
  double current_time = 123.45;
  const double kTimeStep = 1.0;
  ProportionalIntegralController pi_controller(
      &timer, kGainProportional, kGainIntegral, kMinimum, kMaximum);
  
  double input_reference;
  double calculation;

  // Regulate 4 times, which should put the controller beyond its minimum.
  const int kIterationsToMinimum = 4;
  const double kInputActual = 0.0;
  input_reference = -1.0;
  for (int i = 0; i < kIterationsToMinimum; i++) {
    current_time += kTimeStep;
    timer.SetTime(current_time);
    calculation = pi_controller.Solve(kInputActual, input_reference);
  }
  EXPECT_NEAR(calculation, -0.2, kSmallError);

  // Reverse direction, and we should see the regulator immediately back off
  // the minimum, if it correctly avoided windup.
  input_reference = +1.0;
  current_time += kTimeStep;
  timer.SetTime(current_time);
  calculation = pi_controller.Solve(kInputActual, input_reference);
  EXPECT_NEAR(calculation, -0.1, kSmallError);

}

TEST(ProportionalIntegralController,
     BacksOffOfMaximumWithoutIntegratorWindup) {
  const double kGainProportional = 0.0;
  const double kGainIntegral = 0.1;
  const double kMinimum = -DBL_MAX;
  const double kMaximum = 0.2;
  SettableTimer timer;
  double current_time = 123.45;
  const double kTimeStep = 1.0;
  ProportionalIntegralController pi_controller(
      &timer, kGainProportional, kGainIntegral, kMinimum, kMaximum);
  
  double input_reference;
  double calculation;

  // Regulate 4 times, which should put the controller beyond its maximum.
  const int kIterationsToMinimum = 4;
  const double kInputActual = 0.0;
  input_reference = +1.0;
  for (int i = 0; i < kIterationsToMinimum; i++) {
    current_time += kTimeStep;
    timer.SetTime(current_time);
    calculation = pi_controller.Solve(kInputActual, input_reference);
  }
  EXPECT_NEAR(calculation, +0.2, kSmallError);

  // Reverse direction, and we should see the regulator immediately back off
  // the maximum, if it correctly avoided windup.
  input_reference = -1.0;
  current_time += kTimeStep;
  timer.SetTime(current_time);
  calculation = pi_controller.Solve(kInputActual, input_reference);
  EXPECT_NEAR(calculation, +0.1, kSmallError);

}

}  // namespace
}  // namespace signal_processing
}  // namespace electric_vehicle

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
