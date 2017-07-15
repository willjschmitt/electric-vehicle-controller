#include "signal_processing/integrator.h"

#include <chrono>

#include "gtest/gtest.h"

namespace electric_vehicle {
namespace signal_processing {
namespace {

using std::chrono::seconds;
using std::chrono::system_clock;
using ::electric_vehicle::control::SettableTimer;

TEST(Integrator, Integrates) {
  SettableTimer timer;
  system_clock::time_point start = system_clock::now();
  Integrator integrator(&timer);

  constexpr double kInput = 0.1;
  
  timer.SetTime(start);  
  const double integration1 = integrator.Integrate(kInput);
  EXPECT_EQ(integration1, 0.0);
  
  const seconds kOneSecond(1);
  timer.SetTime(start + kOneSecond);
  const double integration2 = integrator.Integrate(kInput);
  EXPECT_EQ(integration2, 0.1);

  const seconds kTwoSeconds(2);
  timer.SetTime(start + kTwoSeconds);
  const double integration3 = integrator.Integrate(kInput);
  EXPECT_EQ(integration3, 0.2);
}

TEST(Integrator, LimitsMinimum) {
  const double kMinimum = 100.0;
  const double kMaximum = DBL_MAX;
  SettableTimer timer;
  const system_clock::time_point start = system_clock::now();
  Integrator integrator(&timer, kMinimum, kMaximum);

  const double kInput = -0.1;
  
  timer.SetTime(start);  
  const double calculation1 = integrator.Integrate(kInput);
  EXPECT_EQ(calculation1, 0.0);
  
  const seconds kOneSecond(1);
  timer.SetTime(start + kOneSecond);
  const double calculation2 = integrator.Integrate(kInput);
  EXPECT_EQ(calculation2, kMinimum);

  const seconds kTwoSeconds(2);
  timer.SetTime(start + kTwoSeconds);
  const double calculation3 = integrator.Integrate(kInput);
  EXPECT_EQ(calculation3, kMinimum);
}

TEST(Integrator, LimitsMaximum) {
  const double kMinimum = -DBL_MAX;
  const double kMaximum = -100.0;
  SettableTimer timer;
  const system_clock::time_point start = system_clock::now();
  Integrator integrator(&timer, kMinimum, kMaximum);

  const double kInput = 0.1;

  timer.SetTime(start);  
  const double calculation1 = integrator.Integrate(kInput);
  EXPECT_EQ(calculation1, 0.0);
  
  const seconds kOneSecond(1);
  timer.SetTime(start + kOneSecond);
  const double calculation2 = integrator.Integrate(kInput);
  EXPECT_EQ(calculation2, kMaximum);

  const seconds kTwoSeconds(2);
  timer.SetTime(start + kTwoSeconds);
  const double calculation3 = integrator.Integrate(kInput);
  EXPECT_EQ(calculation3, kMaximum);
}

TEST(Integrator, BacksOffOfMinimumWithoutIntegratorWindup) {
  const double kMinimum = -0.2;
  const double kMaximum = DBL_MAX;
  SettableTimer timer;
  system_clock::time_point current_time = system_clock::now();
  const seconds kTimeStep(1);
  Integrator integrator(&timer, kMinimum, kMaximum);
  
  double calculation;
  double kInput;

  // Regulate 4 times, which should put the controller beyond its minimum.
  const int kIterationsToMinimum = 4;
  kInput = -0.1;
  for (int i = 0; i < kIterationsToMinimum; i++) {
    current_time += kTimeStep;
    timer.SetTime(current_time);
    calculation = integrator.Integrate(kInput);
  }
  EXPECT_EQ(calculation, -0.2);

  // Reverse direction, and we should see the regulator immediately back off
  // the minimum, if it correctly avoided windup.
  kInput = +0.1;
  current_time += kTimeStep;
  timer.SetTime(current_time);
  calculation = integrator.Integrate(kInput);
  EXPECT_EQ(calculation, -0.1);

}

TEST(Integrator, BacksOffOfMaximumWithoutIntegratorWindup) {
  const double kMinimum = -DBL_MAX;
  const double kMaximum = 0.2;
  SettableTimer timer;
  system_clock::time_point current_time = system_clock::now();
  const seconds kTimeStep(1);
  Integrator integrator(&timer, kMinimum, kMaximum);
  
  double calculation;
  double kInput;

  // Regulate 4 times, which should put the controller beyond its maximum.
  const int kIterationsToMinimum = 4;
  kInput = +0.1;
  for (int i = 0; i < kIterationsToMinimum; i++) {
    current_time += kTimeStep;
    timer.SetTime(current_time);
    calculation = integrator.Integrate(kInput);
  }
  EXPECT_EQ(calculation, +0.2);

  // Reverse direction, and we should see the regulator immediately back off
  // the maximum, if it correctly avoided windup.
  kInput = -0.1;
  current_time += kTimeStep;
  timer.SetTime(current_time);
  calculation = integrator.Integrate(kInput);
  EXPECT_EQ(calculation, +0.1);

}

}  // namespace
}  // namespace signal_processing
}  // namespace electric_vehicle

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
