#include "control/stator_angle_estimator.h"

#include "gtest/gtest.h"

namespace electric_vehicle {
namespace control {
namespace {

using ::electric_vehicle::signal_processing::k2Pi;

TEST(StatorAngleEstimator, Estimates) {
  SettableTimer timer;
  double current_time = 123.45;
  constexpr unsigned int kNumberOfPoles = 2;
  // Tau of 200ms = 0.040 / 0.2.
  constexpr double kRotorInductance = 0.040;  // Henries.
  constexpr double kRotorResistance = 0.2;  // Ohms.
  StatorAngleEstimator estimator(
      &timer, kNumberOfPoles, kRotorInductance, kRotorResistance);

  constexpr double kMechanicalSpeed = 1800.0 * k2Pi / 60.0;  // 1800rpm in rad/sec.

  double estimated_rotor_flux_angle;

  timer.SetTime(current_time);
  constexpr double kDirectCurrentStator = 10.0;  // Amps.
  constexpr double kQuadratureCurrentStator = 40.0;  // Amps.
  
  // Expect the speed to be (1800.0rpm*2pi/60) + (1/tau) * (Iq/Id)
  //   = 188.5rad/s + (1 / 0.2s) * (40.0 A / 10.0 A)
  //   = 188.5rad/s + (4 / 0.2s)
  //   = 188.5rad/s + 20rad/sec
  //   = 208.5rad/s
  // ==> 0.02085 rad per 100uS.
  constexpr double kRadPer100uS = 0.02085;
  
  timer.SetTime(current_time);
  estimated_rotor_flux_angle = estimator.Estimate(
      kMechanicalSpeed, kDirectCurrentStator, kQuadratureCurrentStator);
  EXPECT_DOUBLE_EQ(estimated_rotor_flux_angle, 0.0);

  current_time += 100E-6;
  timer.SetTime(current_time);
  estimated_rotor_flux_angle = estimator.Estimate(
      kMechanicalSpeed, kDirectCurrentStator, kQuadratureCurrentStator);
  EXPECT_NEAR(estimated_rotor_flux_angle, kRadPer100uS, 1E-4);

  current_time += 100E-6;
  timer.SetTime(current_time);
  estimated_rotor_flux_angle = estimator.Estimate(
      kMechanicalSpeed, kDirectCurrentStator, kQuadratureCurrentStator);
  EXPECT_NEAR(estimated_rotor_flux_angle, 2*kRadPer100uS, 1E-4);
}

}  // namespace
}  // namespace signal_processing
}  // namespace electric_vehicle
