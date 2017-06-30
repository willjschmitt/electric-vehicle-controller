#include "signal_processing/pi.h"

#include "gtest/gtest.h"

using electric_vehicle::signal_processing::ProportionalIntegralController;


TEST(ProportionalIntegralController, Integrates) {
  const double kGainProportional = 0.0;
  const double kGainIntegral = 0.1;
  const double kDeltaTimestep = 1.0;
  ProportionalIntegralController pi_controller(kGainProportional, kGainIntegral,
                                               kDeltaTimestep);

  const double kInputActual = 0.0;
  const double kInputReference = 1.0;
  const double integration1 = pi_controller.Solve(
      kInputActual, kInputReference);
  EXPECT_EQ(integration1, 0.1);
  
  const double integration2 = pi_controller.Solve(
      kInputActual, kInputReference);
  EXPECT_EQ(integration2, 0.2);
}
