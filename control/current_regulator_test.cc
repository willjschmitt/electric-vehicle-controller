#include "control/current_regulator.h"

#include <cmath>
#include <ctime>

#include "gtest/gtest.h"
#include "control/timer.h"
#include "signal_processing/clarke_transformations.h"

namespace electric_vehicle {
namespace control {
namespace {

using ::electric_vehicle::signal_processing::DirectQuadrature;

TEST(CurrentRegulator, ControlsDirectAction) {
  const DirectQuadrature current_reference{ 1.0, 0.0, 0.0 };
  const DirectQuadrature current_measurement{ 0.5, 0.0, 0.0 };
  SettableTimer timer;
  const double kProportionalGain = 1.0;
  const double kIntegralGain = 0.1;
  CurrentRegulator regulator(&timer, kProportionalGain, kIntegralGain);
  
  std::time_t start_time = std::time(NULL);

  // First regulation only initializes and will result in no output.
  timer.SetTime(start_time);
  const DirectQuadrature voltage_reference1 = regulator.Regulate(
      current_reference, current_measurement);
  EXPECT_DOUBLE_EQ(voltage_reference1.direct, 0.0);
  EXPECT_DOUBLE_EQ(voltage_reference1.quadrature, 0.0);

  const std::time_t kOneSecond = (std::time_t)1.0;
  timer.SetTime(start_time + kOneSecond);
  const DirectQuadrature voltage_reference2 = regulator.Regulate(
      current_reference, current_measurement);
  // 1.0 * (1.0 - 0.5) + 0.1 * (1.0 - 0.5) * 1sec
  //   = 1.0 * (0.5) + 0.1 * (0.5) * 1sec
  //   = 0.5 + 0.05
  //   = 0.55
  EXPECT_DOUBLE_EQ(voltage_reference2.direct, 0.55);
  EXPECT_DOUBLE_EQ(voltage_reference2.quadrature, 0.0);
}

TEST(CurrentRegulator, ControlsQuadratureAction) {
  const DirectQuadrature current_reference{ 0.0, 1.0, 0.0 };
  const DirectQuadrature current_measurement{ 0.0, 0.5, 0.0 };
  SettableTimer timer;
  const double kProportionalGain = 1.0;
  const double kIntegralGain = 0.1;
  CurrentRegulator regulator(&timer, kProportionalGain, kIntegralGain);
  
  std::time_t start_time = std::time(NULL);

  // First regulation only initializes and will result in no output.
  timer.SetTime(start_time);
  const DirectQuadrature voltage_reference1 = regulator.Regulate(
      current_reference, current_measurement);
  EXPECT_DOUBLE_EQ(voltage_reference1.direct, 0.0);
  EXPECT_DOUBLE_EQ(voltage_reference1.quadrature, 0.0);

  const std::time_t kOneSecond = (std::time_t)1.0;
  timer.SetTime(start_time + kOneSecond);
  const DirectQuadrature voltage_reference2 = regulator.Regulate(
      current_reference, current_measurement);
  // 1.0 * (1.0 - 0.5) + 0.1 * (1.0 - 0.5) * 1sec
  //   = 1.0 * (0.5) + 0.1 * (0.5) * 1sec
  //   = 0.5 + 0.05
  //   = 0.55
  EXPECT_DOUBLE_EQ(voltage_reference2.direct, 0.0);
  EXPECT_DOUBLE_EQ(voltage_reference2.quadrature, 0.55);
}

TEST(CurrentRegulator, ControlsDualAction) {
  const DirectQuadrature current_reference{ 1.0, 1.0, 0.0 };
  const DirectQuadrature current_measurement{ 0.5, 0.5, 0.0 };
  SettableTimer timer;
  const double kProportionalGain = 1.0;
  const double kIntegralGain = 0.1;
  CurrentRegulator regulator(&timer, kProportionalGain, kIntegralGain);
  
  std::time_t start_time = std::time(NULL);

  // First regulation only initializes and will result in no output.
  timer.SetTime(start_time);
  const DirectQuadrature voltage_reference1 = regulator.Regulate(
      current_reference, current_measurement);
  EXPECT_DOUBLE_EQ(voltage_reference1.direct, 0.0);
  EXPECT_DOUBLE_EQ(voltage_reference1.quadrature, 0.0);

  const std::time_t kOneSecond = (std::time_t)1.0;
  timer.SetTime(start_time + kOneSecond);
  const DirectQuadrature voltage_reference2 = regulator.Regulate(
      current_reference, current_measurement);
  // 1.0 * (1.0 - 0.5) + 0.1 * (1.0 - 0.5) * 1sec
  //   = 1.0 * (0.5) + 0.1 * (0.5) * 1sec
  //   = 0.5 + 0.05
  //   = 0.55
  EXPECT_DOUBLE_EQ(voltage_reference2.direct, 0.55);
  EXPECT_DOUBLE_EQ(voltage_reference2.quadrature, 0.55);
}

}  // namespace
}  // namespace control
}  // namespace electric_vehicle

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
