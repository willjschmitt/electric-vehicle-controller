#include "control/modulation.h"

#include <cmath>

#include "gtest/gtest.h"
#include "signal_processing/clarke_transformations.h"
#include "signal_processing/math_constants.h"

namespace electric_vehicle {
namespace control {
namespace {

using ::electric_vehicle::signal_processing::ThreePhase;
using ::electric_vehicle::signal_processing::kSqrt2;
using ::electric_vehicle::signal_processing::kSqrt3;
using ::electric_vehicle::signal_processing::k2PiBy3;

TEST(TwoLevelSineModulator, Modulates) {
  const double kSwitchingPeriod = 1.0;
  TwoLevelSineModulator modulator(kSwitchingPeriod);
  const double kDCVoltage = 1000.0;
  const double kACMagnitude = 120.0;
  const double kAReference = (kSqrt2 / kSqrt3) * kACMagnitude * cos(0.0);
  const double kBReference = (kSqrt2 / kSqrt3) * kACMagnitude * cos(-k2PiBy3);
  const double kCReference = (kSqrt2 / kSqrt3) * kACMagnitude * cos(+k2PiBy3);
  const ThreePhase kACReferences{kAReference, kBReference, kCReference};
  ModulationCommands<6> modulation_commands = modulator.Modulate(
    kACReferences, kDCVoltage);

  ASSERT_EQ(modulation_commands.ForPhase(Phase::A).Peek()->time, 0.0);
  ASSERT_EQ(modulation_commands.ForPhase(Phase::B).Peek()->time, 0.0);
  ASSERT_EQ(modulation_commands.ForPhase(Phase::C).Peek()->time, 0.0);

  const double voltage_a = modulation_commands.ForPhase(Phase::A)
    .ModulationCommandsToVoltage(kDCVoltage, kSwitchingPeriod);
  const double voltage_b = modulation_commands.ForPhase(Phase::B)
    .ModulationCommandsToVoltage(kDCVoltage, kSwitchingPeriod);
  const double voltage_c = modulation_commands.ForPhase(Phase::C)
    .ModulationCommandsToVoltage(kDCVoltage, kSwitchingPeriod);
  EXPECT_DOUBLE_EQ(voltage_a - voltage_b, kAReference - kBReference);
  EXPECT_DOUBLE_EQ(voltage_b - voltage_c, kBReference - kCReference);
  EXPECT_DOUBLE_EQ(voltage_c - voltage_a, kCReference - kAReference);
}

}  // namespace
}  // namespace control
}  // namespace electric_vehicle

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
