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
  const ModulationCommands modulation_commands = modulator.Modulate(
    kACReferences, kDCVoltage);

  ASSERT_EQ(modulation_commands.at(Phase::A).size(), (unsigned int)2);
  ASSERT_EQ(modulation_commands.at(Phase::B).size(), (unsigned int)2);
  ASSERT_EQ(modulation_commands.at(Phase::C).size(), (unsigned int)2);
  ASSERT_EQ(modulation_commands.at(Phase::A)[0].time, 0.0);
  ASSERT_EQ(modulation_commands.at(Phase::B)[0].time, 0.0);
  ASSERT_EQ(modulation_commands.at(Phase::C)[0].time, 0.0);

  const double voltage_a = ModulationCommandsToVoltage(
      modulation_commands.at(Phase::A), kDCVoltage, kSwitchingPeriod);
  const double voltage_b = ModulationCommandsToVoltage(
      modulation_commands.at(Phase::B), kDCVoltage, kSwitchingPeriod);
  const double voltage_c = ModulationCommandsToVoltage(
      modulation_commands.at(Phase::C), kDCVoltage, kSwitchingPeriod);
  EXPECT_DOUBLE_EQ(voltage_a - voltage_b, kAReference - kBReference);
  EXPECT_DOUBLE_EQ(voltage_b - voltage_c, kBReference - kCReference);
  EXPECT_DOUBLE_EQ(voltage_c - voltage_a, kCReference - kAReference);
}


class SwitchOperationToVoltageTest
  : public ::testing::TestWithParam<std::pair<SwitchOperation, double>> {};

TEST_P(SwitchOperationToVoltageTest, CalculatesExpectedVoltage) {
  const double kDCVoltage = 1200.0;
  const SwitchOperation& switch_operation = GetParam().first;
  const double got = SwitchOperationToVoltage(switch_operation, kDCVoltage);
  const double want = GetParam().second;
  EXPECT_DOUBLE_EQ(got, want);
}

INSTANTIATE_TEST_CASE_P(CalculatesExpectedVoltage,
                        SwitchOperationToVoltageTest,
                        ::testing::Values(
                            std::make_pair(SwitchOperation::HI, +600.0),
                            std::make_pair(SwitchOperation::LOW, -600.0)));

TEST(SwitchOperationToVoltageTest, FailsWithOff) {
  const double kDCVoltageDoesntMatter = 0.0;
  ASSERT_ANY_THROW(SwitchOperationToVoltage(SwitchOperation::OFF,
                                            kDCVoltageDoesntMatter));
}

class ModulationCommandsToVoltageTest
  : public ::testing::TestWithParam<std::pair<double, double>> {};

TEST_P(ModulationCommandsToVoltageTest, CalculatesCorrectly75PercentDuty) {
  const double& kTimeToSwitch = GetParam().first;
  std::vector<ModulationCommand> commands;
  const double kSwitchingPeriod = 1.0;
  const double kMidtimeSwitch = kTimeToSwitch;
  const double kStartTimeSwitch = 0.0;
  commands.push_back(ModulationCommand{SwitchOperation::HI, kStartTimeSwitch});
  commands.push_back(ModulationCommand{SwitchOperation::LOW, kMidtimeSwitch});
  const double kDCVoltage = 1200.0;

  const double& kExpectedVoltage = GetParam().second;
  const double got = ModulationCommandsToVoltage(
      commands, kDCVoltage, kSwitchingPeriod);
  ASSERT_DOUBLE_EQ(got, kExpectedVoltage);
}

INSTANTIATE_TEST_CASE_P(CalculatesCorrectly75PercentDuty,
                        ModulationCommandsToVoltageTest,
                        ::testing::Values(
                            std::make_pair(0.50, 0.0),
                            std::make_pair(0.75, +300.0),
                            std::make_pair(0.25, -300.0),
                            std::make_pair(0.00, -600.0),
                            std::make_pair(1.00, +600.0)));

}  // namespace
}  // namespace control
}  // namespace electric_vehicle

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
