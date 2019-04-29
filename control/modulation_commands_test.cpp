#include "control/modulation_commands.h"

#include "gtest/gtest.h"
#include "gmock/gmock.h"

namespace electric_vehicle {
namespace control {
namespace {

using ::testing::Eq;
using ::testing::Ne;


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

TEST(ModulationCommandBuffer, PeekEmpty) {
  const ModulationCommandBuffer<6> buffer;
  ASSERT_THAT(buffer.Peek(), Eq(nullptr));
}

TEST(ModulationCommandBuffer, PeekAfterInsert) {
  ModulationCommandBuffer<6> buffer;
  const ModulationCommand command{ SwitchOperation::HI, 0.0 };
  buffer.PushBack(command);
  ASSERT_THAT(buffer.Peek()->operation, Eq(command.operation));
  ASSERT_THAT(buffer.Peek()->time, Eq(command.time));
}

TEST(ModulationCommandBuffer, PopEmpty) {
  ModulationCommandBuffer<6> buffer;
  ASSERT_ANY_THROW(buffer.Pop());
}

TEST(ModulationCommandBuffer, PopThenPeekIsEmpty) {
  ModulationCommandBuffer<6> buffer;
  buffer.PushBack({ SwitchOperation::HI, 0.0 });
  ASSERT_THAT(buffer.Peek(), Ne(nullptr));
  buffer.Pop();
  ASSERT_THAT(buffer.Peek(), Eq(nullptr));
}

TEST(ModulationCommandBuffer, BufferWrappingAroundEnd) {
  ModulationCommandBuffer<6> buffer;

  std::array<ModulationCommand, 8> commands;
  for (int i = 0; i < 8; ++i) {
    commands[i] = { SwitchOperation::HI, (double)i };
  }

  // First fill the buffer completely.
  for (int i = 0; i < 6; ++i) {
    buffer.PushBack(commands[i]);
  }

  // Pop two from the front, so now our starting index will be in the middle of
  // the buffer.
  for (int i = 0; i < 2; ++i) {
    ASSERT_THAT(buffer.Pop(), Eq(commands[i]));
  }

  // Add the last two objects, which will replace the first two slots in the
  // buffer array.
  for (int i = 6; i < 8; ++i) {
    buffer.PushBack(commands[i]);
  }

  // Check as we wrap around we continuously get the commands.
  for (int i = 2; i < 8; ++i) {
    ASSERT_THAT(buffer.Pop(), Eq(commands[i]));
  }
}

TEST(ModulationCommandBuffer, ThrowsAfterTooManyInserts) {
  ModulationCommandBuffer<6> buffer;
  for (int i = 0; i < 6; i++) {
    const ModulationCommand command{ SwitchOperation::HI, 0.0 };
    buffer.PushBack(command);
  }
  const ModulationCommand command{ SwitchOperation::HI, 0.0 };
  ASSERT_ANY_THROW(buffer.PushBack(command));
}

class ModulationCommandsToVoltageTest
  : public ::testing::TestWithParam<std::pair<double, double>> {};

TEST_P(ModulationCommandsToVoltageTest, CalculatesCorrectly75PercentDuty) {
  const double& kTimeToSwitch = GetParam().first;
  ModulationCommandBuffer<6> commands;
  const double kSwitchingPeriod = 1.0;
  const double kMidtimeSwitch = kTimeToSwitch;
  const double kStartTimeSwitch = 0.0;
  commands.PushBack(ModulationCommand{SwitchOperation::HI, kStartTimeSwitch});
  commands.PushBack(ModulationCommand{SwitchOperation::LOW, kMidtimeSwitch});
  commands.PushBack(ModulationCommand{SwitchOperation::HI, kStartTimeSwitch + 1.0});
  commands.PushBack(ModulationCommand{SwitchOperation::LOW, kMidtimeSwitch  + 1.0});
  commands.PushBack(ModulationCommand{SwitchOperation::HI, kStartTimeSwitch + 2.0});
  commands.PushBack(ModulationCommand{SwitchOperation::LOW, kMidtimeSwitch + 2.0});
  const double kDCVoltage = 1200.0;

  const double& kExpectedVoltage = GetParam().second;
  const double got = commands.ModulationCommandsToVoltage(
      kDCVoltage, kSwitchingPeriod);
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
