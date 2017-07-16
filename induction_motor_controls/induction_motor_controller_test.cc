#include "induction_motor_controls/induction_motor_controller.h"

#include "control/current_regulator.h"
#include "control/timer.h"
#include "gtest/gtest.h"
#include "machines/induction_machine.h"
#include "measurement/dc_voltage.h"
#include "measurement/mechanical_speed.h"
#include "measurement/three_phase_currents.h"
#include "measurement/throttle.h"
#include "signal_processing/clarke_transformations.h"

namespace electric_vehicle {
namespace induction_motor_controls {
namespace {

using ::electric_vehicle::control::CurrentRegulator;
using ::electric_vehicle::control::ModulationCommands;
using ::electric_vehicle::control::SettableTimer;
using ::electric_vehicle::machines::InductionMachine;
using ::electric_vehicle::measurement::DcVoltageMeasurementInterface;
using ::electric_vehicle::measurement::MechanicalSpeedInterface;
using ::electric_vehicle::measurement::ThreePhaseMeasurementInterface;
using ::electric_vehicle::measurement::ThrottleInterface;
using ::electric_vehicle::signal_processing::ThreePhase;
using ::std::chrono::microseconds;

class StubThrottle : public ThrottleInterface {
 public:
  double Sample() const { return throttle_; }

  double throttle_;
};

class StubMechanicalSpeed : public MechanicalSpeedInterface {
 public:
  double Sample() const { return speed_; }

  double speed_;
};

class StubThreePhaseMeasurement : public ThreePhaseMeasurementInterface {
 public:
  ThreePhase Sample() const { return currents_; }

  ThreePhase currents_;
};

class StubDcVoltageMeasurement : public DcVoltageMeasurementInterface {
 public:
  double Sample() const { return voltage_; }

  double voltage_;
};

// This test is mostly to enforce compilation and linking for the composite
// elements.
TEST(InductionMotorController, Creates) {
  SettableTimer timer;
  const StubThrottle throttle_sampler;
  const StubMechanicalSpeed speed_sampler;
  const StubThreePhaseMeasurement current_sampler;
  const StubDcVoltageMeasurement dc_voltage_sampler;
  // TODO(willjschmitt): Set this with real values when the test is actually
  // fleshed out.
  constexpr double kMagnetizingInductance = 0.0;
  constexpr double kStatorInductance = 0.0;
  constexpr double kStatorResistance = 0.0;
  constexpr double kRotorInductance = 0.0;
  constexpr double kRotorResistance = 0.0;
  constexpr unsigned int kNumberOfPoles = 2;
  const InductionMachine induction_machine(
      kMagnetizingInductance, kStatorInductance, kStatorResistance,
      kRotorInductance, kRotorResistance, kNumberOfPoles);
  // TODO(willjschmitt): Tune gains with real values when the test is actually
  // fleshed out.
  constexpr double kCurrentRegulatorProportionalGain = 1.0;
  constexpr double kCurrentRegulatorIntegralGain = 1000.0;
  constexpr double kDirectVoltageMaximum = 1.0;
  constexpr double kQuadratureVoltageMaximum  = 1.0;
  CurrentRegulator current_regulator(
      &timer, kCurrentRegulatorProportionalGain,
      kCurrentRegulatorIntegralGain, -kDirectVoltageMaximum,
      +kDirectVoltageMaximum, -kQuadratureVoltageMaximum,
      +kQuadratureVoltageMaximum);
  const microseconds kCoreControlsTaskRate(100);
  InductionMotorController controller(
      &timer, throttle_sampler, speed_sampler, current_sampler,
      dc_voltage_sampler, induction_machine, current_regulator,
      kCoreControlsTaskRate);

  const ModulationCommands modulation_commands = controller.CoreControlsTask();
}

}  // namespace
}  // namespace induction_motor_controls
}  // namespace electric_vehicle

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
