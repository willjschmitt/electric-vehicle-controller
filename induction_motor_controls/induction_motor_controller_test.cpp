#include "induction_motor_controls/induction_motor_controller.h"

#include <chrono>
#include <iostream>
#include <memory>

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

static const double kCoreControlsTaskRate = 100E-6;

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

class InductionMotorControllerTest : public ::testing::Test {
 protected:
  void SetUp() {
    
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
    throttle_sampler.reset(new StubThrottle());
    speed_sampler.reset(new StubMechanicalSpeed());
    current_sampler.reset(new StubThreePhaseMeasurement());
    dc_voltage_sampler.reset(new StubDcVoltageMeasurement());
    controller.reset(new InductionMotorController(
        &timer, throttle_sampler.get(), speed_sampler.get(),
        current_sampler.get(), dc_voltage_sampler.get(),
        induction_machine, current_regulator,
        kCoreControlsTaskRate));
  }

  SettableTimer timer;
  std::unique_ptr<StubThrottle> throttle_sampler;
  std::unique_ptr<StubMechanicalSpeed> speed_sampler;
  std::unique_ptr<StubThreePhaseMeasurement> current_sampler;
  std::unique_ptr<StubDcVoltageMeasurement> dc_voltage_sampler;
  std::unique_ptr<InductionMotorController> controller;
};

// This test is mostly to enforce compilation and linking for the composite
// elements.
TEST_F(InductionMotorControllerTest, Creates) {
  const ModulationCommands<6> modulation_commands = controller->CoreControlsTask();
}

// Quickly checks that the main control loop executes quickly enough.
// This is likely to be flaky, but it does put some downward pressure on time
// to identify when the computing takes too long. It's performance is, of
// course, dependent on the system. Adjust the count of tries and acceptable
// time as needed.
TEST_F(InductionMotorControllerTest, PerformanceTest) {
  const unsigned int kNumberOfTimes = (unsigned int)1E2;
  const std::chrono::system_clock::time_point start_time
      = std::chrono::system_clock::now();
  for (unsigned int i = 0; i < kNumberOfTimes; i++) {
    const ModulationCommands<6> modulation_commands = controller->CoreControlsTask();
  }
  const std::chrono::system_clock::time_point end_time
      = std::chrono::system_clock::now();
  const std::chrono::system_clock::duration total_time = end_time - start_time;
  const long long time_per_run
      = std::chrono::duration_cast<std::chrono::seconds>(total_time).count() / kNumberOfTimes;
  EXPECT_LE(time_per_run, kCoreControlsTaskRate);
  std::cout << "============PERFORMANCE============" << std::endl;
  std::cout << "Total time: " << total_time.count() << " seconds" << std::endl;
  std::cout << "Timer per run: " << time_per_run << " uSeconds" << std::endl;
  std::cout << "===============END=================" << std::endl;
}

}  // namespace
}  // namespace induction_motor_controls
}  // namespace electric_vehicle
