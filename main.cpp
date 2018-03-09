#include "config.h"

#include <xc.h>

#include "control/current_regulator.h"
#include "control/timer.h"
#include "induction_motor_controls/induction_motor_controller.h"
#include "machines/induction_machine.h"
#include "measurement/throttle.h"

using ::electric_vehicle::control::CurrentRegulator;
using ::electric_vehicle::control::SamplingTimer;
using ::electric_vehicle::induction_motor_controls::InductionMotorController;
using ::electric_vehicle::machines::InductionMachine;
using ::electric_vehicle::measurement::PIC32DcVoltageSampler;
using ::electric_vehicle::measurement::PIC32MechanicalSpeedSampler;
using ::electric_vehicle::measurement::PIC32ThreePhaseCurrentSampler;
using ::electric_vehicle::measurement::PIC32ThrottleSampler;

int main(int argc, char** argv) {
  SamplingTimer timer;
  const PIC32ThrottleSampler throttle_sampler;
  const PIC32MechanicalSpeedSampler speed_sampler;
  const PIC32ThreePhaseCurrentSampler current_sampler;
  const PIC32DcVoltageSampler dc_voltage_sampler;
  
  constexpr double kMagnetizingInductance = 0.0;
  constexpr double kStatorInductance = 0.0;
  constexpr double kStatorResistance = 0.0;
  constexpr double kRotorInductance = 0.0;
  constexpr double kRotorResistance = 0.0;
  constexpr unsigned int kNumberOfPoles = 2;
  const InductionMachine induction_machine(
      kMagnetizingInductance, kStatorInductance, kStatorResistance,
      kRotorInductance, kRotorResistance, kNumberOfPoles);
  
  constexpr double kCurrentRegulatorProportionalGain = 0.0;
  constexpr double kCurrentRegulatorIntegralGain = 0.0;
  constexpr double kCurrentRegulatorDirectMinimum = -1.0;
  constexpr double kCurrentRegulatorDirectMaximum = +1.0;
  constexpr double kCurrentRegulatorQuadratureMinimum = -1.0;
  constexpr double kCurrentRegulatorQuadratureMaximum = +1.0;
  const CurrentRegulator current_regulator(
      &timer, kCurrentRegulatorProportionalGain,
      kCurrentRegulatorIntegralGain, kCurrentRegulatorDirectMinimum,
      kCurrentRegulatorDirectMaximum, kCurrentRegulatorQuadratureMinimum,
      kCurrentRegulatorQuadratureMaximum);
  
  constexpr double kCoreControlsTaskRate = 200E-6;
  
  InductionMotorController controller(
      &timer, &throttle_sampler, &speed_sampler, &current_sampler,
      &dc_voltage_sampler, induction_machine, current_regulator,
      kCoreControlsTaskRate);
    
  while(1) {};
  return 0;
}
