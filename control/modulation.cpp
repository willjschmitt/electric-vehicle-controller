#include "control/modulation.h"

#include <algorithm>
#include <stdexcept>

#include "signal_processing/clarke_transformations.h"

namespace electric_vehicle {
namespace control {

using ::electric_vehicle::signal_processing::ThreePhase;

namespace {

// Converts a integer form of a phase into a phase enum.
Phase IntegerToPhase(const int& integer_phase) {
  switch (integer_phase) {
  case 0:
    return Phase::A;
  case 1:
    return Phase::B;
  case 2:
    return Phase::C;
  default:
    throw std::invalid_argument("Invalid phase index.");
  }
}

// Adds third harmonic to references by subtracting an average of the max and
// min elements of the input vector.
ThreePhase AddThirdHarmonic(ThreePhase raw_voltages) {
  const double voltage_max = *std::max_element(
      raw_voltages.begin(), raw_voltages.end());
  const double voltage_min = *std::min_element(
      raw_voltages.begin(), raw_voltages.end());

  const double third_harmonic = (voltage_max + voltage_min) / 2.0;

  ThreePhase voltages_with_third_harmonic;
  for (unsigned int i = 0; i < voltages_with_third_harmonic.size(); i++) {
    const double& voltage = raw_voltages[i];
    voltages_with_third_harmonic[i] = voltage - third_harmonic;
  }
  return voltages_with_third_harmonic;
}

// Converts a voltage reference into a duty cycle given an ac voltage reference
// and dc voltage measurement, allowing. Assumes 2 level modulation, where duty
// cycle represents time pulsing high vs low.
double TwoLevelSineModulationDutyCycle(
    const double& ac_voltage_reference, const double& dc_voltage) {
  return ac_voltage_reference / dc_voltage + (1.0 / 2.0);
}

}  // namespace

ModulationCommands TwoLevelSineModulator::Modulate(
    const ThreePhase& voltage_references, const double& dc_voltage) const {
  const ThreePhase& voltages_with_third_harmonic
      = AddThirdHarmonic(voltage_references);
  
  // Load the commands per phase into the return structure.
  ModulationCommands commands;
  for (int i = 0; i < 3; i++) {
    const Phase phase = IntegerToPhase(i);
    const double ac_voltage_reference = voltages_with_third_harmonic[i];
    const double& duty_cycle = TwoLevelSineModulationDutyCycle(
        ac_voltage_reference, dc_voltage);
    
    // Always start with the switch turned on.
    // TODO(#1): Add deadband to stay off below a certain duty cycle.
    ModulationCommand turn_on;
    turn_on.operation = SwitchOperation::HI;
    turn_on.time = 0.0;
  
    // Turn the switch off at the duty cycle time.
    // TODO(#1): Add deadband to keep on above a certain duty cycle.
    ModulationCommand turn_off;
    turn_off.operation = SwitchOperation::LOW;
    turn_off.time = duty_cycle * switching_period_;
    
    commands[phase] = {turn_on, turn_off};
  }
  return commands;
}

double SwitchOperationToVoltage(const SwitchOperation& switch_operation,
                                const double& dc_voltage) {
  switch (switch_operation) {
  case SwitchOperation::OFF:
    throw "Invalid SwitchOperation OFF";
  case SwitchOperation::HI:
    return +dc_voltage / 2.0;
  case SwitchOperation::LOW:
    return -dc_voltage / 2.0;
  }
}

double ModulationCommandsToVoltage(
    const ModulationCommandBuffer& commands,
    const double& dc_voltage,
    const double& switching_period) {
  const double calculation_period = ceil(commands[commands.size() - 1].time / switching_period);

  double voltage = 0.0;
  for (unsigned int i = 0; i < commands.size(); i++) {
    const double delta_time = (i == commands.size() - 1)
		                      ? calculation_period - commands[commands.size() - 1].time
		                      : commands[i + 1].time - commands[i].time;
    const double switching_voltage = SwitchOperationToVoltage(
        commands[i].operation, dc_voltage);
    voltage += switching_voltage * (delta_time / calculation_period);
  }

  return voltage;
}

}  // namespace control
}  // namespace electric_vehicle
