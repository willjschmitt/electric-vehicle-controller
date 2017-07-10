#ifndef CONTROL__MODULATION__H
#define CONTROL__MODULATION__H

#include <chrono>
#include <ctime>
#include <map>
#include <vector>

#include "signal_processing/clarke_transformations.h"

namespace electric_vehicle {
namespace control {

// Phase naming in a three-phase system.
enum Phase {
  A = 0,
  B = 1,
  C = 2,
};

// Switch command type for which switches to enable.
enum SwitchOperation {
  OFF,  // No IGBTs enabled.
  HI,   // Only the Positive IGBT enabled.
  LOW,  // Only the Negative IGBT enabled.
};

// A single command for a switching operation at a given time relative to the
// switching period.
struct ModulationCommand {
  // Action to take at the provided time.
  SwitchOperation operation;

  // Time to execute the action relative to the base of the switching period.
  std::chrono::duration<double> time;
};

// A Map of phases to a vector of modulation commands to execute.
using ModulationCommands = std::map<Phase, std::vector<ModulationCommand>>;

// Modulates a three-phase instantaneous voltage reference into modulation
// commands to turn IGBTs on and off
class TwoLevelModulatorInterface {
 public:
  virtual ModulationCommands Modulate(
      const ::electric_vehicle::signal_processing::ThreePhase&
          modulation_references,
      const double& dc_voltage) const = 0;
};

// A simple modulator with sine PWM. Adds a third harmonic into the PWM to
// extend PWM range.
class TwoLevelSineModulator : public TwoLevelModulatorInterface {
 public:
  TwoLevelSineModulator(
    const std::chrono::duration<double>& switching_period)
    : switching_period_(switching_period) {}

  ModulationCommands Modulate(
      const ::electric_vehicle::signal_processing::ThreePhase&
          modulation_references,
      const double& dc_voltage) const override;

 private:
  const std::chrono::duration<double> switching_period_;

  // Converts a duty cycle into a +V, -V set of switching commands.
  std::vector<ModulationCommand> DutyCycleToCommands(
    const double& duty_cycle) const;
};

// Converts a SwitchOperation into's it's generated voltage, using the middle
// point of the capacitor as a reference ground.
double SwitchOperationToVoltage(const SwitchOperation& switch_operation,
                                const double& dc_voltage);

// Converts a series of ModulationCommand's into a voltage. First switch
// operation must be at 0.0.
double ModulationCommandsToVoltage(
    const std::vector<ModulationCommand>& commands,
    const double& dc_voltage,
    const std::chrono::duration<double>& switching_period);

}  // namespace control
}  // namespace electric_vehicle

#endif  // CONTROL__MODULATION__H
