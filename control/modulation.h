#ifndef CONTROL__MODULATION__H
#define CONTROL__MODULATION__H

#include <array>
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
  double time;
};

// A Map of phases to a size-6 array of modulation commands to execute. Size is
// set to 6 to support turn on/off in one cycle and then 3 cycles (finishing
// cycle, current cycle, next cycle).
using ModulationCommands = std::map<Phase, std::array<ModulationCommand, 6>>;

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
  TwoLevelSineModulator(const double& switching_period)
    : switching_period_(switching_period) {}

  ModulationCommands Modulate(
      const ::electric_vehicle::signal_processing::ThreePhase&
          modulation_references,
      const double& dc_voltage) const override;

 private:
  double switching_period_;
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
    const double& switching_period);

}  // namespace control
}  // namespace electric_vehicle

#endif  // CONTROL__MODULATION__H
