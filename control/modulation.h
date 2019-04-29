#ifndef CONTROL__MODULATION__H
#define CONTROL__MODULATION__H

#include <array>
#include <map>
#include <vector>

#include "control/modulation_commands.h"
#include "signal_processing/clarke_transformations.h"

namespace electric_vehicle {
namespace control {

// Modulates a three-phase instantaneous voltage reference into modulation
// commands to turn IGBTs on and off
class TwoLevelModulatorInterface {
 public:
  virtual ModulationCommands<6> Modulate(
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

  ModulationCommands<6> Modulate(
      const ::electric_vehicle::signal_processing::ThreePhase&
          modulation_references,
      const double& dc_voltage) const override;

 private:
  double switching_period_;
};

}  // namespace control
}  // namespace electric_vehicle

#endif  // CONTROL__MODULATION__H
