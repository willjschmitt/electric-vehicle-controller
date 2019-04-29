#include "control/modulation_commands.h"

#include <algorithm>
#include <stdexcept>

#include "signal_processing/clarke_transformations.h"

namespace electric_vehicle {
namespace control {

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

}  // namespace control
}  // namespace electric_vehicle
