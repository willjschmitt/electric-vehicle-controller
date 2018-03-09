#include "control/current_regulator.h"

#include "signal_processing/clarke_transformations.h"
#include "signal_processing/pi.h"

namespace electric_vehicle {
namespace control {

using ::electric_vehicle::signal_processing::DirectQuadrature;
  
DirectQuadrature CurrentRegulator::Regulate(
    const DirectQuadrature& current_reference,
    const DirectQuadrature& current_measured) {
  DirectQuadrature voltage_dq_reference;
  voltage_dq_reference.direct = direct_axis_regulator_.Solve(
      current_measured.direct, current_reference.direct);
  voltage_dq_reference.quadrature = quadrature_axis_regulator_.Solve(
      current_measured.quadrature, current_reference.quadrature);
  return voltage_dq_reference;
}

}  // namespace control
}  // namespace electric_vehicle
