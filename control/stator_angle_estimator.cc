#include "control/stator_angle_estimator.h"

namespace electric_vehicle {
namespace control {

double StatorAngleEstimator::Estimate(
    const double& mechanical_speed, const double& direct_current_stator,
    const double& quadrature_current_stator) {
  const double tau_rotor = rotor_inductance_ / rotor_resistance_;
  const unsigned int number_pole_pairs = number_poles_ / 2;
  const double electrical_speed = 
      (mechanical_speed / (double)number_pole_pairs)
      + ((1.0 / tau_rotor)
         * (quadrature_current_stator / direct_current_stator));
  return integrator_.Integrate(electrical_speed);
}

}  // namespace control
}  // namespace electric_vehicle
