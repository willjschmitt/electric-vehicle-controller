#include "signal_processing/pi.h"

#include <iostream>

namespace electric_vehicle {
namespace signal_processing {

double ProportionalIntegralController::Solve(const double& input_actual,
                                             const double& input_reference) {
  double error = input_reference - input_actual;
  
  const double q_proportional = error * gain_proportional_;
  q_integral_ += error * gain_integral_ * delta_timestep_;

  return q_proportional + q_integral_;
}

}  // namespace signal_processing
}  // namespace electric_vehicle
