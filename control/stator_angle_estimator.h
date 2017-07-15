#ifndef CONTROL__STATOR_ANGLE_ESTIMATOR__H
#define CONTROL__STATOR_ANGLE_ESTIMATOR__H

#include "control/timer.h"
#include "signal_processing/integrator.h"
#include "signal_processing/math_constants.h"

namespace electric_vehicle {
namespace control {

// Estimates the stator angle/rotor flux angle, at which current will be
// transformed using a dq transformation as well as for the voltage to be
// generated using a reverse dq transformation.
class StatorAngleEstimator {
 public:
  StatorAngleEstimator(
    TimerInterface* timer, const double& number_poles,
    const double& rotor_inductance, const double& rotor_resistance)
    : integrator_(timer, ::electric_vehicle::signal_processing::kZero,
      ::electric_vehicle::signal_processing::k2Pi),
      number_poles_(number_poles),
      rotor_inductance_(rotor_inductance),
      rotor_resistance_(rotor_resistance) {}

  // Estimates the stator electrical angle based on the current injected into
  // it and the mechanical speed of the rotor.
  double Estimate(
      const double& mechanical_speed, const double& direct_current_stator,
      const double& quadrature_current_stator);

 private:
  // Timer used for sampling time between estimation calls.
  ::electric_vehicle::signal_processing::LoopingIntegrator integrator_;

  // Number of poles (not pole pairs).
  const unsigned int number_poles_;

  // Inductance of the rotor (in Henries).
  const double rotor_inductance_;

  // Inductance of the rotor, on a stator reference (in Ohms).
  const double rotor_resistance_;
};

}  // namespace control
}  // namespace electric_vehicle

#endif  // CONTROL__STATOR_ANGLE_ESTIMATOR__H
