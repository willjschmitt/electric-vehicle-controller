#ifndef CONTROL__CURRENT_REGULATOR__H
#define CONTROL__CURRENT_REGULATOR__H

#include "control/timer.h"
#include "signal_processing/clarke_transformations.h"
#include "signal_processing/pi.h"

namespace electric_vehicle {
namespace control {

class CurrentRegulator {
 public:
  CurrentRegulator(
      TimerInterface* timer,
      const double& proportional_gain, const double& integral_gain) :
    direct_axis_regulator_(timer, proportional_gain, integral_gain),
    quadrature_axis_regulator_(timer, proportional_gain, integral_gain) {}

  ::electric_vehicle::signal_processing::DirectQuadrature Regulate(
      const ::electric_vehicle::signal_processing::DirectQuadrature&
          current_reference,
      const ::electric_vehicle::signal_processing::DirectQuadrature&
          current_measured);

 private:
  // PI regulators for regulating Id and Iq (direct and quadrature axes).
  ::electric_vehicle::signal_processing::ProportionalIntegralController
      direct_axis_regulator_;
  ::electric_vehicle::signal_processing::ProportionalIntegralController
      quadrature_axis_regulator_;
};

}  // namespace control
}  // namespace electric_vehicle

#endif  // CONTROL__CURRENT_REGULATOR__H
