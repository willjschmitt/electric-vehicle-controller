#ifndef SIGNAL_PROCESSING__INTEGRATOR__H
#define SIGNAL_PROCESSING__INTEGRATOR__H

#include <cfloat>
#include <chrono>

#include "control/timer.h"

namespace electric_vehicle {
namespace signal_processing {

// Integrator representing a transfer function 1/s.
class Integrator {
 public:
  Integrator(
      ::electric_vehicle::control::TimerInterface* timer,
      const double& minimum = -DBL_MAX, const double& maximum = DBL_MAX)
      : timer_(timer),
        minimum_(minimum),
        maximum_(maximum),
        evaluated_(false),
        q_integral_(0.0) {}

  // Solves an integration step from the input.
  double Integrate(const double& input);

 private:
  // Retrieves the delta time since the last time the instance was solved.
  std::chrono::duration<double> DeltaTimestep();

  // Samples and saves the curreent evaluation time as the last evaluation time.
  void SampleLastEvaluationTime();

  // Timer used for sampling time between estimation calls.
  ::electric_vehicle::control::TimerInterface* timer_;

  // Limits on the integrator.
  const double minimum_;
  const double maximum_;

  // Last time the block was solved. Not initialized until Solve is called.
  std::chrono::system_clock::time_point last_evaluation_time_;

  // Boolean indicating if the instance has been solved at least once yet.
  // Used for intializting steps.
  bool evaluated_;

  // Running internal value of the integration.
  double q_integral_;
};

}  // namespace signal_processing
}  // namespace electric_vehicle

#endif  // SIGNAL_PROCESSING__INTEGRATOR__H
