#ifndef SIGNAL_PROCESSING__PI__H_
#define SIGNAL_PROCESSING__PI__H_

#include <cfloat>

#include "control/timer.h"

namespace electric_vehicle {
namespace signal_processing {

// A PI regulator/controller, which has proportional and integral action to
// maintain a reference value.
class ProportionalIntegralController {
 public:
  ProportionalIntegralController(
      ::electric_vehicle::control::TimerInterface* timer,
      const double& gain_proportional, const double& gain_integral,
      const double& minimum = -DBL_MAX, const double& maximum = DBL_MAX) :
    timer_(timer),
    gain_proportional_(gain_proportional),
    gain_integral_(gain_integral),
    minimum_(minimum),
    maximum_(maximum),
    q_integral_(0.0),
    evaluated_(false) {}

  // Solves a current iteration of desired versus actual value for a single
  // timestep. If it is the first time calling Solve, no integration/
  // differentiation will occur.
  double Solve(const double& input_actual, const double& input_reference);

 private:
  // Retrieves the delta time since the last time the instance was solved.
  double DeltaTimestep() const;

  // Samples and saves the curreent evaluation time as the last evaluation time.
  void SampleLastEvaluationTime();
   
  // Timer used for calculation of solution of discrete control elements.
  ::electric_vehicle::control::TimerInterface* timer_;

  // The proportional and integral gains for control.
  const double gain_proportional_;
  const double gain_integral_;

  // Limits on the regulator output.
  const double minimum_;
  const double maximum_;

  double q_integral_;

  // Last time the block was solved. Not initialized until Solve is called.
  double last_evaluation_time_;

  // Boolean indicating if the instance has been solved at least once yet.
  // Used for intializting steps.
  bool evaluated_;
};

}  // namespace signal_processing
}  // namespace electric_vehicle

#endif  // SIGNAL_PROCESSING__PI__H_
