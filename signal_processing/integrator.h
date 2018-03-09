#ifndef SIGNAL_PROCESSING__INTEGRATOR__H
#define SIGNAL_PROCESSING__INTEGRATOR__H

#include <cfloat>

#include "control/timer.h"

namespace electric_vehicle {
namespace signal_processing {

class IntegratorInterface {
  // Solves an integration step from the input.
  virtual double Integrate(const double& input) = 0;
};

// Integrator representing a transfer function 1/s.
class Integrator : public IntegratorInterface {
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
  double Integrate(const double& input) override;

 private:
  // Retrieves the delta time since the last time the instance was solved.
  double DeltaTimestep() const;

  // Samples and saves the curreent evaluation time as the last evaluation time.
  void SampleLastEvaluationTime();

  // Timer used for sampling time between estimation calls.
  ::electric_vehicle::control::TimerInterface* timer_;

  // Limits on the integrator.
  const double minimum_;
  const double maximum_;

  // Last time the block was solved. Not initialized until Solve is called.
  double last_evaluation_time_;

  // Boolean indicating if the instance has been solved at least once yet.
  // Used for intializting steps.
  bool evaluated_;

  // Running internal value of the integration.
  double q_integral_;
};

// Integrator representing a transfer function 1/s with continuous bounds. That
// is, when the maxium is reached, the integration loops to the minimum
// contiuously. This is useful for integrations of frequency into angle.
class LoopingIntegrator : public IntegratorInterface {
 public:
  LoopingIntegrator(
      ::electric_vehicle::control::TimerInterface* timer,
      const double& minimum, const double& maximum)
      : timer_(timer),
        minimum_(minimum),
        maximum_(maximum),
        evaluated_(false),
        q_integral_(0.0) {}

  // Solves an integration step from the input.
  double Integrate(const double& input) override;

 private:
  // Retrieves the delta time since the last time the instance was solved.
  double DeltaTimestep() const;

  // Samples and saves the curreent evaluation time as the last evaluation time.
  void SampleLastEvaluationTime();

  // Timer used for sampling time between estimation calls.
  ::electric_vehicle::control::TimerInterface* timer_;

  // Limits on the integrator.
  const double minimum_;
  const double maximum_;

  // Last time the block was solved. Not initialized until Solve is called.
  double last_evaluation_time_;

  // Boolean indicating if the instance has been solved at least once yet.
  // Used for intializting steps.
  bool evaluated_;

  // Running internal value of the integration.
  double q_integral_;
};

}  // namespace signal_processing
}  // namespace electric_vehicle

#endif  // SIGNAL_PROCESSING__INTEGRATOR__H
