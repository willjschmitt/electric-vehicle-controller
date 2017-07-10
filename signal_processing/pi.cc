#include "signal_processing/pi.h"

#include <chrono>
#include <ctime>
#include <iostream>

namespace electric_vehicle {
namespace signal_processing {

double ProportionalIntegralController::Solve(const double& input_actual,
                                             const double& input_reference) {
  // If the instance hasn't been solved yet, output no value, so we can
  // initialize calculation times.
  if (!evaluated_) {
    SampleLastEvaluationTime();
    evaluated_ = true;
    return 0.0;
  }
  
  // Calculate error and PI component regulation actions.
  double error = input_reference - input_actual;
  const double q_proportional = error * gain_proportional_;
  q_integral_ += error * gain_integral_ * DeltaTimestep().count();

  // Apply limits with anti-windup logic by limiting the integral action.
  const double q_unlimited = q_integral_ + q_proportional;
  if (q_unlimited > maximum_) {
    q_integral_ = maximum_ - q_proportional;
  }
  if (q_unlimited < minimum_) {
    q_integral_ = minimum_ - q_proportional;
  }

  // Re-add the proportional and integral actions after the limiting
  // + anti-windup logic has been applied to the integral component.
  const double q_limited = q_integral_ + q_proportional;

  SampleLastEvaluationTime();
  return q_limited;
}

std::chrono::duration<double> ProportionalIntegralController::DeltaTimestep() {
  return timer_->Time() - last_evaluation_time_;
}

void ProportionalIntegralController::SampleLastEvaluationTime() {
  last_evaluation_time_ = timer_->Time();
}

}  // namespace signal_processing
}  // namespace electric_vehicle
