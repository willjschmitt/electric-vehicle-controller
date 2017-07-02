#include "signal_processing/pi.h"

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
  
  double error = input_reference - input_actual;
  
  const double q_proportional = error * gain_proportional_;
  q_integral_ += error * gain_integral_ * DeltaTimestep();

  SampleLastEvaluationTime();
  return q_proportional + q_integral_;
}

double ProportionalIntegralController::DeltaTimestep() {
  return std::difftime(timer_->Time(), last_evaluation_time_);
}

void ProportionalIntegralController::SampleLastEvaluationTime() {
  last_evaluation_time_ = timer_->Time();
}

}  // namespace signal_processing
}  // namespace electric_vehicle
