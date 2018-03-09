#include "signal_processing/integrator.h"

namespace electric_vehicle {
namespace signal_processing {

double Integrator::Integrate(const double& input) {
  // If the instance hasn't been solved yet, output no value, so we can
  // initialize calculation times.
  if (!evaluated_) {
    SampleLastEvaluationTime();
    evaluated_ = true;
    return 0.0;
  }
  
  q_integral_ += input * DeltaTimestep();

  // Apply limits with anti-windup logic by keeping the internal value within
  // the limits.
  if (q_integral_ > maximum_) {
    q_integral_ = maximum_;
  }
  if (q_integral_ < minimum_) {
    q_integral_ = minimum_;
  }

  SampleLastEvaluationTime();
  return q_integral_;
}

double Integrator::DeltaTimestep() const {
  return timer_->Time() - last_evaluation_time_;
}

void Integrator::SampleLastEvaluationTime() {
  last_evaluation_time_ = timer_->Time();
}

double LoopingIntegrator::Integrate(const double& input) {
  // If the instance hasn't been solved yet, output no value, so we can
  // initialize calculation times.
  if (!evaluated_) {
    SampleLastEvaluationTime();
    evaluated_ = true;
    return 0.0;
  }
  
  q_integral_ += input * DeltaTimestep();

  // Apply limits with anti-windup logic by keeping the internal value within
  // the limits.
  if (q_integral_ > maximum_) {
    q_integral_ -= (maximum_ - minimum_);
  }
  if (q_integral_ < minimum_) {
    q_integral_ += (maximum_ - minimum_);
  }

  SampleLastEvaluationTime();
  return q_integral_;
}

double LoopingIntegrator::DeltaTimestep() const {
  return timer_->Time() - last_evaluation_time_;
}

void LoopingIntegrator::SampleLastEvaluationTime() {
  last_evaluation_time_ = timer_->Time();
}

}  // namespace signal_processing
}  // namespace electric_vehicle
