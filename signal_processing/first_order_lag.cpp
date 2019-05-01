#include "signal_processing/first_order_lag.h"

namespace electric_vehicle {
namespace signal_processing {

double FirstOrderLag::Solve(const double& unfiltered) {
  unfiltered_last_ = unfiltered;

  // If the instance hasn't been solved yet, output the input value, and
  // initialize the filter to the input value.
  if (!evaluated_) {
    SampleLastEvaluationTime();
    evaluated_ = true;
    filtered_last_ = unfiltered;
    return unfiltered;
  }

  filtered_last_ += (unfiltered - filtered_last_) * (DeltaTimestep() / tau_);

  SampleLastEvaluationTime();
  return filtered_last_;
}

double FirstOrderLag::DeltaTimestep() const {
  return timer_->Time() - last_evaluation_time_;
}

void FirstOrderLag::SampleLastEvaluationTime() {
  last_evaluation_time_ = timer_->Time();
}

}  // namespace signal_processing
}  // namespace electric_vehicle
