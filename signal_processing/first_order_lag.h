#ifndef SIGNAL_PROCESSING__FIRST_ORDER_LAG__H
#define SIGNAL_PROCESSING__FIRST_ORDER_LAG__H

#include "control/timer.h"

namespace electric_vehicle {
namespace signal_processing {

using ::electric_vehicle::control::TimerInterface;

// A First-order filter with the transfer function (1/1+sT).
class FirstOrderLag {
 public:
  FirstOrderLag(TimerInterface* timer, const double& tau)
      : timer_(timer),
        tau_(tau),
        filtered_last_(0.0),
        evaluated_ (false) {}

  double Solve(const double& unfiltered);

 private:
  // Retrieves the delta time since the last time the instance was solved.
  double DeltaTimestep() const;

  // Samples and saves the current evaluation time as the last evaluation time.
  void SampleLastEvaluationTime();

  TimerInterface* timer_;
  double tau_;

  double filtered_last_;

  // Boolean indicating if the instance has been solved at least once yet.
  // Used for intializting steps.
  bool evaluated_;

  // Last time the block was solved. Not initialized until Solve is called.
  double last_evaluation_time_;
};

}  // namespace signal_processing
}  // namespace electric_vehicle

#endif  // SIGNAL_PROCESSING__FIRST_ORDER_LAG__H
