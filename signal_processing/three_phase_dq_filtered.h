#ifndef SIGNAL_PROCESSING__THREE_PHASE_DQ_FILTERED_H
#define SIGNAL_PROCESSING__THREE_PHASE_DQ_FILTERED_H

#include "signal_processing/angle_math.h"
#include "signal_processing/clarke_transformations.h"
#include "signal_processing/two_dimensional_first_order_lag.h"

namespace electric_vehicle {
namespace signal_processing {

// A combined dq-transform with filter.
// Filter is applied on the dq (DC) values.
class ThreePhaseDQFiltered {
 public:
  ThreePhaseDQFiltered(TimerInterface* const timer, const double& tau)
      : dq_filter_(timer, tau) {}

  // Converts the three phase values to dq-axis using theta, filtering them,
  // returning the filtered value.
  inline DirectQuadrature Solve(
      const ThreePhase& three_phase, const double theta) {
    three_phase_ = three_phase;
    const DirectQuadrature dq = ParkTransformation(three_phase, theta);
    return DirectQuadrature(dq_filter_.Solve(dq.XY()));
  }

  // Return the last three_phase value passed to Solve.
  inline const ThreePhase& ThreePhaseLast() const {
    return three_phase_;
  }

  // Return the DQ value for the last value of three_phase passed to Solve.
  inline const TwoDimensionalVector DQUnfilteredLast() const {
    return dq_filter_.UnfilteredLast();
  }

 private:
  ThreePhase three_phase_;
  TwoDimensionalFirstOrderLag dq_filter_;
};

}  // namespace signal_processing
}  // namespace electric_vehicle

#endif  // SIGNAL_PROCESSING_THREE_PHASE_DQ_FILTERED_H
