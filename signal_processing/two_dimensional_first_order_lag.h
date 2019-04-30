#ifndef SIGNAL_PROCESSING__TWO_DIMENSIONAL_FIRST_ORDER_LAG_H
#define SIGNAL_PROCESSING__TWO_DIMENSIONAL_FIRST_ORDER_LAG_H

#include <array>

#include "control/timer.h"
#include "signal_processing/angle_math.h"
#include "signal_processing/first_order_lag.h"
#include "../control/timer.h"

namespace electric_vehicle {
namespace signal_processing {

using ::electric_vehicle::control::TimerInterface;

// Filters a 2-dimension value like DQ(direct-quadrature).
// Filters are applied independently on each axis.
class TwoDimensionalFirstOrderLag {
 public:
  TwoDimensionalFirstOrderLag(TimerInterface* const timer, const double& tau)
      : filters_{FirstOrderLag(timer, tau), FirstOrderLag(timer, tau)} {}

  inline TwoDimensionalVector TwoDimensionalFirstOrderLag::Solve(const TwoDimensionalVector& unfiltered) {
    return {
      filters_.at(0).Solve(unfiltered.x),
      filters_.at(1).Solve(unfiltered.y)
    };
  }

 private:
  std::array<FirstOrderLag, 2> filters_;
};

}  // namespace signal_processing
}  // namespace electric_vehicle

#endif // SIGNAL_PROCESSING__TWO_DIMENSIONAL_FIRST_ORDER_LAG_H
