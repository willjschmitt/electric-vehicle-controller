#include "control/timer.h"

namespace electric_vehicle {
namespace control {

const double AbsoluteTimer::Time() {
  // TODO(willjschmitt): Reimplement.
  return 0.0;
}

const double SettableTimer::Time() {
  return time_;
}

void SettableTimer::SetTime(const double& time){
  time_ = time;
}

const double SamplingTimer::Time() {
  return time_;
}

void SamplingTimer::SampleTime() {
  // TODO(willjschmitt): Reimplement.
  time_ = 0.0;
}

}  // namespace control
}  // namespace electric_vehicle
