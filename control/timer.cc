#include "control/timer.h"

#include <ctime>

namespace electric_vehicle {
namespace control {

const std::time_t AbsoluteTimer::Time() {
  return std::clock();
}

const std::time_t SettableTimer::Time() {
  return time_;
}

void SettableTimer::SetTime(std::time_t time){
  time_ = time;
}

const std::time_t SamplingTimer::Time() {
  return time_;
}

void SamplingTimer::SampleTime() {
  time_ = std::clock();
}

}  // namespace control
}  // namespace electric_vehicle
