#include "control/timer.h"

#include <chrono>

namespace electric_vehicle {
namespace control {

using std::chrono::system_clock;

const system_clock::time_point AbsoluteTimer::Time() {
  std::chrono::system_clock::now();
  return system_clock::now();
}

const system_clock::time_point SettableTimer::Time() {
  return time_;
}

void SettableTimer::SetTime(system_clock::time_point time){
  time_ = time;
}

const system_clock::time_point SamplingTimer::Time() {
  return time_;
}

void SamplingTimer::SampleTime() {
  time_ = system_clock::now();
}

}  // namespace control
}  // namespace electric_vehicle
