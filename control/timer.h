#ifndef CONTROL__TIMER__H
#define CONTROL__TIMER__H

namespace electric_vehicle {
namespace control {

// An interface for timing functionality in a discrete control system. Some
// implementions might choose to implement a timer which returns a sampled time
// for a control step, whereas others might always return the current machine
// time.
class TimerInterface {
 public:
  // Provides the current time to use for discrete controls representations.
  virtual const double Time() = 0;
};


// A Timer, which provides the system time anytime Time() is invoked.
class AbsoluteTimer : public TimerInterface {
 public:
  AbsoluteTimer() {}

  const double Time() override;
};

// A Timer, which provides a time, which can be set discretely. For example in
// fixed timestep controls, the control elements throughout the timestep may
// want to share the same time for all actions, rather than differing from the
// timestep execution time and whatever computation time delayed the execution
// of an individual block in the timestep.
class SettableTimer : public TimerInterface {
 public:
  SettableTimer() {}
  
  const double Time() override;

  // Sets the time to be returned for any subsequent calls to Time.
  void SetTime(const double& time);

 private:
  // The time, which has been set on the Timer, and will be returned anytime
  // Time() is invoked.
  double time_;
};

// A timer, which will sample the current time whenever Sample is called and
// use that time for all subsequent calls to Time().
class SamplingTimer : public TimerInterface {
 public:
  SamplingTimer() {}

  const double Time() override;

  // Samples the current time and uses it for any subsequent calls to Time().
  void SampleTime();

 private:
  // The time, which has been set on the Timer by SampleTime, which will be
  // returned anytime Time() is invoked.
  double time_;
};

}  // namespace control
}  // namespace electric_vehicle

#endif  // CONTROL__TIMER__H