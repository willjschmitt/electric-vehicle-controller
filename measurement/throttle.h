#ifndef MEASUREMENT__THROTTLE__H
#define MEASUREMENT__THROTTLE__H

namespace electric_vehicle {
namespace measurement {

// Interface to the mechanical throttle the user will use to request torque.
class ThrottleInterface {
 public:
  // Samples the throttle value for a per-unitized 0-100% throttle position.
  virtual double Sample() const = 0;
};

}  // namespace measurement
}  // namespace electric_vehicle

#endif  // MEASUREMENT__THROTTLE__H