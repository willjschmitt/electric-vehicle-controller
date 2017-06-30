#ifndef SIGNAL_PROCESSING__PI__H_
#define SIGNAL_PROCESSING__PI__H_

namespace electric_vehicle {
namespace signal_processing {

// A PI regulator/controller, which has proportional and integral action to
// maintain a reference value.
class ProportionalIntegralController {
 public:
  ProportionalIntegralController(
      const double& gain_proportional, const double& gain_integral,
      const double& delta_timestep) :
    gain_proportional_(gain_proportional),
    gain_integral_(gain_integral),
    delta_timestep_(delta_timestep) {
    q_integral_ = 0.0;
  }

  // Solves a current iteration of desired versus actual value for a single
  // timestep.
  double Solve(const double& input_actual, const double& input_reference);

 private:
  // The proportional gain for control.
  const double gain_proportional_;

  // The integral gain for control.
  const double gain_integral_;

  // The timestep at which the Solve method will be called.
  const double delta_timestep_;

  double q_integral_;
};

}  // namespace signal_processing
}  // namespace electric_vehicle

#endif  // SIGNAL_PROCESSING__PI__H_
