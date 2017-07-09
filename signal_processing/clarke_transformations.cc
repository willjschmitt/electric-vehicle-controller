#include "signal_processing/clarke_transformations.h"

#include <exception>
#include <vector>

#include "signal_processing/math_constants.h"
#include "signal_processing/angle_math.h"

namespace electric_vehicle {
namespace signal_processing {

AlphaBeta ClarkeTransformation(ThreePhase three_phase) {
  AlphaBeta transformed;
  transformed.alpha = three_phase.A() - 0.5 * (three_phase.B() + three_phase.C());
  transformed.beta = kSqrt3Over2 * (three_phase.B() - three_phase.C());
  transformed.gamma = 0.5 * (three_phase.A() + three_phase.B() + three_phase.C());
  transformed.alpha *= 2.0 / 3.0;
  transformed.beta *= 2.0 / 3.0;
  transformed.gamma *= 2.0 / 3.0;
  return transformed;
}

DirectQuadrature ParkTransformation(const ThreePhase& three_phase,
                                    const double& theta) {
  DirectQuadrature dq0;
  const AlphaBeta alpha_beta = ClarkeTransformation(three_phase);
  const TwoDimensionalVector unrotated{ alpha_beta.alpha, alpha_beta.beta };
  const TwoDimensionalVector rotated = Rotate(unrotated, -theta);
  dq0.direct = rotated.x;
  dq0.quadrature = rotated.y;
  dq0.zero = alpha_beta.gamma;
  return dq0;
}

}  // namespace signal_processing
}  // namespace electric_vehicle
