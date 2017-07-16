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

ThreePhase InverseClarkeTransformation(const AlphaBeta& alpha_beta) {
  const double a = alpha_beta.alpha + alpha_beta.gamma;
  const double b = -0.5 * alpha_beta.alpha + kSqrt3Over2 * alpha_beta.beta
      + alpha_beta.gamma;
  const double c = -0.5 * alpha_beta.alpha - kSqrt3Over2 * alpha_beta.beta
      + alpha_beta.gamma;
  return ThreePhase(a, b, c);
}

ThreePhase InverseParkTransformation(const DirectQuadrature& direct_quadrature,
                                     const double& theta) {
  const TwoDimensionalVector xy_in_dq_frame{
    direct_quadrature.direct,
    direct_quadrature.quadrature,
  };
  const TwoDimensionalVector xy_in_ab_frame = Rotate(xy_in_dq_frame, +theta);
  const AlphaBeta alpha_beta{
    xy_in_ab_frame.x,
    xy_in_ab_frame.y,
  };
  const ThreePhase abc = InverseClarkeTransformation(alpha_beta);
  return abc;
}

}  // namespace signal_processing
}  // namespace electric_vehicle
