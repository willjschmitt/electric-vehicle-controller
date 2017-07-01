#ifndef SIGNAL_PROCESSING__CLARKE_TRANSFORMATIONS__H_
#define SIGNAL_PROCESSING__CLARKE_TRANSFORMATIONS__H_

namespace electric_vehicle {
namespace signal_processing {

// Alpha, Beta (Clarke) transform results in the alpha, beta reference frame.
struct AlphaBeta {
  double alpha;
  double beta;
  double gamma;
};

// Direct Quadrature (Park) transform results in the DQ0 reference frame.
struct DirectQuadrature {
  double direct;
  double quadrature;
  double zero;
};

// Three Phase instantaneous values.
struct ThreePhase {
  double a;
  double b;
  double c;
};

// Converts three-phase instantaneous values into the alpha beta reference
// frame. Values should peak at sqrt(2) * RMS value.
// See: https://en.wikipedia.org/wiki/Alpha%E2%80%93beta_transformation.
AlphaBeta ClarkeTransformation(ThreePhase three_phase);

// Converts three-phase instantaneous values into the dq0 reference frame.
// Values should peak at sqrt(2) * RMS value.
// See: https://en.wikipedia.org/wiki/Direct-quadrature-zero_transformation.
DirectQuadrature ParkTransformation(const ThreePhase& three_phase,
                                    const double& theta);

}  // namespace signal_processing
}  // namespace electric_vehicle

#endif  // SIGNAL_PROCESSING__CLARKE_TRANSFORMATIONS__H_
