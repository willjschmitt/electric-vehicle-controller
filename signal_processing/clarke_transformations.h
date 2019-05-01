#ifndef SIGNAL_PROCESSING__CLARKE_TRANSFORMATIONS__H_
#define SIGNAL_PROCESSING__CLARKE_TRANSFORMATIONS__H_

#include <array>
#include <vector>
#include "angle_math.h"

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

  DirectQuadrature()
      : direct(0.0), quadrature(0.0), zero(0.0) {}

  DirectQuadrature(const double d, const double q)
      : direct(d), quadrature(q), zero(0.0) {}

  DirectQuadrature(const double d, const double q, const double z)
      : direct(d), quadrature(q), zero(z) {}

  DirectQuadrature(const TwoDimensionalVector& xy)
      : direct(xy.x), quadrature(xy.y) {}

  inline DirectQuadrature operator-(const DirectQuadrature& rhs) const {
    DirectQuadrature result;
    result.direct = this->direct - rhs.direct;
    result.quadrature = this->quadrature - rhs.quadrature;
    result.zero = this->zero - rhs.zero;
    return result;
  }

  inline DirectQuadrature operator+(const DirectQuadrature& rhs) const {
    DirectQuadrature result;
    result.direct = this->direct + rhs.direct;
    result.quadrature = this->quadrature + rhs.quadrature;
    result.zero = this->zero + rhs.zero;
    return result;
  }

  // Convert DQ values into generic 2d vector.
  inline TwoDimensionalVector XY() const {
    return { direct, quadrature };
  }
};

// Three Phase instantaneous values.
class ThreePhase {
 public:
  typedef std::array<double, 3>::iterator iterator;
  typedef std::array<double, 3>::const_iterator const_iterator;
  typedef std::array<double, 3>::size_type size_type;
  typedef double value_type;

  ThreePhase() {}
  ThreePhase(const std::array<double, 3> values)
    : values_(values) {}
  ThreePhase(const double& a, const double& b, const double& c)
    : values_{a, b, c} {}

  double& A(){ return values_[0]; };
  double& B(){ return values_[1]; };
  double& C(){ return values_[2]; };

  double A() const { return values_[0]; };
  double B() const { return values_[1]; };
  double C() const { return values_[2]; };

  double& operator[](const std::size_t index) { return values_[index]; };
  const double& operator[](const std::size_t index) const { return values_[index]; };
 
  iterator begin() { return values_.begin(); }
  iterator end() { return values_.end(); }

  const_iterator begin() const { return values_.begin(); }
  const_iterator end() const { return values_.end(); }

  size_type size() { return values_.size(); }
 
 private:
  std::array<double, 3> values_;
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

// Converts a alpha-beta reference frame into three-phase instantaneous values.
// See: https://en.wikipedia.org/wiki/Alpha%E2%80%93beta_transformation.
ThreePhase InverseClarkeTransformation(const AlphaBeta& alpha_beta);

// Converts a dq0 reference frame into three-phase instantaneous values.
// See: https://en.wikipedia.org/wiki/Direct-quadrature-zero_transformation.
ThreePhase InverseParkTransformation(const DirectQuadrature& direct_quadrature,
                                     const double& theta);

}  // namespace signal_processing
}  // namespace electric_vehicle

#endif  // SIGNAL_PROCESSING__CLARKE_TRANSFORMATIONS__H_
