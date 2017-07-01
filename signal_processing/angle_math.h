#ifndef SIGNAL_PROCESSING__ANGLE_MATH__H
#define SIGNAL_PROCESSING__ANGLE_MATH__H

namespace electric_vehicle {
namespace signal_processing {

// A 2D Vector, often a real, imaginary axis pair.
struct TwoDimensionalVector {
  double x;
  double y;
};

// Calculates the amplitude of two orthogonal values as sqrt(x^2 + y^2).
double Amplitude(const TwoDimensionalVector& in);

// Rotates an x,y vector pair by a provided angle.
TwoDimensionalVector Rotate(const TwoDimensionalVector& in,
                            const double& theta);

}  // namespace signal_processing
}  // namespace electric_vehicle

#endif  // SIGNAL_PROCESSING__ANGLE_MATH__H