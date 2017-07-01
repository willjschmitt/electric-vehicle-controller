#include "signal_processing/angle_math.h"

#include <cmath>

namespace electric_vehicle {
namespace signal_processing {

double Amplitude(const TwoDimensionalVector& in) {
  return sqrt(pow(in.x, 2.0) + pow(in.y, 2.0));
}

TwoDimensionalVector Rotate(const TwoDimensionalVector& in,
                            const double& theta) {
  TwoDimensionalVector out;
  out.x = in.x * cos(-theta) + in.y * sin(-theta);
  out.y = -in.x * sin(-theta) + in.y * cos(-theta);
  return out;
}

}  // namespace signal_processing
}  // namespace electric_vehicle
