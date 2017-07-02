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

TwoDimensionalVector Scale(
    const TwoDimensionalVector& in, const double& gain) {
  TwoDimensionalVector scaled;
  scaled.x = in.x * gain;
  scaled.y = in.y * gain;
  return scaled;
}

TwoDimensionalVector Limit(
    const TwoDimensionalVector& in, const double& limit) {
  const double amplitude = Amplitude(in);
  if (amplitude > abs(limit)) {
    const double reduction_ratio = abs(limit / amplitude);
    return Scale(in, reduction_ratio);
  }
  return in;
}

}  // namespace signal_processing
}  // namespace electric_vehicle
