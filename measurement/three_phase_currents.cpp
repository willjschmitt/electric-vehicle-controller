#include "measurement/three_phase_currents.h"

namespace electric_vehicle {
namespace measurement {

using ::electric_vehicle::signal_processing::ThreePhase;
    
ThreePhase PIC32ThreePhaseCurrentSampler::Sample() const {
  return ThreePhase{0.0, 0.0, 0.0};
};
    
}  // namespace measurement
}  // namespace electric_vehicle
