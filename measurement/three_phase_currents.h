#ifndef MEASUREMENT__THREE_PHASE_CURRENTS__H
#define MEASUREMENT__THREE_PHASE_CURRENTS__H

#include "signal_processing/clarke_transformations.h"

namespace electric_vehicle {
namespace measurement {

class ThreePhaseMeasurementInterface {
 public:
   virtual ::electric_vehicle::signal_processing::ThreePhase Sample()
       const = 0;
};


class PIC32ThreePhaseCurrentSampler : public ThreePhaseMeasurementInterface {
 public:
   ::electric_vehicle::signal_processing::ThreePhase Sample() const override;
};
    
}  // namespace measurement
}  // namespace electric_vehicle

#endif  // MEASUREMENT__THREE_PHASE_CURRENTS__H