#ifndef MEASUREMENT__MECHANICAL_SPEED__H
#define MEASUREMENT__MECHANICAL_SPEED__H

namespace electric_vehicle {
namespace measurement {

class MechanicalSpeedInterface {
 public:
  virtual double Sample() const = 0;
};

class PIC32MechanicalSpeedSampler : public MechanicalSpeedInterface{
 public:
  double Sample() const override;
};

}
}

#endif  // MEASUREMENT__MECHANICAL_SPEED__H