#ifndef MEASUREMENT__DC_VOLTAGE__H
#define MEASUREMENT__DC_VOLTAGE__H

namespace electric_vehicle {
namespace measurement {

// Measures a DC Voltage for a bridge or battery pack.
class DcVoltageMeasurementInterface {
 public:
  virtual double Sample() const = 0;
};


class PIC32DcVoltageSampler : public DcVoltageMeasurementInterface {
 public:
  double Sample() const override;
};

}
}

#endif  // MEASUREMENT__DC_VOLTAGE__H