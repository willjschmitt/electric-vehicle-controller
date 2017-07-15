#ifndef MACHINES__INDUCTION_MACHINE__H
#define MACHINES__INDUCTION_MACHINE__H

namespace electric_vehicle {
namespace machines {

// An induction machine, which provides it's electrical parameters in various
// forms. Comments referring to short form circuit parameters should use this
// diagram as a key:
//     R1    L1       L2
// o--wwww--@@@@--o--@@@@---
//                |        |
//                @        z
//                @ Lm     z R2/s
//                @        z
//                |        |
// o------------------------
class InductionMachine {
 public:
  // Inductance of the magnetizing branch of the motor Lm.
  double MagnetizingInductance() const { return magnetizing_inductance_; }

  // Inductance of the stator winding referred to the stator side.
  double StatorInductancePrimary() const { return stator_inductance_; }

  // Resistance of the stator winding referred to the stator side.
  double StatorResistancePrimary() const { return stator_resistance_; }

  // Inductance of the rotor winding referred to the stator side.
  double RotorInductancePrimary() const { return rotor_inductance_; }

  // Resistance of the rotor winding referred to the stator side.
  double RotorResistancePrimary() const { return rotor_resistance_; }

 private:
  InductionMachine(
      const double& magnetizing_inductance, const double& stator_inductance,
      const double& stator_resistance, const double& rotor_inductance,
      const double& rotor_resistance)
      : magnetizing_inductance_(magnetizing_inductance),
        stator_inductance_(stator_inductance),
        stator_resistance_(stator_resistance),
        rotor_inductance_(rotor_inductance),
        rotor_resistance_(rotor_resistance) {}

  // Inductance of the magnetizing branch of the motor Lm referred to the
  // stator side. Units: Henries.
  const double magnetizing_inductance_;

  // Inductance of the stator winding referred to the stator side. Units:
  // Henries.
  const double stator_inductance_;
  
  // Resistance of the stator winding referred to the stator side. Units:
  // Ohms.
  const double stator_resistance_;
  
  // Inductance of the rotor winding referred to the stator side. Units:
  // Henries.
  const double rotor_inductance_;

  // Resistance of the rotor winding referred to the stator side. Units:
  // Ohms.
  const double rotor_resistance_;
};

}  // namespace machines
}  // namespace electric_vehicle

#endif  // MACHINES__INDUCTION_MACHINE__H
