#ifndef INDUCTION_MOTOR_CONTROLS__INDUCTION_MOTOR_CONTROLLER__H
#define INDUCTION_MOTOR_CONTROLS__INDUCTION_MOTOR_CONTROLLER__H

#include <chrono>

#include "control/current_regulator.h"
#include "control/modulation.h"
#include "control/timer.h"
#include "control/stator_angle_estimator.h"
#include "machines/induction_machine.h"
#include "measurement/dc_voltage.h"
#include "measurement/mechanical_speed.h"
#include "measurement/three_phase_currents.h"
#include "measurement/throttle.h"

namespace electric_vehicle {
namespace induction_motor_controls {

class InductionMotorController {
 public:
  InductionMotorController(
      ::electric_vehicle::control::TimerInterface* timer,
      const ::electric_vehicle::measurement::ThrottleInterface*
          throttle_sampler,
      const ::electric_vehicle::measurement::MechanicalSpeedInterface*
          speed_sampler,
      const ::electric_vehicle::measurement::ThreePhaseMeasurementInterface*
          current_sampler,
      const ::electric_vehicle::measurement::DcVoltageMeasurementInterface*
          dc_voltage_sampler,
      const ::electric_vehicle::machines::InductionMachine& induction_machine,
      const ::electric_vehicle::control::CurrentRegulator& current_regulator,
      const std::chrono::system_clock::duration core_controls_task_rate)
      : throttle_sampler_(throttle_sampler),
        speed_sampler_(speed_sampler),
        current_sampler_(current_sampler),
        dc_voltage_sampler_(dc_voltage_sampler),
        stator_angle_estimator_(
            timer, induction_machine.NumberOfPoles(),
            induction_machine.RotorInductancePrimary(),
            induction_machine.RotorResistancePrimary()),
        current_regulator_(current_regulator),
        modulator_(core_controls_task_rate) {}

  // Core controls task loop. To be executed at PWM frequency ~= 10kHz.
  // Generates PWM commands.
  ::electric_vehicle::control::ModulationCommands CoreControlsTask();

 private:
  // Samples the throttle position.
  const ::electric_vehicle::measurement::ThrottleInterface* throttle_sampler_;

  // Samples the current mechanical speed.
  const ::electric_vehicle::measurement::MechanicalSpeedInterface*
      speed_sampler_;

  // Samples the three phase stator current feedback.
  const ::electric_vehicle::measurement::ThreePhaseMeasurementInterface*
      current_sampler_;

  // Samples the DC voltage from the battery/inverter input.
  const ::electric_vehicle::measurement::DcVoltageMeasurementInterface*
      dc_voltage_sampler_;

  // Estimates the rotor flux angle for voltage generation and current
  // regulation.
  ::electric_vehicle::control::StatorAngleEstimator stator_angle_estimator_;
   
  // Regulates the current for the inverter by generating voltage references
  // to pulse width modulate.
  ::electric_vehicle::control::CurrentRegulator current_regulator_;

  // Modulator, which converts the voltage references into switching commands.
  ::electric_vehicle::control::TwoLevelSineModulator modulator_;

  // Gets the torque reference based on the throttle position. Units: per-unit
  // torque.
  double TorqueReference(const double& throttle_position) const;

  // Gets the quadrature current reference based on a torque reference. Units:
  // per-unit current Iq.
  double TorqueController(const double& torque_reference) const;

  // Conditions and calls the current regulator to return a limited voltage
  // reference in the dq0 frame.
  ::electric_vehicle::signal_processing::DirectQuadrature CurrentController(
      const ::electric_vehicle::signal_processing::DirectQuadrature&
          current_reference_dq,
      const ::electric_vehicle::signal_processing::DirectQuadrature& 
          current_feedback_dq);
};

}
}

#endif  // INDUCTION_MOTOR_CONTROLS__INDUCTION_MOTOR_CONTROLLER__H