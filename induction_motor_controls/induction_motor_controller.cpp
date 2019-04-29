#include "induction_motor_controls/induction_motor_controller.h"

#include "control/modulation.h"
#include "signal_processing/angle_math.h"
#include "signal_processing/clarke_transformations.h"

namespace electric_vehicle {
namespace induction_motor_controls {
  
using ::electric_vehicle::control::ModulationCommands;
using ::electric_vehicle::signal_processing::DirectQuadrature;
using ::electric_vehicle::signal_processing::Limit;
using ::electric_vehicle::signal_processing::InverseParkTransformation;
using ::electric_vehicle::signal_processing::ParkTransformation;
using ::electric_vehicle::signal_processing::ThreePhase;
using ::electric_vehicle::signal_processing::TwoDimensionalVector;

ModulationCommands<6> InductionMotorController::CoreControlsTask() {
  const double throttle = throttle_sampler_->Sample();
  
  const double quadrature_current_reference = TorqueController(throttle);
  // TODO(willjschmitt): Set the direct current reference appropriately.
  const double direct_current_reference = 0.0;
  const DirectQuadrature current_reference_dq{
      direct_current_reference, quadrature_current_reference, 0.0};

  const double mechanical_speed = speed_sampler_->Sample();

  const double estimated_stator_angle = stator_angle_estimator_.Estimate(
      mechanical_speed, direct_current_reference,
      quadrature_current_reference);

  const ThreePhase current_feedback = current_sampler_->Sample();
  const DirectQuadrature current_feedback_dq = ParkTransformation(
      current_feedback, estimated_stator_angle);

  const DirectQuadrature voltage_reference = CurrentController(
      current_reference_dq, current_feedback_dq);

  const ThreePhase voltage_reference_abc = InverseParkTransformation(
      voltage_reference, estimated_stator_angle);
  const double dc_voltage = dc_voltage_sampler_->Sample();
  const ModulationCommands<6> modulation_commands = modulator_.Modulate(
      voltage_reference_abc, dc_voltage);
  return modulation_commands;
}

double InductionMotorController::TorqueReference(
    const double& throttle_position) const {
  // For now, the throttle percentage should be equivalent to the torque
  // percentage.
  return throttle_position;
}

double InductionMotorController::TorqueController(
    const double& torque_reference) const {
  // For now, the torque percentage should be equivalent to the amount of
  // torque producing current; i.e: Iq.
  return torque_reference;  
}

DirectQuadrature InductionMotorController::CurrentController(
    const DirectQuadrature& current_reference_dq,
    const DirectQuadrature& current_feedback_dq) {
  const DirectQuadrature voltage_reference_unlimited_dq
      = current_regulator_.Regulate(
            current_reference_dq, current_feedback_dq);
  constexpr double kVoltageLimit = 1.0;
  // TODO(willjschmitt): Replace when conversions are in place for xy vs dq.
  const TwoDimensionalVector voltage_reference_unlimited_xy{
    voltage_reference_unlimited_dq.direct,
    voltage_reference_unlimited_dq.quadrature,
  };
  const TwoDimensionalVector voltage_reference_limited_xy
      = Limit(voltage_reference_unlimited_xy, kVoltageLimit);
  const DirectQuadrature voltage_reference_limited_dq{
    voltage_reference_limited_xy.x,
    voltage_reference_limited_xy.y,
  };
  
  return voltage_reference_limited_dq;
}

}  // namespace induction_motor_controls
}  // namespace electric_vehicle
