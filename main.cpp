#include "config.h"

#include <xc.h>
#include <sys/attribs.h>

#include "control/current_regulator.h"
#include "control/timer.h"
#include "induction_motor_controls/induction_motor_controller.h"
#include "machines/induction_machine.h"
#include "measurement/throttle.h"

using ::electric_vehicle::control::CurrentRegulator;
using ::electric_vehicle::control::SamplingTimer;
using ::electric_vehicle::induction_motor_controls::InductionMotorController;
using ::electric_vehicle::machines::InductionMachine;
using ::electric_vehicle::measurement::PIC32DcVoltageSampler;
using ::electric_vehicle::measurement::PIC32MechanicalSpeedSampler;
using ::electric_vehicle::measurement::PIC32ThreePhaseCurrentSampler;
using ::electric_vehicle::measurement::PIC32ThrottleSampler;

namespace {

// System is configured to 100MHz (10nS ticks).
constexpr double kSystemFrequencyHertz = 80e6;
constexpr double kSystemTickSeconds = 1.0 / kSystemFrequencyHertz;

constexpr double kTimer2Seconds = 400e-6;

// Sets up PORTC.6 and PORTC.7 (RPC6 and RPC7) as output for debugging. These
// pins is exposed on the development board.
void ConfigureIO() {
  TRISCbits.TRISC6 = 0;
  TRISCbits.TRISC7 = 0;
}

// Sets up Timer2 as a 16bit timer with interrupts at a frequency of
// kTimer2Seconds. Sets up Timer2 interrupt with priority 2, subpriority 0.
void ConfigureTimer2() {
  // Stop the timer, clear the control register, and reset the timer.
  T2CON = 0x0;
  TMR2 = 0x0;
  
  // Set the timer to pre-scale by a factor of 1, and resetting at count to get
  // kTimer2Seconds.
  T2CONbits.TCKPS = 0x0;
  T2CONbits.T32 = 0;
  PR2 = (unsigned int) (kTimer2Seconds * kSystemFrequencyHertz);

  // Set Timer 2 Priorities to 2, 0.
  IPC2bits.T2IP = 2;
  IPC2bits.T2IS = 0;

  // Clear the timer interrupt status flag and enable timer interrupts.
  IFS0bits.T2IF = 0;
  IEC0bits.T2IE = 1;
  INTCONbits.MVEC = 1;
  __asm__ volatile ("ei");
  
  // Start the timer.
  T2CONbits.ON = 1;
}

// Controller variables. All are pointers but many would otherwise be declared
// as const. This will allow global access with initialization in the main
// function.
volatile SamplingTimer* timer;
volatile PIC32ThrottleSampler* throttle_sampler;
volatile PIC32MechanicalSpeedSampler* speed_sampler;
volatile PIC32ThreePhaseCurrentSampler* current_sampler;
volatile PIC32DcVoltageSampler* dc_voltage_sampler;
volatile InductionMachine* induction_machine;
volatile CurrentRegulator* current_regulator;
volatile InductionMotorController* controller;

// Executes the core control loops. Should be executed at the Timer2 frequency.
void RunControls() {
  ((InductionMotorController*)controller)->CoreControlsTask();
}

}  // namespace

// Interrupt handlers are outside of namespaces, since they need to be in the
// "C" namespace.

// Interrupt service routine for Timer2 overflows as configured in
// ConfigureTimer2.
extern "C" void __ISR(_TIMER_2_VECTOR, IPL2SOFT) __attribute__((nomips16))
_T2Interrupt(void)
{
  IFS0bits.T2IF = 0;
  PORTCINV = 0x1 << 6;
  RunControls();
  PORTCINV = 0x1 << 7;
}

int main(int argc, char** argv) {
  ConfigureIO();
  ConfigureTimer2();

  timer = new SamplingTimer();
  throttle_sampler = new PIC32ThrottleSampler();
  speed_sampler = new PIC32MechanicalSpeedSampler();
  current_sampler = new PIC32ThreePhaseCurrentSampler();
  dc_voltage_sampler = new PIC32DcVoltageSampler();
  
  constexpr double kMagnetizingInductance = 0.0;
  constexpr double kStatorInductance = 0.0;
  constexpr double kStatorResistance = 0.0;
  constexpr double kRotorInductance = 0.0;
  constexpr double kRotorResistance = 0.0;
  constexpr unsigned int kNumberOfPoles = 2;
  induction_machine = new InductionMachine(
      kMagnetizingInductance, kStatorInductance, kStatorResistance,
      kRotorInductance, kRotorResistance, kNumberOfPoles);
  
  constexpr double kCurrentRegulatorProportionalGain = 0.0;
  constexpr double kCurrentRegulatorIntegralGain = 0.0;
  constexpr double kCurrentRegulatorDirectMinimum = -1.0;
  constexpr double kCurrentRegulatorDirectMaximum = +1.0;
  constexpr double kCurrentRegulatorQuadratureMinimum = -1.0;
  constexpr double kCurrentRegulatorQuadratureMaximum = +1.0;
  current_regulator = new CurrentRegulator(
      (SamplingTimer*)timer, kCurrentRegulatorProportionalGain,
      kCurrentRegulatorIntegralGain, kCurrentRegulatorDirectMinimum,
      kCurrentRegulatorDirectMaximum, kCurrentRegulatorQuadratureMinimum,
      kCurrentRegulatorQuadratureMaximum);
  
  constexpr double kCoreControlsTaskRate = kTimer2Seconds;
  
  controller = new InductionMotorController(
      (SamplingTimer*)timer, (PIC32ThrottleSampler*)throttle_sampler,
      (PIC32MechanicalSpeedSampler*)speed_sampler,
      (PIC32ThreePhaseCurrentSampler*)current_sampler,
      (PIC32DcVoltageSampler*)dc_voltage_sampler,
      *(InductionMachine*)induction_machine,
      *(CurrentRegulator*)current_regulator,
      kCoreControlsTaskRate);
    
  while(1) {
    __asm__ volatile ("nop");
  };
  return 0;
}
