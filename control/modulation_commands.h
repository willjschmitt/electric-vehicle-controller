#ifndef CONTROL__MODULATION__COMMANDS__H
#define CONTROL__MODULATION__COMMANDS_H

#include <array>
#include <cmath>

namespace electric_vehicle {
namespace control {

// Phase naming in a three-phase system.
enum Phase {
  A = 0,
  B = 1,
  C = 2,
};

// Switch command type for which switches to enable.
enum SwitchOperation {
  OFF,  // No IGBTs enabled.
  HI,   // Only the Positive IGBT enabled.
  LOW,  // Only the Negative IGBT enabled.
};

// A single command for a switching operation at a given time relative to the
// switching period.
struct ModulationCommand {
  // Action to take at the provided time.
  SwitchOperation operation;

  // Time to execute the action relative to the base of the switching period.
  double time;

  bool operator==(const ModulationCommand& rhs) const {
    return (this->operation == rhs.operation) && (this->time == rhs.time);
  }
};

// Converts a SwitchOperation into's it's generated voltage, using the middle
// point of the capacitor as a reference ground.
double SwitchOperationToVoltage(const SwitchOperation& switch_operation,
  const double& dc_voltage);

// A circular buffer for modulation commands. Used since the commands generated
// in a control loop will usually include the commands for the next cycle while
// controls are computed. Since dynamically allocating the buffer is expensive,
// this allows for a fixed size buffer with a queue-like API for a clear,
// readable modulation command creation and consumption. Typically for a
// 2-level modulator a size 6 buffer is sufficient to support turn on/off in
// one cycle and then 3 cycles (finishing cycle, current cycle, next cycle).
// Example: ModulationCommandBuffer<6> single_phase_buffer;
template <size_t BUFFER_SIZE> class ModulationCommandBuffer {
 public:
  ModulationCommandBuffer() : buffer_(), current_index_(0), buffer_fill_(0) {}

  // Retrieves the next command in the buffer or std::nullopt if no command is
  // available.
  const ModulationCommand* Peek() const {
    if (buffer_fill_ == 0) {
      return nullptr;
    }
    return &buffer_.at(current_index_);
  }

  // Returns the next modulation command if it exists.
  // Throws an invalid_argument exception if there is no command available. Use
  // Peek to check if one exists.
  // Returns a reference, which can be overwritten by new commands. Make a copy
  // if stability cannot be guaranteed.
  const ModulationCommand& Pop() {
    const ModulationCommand* command = Peek();
    if (command == nullptr) {
      throw std::invalid_argument("No command available in buffer.");
    }
    current_index_ = (current_index_ + 1) % BUFFER_SIZE;
    buffer_fill_--;
    return *command;
  }

  // Adds a new command to the buffer. Throws an invalid_argument exception if
  // the buffer is already full.
  void PushBack(const ModulationCommand& command) {
    if (buffer_fill_ == BUFFER_SIZE) {
      throw std::invalid_argument("No command available in buffer.");
    }
    buffer_.at((current_index_ + buffer_fill_) % BUFFER_SIZE) = command;
    buffer_fill_++;
  }

  // Convenience function to calculate the average voltage created for the
  // current state of the buffer.
  double ModulationCommandsToVoltage(
    const double& dc_voltage,
    const double& switching_period) const {
    const double calculation_period = ceil(buffer_[AbsoluteIndex(buffer_fill_ - 1)].time / switching_period);

    double voltage = 0.0;
    for (unsigned int i = 0; i < buffer_fill_; i++) {
      const double delta_time = (i == buffer_fill_ - 1)
        ? calculation_period - buffer_[AbsoluteIndex(buffer_fill_ - 1)].time
        : buffer_[AbsoluteIndex(i + 1)].time - buffer_[AbsoluteIndex(i)].time;
      const double switching_voltage = SwitchOperationToVoltage(
        buffer_[AbsoluteIndex(i)].operation, dc_voltage);
      voltage += switching_voltage * (delta_time / calculation_period);
    }

    return voltage;
  }

 private:
   // Computes the index in buffer_ for the relative_index relative to
   // current_index, looping around if necessary. Does not sanitize if the
   // relative_index is >= buffer_fill_.
  size_t AbsoluteIndex(const int relative_index) const {
    return (current_index_ + relative_index) % BUFFER_SIZE;
  }

  // Fixed size circular buffer, whose head and size will be tracked by
  // current_index_ and buffer_fill_.
  std::array<ModulationCommand, BUFFER_SIZE> buffer_;

  // Current index at the front of the queue.
  size_t current_index_;

  // Number of indices, which are currently used. If ==0, the buffer is empty.
  // If ==BUFFER_SIZE, the buffer is full and can take no more elements.
  size_t buffer_fill_;
};

// A complete 3-phase command buffer comprising of ModulationCommandBuffer's.
// Each buffer is equally sized to BUFFER_SIZE. See ModulationCommandBuffer
// for sizing recommendations.
// Example: ModulationCommands<6> modulation_commands;
template <size_t BUFFER_SIZE> class ModulationCommands {
 public:
  ModulationCommands() {}

  // Retrieves the command buffer for the requested phase.
  ModulationCommandBuffer<BUFFER_SIZE>& ForPhase(const Phase phase) {
    switch (phase) {
      case Phase::A:
        return buffers_by_phase_.at(0);
      case Phase::B:
        return buffers_by_phase_.at(1);
      case Phase::C:
        return buffers_by_phase_.at(2);
    }
  }

 private:
  // Buffers for each phase, each containing the commands for the phase.
  std::array<ModulationCommandBuffer<BUFFER_SIZE>, 3> buffers_by_phase_;
};

}  // namespace control
}  // namespace electric_vehicle

#endif  // CONTROL__MODULATION__H
