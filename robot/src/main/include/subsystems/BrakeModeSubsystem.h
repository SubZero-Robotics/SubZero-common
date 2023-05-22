#pragma once

#include <frc2/command/SubsystemBase.h>

#include <span>

template <typename MOTOR>
class BrakeModeSubsystem : public frc2::SubsystemBase {
public:
  BrakeModeSubsystem(std::span<MOTOR *> motors, bool initialState)
      : _motors(motors) {
    SetBrakeMode(initialState);
  };

  /**
   * @brief Set/unset brake mode
   *
   * @param state true = Brake mode, false = Coast
   */
  void SetBrakeMode(bool state);

  void SetBrakeMode(uint8_t index, bool state);

private:
  std::span<MOTOR> _motors;
};