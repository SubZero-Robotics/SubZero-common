#pragma once

#include <string>

#include "subzero/singleaxis/BaseSingleAxisSubsystem.h"

namespace subzero {

/**
 * @brief A single axis representing a linear path of motion in meters
 *
 * @tparam TController PidMotorController
 */
template <typename TController>
class LinearSingleAxisSubsystem
    : public BaseSingleAxisSubsystem<TController, units::meter> {
public:
  LinearSingleAxisSubsystem(
      std::string name, TController &controller,
      ISingleAxisSubsystem<units::meter>::SingleAxisConfig config,
      frc::MechanismObject2d *node = nullptr)
      : BaseSingleAxisSubsystem<TController, units::meter>{name, controller,
                                                           config, node} {}

  void Periodic() override;

  /**
   * @brief Not allowed
   *
   * @param speed
   * @param ignoreEncoder
   */
  void RunMotorVelocity(units::meters_per_second_t speed,
                        bool ignoreEncoder = false) override;
};
} // namespace subzero