#pragma once

#include <string>

#include "subzero/singleaxis/BaseSingleAxisSubsystem.h"

namespace subzero
{

  /**
   * @brief A single axis representing a circular path of motion in degrees
   *
   * @tparam TController PidMotorController
   */
  template <typename TController>
  class RotationalSingleAxisSubsystem
      : public BaseSingleAxisSubsystem<TController, units::degree>
  {
  public:
    RotationalSingleAxisSubsystem(
        std::string name,
        TController &controller,
        ISingleAxisSubsystem<units::degree>::SingleAxisConfig config,
        units::meter_t armatureLength, frc::MechanismObject2d *node = nullptr)
        : BaseSingleAxisSubsystem<TController, units::degree>{name,
                                                              controller,
                                                              config, node},
          m_armatureLength{armatureLength} {}

    void Periodic() override;

    void RunMotorVelocity(units::degrees_per_second_t speed,
                          bool ignoreEncoder = false) override;

  protected:
    units::meter_t m_armatureLength;
  };
} // namespace subzero