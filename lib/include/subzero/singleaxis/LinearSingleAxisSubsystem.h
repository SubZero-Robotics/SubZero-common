#pragma once

#include <string>

#include "subzero/singleaxis/BaseSingleAxisSubsystem.h"

namespace subzero {

/**
 * @brief A single axis representing a linear path of motion in meters
 *
 * @tparam TMotor
 * @tparam TController
 * @tparam TRelativeEncoder
 * @tparam TAbsoluteEncoder
 */
template <typename TMotor, typename TController, typename TRelativeEncoder,
          typename TAbsoluteEncoder>
class LinearSingleAxisSubsystem
    : public BaseSingleAxisSubsystem<TMotor, TController, TRelativeEncoder,
                                     TAbsoluteEncoder, units::meter> {
public:
  LinearSingleAxisSubsystem(
      std::string name,
      PidMotorController<TMotor, TController, TRelativeEncoder,
                         TAbsoluteEncoder> &controller,
      ISingleAxisSubsystem<units::meter>::SingleAxisConfig config,
      frc::MechanismObject2d *node = nullptr)
      : BaseSingleAxisSubsystem<TMotor, TController, TRelativeEncoder,
                                TAbsoluteEncoder, units::meter>{
            name, controller, config, node} {}

  void Periodic() override {
    BaseSingleAxisSubsystem<TMotor, TController, TRelativeEncoder,
                            TAbsoluteEncoder, units::meter>::Periodic();

    if (BaseSingleAxisSubsystem<TMotor, TController, TRelativeEncoder,
                                TAbsoluteEncoder, units::meter>::m_ligament2d) {
      BaseSingleAxisSubsystem<TMotor, TController, TRelativeEncoder,
                              TAbsoluteEncoder, units::meter>::m_ligament2d
          ->SetLength(
              ((BaseSingleAxisSubsystem<TMotor, TController, TRelativeEncoder,
                                        TAbsoluteEncoder,
                                        units::meter>::m_config.reversed
                    ? -(BaseSingleAxisSubsystem<
                          TMotor, TController, TRelativeEncoder,
                          TAbsoluteEncoder, units::meter>::GetCurrentPosition())
                    : BaseSingleAxisSubsystem<
                          TMotor, TController, TRelativeEncoder,
                          TAbsoluteEncoder,
                          units::meter>::GetCurrentPosition()) +
               BaseSingleAxisSubsystem<TMotor, TController, TRelativeEncoder,
                                       TAbsoluteEncoder, units::meter>::m_config
                   .mechanismConfig.minimumLength)
                  .value());
    }
  }

  /**
   * @brief Not allowed
   *
   * @param speed
   * @param ignoreEncoder
   */
  void RunMotorVelocity(units::meters_per_second_t speed,
                        bool ignoreEncoder = false) override {
    BaseSingleAxisSubsystem<TMotor, TController, TRelativeEncoder,
                            TAbsoluteEncoder, units::meter>::DisablePid();
    ConsoleWriter.logWarning(
        BaseSingleAxisSubsystem<TMotor, TController, TRelativeEncoder,
                                TAbsoluteEncoder, units::meter>::m_name,
        "Running with a velocity is not supported for linear subsystems!%s",
        "");
  }
};
} // namespace subzero