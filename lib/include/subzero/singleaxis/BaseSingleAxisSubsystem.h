#pragma once

#include <frc/DigitalInput.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/controller/PIDController.h>
#include <frc/smartdashboard/MechanismLigament2d.h>
#include <frc/smartdashboard/MechanismRoot2d.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/TrapezoidProfileSubsystem.h>
#include <rev/SparkClosedLoopController.h>
#include <rev/SparkFlex.h>
#include <rev/SparkMax.h>
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>

#include <memory>
#include <string>

#include "subzero/frc2/command/EmptyCommand.h"
#include "subzero/logging/ConsoleLogger.h"
#include "subzero/logging/ShuffleboardLogger.h"
#include "subzero/motor/PidMotorController.h"
#include "subzero/singleaxis/ISingleAxisSubsystem.h"

namespace subzero {

/**
 * @brief The ultimate solution for turrets, arms, and much more. This class
 * allows for absolute, relative, and joystick control across both linear and
 * rotational axes. Even better, it checks motion limits, outputs real-time
 * stats via SmartDashboard, and allows for out-of-the-box simulations via
 * Mechanism2d
 *
 * @tparam TController PidMotorController
 * @tparam TDistance Distance unit
 */
template <typename TController, typename TDistance>
class BaseSingleAxisSubsystem
    : public ISingleAxisSubsystem<TDistance>,
      public frc2::TrapezoidProfileSubsystem<TDistance> {
public:
  using PidState = typename frc::TrapezoidProfile<TDistance>::State;
  using Distance_t = units::unit_t<TDistance>;
  using Velocity =
      units::compound_unit<TDistance, units::inverse<units::seconds>>;
  using Velocity_t = units::unit_t<Velocity>;
  using Acceleration =
      units::compound_unit<Velocity, units::inverse<units::seconds>>;
  using Acceleration_t = units::unit_t<Acceleration>;

protected:
  bool IsMovementAllowed(double speed, bool ignoreEncoder = false);

  bool IsMovementAllowed(bool ignoreEncoder = false);

public:
  BaseSingleAxisSubsystem(
      std::string name, TController &controller,
      ISingleAxisSubsystem<TDistance>::SingleAxisConfig config,
      frc::MechanismObject2d *mechanismNode = nullptr);

  /**
   * @brief Runs the absolute positioning task and updates relevant info on
   * SmartDashboard
   *
   */
  void Periodic() override;

  virtual void RunMotorVelocity(Velocity_t speed,
                                bool ignoreEncoder = false) = 0;

  inline void UseState(PidState setpoint) override {
    m_controller.RunToPosition(setpoint.position.value());
  }

  inline void RunMotorSpeedDefault(bool ignoreEncoder = false) override {
    RunMotorVelocity(m_config.defaultSpeed, ignoreEncoder);
  }

  void RunMotorPercentage(double percentSpeed,
                          bool ignoreEncoder = false) override;

  inline Distance_t GetCurrentPosition() override {
    return Distance_t(m_controller.GetEncoderPosition());
  }

  void Stop() override;

  void ResetEncoder() override;

  inline bool AtHome() override {
    return AtLimitSwitchMin() || GetCurrentPosition() <= m_config.minDistance;
  }

  inline bool AtMax() override {
    return AtLimitSwitchMax() || GetCurrentPosition() >= m_config.maxDistance;
  }

  bool AtLimitSwitchMin() override;

  bool AtLimitSwitchMax() override;

  frc2::CommandPtr MoveToPositionAbsolute(Distance_t position) override;

  inline frc2::CommandPtr MoveToPositionRelative(Distance_t position) override {
    return MoveToPositionAbsolute(m_goalPosition + position);
  }

  frc2::CommandPtr Home() override;

  inline frc2::CommandPtr ResetRelativeEncoder() {
    return frc2::InstantCommand([this] { ResetEncoder(); }).ToPtr();
  }

  inline frc::MechanismLigament2d *GetLigament() { return m_ligament2d; }

  inline bool IsEnabled() override { return m_pidEnabled; }

  void DisablePid() override;

  void EnablePid() override;

  /**
   * @brief This must be called once upon robot startup
   *
   */
  void OnInit();

protected:
  std::optional<frc::DigitalInput *> m_minLimitSwitch;
  std::optional<frc::DigitalInput *> m_maxLimitSwitch;
  TController &m_controller;
  ISingleAxisSubsystem<TDistance>::SingleAxisConfig m_config;
  std::string m_name;
  Distance_t m_goalPosition;
  bool m_pidEnabled;
  bool m_home;
  bool resetOccurred = false;
  double m_latestSpeed;
  frc2::CommandPtr m_resetEncCmd = EmptyCommand().ToPtr();
  frc::MechanismLigament2d *m_ligament2d;
};
} // namespace subzero