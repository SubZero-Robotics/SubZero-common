#pragma once

#include "subzero/singleaxis/BaseSingleAxisSubsystem.h"
#include "subzero/motor/PidMotorController.cpp"

using namespace subzero;

template <typename TController, typename TDistance>
BaseSingleAxisSubsystem<TController, TDistance>::BaseSingleAxisSubsystem(
    std::string name, TController &controller,
    ISingleAxisSubsystem<TDistance>::SingleAxisConfig config,
    frc::MechanismObject2d *mechanismNode)
    : frc2::TrapezoidProfileSubsystem<TDistance>{config.profileConstraints},
      m_minLimitSwitch{config.minLimitSwitch},
      m_maxLimitSwitch{config.maxLimitSwitch}, m_controller{controller},
      m_config{config}, m_name{name}, m_pidEnabled{false} {
  m_pidEnabled = false;

  if (mechanismNode) {
    m_ligament2d = mechanismNode->Append<frc::MechanismLigament2d>(
        name, config.mechanismConfig.minimumLength.value(),
        config.mechanismConfig.minimumAngle, config.mechanismConfig.lineWidth,
        config.mechanismConfig.color);
  }

  m_resetEncCmd = ResetRelativeEncoder();
  frc::SmartDashboard::PutData(m_name + " Reset Encoder", m_resetEncCmd.get());
  frc2::TrapezoidProfileSubsystem<TDistance>::Disable();
}

template <typename TController, typename TDistance>
void BaseSingleAxisSubsystem<TController, TDistance>::Periodic() {
  frc::SmartDashboard::PutBoolean(m_name + " Pid Enabled", m_pidEnabled);
  frc::SmartDashboard::PutNumber(m_name + " Position",
                                 GetCurrentPosition().value());
  if (m_config.conversionFunction) {
    std::string curPos =
        m_config.conversionFunction.value()(GetCurrentPosition());
    frc::SmartDashboard::PutString(m_name + " Converted Position", curPos);
  }

  if (m_controller.GetAbsoluteEncoderPosition().has_value())
    frc::SmartDashboard::PutNumber(
        m_name + " Absolute Position",
        m_controller.GetAbsoluteEncoderPosition().value());

  if (m_controller.GetAbsoluteEncoderPosition().has_value()) {
    Distance_t absEncValue =
        Distance_t(std::abs(m_controller.GetAbsoluteEncoderPosition().value()));

    if (!resetOccurred && absEncValue <= m_config.tolerance) {
      m_controller.ResetEncoder();
      resetOccurred = true;
    } else if (resetOccurred && absEncValue > m_config.tolerance) {
      resetOccurred = false;
    }
  }

  if (m_pidEnabled) {
    frc2::TrapezoidProfileSubsystem<TDistance>::Periodic();
    m_controller.Update();
  }

  if (!m_pidEnabled && !IsMovementAllowed(m_latestSpeed)) {
    ConsoleWriter.logInfo(m_name,
                          "Periodic: Movement with speed %f is not allowed",
                          m_latestSpeed);

    Stop();
  }
}

template <typename TController, typename TDistance>
void BaseSingleAxisSubsystem<TController, TDistance>::RunMotorPercentage(
    double percentSpeed, bool ignoreEncoder) {
  bool movementAllowed = IsMovementAllowed(percentSpeed, ignoreEncoder);
  frc::SmartDashboard::PutBoolean(m_name + " Movement Allowed",
                                  movementAllowed);

  if (!movementAllowed) {
    Stop();
    return;
  }

  DisablePid();

  frc::SmartDashboard::PutNumber(m_name + " Speed %", percentSpeed);

  m_controller.Set(percentSpeed);
}

template <typename TController, typename TDistance>
void BaseSingleAxisSubsystem<TController, TDistance>::Stop() {
  frc::SmartDashboard::PutNumber(m_name + " Speed %", 0);
  m_controller.Stop();
}

template <typename TController, typename TDistance>
void BaseSingleAxisSubsystem<TController, TDistance>::ResetEncoder() {
  m_controller.ResetEncoder();
}

template <typename TController, typename TDistance>
bool BaseSingleAxisSubsystem<TController, TDistance>::AtLimitSwitchMin() {
  if (m_minLimitSwitch && m_minLimitSwitch.value()) {
    bool value = !m_minLimitSwitch.value()->Get();
    frc::SmartDashboard::PutBoolean(m_name + " Min Limit Switch", value);
    return value;
  }

  return false;
}

template <typename TController, typename TDistance>
bool BaseSingleAxisSubsystem<TController, TDistance>::AtLimitSwitchMax() {
  if (m_maxLimitSwitch && m_maxLimitSwitch.value()) {
    bool value = !m_maxLimitSwitch.value()->Get();
    frc::SmartDashboard::PutBoolean(m_name + " Max Limit Switch", value);
    return value;
  }

  return false;
}

template <typename TController, typename TDistance>
frc2::CommandPtr
BaseSingleAxisSubsystem<TController, TDistance>::MoveToPositionAbsolute(
    Distance_t position) {
  return frc2::InstantCommand(
             [this, position] {
               if (position < m_config.minDistance ||
                   position > m_config.maxDistance) {
                 ConsoleWriter.logWarning(
                     m_name,
                     "Attempting to move to position %f outside of boundary",
                     position.value());
                 return;
               }

               ConsoleWriter.logVerbose(
                   m_name, "Moving to absolute position %f", position.value());

               m_goalPosition = position;
               EnablePid();
               frc2::TrapezoidProfileSubsystem<TDistance>::SetGoal(position);
             },
             {this})
      .ToPtr();
}

template <typename TController, typename TDistance>
frc2::CommandPtr BaseSingleAxisSubsystem<TController, TDistance>::Home() {
  return frc2::FunctionalCommand(
             // OnInit
             [this] { Stop(); },
             // OnExecute
             [this] { RunMotorSpeedDefault(true); },
             // OnEnd
             [this](bool interrupted) {
               Stop();
               ResetEncoder();
             },
             // IsFinished
             [this] { return AtLimitSwitchMin(); }, {this})
      .ToPtr();
}

template <typename TController, typename TDistance>
void BaseSingleAxisSubsystem<TController, TDistance>::DisablePid() {
  m_pidEnabled = false;
  Stop();
  frc2::TrapezoidProfileSubsystem<TDistance>::Disable();
}

template <typename TController, typename TDistance>
void BaseSingleAxisSubsystem<TController, TDistance>::EnablePid() {
  m_pidEnabled = true;
  frc2::TrapezoidProfileSubsystem<TDistance>::Enable();
}

template <typename TController, typename TDistance>
void BaseSingleAxisSubsystem<TController, TDistance>::OnInit() {
  m_controller.SetPidTolerance(m_config.tolerance.value());
  m_controller.SetEncoderConversionFactor(
      m_config.encoderDistancePerRevolution.value());

  if (m_config.absoluteEncoderDistancePerRevolution.has_value())
    m_controller.SetAbsoluteEncoderConversionFactor(
        m_config.absoluteEncoderDistancePerRevolution.value().value());
}

template <typename TController, typename TDistance>
bool BaseSingleAxisSubsystem<TController, TDistance>::IsMovementAllowed(
    double speed, bool ignoreEncoder) {
  if (m_config.ignoreLimit())
    return true;

  bool atMin = ignoreEncoder ? AtLimitSwitchMin() : AtHome();
  bool atMax = ignoreEncoder ? AtLimitSwitchMax() : AtMax();

  frc::SmartDashboard::PutBoolean(m_name + " At Min", atMin);
  frc::SmartDashboard::PutBoolean(m_name + " At Max", atMax);

  if (atMin) {
    return speed >= 0;
  }

  if (atMax) {
    return speed <= 0;
  }

  return true;
}

template <typename TController, typename TDistance>
bool BaseSingleAxisSubsystem<TController, TDistance>::IsMovementAllowed(
    bool ignoreEncoder) {
  if (m_config.ignoreLimit())
    return true;

  bool atMin = ignoreEncoder ? AtLimitSwitchMin() : AtHome();
  bool atMax = ignoreEncoder ? AtLimitSwitchMax() : AtMax();

  if (atMin) {
    return false;
  }

  if (atMax) {
    return false;
  }

  return true;
}