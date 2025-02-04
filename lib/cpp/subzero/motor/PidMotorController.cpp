#pragma once

#include "subzero/motor/PidMotorController.h"

using namespace subzero;

template <typename TMotor, typename TController, typename TRelativeEncoder,
          typename TAbsoluteEncoder, typename TPidConfig>
PidMotorController<
    TMotor, TController, TRelativeEncoder, TAbsoluteEncoder,
    TPidConfig>::PidMotorController(std::string name, TMotor &motor,
                                    TRelativeEncoder &encoder,
                                    TController &controller,
                                    PidSettings pidSettings,
                                    TAbsoluteEncoder *absEncoder,
                                    units::revolutions_per_minute_t maxRpm)
    : IPidMotorController(name), m_motor{motor}, m_controller{controller},
      m_encoder{encoder}, m_absEncoder{absEncoder}, m_settings{pidSettings},
      m_pidController{
          frc::PIDController{pidSettings.p, pidSettings.i, pidSettings.d}},
      m_maxRpm{maxRpm}, m_isInitialized{false} {

  // Doing it here so the PID controllers themselves get updated
  UpdatePidSettings(pidSettings);
}

template <typename TMotor, typename TController, typename TRelativeEncoder,
          typename TAbsoluteEncoder, typename TPidConfig>
void PidMotorController<TMotor, TController, TRelativeEncoder, TAbsoluteEncoder,
                        TPidConfig>::Set(units::volt_t volts) {
  frc::SmartDashboard::PutNumber(m_name + " Commanded volts", volts.value());
  m_motor.SetVoltage(volts);
}

template <typename TMotor, typename TController, typename TRelativeEncoder,
          typename TAbsoluteEncoder, typename TPidConfig>
void PidMotorController<TMotor, TController, TRelativeEncoder, TAbsoluteEncoder,
                        TPidConfig>::Set(double percentage) {
  m_motor.Set(percentage);
}

template <typename TMotor, typename TController, typename TRelativeEncoder,
          typename TAbsoluteEncoder, typename TPidConfig>
void PidMotorController<TMotor, TController, TRelativeEncoder, TAbsoluteEncoder,
                        TPidConfig>::SetPidTolerance(double tolerance) {
  m_pidController.SetTolerance(tolerance);
}

template <typename TMotor, typename TController, typename TRelativeEncoder,
          typename TAbsoluteEncoder, typename TPidConfig>
void PidMotorController<TMotor, TController, TRelativeEncoder, TAbsoluteEncoder,
                        TPidConfig>::Update() {
  if (m_absolutePositionEnabled) {
    // ConsoleWriter.logVerbose(
    //     m_name,
    //     "relative position %0.3f, absolute position %0.3f, absolute target"
    //     "%0.3f",
    //     GetEncoderPosition(), GetAbsoluteEncoderPosition(),
    //     m_absoluteTarget);
    auto effort =
        m_pidController.Calculate(GetEncoderPosition(), m_absoluteTarget);
    double totalEffort = effort;
    Set(units::volt_t(totalEffort));

    if (m_pidController.AtSetpoint()) {
      m_pidController.Reset();
      m_absolutePositionEnabled = false;
      Stop();
    }
  }
}

template <typename TMotor, typename TController, typename TRelativeEncoder,
          typename TAbsoluteEncoder, typename TPidConfig>
void PidMotorController<
    TMotor, TController, TRelativeEncoder, TAbsoluteEncoder,
    TPidConfig>::RunWithVelocity(units::revolutions_per_minute_t rpm) {
  m_absolutePositionEnabled = false;
  frc::SmartDashboard::PutNumber(m_name + "commanded rpm", rpm.value());
  m_controller.SetReference(rpm.value(),
                            rev::spark::SparkLowLevel::ControlType::kVelocity);
}

template <typename TMotor, typename TController, typename TRelativeEncoder,
          typename TAbsoluteEncoder, typename TPidConfig>
void PidMotorController<TMotor, TController, TRelativeEncoder, TAbsoluteEncoder,
                        TPidConfig>::RunWithVelocity(double percentage) {
  if (abs(percentage) > 1.0) {
    ConsoleWriter.logError("PidMotorController",
                           "Incorrect percentages for motor %s: Value=%.4f ",
                           m_name.c_str(), percentage);
    return;
  }
  auto rpm = units::revolutions_per_minute_t(m_maxRpm) * percentage;
  RunWithVelocity(rpm);
}

template <typename TMotor, typename TController, typename TRelativeEncoder,
          typename TAbsoluteEncoder, typename TPidConfig>
void PidMotorController<TMotor, TController, TRelativeEncoder, TAbsoluteEncoder,
                        TPidConfig>::RunToPosition(double position) {
  ConsoleWriter.logVerbose(m_name + " Target position", position);
  Stop();
  m_pidController.Reset();
  m_absolutePositionEnabled = true;
  m_absoluteTarget = position;
}

template <typename TMotor, typename TController, typename TRelativeEncoder,
          typename TAbsoluteEncoder, typename TPidConfig>
std::optional<double>
PidMotorController<TMotor, TController, TRelativeEncoder, TAbsoluteEncoder,
                   TPidConfig>::GetAbsoluteEncoderPosition() {
  if (m_absEncoder) {
    return m_absEncoder->GetPosition();
  }

  return std::nullopt;
}

template <typename TMotor, typename TController, typename TRelativeEncoder,
          typename TAbsoluteEncoder, typename TPidConfig>
void PidMotorController<TMotor, TController, TRelativeEncoder, TAbsoluteEncoder,
                        TPidConfig>::Stop() {
  m_absolutePositionEnabled = false;
  m_motor.Set(0);
}

template <typename TMotor, typename TController, typename TRelativeEncoder,
          typename TAbsoluteEncoder, typename TPidConfig>
void PidMotorController<TMotor, TController, TRelativeEncoder, TAbsoluteEncoder,
                        TPidConfig>::UpdatePidSettings(PidSettings settings) {

  bool changed = false;

  if (settings.p != m_settings.p || !m_isInitialized) {
    ConsoleWriter.logInfo("PidMotorController", "Setting P to %.6f for %s",
                          settings.p, m_name.c_str());

    m_config.closedLoop.P(settings.p);
    changed = true;
  }

  if (settings.i != m_settings.i || !m_isInitialized) {
    ConsoleWriter.logInfo("PidMotorController", "Setting I to %.6f for %s",
                          settings.i, m_name.c_str());
    m_config.closedLoop.I(settings.i);
    changed = true;
  }

  if (settings.d != m_settings.d || !m_isInitialized) {
    ConsoleWriter.logInfo("PidMotorController", "Setting D to %.6f for %s",
                          settings.d, m_name.c_str());
    m_config.closedLoop.D(settings.d);
    changed = true;
  }

  if (settings.iZone != m_settings.iZone || !m_isInitialized) {
    ConsoleWriter.logInfo("PidMotorController", "Setting IZone to %.6f for %s",
                          settings.iZone, m_name.c_str());
    m_config.closedLoop.IZone(settings.iZone);
    changed = true;
  }

  if (settings.ff != m_settings.ff || !m_isInitialized) {
    ConsoleWriter.logInfo("PidMotorController", "Setting FF to %.6f for %s",
                          settings.ff, m_name.c_str());
    m_config.closedLoop.VelocityFF(settings.ff);
    changed = true;
  }

  if (settings.isIdleModeBrake != m_settings.isIdleModeBrake ||
      !m_isInitialized) {
    m_config.SetIdleMode(m_settings.isIdleModeBrake
                             ? rev::spark::SparkBaseConfig::IdleMode::kBrake
                             : rev::spark::SparkBaseConfig::IdleMode::kCoast);
    changed = true;
  }

  if (changed || !m_isInitialized) {
    m_motor.Configure(m_config,
                      rev::spark::SparkBase::ResetMode::kNoResetSafeParameters,
                      rev::spark::SparkBase::PersistMode::kPersistParameters);
    m_isInitialized = true;
  }

  m_settings = settings;
}