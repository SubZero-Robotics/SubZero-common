#include "subzero/motor/SimPidMotorController.h"

using namespace subzero;

constexpr units::revolutions_per_minute_t kSampleRevs = 100_rpm;
constexpr units::second_t kLoopTime = 22_ms;
constexpr auto test = (kSampleRevs / (1_min / 1_s)) * kLoopTime;

SimPidMotorController::SimPidMotorController(
    std::string name, PidSettings pidSettings,
    units::revolutions_per_minute_t maxRpm)
    : IPidMotorController(name),
      m_settings{pidSettings},
      m_pidController{
          frc::PIDController{pidSettings.p, pidSettings.i, pidSettings.d}},
      m_maxRpm{maxRpm} {}

void SimPidMotorController::Update() {
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

  m_currentRelativePosition +=
      ((m_currentRpm / (1_min / 1_s)) * kLoopTime).value();
}

void SimPidMotorController::RunToPosition(double position) {
  Stop();
  m_pidController.Reset();
  m_absolutePositionEnabled = true;
  m_absoluteTarget = position;
}

void SimPidMotorController::Stop() {
  m_absolutePositionEnabled = false;
  Set(0);
}