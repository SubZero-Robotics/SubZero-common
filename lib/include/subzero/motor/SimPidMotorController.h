#pragma once

#include <frc/controller/PIDController.h>
#include <units/voltage.h>

#include <optional>

#include "subzero/motor/IPidMotorController.h"

namespace subzero {
class SimPidMotorController : public IPidMotorController {
public:
  explicit SimPidMotorController(std::string name, PidSettings pidSettings,
                                 units::revolutions_per_minute_t maxRpm);

  inline void Set(double percentage) override {
    m_currentRpm = m_maxRpm * percentage;
  }

  inline void Set(units::volt_t volts) override {
    m_currentRpm = m_maxRpm * (volts / 12.7).value();
  }

  inline void SetPidTolerance(double tolerance) override {
    m_pidController.SetTolerance(tolerance);
  }

  void Update() override;

  inline void RunWithVelocity(units::revolutions_per_minute_t rpm) override {
    m_currentRpm = rpm;
  }

  inline void RunWithVelocity(double percentage) override { Set(percentage); }

  void RunToPosition(double position) override;

  inline void ResetEncoder() override { m_currentRelativePosition = 0; }

  inline double GetEncoderPosition() override {
    return m_currentRelativePosition * m_conversionFactor;
  }

  inline std::optional<double> GetAbsoluteEncoderPosition() override {
    return std::nullopt;
  }

  inline void SetEncoderConversionFactor(double factor) override {
    m_conversionFactor = factor;
  }

  inline void SetAbsoluteEncoderConversionFactor(double factor) override {}

  void Stop() override;

  inline const PidSettings &GetPidSettings() override { return m_settings; }

  inline void UpdatePidSettings(PidSettings settings) override {
    m_settings = settings;
  }

private:
  PidSettings m_settings;
  frc::PIDController m_pidController;
  double m_currentRelativePosition = 0;
  bool m_absolutePositionEnabled = false;
  double m_conversionFactor = 1.0;
  double m_absoluteTarget = 0;
  const units::revolutions_per_minute_t m_maxRpm;
  units::revolutions_per_minute_t m_currentRpm = 0_rpm;
};
} // namespace subzero