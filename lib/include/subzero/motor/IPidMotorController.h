#pragma once

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/voltage.h>

#include <string>

namespace subzero {

// TODO: Add slot support inside of PidSettings struct
struct PidSettings {
  double p, i, d, iZone, ff;
  bool isIdleModeBrake;
};

class IPidMotorController {
public:
  explicit IPidMotorController(std::string name) : m_name{name} {}

  virtual void Set(double percentage) = 0;
  virtual void Set(units::volt_t volts) = 0;
  virtual void SetPidTolerance(double tolerance) = 0;
  virtual void Update(void) = 0;
  virtual void RunWithVelocity(units::revolutions_per_minute_t rpm) = 0;
  virtual void RunWithVelocity(double percentage) = 0;
  virtual void RunToPosition(double position) = 0;
  virtual void ResetEncoder(void) = 0;
  virtual double GetEncoderPosition(void) = 0;
  virtual std::optional<double> GetAbsoluteEncoderPosition(void) = 0;
  virtual void SetEncoderConversionFactor(double factor) = 0;
  virtual void SetAbsoluteEncoderConversionFactor(double factor) = 0;
  virtual void Stop(void) = 0;
  virtual const PidSettings &GetPidSettings(void) = 0;
  virtual void UpdatePidSettings(PidSettings settings) = 0;

  const std::string m_name;
};
} // namespace subzero