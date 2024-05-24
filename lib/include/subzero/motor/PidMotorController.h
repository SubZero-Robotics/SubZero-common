#pragma once

#include <frc/controller/PIDController.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/CANSparkBase.h>
#include <units/angle.h>
#include <units/angular_velocity.h>

#include <string>

#include "subzero/logging/ConsoleLogger.h"

namespace subzero {
struct PidSettings {
  double p, i, d, iZone, ff;
};

// TODO: Group into a single, combined typename
/**
 * @brief Combines a motor, motor drvier, relative encoder, and absolute encoder
 * into a single wrapper; helpful for absolute positioning, setting velocities,
 * or tuning PID values
 *
 * @tparam TMotor
 * @tparam TController
 * @tparam TRelativeEncoder
 * @tparam TAbsoluteEncoder
 */
template <typename TMotor, typename TController, typename TRelativeEncoder,
          typename TAbsoluteEncoder>
class PidMotorController {
public:
  /**
   * @brief Construct a new PidMotorController
   *
   * @param name Display name in SmartDashboard
   * @param motor
   * @param encoder
   * @param controller
   * @param pidSettings Initial PID settings
   * @param absEncoder Set to nullptr for a disconnected encoder
   * @param maxRpm Max RPM of the motor; used to set velocity based on
   * percentage
   */
  explicit PidMotorController(std::string name, TMotor &motor,
                              TRelativeEncoder &encoder,
                              TController &controller, PidSettings pidSettings,
                              TAbsoluteEncoder *absEncoder,
                              units::revolutions_per_minute_t maxRpm);

  /**
   * @brief Set the motor to a percentage of max voltage
   *
   * @param percentage
   */
  inline void Set(double percentage) { m_motor.Set(percentage); }

  /**
   * @brief Set a motor to a voltage
   *
   * @param volts
   */
  void Set(units::volt_t volts);

  /**
   * @brief Absolute positioning is considered 'Done' when within this zone
   *
   * @param tolerance
   */
  void SetPidTolerance(double tolerance);

  /**
   * @brief ! Call this every loop in Periodic !
   *
   */
  void Update();

  /**
   * @brief Set to this velocity
   *
   * @param rpm
   */
  void RunWithVelocity(units::revolutions_per_minute_t rpm);

  /**
   * @brief Set to a percentage of max RPM
   *
   * @param percentage
   */
  void RunWithVelocity(double percentage);

  /**
   * @brief Enables absolute positioning and sets the target to the position
   *
   * @param position
   */
  void RunToPosition(double position);

  inline virtual void ResetEncoder() {
    m_encoder.SetPosition(0);
    ConsoleWriter.logInfo(m_name + " PID Controller", "Reset encoder%s", "");
  }

  inline double GetEncoderPosition() { return m_encoder.GetPosition(); }

  std::optional<double> GetAbsoluteEncoderPosition();

  /**
   * @brief Sets the multiplier for going between encoder ticks and actual
   * distance
   *
   * @param factor
   */
  inline void SetEncoderConversionFactor(double factor) {
    m_encoder.SetPositionConversionFactor(factor);
  }

  /**
   * @brief Sets the multiplier for going between encoder ticks and actual
   * distance
   *
   * @param factor
   */
  inline void SetAbsoluteEncoderConversionFactor(double factor) {
    if (m_absEncoder) {
      m_absEncoder->SetPositionConversionFactor(factor);
    }
  }

  /**
   * @brief Disable absolute positioning and stop the motor
   *
   */
  void Stop();

  inline const PidSettings &GetPidSettings() { return m_settings; }

  void UpdatePidSettings(PidSettings settings);

  const std::string m_name;

protected:
  TMotor &m_motor;
  TController &m_controller;
  TRelativeEncoder &m_encoder;
  TAbsoluteEncoder *m_absEncoder;
  PidSettings m_settings;
  frc::PIDController m_pidController;
  bool m_absolutePositionEnabled = false;
  double m_absoluteTarget = 0;
  const units::revolutions_per_minute_t m_maxRpm;
};

/**
 * @brief Intended for use alongside a PidMotorController for simple tuning
 * through SmartDashboard
 *
 * @tparam TMotor
 * @tparam TController
 * @tparam TRelativeEncoder
 * @tparam TAbsoluteEncoder
 */
template <typename TMotor, typename TController, typename TRelativeEncoder,
          typename TAbsoluteEncoder>
class PidMotorControllerTuner {
public:
  explicit PidMotorControllerTuner(
      PidMotorController<TMotor, TController, TRelativeEncoder,
                         TAbsoluteEncoder> &controller)
      : m_controller{controller} {
    frc::SmartDashboard::PutNumber(m_controller.m_name + " P Gain",
                                   m_controller.GetPidSettings().p);
    frc::SmartDashboard::PutNumber(m_controller.m_name + " I Gain",
                                   m_controller.GetPidSettings().i);
    frc::SmartDashboard::PutNumber(m_controller.m_name + " D Gain",
                                   m_controller.GetPidSettings().d);
    frc::SmartDashboard::PutNumber(m_controller.m_name + " IZone",
                                   m_controller.GetPidSettings().iZone);
    frc::SmartDashboard::PutNumber(m_controller.m_name + " Feed Forward",
                                   m_controller.GetPidSettings().ff);
  }

  /**
   * @brief Call this within the Periodic method of the encapsulating subsystem
   *
   */
  void UpdateFromShuffleboard() {
    double tP = frc::SmartDashboard::GetNumber(m_controller.m_name + " P Gain",
                                               m_controller.GetPidSettings().p);
    double tI = frc::SmartDashboard::GetNumber(m_controller.m_name + " I Gain",
                                               m_controller.GetPidSettings().i);
    double tD = frc::SmartDashboard::GetNumber(m_controller.m_name + " D Gain",
                                               m_controller.GetPidSettings().d);
    double tIZone = frc::SmartDashboard::GetNumber(
        m_controller.m_name + " IZone", m_controller.GetPidSettings().iZone);
    double tFeedForward =
        frc::SmartDashboard::GetNumber(m_controller.m_name + " Feed Forward",
                                       m_controller.GetPidSettings().ff);

    m_controller.UpdatePidSettings(
        {.p = tP, .i = tI, .d = tD, .iZone = tIZone, .ff = tFeedForward});
  }

private:
  PidMotorController<TMotor, TController, TRelativeEncoder, TAbsoluteEncoder>
      &m_controller;
};

class RevPidMotorController
    : public PidMotorController<rev::CANSparkMax, rev::SparkPIDController,
                                rev::SparkRelativeEncoder,
                                rev::SparkAbsoluteEncoder> {};
} // namespace subzero