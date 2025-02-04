#pragma once

#include <frc/controller/PIDController.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/ClosedLoopSlot.h>
#include <rev/SparkBase.h>
#include <rev/SparkClosedLoopController.h>
#include <rev/SparkFlex.h>
#include <rev/SparkMax.h>
#include <rev/config/SparkFlexConfig.h>
#include <rev/config/SparkMaxConfig.h>
#include <units/angle.h>
#include <units/angular_velocity.h>

#include <string>

#include "subzero/logging/ConsoleLogger.h"
#include "subzero/motor/IPidMotorController.h"

namespace subzero {
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
          typename TAbsoluteEncoder, typename TPidConfig>
class PidMotorController : public IPidMotorController {
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

  // TODO: make `absEncoder` an `std::optional<>` instead of a raw pointer
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
  void Set(double percentage) override;

  /**
   * @brief Set a motor to a voltage
   *
   * @param volts
   */
  void Set(units::volt_t volts) override;

  /**
   * @brief Absolute positioning is considered 'Done' when within this zone
   *
   * @param tolerance
   */
  void SetPidTolerance(double tolerance) override;

  /**
   * @brief ! Call this every loop in Periodic !
   *
   */
  void Update() override;

  /**
   * @brief Set to this velocity
   *
   * @param rpm
   */
  void RunWithVelocity(units::revolutions_per_minute_t rpm) override;

  /**
   * @brief Set to a percentage of max RPM
   *
   * @param percentage
   */
  void RunWithVelocity(double percentage) override;

  /**
   * @brief Enables absolute positioning and sets the target to the position
   *
   * @param position
   */
  void RunToPosition(double position) override;

  inline virtual void ResetEncoder() override {
    m_encoder.SetPosition(0);
    ConsoleWriter.logInfo(m_name + " PID Controller", "Reset encoder%s", "");
  }

  inline double GetEncoderPosition() override {
    return m_encoder.GetPosition();
  }

  std::optional<double> GetAbsoluteEncoderPosition() override;

  /**
   * @brief Sets the multiplier for going between encoder ticks and actual
   * distance
   *
   * @param factor
   */
  inline void SetEncoderConversionFactor(double factor) override {
    m_config.encoder.PositionConversionFactor(factor);
    m_config.encoder.VelocityConversionFactor(factor);

    m_motor.Configure(m_config,
                      rev::spark::SparkBase::ResetMode::kNoResetSafeParameters,
                      rev::spark::SparkBase::PersistMode::kPersistParameters);
  }

  /**
   * @brief Sets the multiplier for going between encoder ticks and actual
   * distance
   *
   * @param factor
   */
  inline void SetAbsoluteEncoderConversionFactor(double factor) override {
    if (m_absEncoder) {
      m_config.absoluteEncoder.PositionConversionFactor(factor);
      m_config.absoluteEncoder.VelocityConversionFactor(factor);

      m_motor.Configure(
          m_config, rev::spark::SparkBase::ResetMode::kNoResetSafeParameters,
          rev::spark::SparkBase::PersistMode::kPersistParameters);
    }
  }

  /**
   * @brief Disable absolute positioning and stop the motor
   *
   */
  void Stop() override;

  inline const PidSettings &GetPidSettings() override { return m_settings; }

  void UpdatePidSettings(PidSettings settings);

protected:
  TMotor &m_motor;
  TController &m_controller;
  TRelativeEncoder &m_encoder;
  TAbsoluteEncoder *m_absEncoder;
  TPidConfig m_config;
  PidSettings m_settings;
  frc::PIDController m_pidController;
  bool m_absolutePositionEnabled = false;
  double m_absoluteTarget = 0;
  const units::revolutions_per_minute_t m_maxRpm;
  bool m_isInitialized;
};

// TODO: Move to its own file and make it work with IPidMotorController
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
          typename TAbsoluteEncoder, typename TPidConfig>
class PidMotorControllerTuner {
public:
  explicit PidMotorControllerTuner(
      PidMotorController<TMotor, TController, TRelativeEncoder,
                         TAbsoluteEncoder, TPidConfig> &controller)
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
  PidMotorController<TMotor, TController, TRelativeEncoder, TAbsoluteEncoder,
                     rev::spark::SparkFlexConfig> &m_controller;
};

class RevPidMotorController
    : public PidMotorController<
          rev::spark::SparkMax, rev::spark::SparkClosedLoopController,
          rev::spark::SparkRelativeEncoder, rev::spark::SparkAbsoluteEncoder,
          rev::spark::SparkMaxConfig> {};
} // namespace subzero