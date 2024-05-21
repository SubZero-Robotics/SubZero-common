#pragma once

#include <frc/DigitalInput.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>

#include <functional>
#include <memory>
#include <string>

/**
 * @brief Describes the axis for simulation
 *
 */
struct SingleAxisMechanism {
  units::meter_t minimumLength;
  units::degree_t minimumAngle;
  double lineWidth;
  frc::Color8Bit color;
};

/**
 * @brief Single axis interface
 *
 * @tparam Distance The distance unit- typically meter, inch, degree, radian,
 * etc.
 */
template <typename Distance> class ISingleAxisSubsystem {
public:
  using Distance_t = units::unit_t<Distance>;
  using Velocity =
      units::compound_unit<Distance, units::inverse<units::seconds>>;
  using Velocity_t = units::unit_t<Velocity>;
  using Acceleration =
      units::compound_unit<Velocity, units::inverse<units::seconds>>;
  using Acceleration_t = units::unit_t<Acceleration>;

  /**
   * @brief The configuration for single-axis mechanisms
   *
   */
  struct SingleAxisConfig {
    /**
     * @brief Minimum extent of motion
     *
     */
    Distance_t minDistance;
    /**
     * @brief Maximum extent of motion
     *
     */
    Distance_t maxDistance;
    /**
     * @brief Scaling factor for going between encoder ticks and distance
     *
     * @example For a 10:1 reduction, value would be 36 degrees rather than 360
     * for 1:1
     */
    Distance_t encoderDistancePerRevolution;
    /**
     * @brief Similar to encoderDistancePerRevolution but using the absolute
     * encoder
     *
     */
    std::optional<Distance_t> absoluteEncoderDistancePerRevolution;
    /**
     * @brief Default movement speed; used when homing
     *
     */
    Velocity_t defaultSpeed;
    /**
     * @brief Multiply the motor output by this factor
     *
     */
    double velocityScalar;
    /**
     * @brief Absolute positioning goal is reached within a range of +/- this
     * value
     *
     */
    Distance_t tolerance;
    /**
     * @brief Optional. Will check for the limit switch being active when seeing
     * if at limit
     *
     */
    std::optional<frc::DigitalInput *> minLimitSwitch;
    /**
     * @brief Optional. Will check for the limit switch being active when seeing
     * if at limit
     *
     */
    std::optional<frc::DigitalInput *> maxLimitSwitch;
    /**
     * @brief Motion is reversed if true; only used for simulations currently
     *
     */
    bool reversed;
    /**
     * @brief Simulation configuration
     *
     */
    SingleAxisMechanism mechanismConfig;
    /**
     * @brief Optional. Will be called on each loop; useful for outputting
     * values to SmartDashboard in a different unit than the given one
     *
     */
    std::optional<std::function<std::string(Distance_t)>> conversionFunction;
    /**
     * @brief If true, soft limits will be disabled
     *
     */
    std::function<bool()> ignoreLimit;
    /**
     * @brief Motion profile constraints
     *
     */
    frc::TrapezoidProfile<Distance>::Constraints profileConstraints;

    SingleAxisConfig(
        Distance_t _minDistance, Distance_t _maxDistance,
        Distance_t _encoderDistancePerRevolution,
        std::optional<Distance_t> _absoluteEncoderDistancePerRevolution,
        Velocity_t _defaultSpeed, double _velocityScalar, Distance_t _tolerance,
        std::optional<frc::DigitalInput *> _minLimitSwitch,
        std::optional<frc::DigitalInput *> _maxLimitSwitch, bool _reversed,
        SingleAxisMechanism _mechanismConfig,
        std::optional<std::function<std::string(Distance_t)>>
            _conversionFunction,
        std::function<bool()> _ignoreLimit,
        frc::TrapezoidProfile<Distance>::Constraints _profileConstraints)
        : minDistance{_minDistance}, maxDistance{_maxDistance},
          encoderDistancePerRevolution{_encoderDistancePerRevolution},
          absoluteEncoderDistancePerRevolution{
              _absoluteEncoderDistancePerRevolution},
          defaultSpeed{_defaultSpeed}, velocityScalar{_velocityScalar},
          tolerance{_tolerance}, minLimitSwitch{_minLimitSwitch},
          maxLimitSwitch{_maxLimitSwitch}, reversed{_reversed},
          mechanismConfig{_mechanismConfig},
          conversionFunction{_conversionFunction}, ignoreLimit{_ignoreLimit},
          profileConstraints{_profileConstraints} {}
  };

  /**
   * @brief Run at the given velocity; disables any in-progress absolute
   * movements
   *
   * @param speed
   * @param ignoreEncoder
   */
  virtual void RunMotorVelocity(Velocity_t speed,
                                bool ignoreEncoder = false) = 0;
  /**
   * @brief Run at the given velocity percentage; disables any in-progress
   * absolute movements
   *
   * @param speed
   * @param ignoreEncoder
   */
  virtual void RunMotorPercentage(double percentSpeed,
                                  bool ignoreEncoder = false) = 0;
  /**
   * @brief Run at the default motor speed
   *
   * @param ignoreEncoder
   */
  virtual void RunMotorSpeedDefault(bool ignoreEncoder = false) = 0;

  /**
   * @brief Reset encoder back to 0 ticks
   *
   */
  virtual void ResetEncoder() = 0;

  /**
   * @brief Get the current position in terms of converted distance
   *
   * @return Distance_t
   */
  virtual Distance_t GetCurrentPosition() = 0;

  /**
   * @brief Check if axis is at the mimimum extent of motion
   *
   * @return true
   * @return false
   */
  virtual bool AtHome() = 0;

  /**
   * @brief Check if axis is at the maximum extent of motion
   *
   * @return true
   * @return false
   */
  virtual bool AtMax() = 0;

  /**
   * @brief Only check the limit switch for mimimum extent
   *
   * @return true
   * @return false
   */
  virtual bool AtLimitSwitchMin() = 0;

  /**
   * @brief Only check the limit switch for maximum extent
   *
   * @return true
   * @return false
   */
  virtual bool AtLimitSwitchMax() = 0;

  /**
   * @brief Start moving to the absolute position
   *
   * @param position
   * @return frc2::CommandPtr
   */
  virtual frc2::CommandPtr MoveToPositionAbsolute(Distance_t position) = 0;

  /**
   * @brief Start moving to the position given the current distance and relative
   * position
   *
   * @param position
   * @return frc2::CommandPtr
   */
  virtual frc2::CommandPtr MoveToPositionRelative(Distance_t position) = 0;

  /**
   * @brief Start the homing sequence
   *
   * @return frc2::CommandPtr
   */
  virtual frc2::CommandPtr Home() = 0;

  /**
   * @brief Check if absolute positioning is active
   *
   * @return true
   * @return false
   */
  virtual bool IsEnabled() = 0;

  /**
   * @brief Stop and disable any in-progress absolute movements
   *
   */
  virtual void DisablePid() = 0;

  /**
   * @brief Enable absolute movements
   *
   */
  virtual void EnablePid() = 0;

  /**
   * @brief Stop the axis
   *
   */
  virtual void Stop() = 0;
};