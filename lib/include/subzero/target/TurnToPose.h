#pragma once

#include <frc/controller/HolonomicDriveController.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/smartdashboard/Field2d.h>

#include <functional>
#include <memory>

#include "subzero/target/ITurnToTarget.h"

/**
 * @brief Allows the robot/an axis to face an arbitrary Pose2d
 *
 */
class TurnToPose : public ITurnToTarget {
public:
  struct TurnToPoseConfig {
    frc::TrapezoidProfile<units::radians>::Constraints rotationConstraints;
    double turnP;
    double turnI;
    double turnD;
    double translationP;
    double translationI;
    double translationD;
    /**
     * @brief A Pose2d within this range will be considered "at-goal"
     *
     */
    frc::Pose2d poseTolerance;
  };

  /**
   * @brief Construct a new TurnToPose instance
   *
   * @param config
   * @param poseGetter Returns the robot's current Pose2d
   * @param fieldGetter Returns the current Field2d
   */
  explicit TurnToPose(TurnToPoseConfig config,
                      std::function<frc::Pose2d()> poseGetter,
                      std::function<frc::Field2d *()> fieldGetter);

  /**
   * @brief Call this every loop to get the latest angle relative to the robot's
   * position
   *
   */
  void Update() override;

  /**
   * @brief Set the pose the robot/axis should be facing
   *
   * @param pose
   */
  void SetTargetPose(frc::Pose2d pose);

  /**
   * @brief Set the angle the robot/axis should be facing relative to its
   * current rotation
   *
   * @param angle
   */
  void SetTargetAngleRelative(units::degree_t angle);

  /**
   * @brief Set the angle the robot/axis should be facing relative to the
   * field's coordinate system
   *
   * @param angle
   */
  void SetTargetAngleAbsolute(units::degree_t angle);

  /**
   * @brief Get the Speed Correction object
   *
   * @return frc::ChassisSpeeds
   */
  frc::ChassisSpeeds GetSpeedCorrection() override;

  /**
   * @brief Helper to calculate the relative angle between two poses
   *
   * @param currentPose
   * @param targetPose
   */
  static units::degree_t GetAngleFromOtherPose(const frc::Pose2d &,
                                               const frc::Pose2d &);

  /**
   * @brief Combines two ChassisSpeeds into a single result via a linear
   * regression and coefficient
   *
   * @param other The initial ChassisSpeeds
   * @param correctionFactor Ranges from 0 to 1; percentage of the TurnToPose
   * correction to apply
   */
  frc::ChassisSpeeds BlendWithInput(const frc::ChassisSpeeds &other,
                                    double correctionFactor) override;

  /**
   * @brief Check if the robot is at the goal angle
   *
   * @return true
   * @return false
   */
  inline bool AtGoal() override { return m_driveController->AtReference(); }

  inline std::optional<frc::Pose2d> GetTargetPose() const {
    return m_targetPose;
  }

  inline std::optional<units::degree_t> GetTargetAngle() const {
    return m_targetAngle;
  }

  inline units::degree_t GetTargetHeading() const { return m_targetHeading; }

  /**
   * @brief Helper to normalize a value to a new range
   *
   * @param x Input
   * @param from_min Input min
   * @param from_max Input max
   * @param to_min Output min
   * @param to_max Output max
   * @return double
   */
  static double NormalizeScalar(double x, double from_min, double from_max,
                                double to_min, double to_max) {
    return (x - from_min) * (to_max - to_min) / (from_max - from_min) + to_min;
  }

private:
  TurnToPoseConfig m_config;
  std::function<frc::Pose2d()> m_poseGetter;
  std::function<frc::Field2d *()> m_fieldGetter;
  std::unique_ptr<frc::HolonomicDriveController> m_driveController;
  frc::Pose2d m_startPose;
  std::optional<frc::Pose2d> m_targetPose;
  std::optional<units::degree_t> m_targetAngle;
  units::degree_t m_targetHeading;
  frc::ChassisSpeeds m_speeds;
};