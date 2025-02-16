#include "subzero/target/TurnToPose.h"

#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/trajectory/TrajectoryGenerator.h>

using namespace subzero;

TurnToPose::TurnToPose(TurnToPoseConfig config,
                       std::function<frc::Pose2d()> poseGetter,
                       std::function<frc::Field2d *()> fieldGetter)
    : m_config{config}, m_poseGetter{poseGetter}, m_fieldGetter{fieldGetter} {
  auto xController = frc::PIDController(
      m_config.translationP, m_config.translationI, m_config.translationD);
  auto yController = frc::PIDController(
      m_config.translationP, m_config.translationI, m_config.translationD);
  auto profile = m_config.rotationConstraints;
  auto profiledController = frc::ProfiledPIDController<units::radians>(
      m_config.turnP, m_config.turnI, m_config.turnD, profile);

  m_driveController = std::make_unique<frc::HolonomicDriveController>(
      xController, yController, profiledController);
  m_driveController->SetTolerance(m_config.poseTolerance);
}

void TurnToPose::Update() {
  if (!m_targetPose && !m_targetAngle)
    return;

  auto currentPose = m_poseGetter();
  auto *field = m_fieldGetter();

  units::degree_t targetAngle = 0_deg;
  if (m_targetPose) {
    targetAngle = GetAngleFromOtherPose(currentPose, m_targetPose.value());
    field->GetObject("pose_target")->SetPose(m_targetPose.value());
  } else if (m_targetAngle) {
    targetAngle = m_targetAngle.value();
  }

  frc::SmartDashboard::PutNumber("Angle offset norm", targetAngle.value());
  m_targetHeading = targetAngle;
  frc::Pose2d newTargetPose(currentPose.Translation(),
                            frc::Rotation2d(targetAngle));
  field->GetObject("angle_target")->SetPose(newTargetPose);

  m_speeds = m_driveController->Calculate(currentPose, newTargetPose, 0_mps,
                                          newTargetPose.Rotation());
}

units::degree_t
TurnToPose::GetAngleFromOtherPose(const frc::Pose2d &currentPose,
                                  const frc::Pose2d &otherPose) {
  auto diff = currentPose.Translation() - otherPose.Translation();

  auto newDegree = units::radian_t(atan2(diff.Y().value(), diff.X().value()))
                       .convert<units::degree>();
  return otherPose.Rotation().Degrees() + newDegree;
}

void TurnToPose::SetTargetPose(frc::Pose2d pose) {
  m_startPose = m_poseGetter();
  m_targetPose = pose;
  m_targetAngle = std::nullopt;
}

void TurnToPose::SetTargetAngleRelative(units::degree_t angle) {
  m_startPose = m_poseGetter();
  m_targetAngle = m_startPose.RotateBy(angle).Rotation().Degrees();
  m_targetPose = std::nullopt;
}

void TurnToPose::SetTargetAngleAbsolute(units::degree_t angle) {
  m_startPose = m_poseGetter();
  m_targetAngle = angle;
  m_targetPose = std::nullopt;
}

frc::ChassisSpeeds TurnToPose::GetSpeedCorrection() { return m_speeds; }

frc::ChassisSpeeds TurnToPose::BlendWithInput(const frc::ChassisSpeeds &other,
                                              double correctionFactor) {
  frc::ChassisSpeeds speeds{
      .vx = other.vx, .vy = other.vy, .omega = other.omega};

  speeds.omega = (speeds.omega * (1 - correctionFactor)) +
                 (m_speeds.omega * correctionFactor);

  return speeds;
}