#pragma once

#include "subzero/singleaxis/LinearSingleAxisSubsystem.h"

#include <frc/geometry/Pose3d.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "subzero/singleaxis/BaseSingleAxisSubsystem.cpp"

using namespace subzero;

template <typename TController>
void LinearSingleAxisSubsystem<TController>::Periodic() {
  using LinearBase = BaseSingleAxisSubsystem<TController, units::meter>;

  LinearBase::Periodic();

  auto length =
      ((LinearBase::m_config.reversed ? -(LinearBase::GetCurrentPosition())
                                      : LinearBase::GetCurrentPosition()) +
       LinearBase::m_config.mechanismConfig.minimumLength);

  if (LinearBase::m_ligament2d) {
    LinearBase::m_ligament2d->SetLength(length.value());
  }

  frc::Rotation3d rot{0_rad, 0_rad, 0_rad};
  frc::Pose3d pose{
      0_m, 0_m, (length - LinearBase::m_config.mechanismConfig.minimumLength),
      rot};
  std::vector<double> data{pose.X().value(),        pose.Y().value(),
                           pose.Z().value(),        rot.GetQuaternion().W(),
                           rot.GetQuaternion().X(), rot.GetQuaternion().Y(),
                           rot.GetQuaternion().Z()};
  frc::SmartDashboard::PutNumberArray(LinearBase::m_name + "_position", data);
}

template <typename TController>
void LinearSingleAxisSubsystem<TController>::RunMotorVelocity(
    units::meters_per_second_t speed, bool ignoreEncoder) {
  using LinearBase = BaseSingleAxisSubsystem<TController, units::meter>;

  LinearBase::DisablePid();
  ConsoleWriter.logWarning(
      LinearBase::m_name,
      "Running with a velocity is not supported for linear subsystems!%s", "");
}