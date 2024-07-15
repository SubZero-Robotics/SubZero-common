#pragma once

#include "subzero/singleaxis/RotationalSingleAxisSubsystem.h"

#include <frc/geometry/Pose3d.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "subzero/singleaxis/BaseSingleAxisSubsystem.cpp"

using namespace subzero;

template <typename TController>
void RotationalSingleAxisSubsystem<TController>::Periodic() {
  using RotationalBase = BaseSingleAxisSubsystem<TController, units::degree>;

  RotationalBase::Periodic();

  auto angle = (RotationalBase::m_config.reversed
                    ? -RotationalBase::GetCurrentPosition()
                    : RotationalBase::GetCurrentPosition()) +
               RotationalBase::m_config.mechanismConfig.minimumAngle;
  if (RotationalBase::m_ligament2d) {
    RotationalBase::m_ligament2d->SetAngle(angle);
  }

  frc::Rotation3d rot{0_rad, -angle - 30_deg, 0_rad};
  frc::Pose3d pose{0_m, 0_m, 0_m, rot};
  std::vector<double> data{pose.X().value(),        pose.Y().value(),
                           pose.Z().value(),        rot.GetQuaternion().W(),
                           rot.GetQuaternion().X(), rot.GetQuaternion().Y(),
                           rot.GetQuaternion().Z()};
  frc::SmartDashboard::PutNumberArray(RotationalBase::m_name + "_position",
                                      data);
}

template <typename TController>
void RotationalSingleAxisSubsystem<TController>::RunMotorVelocity(
    units::degrees_per_second_t speed, bool ignoreEncoder) {
  if (!BaseSingleAxisSubsystem<TController, units::degree>::IsMovementAllowed(
          speed.value(), ignoreEncoder)) {
    return;
  }

  BaseSingleAxisSubsystem<TController, units::degree>::DisablePid();

  BaseSingleAxisSubsystem<TController, units::degree>::m_controller
      .RunWithVelocity(speed);
}