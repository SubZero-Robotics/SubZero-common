#pragma once

#include "subzero/singleaxis/LinearSingleAxisSubsystem.h"

#include "subzero/singleaxis/BaseSingleAxisSubsystem.cpp"

using namespace subzero;

template <typename TController>
void LinearSingleAxisSubsystem<TController>::Periodic() {
  using LinearBase = BaseSingleAxisSubsystem<TController, units::meter>;

  LinearBase::Periodic();

  if (LinearBase::m_ligament2d) {
    LinearBase::m_ligament2d->SetLength(
        ((LinearBase::m_config.reversed ? -(LinearBase::GetCurrentPosition())
                                        : LinearBase::GetCurrentPosition()) +
         LinearBase::m_config.mechanismConfig.minimumLength)
            .value());
  }
}

template <typename TController>
void LinearSingleAxisSubsystem<TController>::RunMotorVelocity(
    units::meters_per_second_t speed, bool ignoreEncoder) {
  BaseSingleAxisSubsystem<TController, units::meter>::DisablePid();
  ConsoleWriter.logWarning(
      BaseSingleAxisSubsystem<TController, units::meter>::m_name,
      "Running with a velocity is not supported for linear subsystems!%s", "");
}