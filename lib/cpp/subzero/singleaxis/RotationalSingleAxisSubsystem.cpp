#pragma once

#include "subzero/singleaxis/RotationalSingleAxisSubsystem.h"
#include "subzero/singleaxis/BaseSingleAxisSubsystem.cpp"

using namespace subzero;

template <typename TController>
void RotationalSingleAxisSubsystem<TController>::Periodic()
{
    using RotationalBase = BaseSingleAxisSubsystem<TController, units::degree>;

    RotationalBase::Periodic();

    if (RotationalBase::m_ligament2d)
    {
        RotationalBase::m_ligament2d
            ->SetAngle(
                (RotationalBase::m_config.reversed
                     ? -RotationalBase::GetCurrentPosition()
                     : RotationalBase::GetCurrentPosition()) +
                RotationalBase::m_config
                    .mechanismConfig.minimumAngle);
    }
}

template <typename TController>
void RotationalSingleAxisSubsystem<TController>::RunMotorVelocity(units::degrees_per_second_t speed,
                                                                  bool ignoreEncoder)
{
    if (!BaseSingleAxisSubsystem<TController,
                                 units::degree>::IsMovementAllowed(speed.value(), ignoreEncoder))
    {
        return;
    }

    BaseSingleAxisSubsystem<TController, units::degree>::DisablePid();

    BaseSingleAxisSubsystem<TController, units::degree>::m_controller
        .RunWithVelocity(speed);
}