#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/RunCommand.h>

#include <memory>
#include <span>

#include "subsystems/ISingleAxisSubsystem.h"

class MultiAxisSubsystem : frc2::SubsystemBase {
public:
  typedef uint8_t AxisIndex;

  struct Pose {
    AxisIndex index;
    double position;
  };

  MultiAxisSubsystem(std::span<ISingleAxisSubsystem *> axes) : _axes(axes){};

  frc2::CommandPtr Stop();

  frc2::CommandPtr Stop(AxisIndex index);

  frc2::CommandPtr Home();

  frc2::CommandPtr Home(AxisIndex index);

  // TODO: @WowMuchDoge look at SetPose in CompleteArmSubsystem to see how
  // blocking is done
  /**
   * @brief Sets a single pose and BLOCKS until done
   *
   * @param pose
   * @return frc2::CommandPtr
   */
  frc2::CommandPtr SetPose(Pose pose);

  /**
   * @brief Set poses for multiple axes and BLOCKS until ALL are done
   *
   * @param poses Iterable of poses
   * @return frc2::CommandPtr
   */
  frc2::CommandPtr SetPose(std::span<Pose> poses);

  bool IsMoving(AxisIndex index) const;

  void Periodic() override;

private:
  std::span<ISingleAxisSubsystem *> _axes;
};