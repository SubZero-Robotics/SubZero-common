#include "subsystems/MultiAxisSubsystem.h"

frc2::CommandPtr MultiAxisSubsystem::SetPose(Pose pose) {
  auto axis = _axes[pose.index];
  return frc2
      : FunctionalCommand([pose, axis] { axis->MoveToPosition(pose.position); },

                          [] {},

                          [axis](bool interrupted) { axis->StopMovement(); },

                          [axis] { return !axis->GetIsMovingToPosition(); },
                          {axis})
            .ToPtr();
}

frc2::CommandPtr MultiAxisSubsystem::SetPose(std::span<Pose> poses) {
  auto cmd = frc2::RunCommand([] {}, {}).ToPtr();
  if (poses.empty()) {
    return cmd;
  }
  for (auto pose : poses) {
    cmd = std::move(cmd).AlongWith(SetPose(pose))
  }
  return cmd;
}