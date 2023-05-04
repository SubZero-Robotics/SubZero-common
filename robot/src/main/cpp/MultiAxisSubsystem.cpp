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

frc2::CommandPtr MultiAxisSubsystem::Stop(AxisIndex index) {
  return frc2::InstantCommand([this] { _axes[index]->StopMovement(); },
                              {this, _axes})
      .ToPtr();
}

frc2::CommandPtr MultiAxisSubsystem::Stop() {
  auto cmd = frc2::RunCommand([] {}, {}).ToPtr();
  if (poses.empty()) {
    return cmd;
  }
  for (int i = 0; i < size(_axes); i++) {
    cmd = std::move(cmd).AlongWith(Stop(i));
  };
  return cmd;
}