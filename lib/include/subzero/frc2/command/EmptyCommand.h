#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/Requirements.h>

#include <functional>

namespace subzero {
class EmptyCommand
    : public frc2::CommandHelper<frc2::FunctionalCommand, EmptyCommand> {
public:
  explicit EmptyCommand();

  EmptyCommand(EmptyCommand &&other) = default;
};
} // namespace subzero