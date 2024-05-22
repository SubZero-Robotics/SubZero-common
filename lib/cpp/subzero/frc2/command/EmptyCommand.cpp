#include "subzero/frc2/command/EmptyCommand.h"

using namespace subzero;

EmptyCommand::EmptyCommand()
    : CommandHelper(
          std::move([] {}), [] {}, [](bool interrupted) {}, [] { return true; },
          {}) {}