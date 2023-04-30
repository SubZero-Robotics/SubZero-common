#pragma once

#include "Arduino.h"
#include "Commands.h"

namespace CommandParser {
static void parseCommand(uint8_t *buf, size_t len, Command *cmd) {
  auto type = (CommandType)buf[0];
  Serial.print("Received command type=");
  Serial.println(buf[0]);
  cmd->commandType = type;
  switch (type) {
  case CommandType::On:
    cmd->commandData.commandOn = {};
    break;

  case CommandType::Off:
    cmd->commandData.commandOff = {};
    break;

  case CommandType::Pattern:
    memcpy(&cmd->commandData.commandPattern.pattern, &buf[1],
           sizeof(CommandPattern));
    break;

  case CommandType::ChangeColor:
    memcpy(&cmd->commandData.commandColor.red, &buf[1], sizeof(CommandColor));
    break;

  case CommandType::ReadPatternDone:
    cmd->commandData.commandReadPatternDone = {};
    break;

  case CommandType::SetLedPort:
    memcpy(&cmd->commandData.commandSetLedPort.port, &buf[1],
           sizeof(CommandSetLedPort));
    break;

  case CommandType::ReadAnalog:
    memcpy(&cmd->commandData.commandReadAnalog.port, &buf[1],
           sizeof(CommandReadAnalog));
    break;

  case CommandType::DigitalSetup:
    memcpy(&cmd->commandData.commandDigitalSetup.port, &buf[1],
           sizeof(CommandDigitalSetup));
    break;

  case CommandType::DigitalWrite:
    memcpy(&cmd->commandData.commandDigitalWrite.port, &buf[1],
           sizeof(CommandDigitalWrite));
    break;

  case CommandType::DigitalRead:
    memcpy(&cmd->commandData.commandDigitalRead.port, &buf[1],
           sizeof(CommandDigitalRead));
    break;

    case CommandType::SetConfig:
        memcpy(&cmd->commandData.commandSetConfig.config, &buf[1], sizeof(CommandSetConfig));
        break;

  default:
    break;
  }
}
} // namespace CommandParser