#pragma once

#include "Arduino.h"
#include "Commands.h"

namespace CommandParser {
    static void parseCommand(uint8_t *buf, size_t len, Command* cmd)
    {
        auto type = (CommandType)buf[0];
        Serial.print("Received command type=");
        Serial.println(buf[0]);
        cmd->commandType = type;
        switch (type)
        {
        case CommandType::On:
            cmd->commandData.commandOn = {};
            break;

        case CommandType::Off:
            cmd->commandData.commandOff = {};
            break;

        case CommandType::Pattern:
            memcpy(&cmd->commandData.commandPattern.pattern, &buf[1], sizeof(CommandPattern));
            break;

        case CommandType::ChangeColor:
            memcpy(&cmd->commandData.commandColor.red, &buf[1], sizeof(CommandColor));
            break;

        case CommandType::ReadPatternDone:
            cmd->commandData.commandReadPatternDone = {};
            break;

        default:
            break;
        }
    }
}