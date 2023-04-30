#pragma once

#include <Arduino.h>

enum class CommandType
{
    On = 0,
    Off = 1,
    Pattern = 2,
    ChangeColor = 3,
    ReadPatternDone = 4,
    SetLedPort = 5,
    ReadAnalog = 6,
    DigitalSetup = 7,
    DigitalWrite = 8,
    DigitalRead = 9
};

struct CommandOn
{
};

struct CommandOff
{
};

// * Set delay to -1 to use default delay
struct CommandPattern
{
    uint8_t pattern;
    uint8_t oneShot;
    int16_t delay;
};

struct CommandColor
{
    uint8_t red;
    uint8_t green;
    uint8_t blue;
};

struct CommandReadPatternDone
{
};

struct CommandSetLedPort {
    uint8_t port;
};

struct CommandReadAnalog {
    uint8_t port;
};

struct CommandDigitalSetup {
    uint8_t port;
    /** Follows the Arduino-defined values for pinMode
     * INPUT = 0
     * INPUT_PULLUP = 2
     * INPUT_PULLDOWN = 3
     * OUTPUT = 1
     * OUTPUT_2MA = 4
     * OUTPUT_4MA = 5
     * OUTPUT_8MA = 6
     * OUTPUT_12MA = 7
     */
    uint8_t mode;
};

struct CommandDigitalWrite {
    uint8_t port;
    uint8_t value;
};

struct CommandDigitalRead {
    uint8_t port;
};

union CommandData
{
    CommandOn commandOn;
    CommandOff commandOff;
    CommandPattern commandPattern;
    CommandColor commandColor;
    CommandReadPatternDone commandReadPatternDone;
    CommandSetLedPort commandSetLedPort;
    CommandReadAnalog commandReadAnalog;
    CommandDigitalSetup commandDigitalSetup;
    CommandDigitalWrite commandDigitalWrite;
    CommandDigitalRead commandDigitalRead;
};

struct Command
{
    CommandType commandType;
    CommandData commandData;
};