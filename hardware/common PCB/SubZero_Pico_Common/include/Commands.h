#pragma once

#include <Arduino.h>

// TODO: Add commands for Digital IO
// TODO: Add command for Analog Input
// TODO: Add command to set active LED port
enum class CommandType
{
    On = 0,
    Off = 1,
    Pattern = 2,
    ChangeColor = 3,
    ReadPatternDone = 4
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

union CommandData
{
    CommandOn commandOn;
    CommandOff commandOff;
    CommandPattern commandPattern;
    CommandColor commandColor;
    CommandReadPatternDone commandReadPatternDone;
};

struct Command
{
    CommandType commandType;
    CommandData commandData;
};