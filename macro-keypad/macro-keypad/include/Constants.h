#pragma once

#include <Arduino.h>

constexpr uint8_t ColCount = 3;
constexpr uint8_t RowCount = 5;

namespace Pins {
constexpr uint8_t Led = 27;

static const uint8_t ColumnPins[ColCount] = {
    0,
    1,
    2,
};
static const uint8_t RowPins[RowCount] = {
    18, 19, 20, 21, 22,
};
} // namespace Pins

namespace Keys {
constexpr uint16_t DebounceDelay = 50;

enum class GamepadButton {
  None = -1,
  ButtonA = 0,
  ButtonB = 1,
  ButtonX = 2,
  ButtonY = 3,
  ButtonBumperLeft = 4,
  ButtonBumperRight = 5,
  ButtonBack = 6,
  ButtonStart = 7,
  ButtonLeftStick = 8,
  ButtonRightStick = 9,
  ButtonVirtual0 = 15,
  ButtonVirtual1 = 16,
  ButtonVirtual2 = 17,
  ButtonVirtual3 = 18,
  ButtonVirtual4 = 19,
  ButtonVirtual5 = 20
};
} // namespace Keys

typedef struct {
  uint8_t currentState;
  uint8_t lastSteadyState = 0;
  uint8_t lastFlickerState = 0;
  Keys::GamepadButton button = Keys::GamepadButton::None;
  uint32_t timestamp = 0;
} KeypadKey_t;

static KeypadKey_t keys[RowCount][ColCount];
