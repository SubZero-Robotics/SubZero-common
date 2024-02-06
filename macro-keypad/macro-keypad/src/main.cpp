#include <Arduino.h>
#include <Joystick.h>

#include "Constants.h"

using namespace Keys;

// Col 0 starts on the left
// Row 0 starts at the top

// Row 4 column 1 is not connected to anything

static GamepadButton keyMapping[RowCount][ColCount] = {
    /* Row 0 */ {
        GamepadButton::ButtonVirtual0,
        GamepadButton::ButtonVirtual1,
        GamepadButton::ButtonVirtual2,
    },
    /* Row 1 */
    {
        GamepadButton::ButtonX,
        GamepadButton::ButtonB,
        GamepadButton::ButtonA,
    },
    /* Row 2 */
    {
        GamepadButton::ButtonY,
        GamepadButton::None,
        GamepadButton::None,
    },
    /* Row 3 */
    {
        GamepadButton::ButtonBumperLeft,
        GamepadButton::ButtonBumperRight,
        GamepadButton::ButtonLeftStick,
    },
    /* Row 4 */
    {
        GamepadButton::ButtonRightStick,
        GamepadButton::ButtonRightStick,
        GamepadButton::ButtonRightStick,
    },
};

static Joystick_ gamepad;

static void configureKeys(void);
static void processKey(uint8_t, uint8_t);

void setup() {
  pinMode(Pins::Led, OUTPUT);
  digitalWriteFast(Pins::Led, HIGH);

  for (uint8_t i = 0; i < ColCount; i++) {
    pinMode(Pins::ColumnPins[i], OUTPUT);
    digitalWriteFast(Pins::ColumnPins[i], LOW);
  }

  for (uint8_t i = 0; i < RowCount; i++) {
    pinMode(Pins::RowPins[i], INPUT_PULLDOWN);
  }

  configureKeys();

  gamepad.begin();
}

void loop() {
  for (uint8_t col = 0; col < ColCount; col++) {
    digitalWriteFast(Pins::ColumnPins[col], HIGH);
    for (uint8_t row = 0; row < RowCount; row++) {
      processKey(row, col);
    }
    digitalWriteFast(Pins::ColumnPins[col], LOW);
  }
}

void configureKeys() {
  for (uint8_t row = 0; row < RowCount; row++) {
    for (uint8_t col = 0; col < ColCount; col++) {
      keys[row][col] = {.currentState = 0,
                        .lastSteadyState = 0,
                        .lastFlickerState = 0,
                        .button = keyMapping[row][col],
                        .timestamp = 0};
    }
  }
}

void processKey(uint8_t row, uint8_t col) {
  auto *key = &keys[row][col];

  if (key->button == GamepadButton::None) {
    return;
  }

  key->currentState = digitalRead(Pins::RowPins[row]);

  if (key->currentState != key->lastFlickerState) {
    key->timestamp = millis();
    key->lastFlickerState = key->currentState;
  }

  if ((millis() - key->timestamp) > DebounceDelay) {
    if (key->lastSteadyState && !key->currentState) {
      // Released
      gamepad.setButton(static_cast<uint8_t>(key->button), 0);
    } else if (!key->lastSteadyState && key->currentState) {
      // Pressed
      gamepad.setButton(static_cast<uint8_t>(key->button), 1);
    }

    key->lastSteadyState = key->currentState;
  }
}