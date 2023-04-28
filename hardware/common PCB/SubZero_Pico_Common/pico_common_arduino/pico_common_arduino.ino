#include <Arduino.h>
#include <Wire.h>

#include <pico/mutex.h>

#include "Constants.h"
#include "Commands.h"
#include "PatternRunner.h"

static mutex_t mtx;

// TODO: Move I2C parsing logic to separate class

// Forward declarations
void receiveEvent(int);
void requestEvent(void);
void parseCommand(uint8_t *, size_t);

static volatile uint8_t receiveBuf[receiveBufSize];
static volatile bool newData = false;
static volatile bool newDataToParse = false;
static Adafruit_NeoPixel pixels(ledNum, ledDataOutPin, NEO_GRB + NEO_KHZ800);
static Command command;
static PatternRunner patternRunner(&pixels, Animation::patterns);
static bool systemOn = true;

void setup() {
    // TODO: Set I2C pins
    // TODO: Set SPI pins
    // TODO: Set UART pins
    mutex_init(&mtx);
    Wire.begin(slaveAddress);      // join i2c bus with address #4
    pixels.begin();
    pixels.setBrightness(ledBrightness);
    Wire.onReceive(receiveEvent);  // register event
    Wire.onRequest(requestEvent);
    Serial.begin(115200);  // start serial for output

    // Initialize all LEDs to black
    Animation::executePatternSetAll(pixels, 0, 0, ledNum);
    pixels.show();
}

// TODO: Use second core to handle slave I2C communication
void setup1() {
    
}

void loop() {
    // If there's new data, process it
    if (newData) {
        newData = false;
        uint32_t owner;
        // TODO: Use mutex_enter_blocking for core1 when parsing new I2C data
        if(mutex_try_enter(&mtx, &owner)) {
          switch (command.commandType) {
              case CommandType::On:
                  // Go back to running the current color and pattern
                  patternRunner.reset();
                  systemOn = true;
                  break;

              case CommandType::Off:
                  // Set LEDs to black and stop running the pattern
                  Animation::executePatternSetAll(pixels, 0, 0, ledNum);
                  pixels.show();
                  systemOn = false;
                  break;

              case CommandType::Pattern: {
                  // To set everything to a certain color, change color then call
                  // the 'set all' pattern
                  uint16_t delay = command.commandData.commandPattern.delay == -1 ?
                      patternRunner.getPattern(command.commandData.commandPattern.pattern)->changeDelayDefault :
                      command.commandData.commandPattern.delay;

                  patternRunner.setCurrentPattern(
                      command.commandData.commandPattern.pattern,
                      command.commandData.commandPattern.oneShot,
                      delay);
              } break;

              case CommandType::ChangeColor: {
                  auto colors = command.commandData.commandColor;
                  patternRunner.setCurrentColor(Adafruit_NeoPixel::Color(colors.red, colors.green, colors.blue));
              } break;

              default:
                  break;
          }

          mutex_exit(&mtx);
        }

        Serial.print(F("ON="));
        Serial.println(systemOn);
    }

    if (systemOn) {
        patternRunner.update();
    }
}

void loop1() {
    if (newDataToParse) {
        // TODO: Parse it here instead
        uint8_t buf[receiveBufSize];
        // Safely copy our new data
        noInterrupts();
        memcpy(buf, (const void *)receiveBuf, receiveBufSize);
        interrupts();
        mutex_enter_blocking(&mtx);
        parseCommand(buf, receiveBufSize);
        mutex_exit(&mtx);

        newDataToParse = false;
        newData = true;
    }
}

void receiveEvent(int howMany) {
    Wire.readBytes((uint8_t *)receiveBuf, howMany);
    newDataToParse = true;
}

void requestEvent() {
    switch (command.commandType) {
        case CommandType::ReadPatternDone:
            Wire.write(patternRunner.patternDone());
            break;

        default:
            // Send back 255 (-1 signed) to indicate bad/no data
            Wire.write(0xff);
    }
}

void parseCommand(uint8_t *buf, size_t len) {
    auto type = (CommandType)buf[0];
    Serial.print("Received command type=");
    Serial.println(buf[0]);
    command.commandType = type;
    switch (type) {
        case CommandType::On:
            command.commandData.commandOn = {};
            break;

        case CommandType::Off:
            command.commandData.commandOff = {};
            break;

        case CommandType::Pattern:
            memcpy(&command.commandData.commandPattern.pattern, &buf[1], sizeof(CommandPattern));
            break;

        case CommandType::ChangeColor:
            memcpy(&command.commandData.commandColor.red, &buf[1], sizeof(CommandColor));
            break;

        case CommandType::ReadPatternDone:
            command.commandData.commandReadPatternDone = {};
            break;

        default:
            break;
    }
}
