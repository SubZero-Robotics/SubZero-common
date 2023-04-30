#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include <Wire.h>

#include <pico/mutex.h>

#include "CommandParser.h"
#include "Commands.h"
#include "Configurator.h"
#include "Constants.h"
#include "PatternRunner.h"

static mutex_t mtx;

// Forward declarations
void receiveEvent(int);
void requestEvent(void);
Adafruit_NeoPixel *getPixels(uint8_t port);
PatternRunner *getPatternRunner(uint8_t port);

static volatile uint8_t receiveBuf[i2cReceiveBufSize];
static volatile bool newData = false;
static volatile bool newDataToParse = false;

static constexpr EEPROMConfiguration eepromConfig = {
    .size = kbits_2, .numDevices = 1, .pageSize = 8, .address = eepromAddr};

static Configuration config;
static Configurator *configurator;
static uint8_t ledPort = 0;

static Adafruit_NeoPixel *pixels0;
static Adafruit_NeoPixel *pixels1;
static PatternRunner *patternRunner0;
static PatternRunner *patternRunner1;

static Command command;

static bool systemOn = true;

void setup() {
  Wire1.setSDA(Pin::I2C::Port1::SDA);
  Wire1.setSCL(Pin::I2C::Port1::SCL);
  Wire1.begin();

  configurator = new Configurator(eepromConfig, &Wire1);

  Serial.begin(115200);
  pinMode(Pin::CONFIG::CONFIG_SETUP, INPUT_PULLUP);
  if (!configurator->checkIfValid() ||
      !digitalRead(Pin::CONFIG::CONFIG_SETUP)) {
    configurator->configSetup();
  }

  config = configurator->readConfig();

  pixels0 = new Adafruit_NeoPixel(config.led0.count, Pin::LED::Dout0,
                                  NEO_GRB + NEO_KHZ800);
  pixels1 = new Adafruit_NeoPixel(config.led1.count, Pin::LED::Dout1,
                                  NEO_GRB + NEO_KHZ800);
  patternRunner0 = new PatternRunner(pixels0, Animation::patterns);
  patternRunner1 = new PatternRunner(pixels1, Animation::patterns);

  // Peripherals
  Wire.setSDA(Pin::I2C::Port0::SDA);
  Wire.setSCL(Pin::I2C::Port0::SCL);
  Wire.onReceive(receiveEvent); // register event
  Wire.onRequest(requestEvent);
  Wire.begin(config.i2c0Addr); // join i2c bus as slave

  SPI1.setTX(Pin::SPI::MOSI);
  SPI1.setRX(Pin::SPI::MISO);
  SPI1.setSCK(Pin::SPI::CLK);
  SPI1.setCS(Pin::SPI::CS0);
  SPI1.begin();

  Serial1.setTX(Pin::UART::Tx);
  Serial1.setRX(Pin::UART::Rx);
  Serial1.begin(uartBaudRate);

  mutex_init(&mtx);

  pinMode(Pin::SPI::CS0, OUTPUT);
  digitalWrite(Pin::SPI::CS0, HIGH);
  pinMode(Pin::SPI::CS1, OUTPUT);
  digitalWrite(Pin::SPI::CS1, HIGH);
  pinMode(Pin::SPI::SdCardCS, OUTPUT);
  digitalWrite(Pin::SPI::SdCardCS, HIGH);
  SD.begin(Pin::SPI::SdCardCS, SPI);

  pinMode(Pin::DIGITALIO::P0, OUTPUT);
  digitalWrite(Pin::DIGITALIO::P0, LOW);
  pinMode(Pin::DIGITALIO::P1, OUTPUT);
  digitalWrite(Pin::DIGITALIO::P1, LOW);
  pinMode(Pin::DIGITALIO::P2, OUTPUT);
  digitalWrite(Pin::DIGITALIO::P2, LOW);
  pinMode(Pin::DIGITALIO::P3, OUTPUT);
  digitalWrite(Pin::DIGITALIO::P3, LOW);
  pinMode(Pin::DIGITALIO::P4, OUTPUT);
  digitalWrite(Pin::DIGITALIO::P4, LOW);
  pinMode(Pin::DIGITALIO::P5, OUTPUT);
  digitalWrite(Pin::DIGITALIO::P5, LOW);

  pinMode(Pin::ANALOGIO::ADC0, INPUT);
  pinMode(Pin::ANALOGIO::ADC1, INPUT);
  pinMode(Pin::ANALOGIO::ADC2, INPUT);

  pixels0->begin();
  pixels0->setBrightness(config.led0.brightness);
  // Initialize all LEDs to black
  Animation::executePatternSetAll(*pixels0, 0, 0, pixels0->numPixels());
  pixels0->show();

  pixels1->begin();
  pixels1->setBrightness(config.led1.brightness);
  // Initialize all LEDs to black
  Animation::executePatternSetAll(*pixels1, 0, 0, pixels1->numPixels());
  pixels1->show();
}

void setup1() {}

Adafruit_NeoPixel *getPixels(uint8_t port) {
  if (port == 0) {
    return pixels0;
  }

  return pixels1;
}

PatternRunner *getPatternRunner(uint8_t port) {
  if (port == 0) {
    return patternRunner0;
  }

  return patternRunner1;
}

void loop() {
  // If there's new data, process it
  if (newData) {
    newData = false;
    uint32_t owner;
    if (mutex_try_enter(&mtx, &owner)) {
      switch (command.commandType) {
      case CommandType::On: {
        // Go back to running the current color and pattern
        patternRunner0->reset();
        patternRunner1->reset();
        systemOn = true;
        break;
      }

      case CommandType::Off: {
        // Set LEDs to black and stop running the pattern
        for (uint8_t port = 0; port < 2; port++) {
          auto pixels = getPixels(ledPort);
          Animation::executePatternSetAll(*pixels, 0, 0, pixels->numPixels());
          pixels->show();
        }
        systemOn = false;
        break;
      }

      case CommandType::Pattern: {
        // To set everything to a certain color, change color then call
        // the 'set all' pattern
        auto runner = getPatternRunner(ledPort);
        uint16_t delay =
            command.commandData.commandPattern.delay == -1
                ? runner->getPattern(command.commandData.commandPattern.pattern)
                      ->changeDelayDefault
                : command.commandData.commandPattern.delay;

        runner->setCurrentPattern(command.commandData.commandPattern.pattern,
                                  command.commandData.commandPattern.oneShot,
                                  delay);
        break;
      }

      case CommandType::ChangeColor: {
        auto runner = getPatternRunner(ledPort);
        auto colors = command.commandData.commandColor;
        runner->setCurrentColor(
            Adafruit_NeoPixel::Color(colors.red, colors.green, colors.blue));
        break;
      }

      case CommandType::SetLedPort: {
        ledPort = command.commandData.commandSetLedPort.port;
        break;
      }

      case CommandType::DigitalSetup: {
        auto cfg = command.commandData.commandDigitalSetup;
        auto pin = Pin::DIGITALIO::digitalIOMap.at(cfg.port);
        pinMode(pin, cfg.mode);
        break;
      }

      case CommandType::DigitalWrite: {
        auto cfg = command.commandData.commandDigitalWrite;
        auto pin = Pin::DIGITALIO::digitalIOMap.at(cfg.port);
        digitalWrite(pin, cfg.value);
        break;
      }

      case CommandType::SetConfig: {
        config = command.commandData.commandSetConfig.config;
        Serial.println("Storing new config=");
        Serial.println(configurator->toString(config).c_str());
        configurator->storeConfig(config);
        break;
      }

      default:
        break;
      }

      mutex_exit(&mtx);
    }

    Serial.print(F("ON="));
    Serial.println(systemOn);
  }

  if (systemOn) {
    patternRunner0->update();
    patternRunner1->update();
  }
}

void loop1() {
  if (newDataToParse) {
    uint8_t buf[i2cReceiveBufSize];
    // Safely copy our new data
    noInterrupts();
    memcpy(buf, (const void *)receiveBuf, i2cReceiveBufSize);
    interrupts();
    mutex_enter_blocking(&mtx);
    CommandParser::parseCommand(buf, i2cReceiveBufSize, &command);
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
  /**
   * @brief Returns uint8_t
   *
   */
  case CommandType::ReadPatternDone: {
    auto runner = getPatternRunner(ledPort);
    Wire.write(runner->patternDone());
    break;
  }

  /**
   * @brief Returns uint16_t
   *
   */
  case CommandType::ReadAnalog: {
    uint16_t value = analogRead(Pin::ANALOGIO::analogIOMap.at(
        command.commandData.commandReadAnalog.port));
    Wire.write((uint8_t *)&value, sizeof(value));
    break;
  }

  /**
   * @brief Returns uint8_t
   *
   */
  case CommandType::DigitalRead: {
    uint8_t value = digitalRead(Pin::DIGITALIO::digitalIOMap.at(
        command.commandData.commandDigitalRead.port));
    Wire.write(value);
    break;
  }

  default:
    // Send back 255 (-1 signed) to indicate bad/no data
    Wire.write(0xff);
  }
}
