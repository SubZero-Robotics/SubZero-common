#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include <Wire.h>

#include <pico/mutex.h>

#include "CommandParser.h"
#include "Commands.h"
#include "Configurator.h"
#include "Constants.h"
#include "PacketRadio.h"
#include "PatternRunner.h"

static mutex_t i2cCommandMtx;
static mutex_t radioDataMtx;

// Forward declarations
void receiveEvent(int);
void requestEvent(void);
void handleRadioDataReceive(Message msg);
void centralRespond(Response response);

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

static PacketRadio *radio;

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

  radio = new PacketRadio(&SPI1, config, handleRadioDataReceive);

  for (int i = 0; i < 2; i++) {
    radio->addTeam(config.initialTeams[i]);
  }

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

  mutex_init(&i2cCommandMtx);
  mutex_init(&radioDataMtx);

  pinMode(Pin::SPI::CS0, OUTPUT);
  digitalWrite(Pin::SPI::CS0, HIGH);
  pinMode(Pin::SPI::CS1, OUTPUT);
  digitalWrite(Pin::SPI::CS1, HIGH);
  pinMode(Pin::SPI::SdCardCS, OUTPUT);
  digitalWrite(Pin::SPI::SdCardCS, HIGH);
  SD.begin(Pin::SPI::SdCardCS, SPI);

  for (auto pin : Pin::DIGITALIO::digitalIOMap) {
    pinMode(pin.second, OUTPUT);
    digitalWrite(pin.second, LOW);
  }

  for (auto pin : Pin::ANALOGIO::analogIOMap) {
    pinMode(pin.second, INPUT);
  }

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
    if (mutex_try_enter(&i2cCommandMtx, &owner)) {
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

      case CommandType::RadioSend: {
        auto message = command.commandData.commandRadioSend;
        Message msg;
        msg.teamNumber = message.teamNumber;
        msg.len = message.dataLen;
        memcpy(&msg, message.data, message.dataLen);
        if (msg.teamNumber == Radio::SendToAll) {
          radio->sendToAll(msg);
        } else {
          radio->send(msg);
        }
      }

      default:
        break;
      }

      mutex_exit(&i2cCommandMtx);
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
    mutex_enter_blocking(&i2cCommandMtx);
    CommandParser::parseCommand(buf, i2cReceiveBufSize, &command);
    mutex_exit(&i2cCommandMtx);

    newDataToParse = false;
    newData = true;
  }

  radio->update();
}

void handleRadioDataReceive(Message msg) {
  mutex_enter_blocking(&radioDataMtx);
  Serial.println("New data:");
  Serial.printf("Team = %d\n", msg.teamNumber);
  Serial.printf("Data len = %d\n", msg.len);
  Serial.print("Data (HEX) = ");

  for (uint8_t i = 0; i < msg.len; i++) {
    Serial.printf("%02X ", msg.data[i]);
  }

  Serial.println();

  mutex_exit(&radioDataMtx);
}

void receiveEvent(int howMany) {
  Wire.readBytes((uint8_t *)receiveBuf, howMany);
  newDataToParse = true;
}

void requestEvent() {
    Response res;
    res.commandType = command.commandType;

  switch (command.commandType) {
  /**
   * @brief Returns uint8_t
   *
   */
  case CommandType::ReadPatternDone: {
    auto done = getPatternRunner(ledPort)->patternDone();
    res.responseData.responsePatternDone = done;
    break;
  }

  /**
   * @brief Returns uint16_t
   *
   */
  case CommandType::ReadAnalog: {
    uint16_t value = analogRead(Pin::ANALOGIO::analogIOMap.at(
        command.commandData.commandReadAnalog.port));
    res.responseData.responseReadAnalog.value = value;
    break;
  }

  /**
   * @brief Returns uint8_t
   *
   */
  case CommandType::DigitalRead: {
    uint8_t value = digitalRead(Pin::DIGITALIO::digitalIOMap.at(
        command.commandData.commandDigitalRead.port));
    res.responseData.responseDigitalRead.value = value;
    break;
  }

  case CommandType::RadioGetLatestReceived: {
    mutex_enter_blocking(&radioDataMtx);
    auto msg = radio->getLastReceived();
    mutex_exit(&radioDataMtx);

    res.responseData.responseRadioLastReceived.msg = msg;
    break;
  }

  case CommandType::ReadConfig: {
    auto cfg = configurator->readConfig();
    res.responseData.responseReadConfiguration.config = cfg;
    break;
  }

  default:
    // Send back 255 (-1 signed) to indicate bad/no data
    Wire.write(0xff);
    return;
  }

  centralRespond(res);
}

void centralRespond(Response response) {
    uint16_t size;

    switch (response.commandType) {
        case CommandType::ReadPatternDone:
            size = sizeof(ResponsePatternDone);
            break;
        case CommandType::ReadAnalog:
            size = sizeof(ResponseReadAnalog);
            break;
        case CommandType::DigitalRead:
            size = sizeof(ResponseDigitalRead);
            break;
        case CommandType::RadioGetLatestReceived:
            size = sizeof(ResponseRadioLastReceived);
            break;
        case CommandType::ReadConfig:
            size = sizeof(ResponseReadConfiguration);
            break;
        default:
            size = 0;
    }

    Wire.write((uint8_t*)&response, size + sizeof(response.commandType));
}