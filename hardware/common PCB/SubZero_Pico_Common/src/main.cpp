#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>

#include <pico/mutex.h>

#include "Constants.h"
#include "Commands.h"
#include "CommandParser.h"
#include "Configurator.h"
#include "PatternRunner.h"

static mutex_t mtx;

// Forward declarations
void receiveEvent(int);
void requestEvent(void);

static volatile uint8_t receiveBuf[i2cReceiveBufSize];
static volatile bool newData = false;
static volatile bool newDataToParse = false;

static constexpr EEPROMConfiguration eepromConfig = {
    .size = kbits_2,
    .numDevices = 1,
    .pageSize = 8,
    .address = eepromAddr
};

static Configuration config;
static Configurator configurator;

// TODO: Set up dynamically after reading config data
static Adafruit_NeoPixel pixels0(config.led0.count, Pin::LED::Dout0, NEO_GRB + NEO_KHZ800);
static Adafruit_NeoPixel pixels1(config.led1.count, Pin::LED::Dout1, NEO_GRB + NEO_KHZ800);
static PatternRunner patternRunner0(&pixels0, Animation::patterns);
static PatternRunner patternRunner1(&pixels1, Animation::patterns);

static Command command;

static bool systemOn = true;

void setup()
{
    Wire1.setSDA(Pin::I2C::Port1::SDA);
    Wire1.setSCL(Pin::I2C::Port1::SCL);
    Wire1.begin();

    configurator = Configurator(eepromConfig, &Wire1);

    Serial.begin(115200);
    pinMode(Pin::CONFIG::CONFIG_SETUP, INPUT_PULLUP);
    if (!configurator.checkIfValid() || !digitalRead(Pin::CONFIG::CONFIG_SETUP)) {
        configurator.configSetup();
    }

    config = configurator.readConfig();

    // Peripherals
    Wire.setSDA(Pin::I2C::Port0::SDA);
    Wire.setSCL(Pin::I2C::Port0::SCL);
    Wire.onReceive(receiveEvent); // register event
    Wire.onRequest(requestEvent);
    Wire.begin(config.i2c0Addr);  // join i2c bus as slave

    SPI1.setTX(Pin::SPI::MOSI);
    SPI1.setRX(Pin::SPI::MISO);
    SPI1.setSCK(Pin::SPI::CLK);
    SPI1.setCS(Pin::SPI::CS0);
    SPI1.begin();

    Serial1.setTx(Pin::UART::Tx);
    Serial1.setRx(Pin::UART::Rx);
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

    pixels0.begin();
    pixels0.setBrightness(config.led0.brightness);
    // Initialize all LEDs to black
    Animation::executePatternSetAll(pixels0, 0, 0, pixels0.numPixels());
    pixels0.show();

    pixels1.begin();
    pixels1.setBrightness(config.led1.brightness);
    // Initialize all LEDs to black
    Animation::executePatternSetAll(pixels1, 0, 0, pixels1.numPixels());
    pixels1.show();
}

void setup1()
{
}

void loop()
{
    // If there's new data, process it
    if (newData)
    {
        newData = false;
        uint32_t owner;
        if (mutex_try_enter(&mtx, &owner))
        {
            switch (command.commandType)
            {
            case CommandType::On:
                // Go back to running the current color and pattern
                patternRunner0.reset();
                patternRunner1.reset();
                systemOn = true;
                break;

            case CommandType::Off:
                // Set LEDs to black and stop running the pattern
                Animation::executePatternSetAll(pixels, 0, 0, ledNum);
                pixels.show();
                systemOn = false;
                break;

            case CommandType::Pattern:
            {
                // To set everything to a certain color, change color then call
                // the 'set all' pattern
                uint16_t delay = command.commandData.commandPattern.delay == -1 ? patternRunner.getPattern(command.commandData.commandPattern.pattern)->changeDelayDefault : command.commandData.commandPattern.delay;

                patternRunner.setCurrentPattern(
                    command.commandData.commandPattern.pattern,
                    command.commandData.commandPattern.oneShot,
                    delay);
            }
            break;

            case CommandType::ChangeColor:
            {
                auto colors = command.commandData.commandColor;
                patternRunner.setCurrentColor(Adafruit_NeoPixel::Color(colors.red, colors.green, colors.blue));
            }
            break;

            default:
                break;
            }

            mutex_exit(&mtx);
        }

        Serial.print(F("ON="));
        Serial.println(systemOn);
    }

    if (systemOn)
    {
        patternRunner0.update();
        patternRunner1.update();
    }
}

void loop1()
{
    if (newDataToParse)
    {
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

void receiveEvent(int howMany)
{
    Wire.readBytes((uint8_t *)receiveBuf, howMany);
    newDataToParse = true;
}

void requestEvent()
{
    switch (command.commandType)
    {
    case CommandType::ReadPatternDone:
        Wire.write(patternRunner.patternDone());
        break;

    default:
        // Send back 255 (-1 signed) to indicate bad/no data
        Wire.write(0xff);
    }
}
