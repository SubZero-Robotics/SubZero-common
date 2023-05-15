#pragma once

// TODO: Make this file into Generic PCB Driver instead!

#include <frc/I2C.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SubsystemBase.h>

#include <memory>

#include "Constants.h"
#include "utils/Logging.h"

class LEDControllerSubsystem : public frc2::SubsystemBase {
public:
  enum class CommandType {
    On = 0,
    Off = 1,
    Pattern = 2,
    ChangeColor = 3,
    ReadPatternDone = 4,
    SetLedPort = 5,
    ReadAnalog = 6,
    DigitalSetup = 7,
    DigitalWrite = 8,
    DigitalRead = 9,
    SetConfig = 10,
    RadioSend = 11,
    RadioGetLatestReceived = 12
};

  enum class PatternType {
    None = 0,
    SetAll = 1,
    Blink = 2,
    RGBFade = 3,
    HackerMode = 4
  };

  enum class PinMode {
    INPUT = 0,
    OUTPUT = 1,
    INPUT_PULLUP = 2,
    INPUT_PULLDOWN = 3,
    OUTPUT_2MA = 4,
    OUTPUT_4MA = 5,
    OUTPUT_8MA = 6,
    OUTPUT_12MA = 7
  };

  enum class DigitalPort {
    P0 = 0,
    P1 = 1,
    P2 = 2,
    P3 = 3,
    P4 = 4,
    P5 = 5
  };

  enum class AnalogPort {
    A0 = 0,
    A1 = 1,
    A2 = 2
  };

  enum class LedPort {
    P0 = 0,
    P1 = 1
  };

  struct LedConfiguration {
    uint16_t count;
    uint8_t brightness;
    };

    struct EEPROMConfiguration {
    eeprom_size_t size = kbits_2;
    uint8_t numDevices = 1;
    uint16_t pageSize = 8;
    uint8_t address = eepromAddr;
    };

    struct Configuration {
    int8_t valid;
    uint16_t teamNumber;
    // Send messages to 2 other teams
    uint16_t initialTeams[2];
    uint8_t i2c0Addr;
    LedConfiguration led0;
    LedConfiguration led1;
    };

  ConnectorX(uint8_t slaveAddress,
                         frc::I2C::Port port = frc::I2C::kMXP);

  /**
   * @brief Start communication with the controller
   *
   * @return true if successful init
   */
  bool initialize();

  /**
   * @brief Get the last command sent
   *
   * @return CommandType
   */
  inline CommandType lastCommand(LedPort port) const { return _lastCommand[(uint8_t)port]; }

  inline PatternType lastPattern(LedPort port) const { return _lastPattern[(uint8_t)port]; }

  void configureDigitalPin(DigitalPort port, PinMode mode);

    void writeDigitalPin(DigitalPort value);

  bool readDigitalPin(DigitalPort port);

  uint16_t readAnalogPin(AnalogPort port);

  /**
   * @brief Send the ON command
   *
   */
  void setOn(LedPort port);

  /**
   * @brief Send the OFF command
   *
   */
  void setOff(LedPort port);

  /**
   * @brief Send the PATTERN command
   *
   * @param pattern
   * @param oneShot Only run the pattern once
   */
  void setPattern(LedPort port, PatternType pattern, bool oneShot = false,
                  int16_t delay = -1);

  /**
   * @brief Send the COLOR command
   *
   */
  void setColor(LedPort port, uint8_t red, uint8_t green, uint8_t blue);
  /**
   * @brief Send the COLOR command
   *
   * @param color Color data in the form of 0x00RRGGBB
   */
  bool setColor(LedPort port, uint32_t color);

  /**
   * @brief Send the READPATTERNDONE command
   *
   * @return true if pattern is done
   */
  bool getPatternDone(LedPort port);

  void 

private:
  std::unique_ptr<frc::I2C> _i2c;
  uint8_t _slaveAddress;
  CommandType _lastCommands[2];
  PatternType _lastPattern[2];
};
