#pragma once

#include <Arduino.h>
#include <EEPROM.h>
#include <extEEPROM.h>

#include <memory>
#include <string>

#include "Constants.h"

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
  uint8_t valid;
  uint16_t teamNumber;
  // Send messages to 2 other teams
  uint16_t initialTeams[2];
  uint8_t i2c0Addr;
  LedConfiguration led0;
  LedConfiguration led1;
};

class Configurator {
public:
  Configurator() {}

  Configurator(EEPROMConfiguration config, TwoWire *wire) {
    eeprom = std::make_unique<extEEPROM>(config.size, config.numDevices,
                                         config.pageSize, config.address);
    // eeprom->begin(eeprom->twiClock400kHz, wire);
    EEPROM.begin(256);
  }

  Configuration readConfig() {
    Configuration config;

    // eeprom->read(0, (byte *)&config, sizeof(Configuration));
    EEPROM.get(0, config);

    for (uint8_t i = 0; i < sizeof(Configuration); i++) {
      Serial.printf("%X\r\n", *((uint8_t *)&config + i));
    }

    return config;
  }

  void storeConfig(Configuration config) {
    config.valid = 1;

    EEPROM.put(0, config);
    EEPROM.commit();
  }

  bool checkIfValid() {
    auto cfg = readConfig();
    return cfg.valid == 1;
  }

  void configSetup();

  static std::string toString(Configuration);

private:
  std::unique_ptr<extEEPROM> eeprom;
};