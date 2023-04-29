#pragma once

#include <Arduino.h>
#include <extEEPROM.h>

#include <memory>
#include <string>

#include "Constants.h"

struct LedConfiguration
{
    uint16_t count;
    uint8_t brightness;
};

struct EEPROMConfiguration
{
    eeprom_size_t size = kbits_2;
    uint8_t numDevices = 1;
    uint16_t pageSize = 8;
    uint8_t address = eepromAddr;
};

struct Configuration
{
    int8_t valid;
    uint8_t i2c0Addr;
    LedConfiguration led0;
    LedConfiguration led1;
};

class Configurator
{
public:
    Configurator();

    Configurator(EEPROMConfiguration config, TwoWire* wire)
    {
        eeprom = std::make_unique<extEEPROM>(config.size,
                                             config.numDevices, config.pageSize, config.address);
        eeprom->begin(eeprom->twiClock400kHz, wire);
    }

    Configuration readConfig() {
        Configuration config;

        eeprom->read(0, (byte*)&config, sizeof(Configuration));

        return config;
    }

    void storeConfig(Configuration config) {
        config.valid = 1;

        eeprom->write(0, (byte*)&config, sizeof(Configuration));
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