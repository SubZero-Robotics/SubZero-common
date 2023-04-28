#pragma once

#include <Arduino.h>

constexpr uint8_t receiveBufSize = 8;
// Make sure this is unique on the bus
// TODO: Use DIP switch to set address
constexpr uint8_t slaveAddress = 0x01;

// TODO: Set I2C0/1 pins, UART pins, and SPI0 pins

constexpr uint8_t ledDataOutPin = 2;
constexpr uint8_t ledBrightness = 50;

constexpr uint16_t ledNum = 150;
constexpr uint8_t patternCount = 7;
constexpr uint8_t chaseLEDWidth = ceil((float)ledNum / 20);