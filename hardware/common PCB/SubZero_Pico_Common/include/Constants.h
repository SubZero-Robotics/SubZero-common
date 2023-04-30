#pragma once

#include <Arduino.h>
#include <unordered_map>

namespace Pin
{
    namespace I2C
    {
        namespace Port0
        {
            constexpr uint8_t SDA = 4;
            constexpr uint8_t SCL = 5;
        }

        namespace Port1
        {
            constexpr uint8_t SDA = 14;
            constexpr uint8_t SCL = 15;
        }
    }

    namespace SPI
    {
        constexpr uint8_t MISO = 12;
        constexpr uint8_t MOSI = 11;
        constexpr uint8_t CLK = 10;
        constexpr uint8_t CS0 = 13;
        constexpr uint8_t CS1 = 9;
        constexpr uint8_t SdCardCS = 22;
    }

    namespace UART
    {
        constexpr uint8_t Tx = 0;
        constexpr uint8_t Rx = 1;
    }

    namespace LED
    {
        constexpr uint8_t Dout0 = 20;
        constexpr uint8_t Dout1 = 27;
    }

    namespace DIGITALIO
    {
        constexpr uint8_t P0 = 2;
        constexpr uint8_t P1 = 3;
        constexpr uint8_t P2 = 16;
        constexpr uint8_t P3 = 17;
        constexpr uint8_t P4 = 18;
        constexpr uint8_t P5 = 19;

        static const std::unordered_map<uint8_t, uint8_t> digitalIOMap = {
            {0, P0},
            {1, P1},
            {2, P2},
            {3, P3},
            {4, P4},
            {5, P5}};
    }

    namespace ANALOGIO
    {
        constexpr uint8_t ADC0 = A0;
        constexpr uint8_t ADC1 = A1;
        constexpr uint8_t ADC2 = A2;

        static const std::unordered_map<uint8_t, uint8_t> analogIOMap = {
            {0, ADC0},
            {1, ADC1},
            {2, ADC2}
        };
    }

    namespace CONFIG
    {
        constexpr uint8_t CONFIG_SETUP = 8;
    }
}

constexpr uint8_t i2cReceiveBufSize = 8;
constexpr uint8_t eepromAddr = 0b01011000;

constexpr uint32_t uartBaudRate = 115200;

constexpr uint8_t patternCount = 5;
// constexpr uint8_t chaseLEDWidth = ceil((float)ledNum / 20);