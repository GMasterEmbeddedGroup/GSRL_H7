#pragma once

#include "spi.h"
#include <stdbool.h>
#include <stdint.h>

class Ws2812Spi
{
public:
    explicit Ws2812Spi(SPI_HandleTypeDef *hspi = &hspi6);

    bool init();
    bool setColor(uint8_t red, uint8_t green, uint8_t blue);
    bool off();

private:
    static uint8_t encodeTriBit(uint8_t wsBit);
    bool transmitFrame();

    SPI_HandleTypeDef *m_hspi = nullptr;
    uint8_t m_txFrame[33]     = {0};
};
