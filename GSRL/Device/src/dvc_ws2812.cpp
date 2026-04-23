#include "dvc_ws2812.hpp"

namespace {
constexpr uint32_t WS2812_SPI_TIMEOUT_MS = 10U;

constexpr uint8_t WS_BIT_0 = 0b100;
constexpr uint8_t WS_BIT_1 = 0b110;

constexpr uint16_t DATA_BYTES  = 9U;
constexpr uint16_t RESET_BYTES = 24U;

inline void writeStreamBit(uint8_t *buffer, uint16_t bitIndex, uint8_t bitValue)
{
    const uint16_t byteIndex = bitIndex / 8U;
    const uint8_t bitInByte = static_cast<uint8_t>(7U - (bitIndex % 8U));
    if (bitValue != 0U) {
        buffer[byteIndex] |= static_cast<uint8_t>(1U << bitInByte);
    }
}
} // namespace

Ws2812Spi::Ws2812Spi(SPI_HandleTypeDef *hspi) : m_hspi(hspi)
{
}

bool Ws2812Spi::init()
{
    if (m_hspi == nullptr) {
        return false;
    }

    return off();
}

bool Ws2812Spi::setColor(uint8_t red, uint8_t green, uint8_t blue)
{
    const uint8_t grb[3] = {green, red, blue};
    for (uint16_t i = 0U; i < DATA_BYTES; ++i) {
        m_txFrame[i] = 0U;
    }

    uint16_t outBitPos = 0U;
    for (uint8_t byteIndex = 0U; byteIndex < 3U; ++byteIndex) {
        for (int bit = 7; bit >= 0; --bit) {
            const uint8_t tri = encodeTriBit((grb[byteIndex] >> bit) & 0x01U);
            writeStreamBit(m_txFrame, outBitPos++, (tri >> 2) & 0x01U);
            writeStreamBit(m_txFrame, outBitPos++, (tri >> 1) & 0x01U);
            writeStreamBit(m_txFrame, outBitPos++, tri & 0x01U);
        }
    }

    return transmitFrame();
}

bool Ws2812Spi::off()
{
    return setColor(0U, 0U, 0U);
}

uint8_t Ws2812Spi::encodeTriBit(uint8_t wsBit)
{
    return (wsBit == 0U) ? WS_BIT_0 : WS_BIT_1;
}

bool Ws2812Spi::transmitFrame()
{
    if (m_hspi == nullptr) {
        return false;
    }

    for (uint16_t i = DATA_BYTES; i < (DATA_BYTES + RESET_BYTES); ++i) {
        m_txFrame[i] = 0U;
    }

    if (HAL_SPI_Transmit(m_hspi, m_txFrame, DATA_BYTES + RESET_BYTES, WS2812_SPI_TIMEOUT_MS) != HAL_OK) {
        return false;
    }

    for (uint16_t i = 0U; i < DATA_BYTES; ++i) {
        m_txFrame[i] = 0U;
    }

    return true;
}
