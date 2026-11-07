/**
 ******************************************************************************
 * @file           : dvc_rangefinder.cpp
 * @brief          : 测距传感器
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 GMaster
 * All rights reserved.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "dvc_rangefinder.hpp"
#include <cstring>

/* Typedef -------------------------------------------------------------------*/

/* Define --------------------------------------------------------------------*/

/* Macro ---------------------------------------------------------------------*/

/* Variables -----------------------------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/* User code -----------------------------------------------------------------*/

/******************************************************************************
 *                          RangefinderTFMiniPlus实现
 ******************************************************************************/

/**
 * @brief 构造函数，初始化成员变量
 */
TFMiniPlus::TFMiniPlus()
    : m_rxData{},
      m_rxDataLength(0),
      m_lastDecodeTimestamp(0),
      m_distance(0),
      m_strength(0),
      m_temperature(0.0f),
      m_isConnected(false),
      m_isDataValid(false) {}

/**
 * @brief 在UART接收中断中调用，解码接收到的数据
 * @param data   本段接收数据指针
 * @param length 本段接收数据的字节数
 * @param event UART接收事件
 */
void TFMiniPlus::receiveRxDataFromISR(const uint8_t *data, uint16_t length, UART_RxEvent_t event)
{
    if (event == UART_RX_EVENT_ERROR) {
        m_rxDataLength = 0;
        m_isConnected = m_isDataValid = false;
        m_distance = m_strength = 0;
        m_temperature = 0.0f;
        return;
    }
    while (length != 0) {
        uint16_t count = FRAME_LENGTH - m_rxDataLength;
        if (count > length) count = length;
        memcpy(m_rxData + m_rxDataLength, data, count);
        m_rxDataLength += count;
        data += count;
        length -= count;
        if (m_rxDataLength != FRAME_LENGTH) continue;
        if (decodeRxData()) {
            m_rxDataLength = 0;
        } else {
            memmove(m_rxData, m_rxData + 1, FRAME_LENGTH - 1);
            m_rxDataLength = FRAME_LENGTH - 1;
        }
    }
}

/**
 * @brief 解析原始数据帧，更新距离、信号强度及传感器温度
 * @return 帧校验是否通过, 通过后更新时间戳和数据
 */
bool TFMiniPlus::decodeRxData()
{
    const uint8_t *pFrame = m_rxData;
    if (pFrame[0] != FRAME_HEADER || pFrame[1] != FRAME_HEADER) return false;
    uint8_t checksum = 0;
    for (uint8_t i = 0; i < FRAME_LENGTH - 1; i++) checksum += pFrame[i];
    if (checksum != pFrame[FRAME_LENGTH - 1]) return false;

    // 解析数据
    m_distance       = (int16_t)((uint16_t)pFrame[2] | ((uint16_t)pFrame[3] << 8));
    m_strength       = (uint16_t)((uint16_t)pFrame[4] | ((uint16_t)pFrame[5] << 8));
    uint16_t tempRaw = (uint16_t)((uint16_t)pFrame[6] | ((uint16_t)pFrame[7] << 8));
    m_temperature    = static_cast<fp32>(tempRaw) / 8.0f - 256.0f;

    // 数据有效性判断: 信号强度 >= 100 且不等于 65535(过曝)
    m_isDataValid = (m_strength >= 100 && m_strength != 65535);

    // 更新连接状态和时间戳
    m_lastDecodeTimestamp = HAL_GetTick();
    m_isConnected         = true;
    return true;
}

/**
 * @brief 获取测距结果
 * @return int16_t 距离值(cm)，数据无效时返回0
 */
int16_t TFMiniPlus::getDistance()
{
    isConnected();
    return m_isDataValid ? m_distance : 0;
}

/**
 * @brief 获取信号强度
 * @return uint16_t 信号强度，范围0~65535
 */
uint16_t TFMiniPlus::getStrength()
{
    isConnected();
    return m_strength;
}

/**
 * @brief 获取传感器连接状态，超时未收到数据则认为断开连接
 * @return bool 连接状态
 */
bool TFMiniPlus::isConnected()
{
    if (HAL_GetTick() - m_lastDecodeTimestamp > RX_TIMEOUT_MS) {
        m_distance    = 0;
        m_strength    = 0;
        m_temperature = 0.0f;
        m_isConnected = false;
        m_isDataValid = false;
    }
    return m_isConnected;
}

/**
 * @brief 获取芯片内部温度
 * @return fp32 温度值(℃)
 */
fp32 TFMiniPlus::getTemperature()
{
    isConnected();
    return m_temperature;
}

/**
 * @brief 获取以米为单位的测距结果
 * @return fp32 距离值(m)，数据无效时返回0
 */
fp32 TFMiniPlus::getDistanceM()
{
    return static_cast<fp32>(getDistance()) / 100.0f;
}

/**
 * @brief 获取数据有效性，信号强度不足或过曝时数据不可信
 * @return bool 数据有效性，Strength >= 100 且 Strength != 65535 时为true
 */
bool TFMiniPlus::isDataValid()
{
    isConnected();
    return m_isDataValid;
}
