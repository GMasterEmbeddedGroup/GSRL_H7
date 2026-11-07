/**
 ******************************************************************************
 * @file           : dvc_remotecontrol.cpp
 * @brief          : 遥控器驱动
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 GMaster
 * All rights reserved.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "dvc_remotecontrol.hpp"
#include "alg_crc.hpp"
#include "cmsis_os.h"
#include <cstring>

/* Typedef -------------------------------------------------------------------*/

/* Define --------------------------------------------------------------------*/

/* Macro ---------------------------------------------------------------------*/

/* Variables -----------------------------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/* User code -----------------------------------------------------------------*/

/******************************************************************************
 *                          RemoteControl遥控器基类实现
 ******************************************************************************/

/**
 * @brief 构造函数，初始化成员变量，设置摇杆死区
 * @param stickDeadZone 遥控器摇杆死区, 范围:[0.0, 1.0), 默认0.0f
 * @note 该构造函数不可被外部调用，需由派生类构造函数调用
 */
RemoteControl::RemoteControl(fp32 stickDeadZone)
    : m_uartRxTimestamp(0),
      m_isConnected(false),
      m_isEventInitialized(false),
      m_stickDeadZone(stickDeadZone),
      m_rxWriteIndex(0),
      m_rxPendingIndex(1),
      m_rxReadIndex(2),
      m_hasRxData(false),
      m_hasNewRxData(false),
      m_rxInterrupted(false),
      m_rxLength(0),
      m_isRxSynced(false) {}

/**
 * @brief 将本段接收数据组装到写入槽, 完整帧校验通过后发布
 * @note DR16先等待IDLE建立边界; 有帧头的协议通过滑动窗口恢复同步
 */
void RemoteControl::assembleRxFramesFromISR(const uint8_t *data, uint16_t length, UART_RxEvent_t event,
                                           void *rxFrames, uint16_t frameSize, RxSyncMode syncMode)
{
    if (event == UART_RX_EVENT_ERROR) {
        resetRxStateFromISR();
        return;
    }
    const bool syncOnIdle = syncMode == RxSyncMode::IDLE;
    while (length != 0 && (!syncOnIdle || m_isRxSynced)) {
        uint8_t *buffer = static_cast<uint8_t *>(rxFrames) + m_rxWriteIndex * frameSize;
        uint16_t count = frameSize - m_rxLength;
        if (count > length) count = length;
        memcpy(buffer + m_rxLength, data, count);
        m_rxLength += count;
        data       += count;
        length     -= count;
        if (m_rxLength != frameSize) continue;

        if (isRxFrameValid(buffer)) {
            publishRxFrameFromISR();
        } else if (syncOnIdle) {
            resetRxStateFromISR();
            break;
        } else {
            // 保留可能的下一帧起点, 不把DMA分段或错误帧末尾当作协议边界
            memmove(buffer, buffer + 1, frameSize - 1);
            m_rxLength = frameSize - 1;
        }
    }
    if (syncOnIdle && event == UART_RX_EVENT_IDLE) {
        if (m_rxLength != 0) resetRxStateFromISR();
        m_isRxSynced = true;
    }
}

void RemoteControl::resetRxStateFromISR()
{
    m_rxLength      = 0;
    m_isRxSynced    = false;
    m_hasRxData     = false;
    m_hasNewRxData  = false;
    m_rxInterrupted = true;
}

void RemoteControl::publishRxFrameFromISR()
{
    uint32_t timestamp = HAL_GetTick();
    if (!m_hasRxData || timestamp - m_uartRxTimestamp > 100) m_rxInterrupted = true;
    uint8_t previous = m_rxPendingIndex;
    m_rxPendingIndex  = m_rxWriteIndex;
    m_rxWriteIndex    = previous;
    m_rxLength        = 0;
    m_uartRxTimestamp = timestamp;
    m_hasRxData       = true;
    m_hasNewRxData    = true;
}

/**
 * @brief 每轮获取最新完整帧, 临界区内只交换槽位和接收状态
 */
bool RemoteControl::acquireRxFrame()
{
    taskENTER_CRITICAL();
    m_isConnected = m_hasRxData && HAL_GetTick() - m_uartRxTimestamp <= 100;
    if (!m_isConnected || m_rxInterrupted) m_isEventInitialized = false;
    m_rxInterrupted = false;
    bool hasNewData = m_hasNewRxData;
    if (hasNewData) {
        uint8_t previous = m_rxReadIndex;
        m_rxReadIndex    = m_rxPendingIndex;
        m_rxPendingIndex = previous;
        m_hasNewRxData   = false;
    }
    taskEXIT_CRITICAL();
    return m_isConnected && hasNewData;
}

RemoteControl::SwitchEvent2Pos RemoteControl::judgeSwitchEvent(SwitchStatus2Pos currentStatus, SwitchStatus2Pos lastStatus)
{
    if (!m_isEventInitialized) return SwitchEvent2Pos::SWITCH_NO_CHANGE;
    // SwitchStatus2Pos枚举值:
    // SWITCH_UP     = 1
    // SWITCH_DOWN   = 2
    // SWITCH_ERROR  = -1
    // 使用枚举值差值判断跳变事件
    switch (static_cast<int8_t>(currentStatus) - static_cast<int8_t>(lastStatus)) {
        case 0:
            return SwitchEvent2Pos::SWITCH_NO_CHANGE;
        case -1:
            return SwitchEvent2Pos::SWITCH_TOGGLE_DOWN_UP;
        case 1:
            return SwitchEvent2Pos::SWITCH_TOGGLE_UP_DOWN;
        default:
            return SwitchEvent2Pos::SWITCH_EVENT_OUT_OF_RANGE_ERROR;
    }
}

/**
 * @brief 根据当前状态和上一次状态返回拨杆的跳变事件
 * @note 本函数为内部函数, 请勿在外部调用
 */
RemoteControl::SwitchEvent3Pos RemoteControl::judgeSwitchEvent(SwitchStatus3Pos currentStatus, SwitchStatus3Pos lastStatus)
{
    if (!m_isEventInitialized) return SwitchEvent3Pos::SWITCH_NO_CHANGE;
    // SwitchStatus3Pos枚举值:
    // SWITCH_UP     = 1
    // SWITCH_DOWN   = 2
    // SWITCH_MIDDLE = 3
    // SWITCH_ERROR  = -1
    if (currentStatus == SwitchStatus3Pos::SWITCH_ERROR || lastStatus == SwitchStatus3Pos::SWITCH_ERROR) {
        return SwitchEvent3Pos::SWITCH_EVENT_STATUS_ERROR;
    }
    // 三档拨杆的正常跳变必须经过中档
    if ((lastStatus == SwitchStatus3Pos::SWITCH_UP && currentStatus == SwitchStatus3Pos::SWITCH_DOWN) ||
        (lastStatus == SwitchStatus3Pos::SWITCH_DOWN && currentStatus == SwitchStatus3Pos::SWITCH_UP)) {
        return SwitchEvent3Pos::SWITCH_EVENT_OUT_OF_RANGE_ERROR;
    }
    // 使用枚举值差值判断跳变事件
    switch (static_cast<int8_t>(currentStatus) - static_cast<int8_t>(lastStatus)) {
        case 0:
            return SwitchEvent3Pos::SWITCH_NO_CHANGE;
        case -2:
            return SwitchEvent3Pos::SWITCH_TOGGLE_MIDDLE_UP;
        case -1:
            return SwitchEvent3Pos::SWITCH_TOGGLE_MIDDLE_DOWN;
        case 1:
            return SwitchEvent3Pos::SWITCH_TOGGLE_DOWN_MIDDLE;
        case 2:
            return SwitchEvent3Pos::SWITCH_TOGGLE_UP_MIDDLE;
        default:
            return SwitchEvent3Pos::SWITCH_EVENT_OUT_OF_RANGE_ERROR;
    }
}

/**
 * @brief 根据当前状态和上一次状态返回按键的跳变事件
 * @note 本函数为内部函数, 请勿在外部调用
 */
RemoteControl::KeyEvent RemoteControl::judgeKeyEvent(KeyStatus currentStatus, KeyStatus lastStatus)
{
    if (!m_isEventInitialized) return KeyEvent::KEY_NO_CHANGE;
    switch (static_cast<int8_t>(currentStatus) - static_cast<int8_t>(lastStatus)) {
        case 0:
            return KeyEvent::KEY_NO_CHANGE;
        case -1:
            return KeyEvent::KEY_TOGGLE_PRESS_RELEASE;
        case 1:
            return KeyEvent::KEY_TOGGLE_RELEASE_PRESS;
        default:
            return KeyEvent::KEY_NO_CHANGE;
    }
}

/**
 * @brief 应用摇杆死区
 * @param stickValue 处理前摇杆输入值，范围[-1.0, 1.0]
 * @return fp32 移除死区后的摇杆值，重映射到范围[-1.0, 1.0]
 */
fp32 RemoteControl::applyStickDeadZone(fp32 stickValue)
{
    if (fabs(stickValue) < m_stickDeadZone) {
        return 0.0f;
    }
    if (stickValue > 0.0f) {
        return (stickValue - m_stickDeadZone) / (1.0f - m_stickDeadZone);
    }
    return (stickValue + m_stickDeadZone) / (1.0f - m_stickDeadZone);
}

/******************************************************************************
 *                           DR16RemoteControl类实现
 ******************************************************************************/

/**
 * @brief 构造函数，初始化接收缓冲区、控制值和事件状态
 */
DR16RemoteControl::DR16RemoteControl(fp32 stickDeadZone)
    : RemoteControl(stickDeadZone),
      m_rxFrames{},
      m_rightStickX(0.0f),
      m_rightStickY(0.0f),
      m_leftStickX(0.0f),
      m_leftStickY(0.0f),
      m_scrollWheel(0.0f),
      m_rightSwitchStatus(SwitchStatus3Pos::SWITCH_ERROR),
      m_lastRightSwitchStatus(SwitchStatus3Pos::SWITCH_ERROR),
      m_rightSwitchEvent(SwitchEvent3Pos::SWITCH_EVENT_NO_UPDATE_ERROR),
      m_leftSwitchStatus(SwitchStatus3Pos::SWITCH_ERROR),
      m_lastLeftSwitchStatus(SwitchStatus3Pos::SWITCH_ERROR),
      m_leftSwitchEvent(SwitchEvent3Pos::SWITCH_EVENT_NO_UPDATE_ERROR),
      m_mouseXSpeed(0.0f),
      m_mouseYSpeed(0.0f),
      m_mouseWheelSpeed(0.0f),
      m_mouseLeftKeyStatus(KeyStatus::KEY_ERROR),
      m_lastMouseLeftKeyStatus(KeyStatus::KEY_ERROR),
      m_mouseLeftKeyEvent(KeyEvent::KEY_EVENT_NO_UPDATE_ERROR),
      m_mouseRightKeyStatus(KeyStatus::KEY_ERROR),
      m_lastMouseRightKeyStatus(KeyStatus::KEY_ERROR),
      m_mouseRightKeyEvent(KeyEvent::KEY_EVENT_NO_UPDATE_ERROR)
{
    // 初始化键盘按键状态数组
    for (uint8_t i = 0; i < static_cast<uint8_t>(KeyboardKeyIndex::KEY_TOTAL_NUMBER); i++) {
        m_keyboardKeyStatus[i]     = KeyStatus::KEY_ERROR;
        m_lastKeyboardKeyStatus[i] = KeyStatus::KEY_ERROR;
        m_keyboardKeyEvent[i]      = KeyEvent::KEY_EVENT_NO_UPDATE_ERROR;
    }
}

/**
 * @brief 在UART接收回调中组帧并发布最新完整帧, 不解包控制值
 * @param data 本段接收数据, ERROR通知时为nullptr
 * @param length 本段接收数据的字节数, IDLE通知允许为0
 * @param event UART接收事件
 */
void DR16RemoteControl::receiveRxDataFromISR(const uint8_t *data, uint16_t length, UART_RxEvent_t event)
{
    assembleRxFramesFromISR(data, length, event, m_rxFrames, sizeof(DR16OriginalRxData), RxSyncMode::IDLE);
}

bool DR16RemoteControl::isRxFrameValid(const uint8_t *data) const
{
    const auto &rxData = *reinterpret_cast<const DR16OriginalRxData *>(data);
    return rxData.Channel_0 >= 364 && rxData.Channel_0 <= 1684 &&
           rxData.Channel_1 >= 364 && rxData.Channel_1 <= 1684 &&
           rxData.Channel_2 >= 364 && rxData.Channel_2 <= 1684 &&
           rxData.Channel_3 >= 364 && rxData.Channel_3 <= 1684 &&
           rxData.Channel_4 >= 364 && rxData.Channel_4 <= 1684 &&
           rxData.Switch_1 >= 1 && rxData.Switch_1 <= 3 &&
           rxData.Switch_2 >= 1 && rxData.Switch_2 <= 3 &&
           rxData.Mouse_Left_Key <= 1 && rxData.Mouse_Right_Key <= 1;
}

/**
 * @brief 在控制周期开头调用一次, 获取新帧并更新控制值和本轮事件
 * @note 无新帧时仅检查连接状态并清除上轮事件, getter不触发解包
 */
void DR16RemoteControl::updateRxData()
{
    clearEvent();
    if (!acquireRxFrame()) return;

    const DR16OriginalRxData &rxData = m_rxFrames[m_rxReadIndex];
    decodeControlData(rxData);
    updateEvent(rxData);
}

/**
 * @brief 解包当前读取帧的摇杆、拨轮和鼠标速度, 缓存归一化控制值
 */
void DR16RemoteControl::decodeControlData(const DR16OriginalRxData &rxData)
{
    m_rightStickX     = (fp32)(rxData.Channel_0 - 1024) / 660.0f;
    m_rightStickY     = (fp32)(rxData.Channel_1 - 1024) / 660.0f;
    m_leftStickX      = (fp32)(rxData.Channel_2 - 1024) / 660.0f;
    m_leftStickY      = (fp32)(rxData.Channel_3 - 1024) / 660.0f;
    m_scrollWheel     = (fp32)(rxData.Channel_4 - 1024) / 660.0f;
    m_mouseXSpeed     = (fp32)(rxData.Mouse_X / 32768.0f);
    m_mouseYSpeed     = (fp32)(rxData.Mouse_Y / 32768.0f);
    m_mouseWheelSpeed = (fp32)(rxData.Mouse_Z / 32768.0f);
}

void DR16RemoteControl::clearEvent()
{
    m_rightSwitchEvent = m_leftSwitchEvent = SwitchEvent3Pos::SWITCH_NO_CHANGE;
    m_mouseLeftKeyEvent = m_mouseRightKeyEvent = KeyEvent::KEY_NO_CHANGE;
    for (auto &event : m_keyboardKeyEvent) event = KeyEvent::KEY_NO_CHANGE;
}

/**
 * @brief 更新所有按键和拨杆的跳变事件并缓存
 * @note 仅由updateRxData在取得新的有效帧后调用
 */
void DR16RemoteControl::updateEvent(const DR16OriginalRxData &rxData)
{
    m_lastRightSwitchStatus   = m_rightSwitchStatus;
    m_rightSwitchStatus       = (SwitchStatus3Pos)rxData.Switch_2;
    m_rightSwitchEvent        = judgeSwitchEvent(m_rightSwitchStatus, m_lastRightSwitchStatus);
    m_lastLeftSwitchStatus    = m_leftSwitchStatus;
    m_leftSwitchStatus        = (SwitchStatus3Pos)rxData.Switch_1;
    m_leftSwitchEvent         = judgeSwitchEvent(m_leftSwitchStatus, m_lastLeftSwitchStatus);
    m_lastMouseLeftKeyStatus  = m_mouseLeftKeyStatus;
    m_mouseLeftKeyStatus      = (KeyStatus)rxData.Mouse_Left_Key;
    m_mouseLeftKeyEvent       = judgeKeyEvent(m_mouseLeftKeyStatus, m_lastMouseLeftKeyStatus);
    m_lastMouseRightKeyStatus = m_mouseRightKeyStatus;
    m_mouseRightKeyStatus     = (KeyStatus)rxData.Mouse_Right_Key;
    m_mouseRightKeyEvent      = judgeKeyEvent(m_mouseRightKeyStatus, m_lastMouseRightKeyStatus);
    for (uint8_t keyIndex = 0; keyIndex < static_cast<uint8_t>(KeyboardKeyIndex::KEY_TOTAL_NUMBER); keyIndex++) {
        m_lastKeyboardKeyStatus[keyIndex] = m_keyboardKeyStatus[keyIndex];
        m_keyboardKeyStatus[keyIndex]     = (KeyStatus)(rxData.Keyboard_Key >> keyIndex & 0x01);
        m_keyboardKeyEvent[keyIndex]      = judgeKeyEvent(m_keyboardKeyStatus[keyIndex], m_lastKeyboardKeyStatus[keyIndex]);
    }
    m_isEventInitialized = true;
}

/******************************************************************************
 *                          ET08ARemoteControl类实现
 ******************************************************************************/

/**
 * @brief 双参数构造函数，自定义通道配置
 * @param config 遥控器通道配置结构体
 * @param stickDeadZone 遥控器摇杆死区, 范围:[0.0, 1.0), 默认0.0f
 */
ET08ARemoteControl::ET08ARemoteControl(Config config, fp32 stickDeadZone)
    : RemoteControl(stickDeadZone),
      m_rxFrames{},
      m_protocolData{},
      m_config(config),
      m_switchSA(SwitchStatus2Pos::SWITCH_ERROR),
      m_switchSD(SwitchStatus2Pos::SWITCH_ERROR),
      m_lastSwitchSA(SwitchStatus2Pos::SWITCH_ERROR),
      m_lastSwitchSD(SwitchStatus2Pos::SWITCH_ERROR),
      m_eventSA(SwitchEvent2Pos::SWITCH_EVENT_NO_UPDATE_ERROR),
      m_eventSD(SwitchEvent2Pos::SWITCH_EVENT_NO_UPDATE_ERROR),
      m_switchSB(SwitchStatus3Pos::SWITCH_ERROR),
      m_switchSC(SwitchStatus3Pos::SWITCH_ERROR),
      m_lastSwitchSB(SwitchStatus3Pos::SWITCH_ERROR),
      m_lastSwitchSC(SwitchStatus3Pos::SWITCH_ERROR),
      m_eventSB(SwitchEvent3Pos::SWITCH_EVENT_NO_UPDATE_ERROR),
      m_eventSC(SwitchEvent3Pos::SWITCH_EVENT_NO_UPDATE_ERROR),
      m_knobLD(0.0f),
      m_knobRD(0.0f),
      m_trimmerT1(0.0f),
      m_trimmerT2(0.0f),
      m_trimmerT3(0.0f),
      m_trimmerT4(0.0f),
      m_rightStickX(0.0f),
      m_rightStickY(0.0f),
      m_leftStickX(0.0f),
      m_leftStickY(0.0f),
      m_rightSwitchStatus(SwitchStatus3Pos::SWITCH_ERROR),
      m_rightSwitchEvent(SwitchEvent3Pos::SWITCH_EVENT_NO_UPDATE_ERROR),
      m_leftSwitchStatus(SwitchStatus3Pos::SWITCH_ERROR),
      m_leftSwitchEvent(SwitchEvent3Pos::SWITCH_EVENT_NO_UPDATE_ERROR) {}

/**
 * @brief 单参数构造函数，使用默认通道配置
 * @param stickDeadZone 遥控器摇杆死区, 范围:[0.0, 1.0), 默认0.0f
 */
ET08ARemoteControl::ET08ARemoteControl(fp32 stickDeadZone)
    : ET08ARemoteControl(Config{}, stickDeadZone) {}

/**
 * @brief 在UART接收回调中组帧并发布最新完整帧, 不解包控制值
 * @param data 本段接收数据, ERROR通知时为nullptr
 * @param length 本段接收数据的字节数, IDLE通知允许为0
 * @param event UART接收事件
 */
void ET08ARemoteControl::receiveRxDataFromISR(const uint8_t *data, uint16_t length, UART_RxEvent_t event)
{
    assembleRxFramesFromISR(data, length, event, m_rxFrames, sizeof(ET08ARawPacket), RxSyncMode::FRAME_HEADER);
}

bool ET08ARemoteControl::isRxFrameValid(const uint8_t *data) const
{
    return data[0] == 0x0F && data[sizeof(ET08ARawPacket) - 1] == 0x00;
}

/**
 * @brief 在控制周期开头调用一次, 获取新帧并更新控制值和本轮事件
 * @note 丢帧/失控保护标志使数据不可用, 控制值保持上次缓存
 */
void ET08ARemoteControl::updateRxData()
{
    clearEvent();
    const bool hasNewData = acquireRxFrame();
    const ET08ARawPacket &rxData = m_rxFrames[m_rxReadIndex];
    updateConnectionStatus(rxData);
    if (!m_isConnected || !hasNewData) return;

    decodeControlData(rxData);
    updateEvent();
}

/**
 * @brief 根据帧中的丢帧和失控保护标志更新连接状态及事件基准
 */
void ET08ARemoteControl::updateConnectionStatus(const ET08ARawPacket &rxData)
{
    if ((rxData.flags & 0x0C) != 0) {
        m_isConnected        = false;
        m_isEventInitialized = false;
    }
}

/**
 * @brief 解包当前读取帧的通道数据, 缓存归一化摇杆、旋钮和微调值
 */
void ET08ARemoteControl::decodeControlData(const ET08ARawPacket &rxData)
{
    parseET08AProtocol(rxData, m_protocolData);

    // ET08A遥控器通道值范围是[353, 1694], 中点1024
    // 为保证数值在[-1.0, 1.0]范围内且保证线性性, 取较大的下限偏移量671, 最终归一化数值范围是[-1.0, 0.9985]
    m_rightStickX = (m_protocolData.rightStickX - 1024) / 671.0f;
    m_rightStickY = (m_protocolData.rightStickY - 1024) / 671.0f;
    m_leftStickX  = (m_protocolData.leftStickX - 1024) / 671.0f;
    m_leftStickY  = (m_protocolData.leftStickY - 1024) / 671.0f;
    m_knobLD      = (m_protocolData.knobLD - 1024) / 671.0f;
    m_knobRD      = (m_protocolData.knobRD - 1024) / 671.0f;
    m_trimmerT1   = (m_protocolData.trimmerT1 - 1024) / 671.0f;
    m_trimmerT2   = (m_protocolData.trimmerT2 - 1024) / 671.0f;
    m_trimmerT3   = (m_protocolData.trimmerT3 - 1024) / 671.0f;
    m_trimmerT4   = (m_protocolData.trimmerT4 - 1024) / 671.0f;
}

void ET08ARemoteControl::clearEvent()
{
    m_eventSA = m_eventSD = SwitchEvent2Pos::SWITCH_NO_CHANGE;
    m_eventSB = m_eventSC = SwitchEvent3Pos::SWITCH_NO_CHANGE;
}

/**
 * @brief 更新所有按键和拨杆的跳变事件并缓存
 * @note 仅由updateRxData在取得新的有效帧后调用
 */
void ET08ARemoteControl::updateEvent()
{
    m_lastSwitchSA = m_switchSA;
    m_lastSwitchSB = m_switchSB;
    if (m_config.switchSASB != ET08AChannelIndex::CH_NONE) { // SA和SB复用通道, SB微调
        if (m_protocolData.switchSASB < 1024) {
            m_switchSA = SwitchStatus2Pos::SWITCH_UP;
            switch ((m_protocolData.switchSASB - 151) / 201) {
                case 0:
                    m_switchSB = SwitchStatus3Pos::SWITCH_UP;
                    break;
                case 1:
                    m_switchSB = SwitchStatus3Pos::SWITCH_MIDDLE;
                    break;
                case 2:
                    m_switchSB = SwitchStatus3Pos::SWITCH_DOWN;
                    break;
                default:
                    m_switchSB = SwitchStatus3Pos::SWITCH_ERROR;
                    break;
            }
        } else {
            m_switchSA = SwitchStatus2Pos::SWITCH_DOWN;
            switch ((m_protocolData.switchSASB - 1493) / 201) {
                case 0:
                    m_switchSB = SwitchStatus3Pos::SWITCH_UP;
                    break;
                case 1:
                    m_switchSB = SwitchStatus3Pos::SWITCH_MIDDLE;
                    break;
                case 2:
                    m_switchSB = SwitchStatus3Pos::SWITCH_DOWN;
                    break;
                default:
                    m_switchSB = SwitchStatus3Pos::SWITCH_ERROR;
                    break;
            }
        }
    } else { // SA和SB使用独立通道
        if (m_protocolData.switchSA < 1024) {
            m_switchSA = SwitchStatus2Pos::SWITCH_UP;
        } else {
            m_switchSA = SwitchStatus2Pos::SWITCH_DOWN;
        }
        if (m_protocolData.switchSB < 1024) {
            m_switchSB = SwitchStatus3Pos::SWITCH_UP;
        } else if (m_protocolData.switchSB > 1024) {
            m_switchSB = SwitchStatus3Pos::SWITCH_DOWN;
        } else {
            m_switchSB = SwitchStatus3Pos::SWITCH_MIDDLE;
        }
    }
    m_eventSA = judgeSwitchEvent(m_switchSA, m_lastSwitchSA);
    m_eventSB = judgeSwitchEvent(m_switchSB, m_lastSwitchSB);

    m_lastSwitchSC = m_switchSC;
    m_lastSwitchSD = m_switchSD;
    if (m_config.switchSCSD != ET08AChannelIndex::CH_NONE) { // SC和SD复用通道, SC微调
        if (m_protocolData.switchSCSD < 1024) {
            m_switchSD = SwitchStatus2Pos::SWITCH_UP;
            switch ((m_protocolData.switchSCSD - 151) / 201) {
                case 0:
                    m_switchSC = SwitchStatus3Pos::SWITCH_UP;
                    break;
                case 1:
                    m_switchSC = SwitchStatus3Pos::SWITCH_MIDDLE;
                    break;
                case 2:
                    m_switchSC = SwitchStatus3Pos::SWITCH_DOWN;
                    break;
                default:
                    m_switchSC = SwitchStatus3Pos::SWITCH_ERROR;
                    break;
            }
        } else {
            m_switchSD = SwitchStatus2Pos::SWITCH_DOWN;
            switch ((m_protocolData.switchSCSD - 1493) / 201) {
                case 0:
                    m_switchSC = SwitchStatus3Pos::SWITCH_UP;
                    break;
                case 1:
                    m_switchSC = SwitchStatus3Pos::SWITCH_MIDDLE;
                    break;
                case 2:
                    m_switchSC = SwitchStatus3Pos::SWITCH_DOWN;
                    break;
                default:
                    m_switchSC = SwitchStatus3Pos::SWITCH_ERROR;
                    break;
            }
        }
    } else { // SC和SD使用独立通道
        if (m_protocolData.switchSC < 1024) {
            m_switchSC = SwitchStatus3Pos::SWITCH_UP;
        } else if (m_protocolData.switchSC > 1024) {
            m_switchSC = SwitchStatus3Pos::SWITCH_DOWN;
        } else {
            m_switchSC = SwitchStatus3Pos::SWITCH_MIDDLE;
        }
        if (m_protocolData.switchSD < 1024) {
            m_switchSD = SwitchStatus2Pos::SWITCH_UP;
        } else {
            m_switchSD = SwitchStatus2Pos::SWITCH_DOWN;
        }
    }
    m_eventSC = judgeSwitchEvent(m_switchSC, m_lastSwitchSC);
    m_eventSD = judgeSwitchEvent(m_switchSD, m_lastSwitchSD);
    m_isEventInitialized = true;
}

/**
 * @brief 解包已校验的ET08A完整帧
 * @param packet 任务侧接收快照
 * @param out 按配置映射后的通道数据
 */
void ET08ARemoteControl::parseET08AProtocol(const ET08ARawPacket &packet, ET08AProtocolData &out) const
{
    const uint8_t *buffer = packet.data;
    uint16_t temp_ch[16];

    temp_ch[0]  = ((buffer[0] | buffer[1] << 8) & 0x07FF);
    temp_ch[1]  = ((buffer[1] >> 3 | buffer[2] << 5) & 0x07FF);
    temp_ch[2]  = ((buffer[2] >> 6 | buffer[3] << 2 | buffer[4] << 10) & 0x07FF);
    temp_ch[3]  = ((buffer[4] >> 1 | buffer[5] << 7) & 0x07FF);
    temp_ch[4]  = ((buffer[5] >> 4 | buffer[6] << 4) & 0x07FF);
    temp_ch[5]  = ((buffer[6] >> 7 | buffer[7] << 1 | buffer[8] << 9) & 0x07FF);
    temp_ch[6]  = ((buffer[8] >> 2 | buffer[9] << 6) & 0x07FF);
    temp_ch[7]  = ((buffer[9] >> 5 | buffer[10] << 3) & 0x07FF);
    temp_ch[8]  = ((buffer[11] | buffer[12] << 8) & 0x07FF);
    temp_ch[9]  = ((buffer[12] >> 3 | buffer[13] << 5) & 0x07FF);
    temp_ch[10] = ((buffer[13] >> 6 | buffer[14] << 2 | buffer[15] << 10) & 0x07FF);
    temp_ch[11] = ((buffer[15] >> 1 | buffer[16] << 7) & 0x07FF);
    temp_ch[12] = ((buffer[16] >> 4 | buffer[17] << 4) & 0x07FF);
    temp_ch[13] = ((buffer[17] >> 7 | buffer[18] << 1 | buffer[19] << 9) & 0x07FF);
    temp_ch[14] = ((buffer[19] >> 2 | buffer[20] << 6) & 0x07FF);
    temp_ch[15] = ((buffer[20] >> 5 | buffer[21] << 3) & 0x07FF);

    // 根据配置映射通道数据
    auto getChannelValue = [&temp_ch](ET08AChannelIndex index) -> uint16_t {
        return (index == ET08AChannelIndex::CH_NONE) ? 1024 : temp_ch[static_cast<uint8_t>(index)];
    };

    out.rightStickX = getChannelValue(m_config.rightStickJ1X);
    out.rightStickY = getChannelValue(m_config.rightStickJ2Y);
    out.leftStickX  = getChannelValue(m_config.leftStickJ4X);
    out.leftStickY  = getChannelValue(m_config.leftStickJ3Y);

    out.switchSA = getChannelValue(m_config.switchSA);
    out.switchSB = getChannelValue(m_config.switchSB);
    out.switchSC = getChannelValue(m_config.switchSC);
    out.switchSD = getChannelValue(m_config.switchSD);

    out.switchSASB = getChannelValue(m_config.switchSASB);
    out.switchSCSD = getChannelValue(m_config.switchSCSD);

    out.knobLD = getChannelValue(m_config.knobLD);
    out.knobRD = getChannelValue(m_config.knobRD);

    out.trimmerT1 = getChannelValue(m_config.trimmerT1);
    out.trimmerT2 = getChannelValue(m_config.trimmerT2);
    out.trimmerT3 = getChannelValue(m_config.trimmerT3);
    out.trimmerT4 = getChannelValue(m_config.trimmerT4);
}

/******************************************************************************
 *                         VT13RemoteControl类实现
 ******************************************************************************/

/**
 * @brief 构造函数，初始化所有成员变量
 */
VT13RemoteControl::VT13RemoteControl(fp32 stickDeadZone)
    : RemoteControl(stickDeadZone),
      m_rxFrames{},
      m_rightStickX(0.0f),
      m_rightStickY(0.0f),
      m_leftStickX(0.0f),
      m_leftStickY(0.0f),
      m_scrollWheel(0.0f),
      m_modeSwitchStatus(SwitchStatus3Pos::SWITCH_ERROR),
      m_lastModeSwitchStatus(SwitchStatus3Pos::SWITCH_ERROR),
      m_modeSwitchEvent(SwitchEvent3Pos::SWITCH_EVENT_NO_UPDATE_ERROR),
      m_pauseKeyStatus(KeyStatus::KEY_ERROR),
      m_lastPauseKeyStatus(KeyStatus::KEY_ERROR),
      m_pauseKeyEvent(KeyEvent::KEY_EVENT_NO_UPDATE_ERROR),
      m_fn1KeyStatus(KeyStatus::KEY_ERROR),
      m_lastFn1KeyStatus(KeyStatus::KEY_ERROR),
      m_fn1KeyEvent(KeyEvent::KEY_EVENT_NO_UPDATE_ERROR),
      m_fn2KeyStatus(KeyStatus::KEY_ERROR),
      m_lastFn2KeyStatus(KeyStatus::KEY_ERROR),
      m_fn2KeyEvent(KeyEvent::KEY_EVENT_NO_UPDATE_ERROR),
      m_triggerKeyStatus(KeyStatus::KEY_ERROR),
      m_lastTriggerKeyStatus(KeyStatus::KEY_ERROR),
      m_triggerKeyEvent(KeyEvent::KEY_EVENT_NO_UPDATE_ERROR),
      m_mouseXSpeed(0.0f),
      m_mouseYSpeed(0.0f),
      m_mouseWheelSpeed(0.0f),
      m_mouseLeftKeyStatus(KeyStatus::KEY_ERROR),
      m_lastMouseLeftKeyStatus(KeyStatus::KEY_ERROR),
      m_mouseLeftKeyEvent(KeyEvent::KEY_EVENT_NO_UPDATE_ERROR),
      m_mouseRightKeyStatus(KeyStatus::KEY_ERROR),
      m_lastMouseRightKeyStatus(KeyStatus::KEY_ERROR),
      m_mouseRightKeyEvent(KeyEvent::KEY_EVENT_NO_UPDATE_ERROR),
      m_mouseMiddleKeyStatus(KeyStatus::KEY_ERROR),
      m_lastMouseMiddleKeyStatus(KeyStatus::KEY_ERROR),
      m_mouseMiddleKeyEvent(KeyEvent::KEY_EVENT_NO_UPDATE_ERROR)
{
    for (uint8_t i = 0; i < static_cast<uint8_t>(KeyboardKeyIndex::KEY_TOTAL_NUMBER); i++) {
        m_keyboardKeyStatus[i]     = KeyStatus::KEY_ERROR;
        m_lastKeyboardKeyStatus[i] = KeyStatus::KEY_ERROR;
        m_keyboardKeyEvent[i]      = KeyEvent::KEY_EVENT_NO_UPDATE_ERROR;
    }
}

/**
 * @brief 在UART接收回调中组帧并发布最新完整帧, 不解包控制值
 * @param data 本段接收数据, ERROR通知时为nullptr
 * @param length 本段接收数据的字节数, IDLE通知允许为0
 * @param event UART接收事件
 */
void VT13RemoteControl::receiveRxDataFromISR(const uint8_t *data, uint16_t length, UART_RxEvent_t event)
{
    assembleRxFramesFromISR(data, length, event, m_rxFrames, sizeof(VT13OriginalRxData), RxSyncMode::FRAME_HEADER);
}

bool VT13RemoteControl::isRxFrameValid(const uint8_t *data) const
{
    return data[0] == SOF_1 && data[1] == SOF_2 && CRCCalculator::verifyCRC16(data, FRAME_SIZE);
}

/**
 * @brief 在控制周期开头调用一次, 获取新帧并更新控制值和本轮事件
 * @note 无新帧时仅检查连接状态并清除上轮事件, getter不触发解包
 */
void VT13RemoteControl::updateRxData()
{
    clearEvent();
    if (!acquireRxFrame()) return;

    const VT13OriginalRxData &rxData = m_rxFrames[m_rxReadIndex];
    decodeControlData(rxData);
    updateEvent(rxData);
}

/**
 * @brief 解包当前读取帧的摇杆、拨轮和鼠标速度, 缓存归一化控制值
 */
void VT13RemoteControl::decodeControlData(const VT13OriginalRxData &rxData)
{
    m_rightStickX     = (fp32)(rxData.Channel_0 - 1024) / 660.0f;
    m_rightStickY     = (fp32)(rxData.Channel_1 - 1024) / 660.0f;
    m_leftStickY      = (fp32)(rxData.Channel_2 - 1024) / 660.0f;
    m_leftStickX      = (fp32)(rxData.Channel_3 - 1024) / 660.0f;
    m_scrollWheel     = (fp32)(rxData.Wheel - 1024) / 660.0f;
    m_mouseXSpeed     = (fp32)(rxData.Mouse_X / 32768.0f);
    m_mouseYSpeed     = (fp32)(rxData.Mouse_Y / 32768.0f);
    m_mouseWheelSpeed = (fp32)(rxData.Mouse_Z / 32768.0f);
}

void VT13RemoteControl::clearEvent()
{
    m_modeSwitchEvent = SwitchEvent3Pos::SWITCH_NO_CHANGE;
    m_pauseKeyEvent = m_fn1KeyEvent = m_fn2KeyEvent = m_triggerKeyEvent = KeyEvent::KEY_NO_CHANGE;
    m_mouseLeftKeyEvent = m_mouseRightKeyEvent = m_mouseMiddleKeyEvent = KeyEvent::KEY_NO_CHANGE;
    for (auto &event : m_keyboardKeyEvent) event = KeyEvent::KEY_NO_CHANGE;
}

/**
 * @brief 更新所有按键和拨杆的跳变事件并缓存
 * @note 仅由updateRxData在取得新的有效帧后调用
 */
void VT13RemoteControl::updateEvent(const VT13OriginalRxData &rxData)
{
    // 档位开关
    m_lastModeSwitchStatus = m_modeSwitchStatus;
    m_modeSwitchStatus     = mapModeSwitch(rxData.Mode_Switch);
    m_modeSwitchEvent      = judgeSwitchEvent(m_modeSwitchStatus, m_lastModeSwitchStatus);
    // 暂停键
    m_lastPauseKeyStatus = m_pauseKeyStatus;
    m_pauseKeyStatus     = (KeyStatus)rxData.Pause;
    m_pauseKeyEvent      = judgeKeyEvent(m_pauseKeyStatus, m_lastPauseKeyStatus);
    // 自定义按键(左)
    m_lastFn1KeyStatus = m_fn1KeyStatus;
    m_fn1KeyStatus     = (KeyStatus)rxData.Fn_1;
    m_fn1KeyEvent      = judgeKeyEvent(m_fn1KeyStatus, m_lastFn1KeyStatus);
    // 自定义按键(右)
    m_lastFn2KeyStatus = m_fn2KeyStatus;
    m_fn2KeyStatus     = (KeyStatus)rxData.Fn_2;
    m_fn2KeyEvent      = judgeKeyEvent(m_fn2KeyStatus, m_lastFn2KeyStatus);
    // 瞄准键
    m_lastTriggerKeyStatus = m_triggerKeyStatus;
    m_triggerKeyStatus     = (KeyStatus)rxData.Trigger;
    m_triggerKeyEvent      = judgeKeyEvent(m_triggerKeyStatus, m_lastTriggerKeyStatus);
    // 鼠标按键
    m_lastMouseLeftKeyStatus   = m_mouseLeftKeyStatus;
    m_mouseLeftKeyStatus       = (KeyStatus)rxData.Mouse_Left;
    m_mouseLeftKeyEvent        = judgeKeyEvent(m_mouseLeftKeyStatus, m_lastMouseLeftKeyStatus);
    m_lastMouseRightKeyStatus  = m_mouseRightKeyStatus;
    m_mouseRightKeyStatus      = (KeyStatus)rxData.Mouse_Right;
    m_mouseRightKeyEvent       = judgeKeyEvent(m_mouseRightKeyStatus, m_lastMouseRightKeyStatus);
    m_lastMouseMiddleKeyStatus = m_mouseMiddleKeyStatus;
    m_mouseMiddleKeyStatus     = (KeyStatus)rxData.Mouse_Middle;
    m_mouseMiddleKeyEvent      = judgeKeyEvent(m_mouseMiddleKeyStatus, m_lastMouseMiddleKeyStatus);
    // 键盘按键
    for (uint8_t keyIndex = 0; keyIndex < static_cast<uint8_t>(KeyboardKeyIndex::KEY_TOTAL_NUMBER); keyIndex++) {
        m_lastKeyboardKeyStatus[keyIndex] = m_keyboardKeyStatus[keyIndex];
        m_keyboardKeyStatus[keyIndex]     = (KeyStatus)(rxData.Keyboard_Key >> keyIndex & 0x01);
        m_keyboardKeyEvent[keyIndex]      = judgeKeyEvent(m_keyboardKeyStatus[keyIndex], m_lastKeyboardKeyStatus[keyIndex]);
    }
    m_isEventInitialized = true;
}

/**
 * @brief 档位原始值到SwitchStatus3Pos的映射
 * @param raw 档位原始值 C:0 N:1 S:2
 * @return SwitchStatus3Pos 对应的拨杆状态
 */
RemoteControl::SwitchStatus3Pos VT13RemoteControl::mapModeSwitch(uint8_t raw)
{
    switch (raw) {
        case 0:
            return SwitchStatus3Pos::SWITCH_UP; // C档
        case 1:
            return SwitchStatus3Pos::SWITCH_MIDDLE; // N档
        case 2:
            return SwitchStatus3Pos::SWITCH_DOWN; // S档
        default:
            return SwitchStatus3Pos::SWITCH_ERROR;
    }
}
