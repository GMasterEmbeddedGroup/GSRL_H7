/**
 ******************************************************************************
 * @file           : dvc_remotecontrol.hpp
 * @brief          : header file for dvc_remotecontrol.cpp
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 GMaster
 * All rights reserved.
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#pragma once

/* Includes ------------------------------------------------------------------*/
#include "gsrl_common.h"
#include "drv_uart.h"
#include <cstdint>
#include <math.h>

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 遥控器基类
 * @note 该类不可实例化
 * @note 同一实例由一路不嵌套的UART/DMA接收回调写入, 由一个RTOS任务每轮调用一次updateRxData
 * @note UART/DMA抢占优先级须相同且数值不小于configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY
 * @note F4库使用decodeRxData解包并单独调用updateEvent, 数值getter/isConnected会触发解包
 * @note 本库由updateRxData统一更新控制值和事件, 不单独调用updateEvent; getter/isConnected只读本轮缓存
 */
class RemoteControl
{
public:
    // 遥控器双档拨杆状态
    enum class SwitchStatus2Pos : int8_t {
        SWITCH_UP = 1,
        SWITCH_DOWN,
        SWITCH_ERROR = -1 // 尚无有效拨杆状态
    };

    // 遥控器双档拨杆跳变事件
    enum class SwitchEvent2Pos : int8_t {
        SWITCH_NO_CHANGE,
        SWITCH_TOGGLE_UP_DOWN,
        SWITCH_TOGGLE_DOWN_UP,
        SWITCH_EVENT_NO_UPDATE_ERROR    = -1, // 检查updateRxData()是否已调用
        SWITCH_EVENT_STATUS_ERROR       = -2, // 检查当前或上一次拨杆状态是否为SWITCH_ERROR
        SWITCH_EVENT_OUT_OF_RANGE_ERROR = -3  // 检查当前或上一次拨杆状态是否超出枚举值范围(遥控数据解包错误)
    };

    // 遥控器三档拨杆状态
    enum class SwitchStatus3Pos : int8_t {
        SWITCH_UP = 1,
        SWITCH_DOWN,
        SWITCH_MIDDLE,
        SWITCH_ERROR = -1 // 尚无有效拨杆状态
    };

    // 遥控器三档拨杆跳变事件
    enum class SwitchEvent3Pos : int8_t {
        SWITCH_NO_CHANGE,
        SWITCH_TOGGLE_UP_MIDDLE,
        SWITCH_TOGGLE_MIDDLE_UP,
        SWITCH_TOGGLE_DOWN_MIDDLE,
        SWITCH_TOGGLE_MIDDLE_DOWN,
        SWITCH_EVENT_NO_UPDATE_ERROR    = -1, // 检查updateRxData()是否已调用
        SWITCH_EVENT_STATUS_ERROR       = -2, // 检查当前或上一次拨杆状态是否为SWITCH_ERROR
        SWITCH_EVENT_OUT_OF_RANGE_ERROR = -3  // 档位越界或上、下档之间未观察到中档的直接跳变
    };

    // 遥控器按键状态
    enum class KeyStatus : int8_t {
        KEY_RELEASE = 0,
        KEY_PRESS,
        KEY_ERROR = -1 // 尚无有效按键状态
    };

    // 遥控器按键跳变事件
    enum class KeyEvent : int8_t {
        KEY_NO_CHANGE,
        KEY_TOGGLE_PRESS_RELEASE,
        KEY_TOGGLE_RELEASE_PRESS,
        KEY_EVENT_NO_UPDATE_ERROR = -1 // 检查updateRxData()是否已调用
    };

protected:
    enum class RxSyncMode : uint8_t {
        FRAME_HEADER,
        IDLE
    };

    // 遥控器连接状态检测
    volatile uint32_t m_uartRxTimestamp; // 完整帧接收时间, 使用HAL_GetTick()
    bool m_isConnected;
    bool m_isEventInitialized; // 本轮任务是否已有可比较的有效状态
    fp32 m_stickDeadZone; // 遥控器摇杆死区

    // 三缓冲: m_rxFrames[3]分别用于写入、待取、读取, 接收和解包各用各的内存
    // 中断收完并校验一帧后, 交换写入区和待取区; 待取区只保留最新完整帧
    // 任务有新帧时交换待取区和读取区, 再解包; 无新帧时跳过解包
    // 交换的只是编号, 不复制整帧; 任务交换时用短临界区防止中断打断
    uint8_t m_rxWriteIndex; // 拼装正在接收的一帧
    volatile uint8_t m_rxPendingIndex; // 留住最新完整帧, 等待任务取走
    uint8_t m_rxReadIndex; // 任务解包使用, 中断不会覆写
    volatile bool m_hasRxData;
    volatile bool m_hasNewRxData;
    volatile bool m_rxInterrupted; // 两轮任务之间发生接收间断, 下轮重新建立事件基准
    uint16_t m_rxLength; // 写入槽中已组装的字节数
    bool m_isRxSynced;   // DR16通过IDLE建立帧边界

public:
    virtual ~RemoteControl() = default;

    // F4库接收入口只传data; 本接口须传入data、length和event
    // length为本段接收数据的字节数; 必须转发IDLE(包括零字节)和ERROR通知
    virtual void receiveRxDataFromISR(const uint8_t *data, uint16_t length, UART_RxEvent_t event) = 0;
    virtual void updateRxData() = 0;
    bool isConnected() const
    {
        return m_isConnected;
    }

protected:
    RemoteControl(fp32 stickDeadZone = 0.0f);
    void assembleRxFramesFromISR(const uint8_t *data, uint16_t length, UART_RxEvent_t event,
                                 void *rxFrames, uint16_t frameSize, RxSyncMode syncMode);
    virtual bool isRxFrameValid(const uint8_t *data) const = 0;
    void resetRxStateFromISR();
    void publishRxFrameFromISR();
    bool acquireRxFrame();
    SwitchEvent2Pos judgeSwitchEvent(SwitchStatus2Pos currentStatus, SwitchStatus2Pos lastStatus);
    SwitchEvent3Pos judgeSwitchEvent(SwitchStatus3Pos currentStatus, SwitchStatus3Pos lastStatus);
    KeyEvent judgeKeyEvent(KeyStatus currentStatus, KeyStatus lastStatus);
    fp32 applyStickDeadZone(fp32 stickValue);
};

/**
 * @brief 大疆DR16遥控器类，用于解码DR16遥控器接收数据
 * @note 使用前需确保receiveRxDataFromISR方法在对应UART接收中断服务函数中被调用
 * @note 与F4库不同: 启动或失步后先等待IDLE, 从下一帧开始接收; 检查18字节帧的通道和拨杆范围
 */
class DR16RemoteControl : public RemoteControl
{
public:
    // DR16遥控器原始数据结构体
    struct DR16OriginalRxData {
        uint64_t Channel_0 : 11;
        uint64_t Channel_1 : 11;
        uint64_t Channel_2 : 11;
        uint64_t Channel_3 : 11;
        uint64_t Switch_2 : 2;
        uint64_t Switch_1 : 2;
        int16_t Mouse_X;
        int16_t Mouse_Y;
        int16_t Mouse_Z;
        uint64_t Mouse_Left_Key : 8;
        uint64_t Mouse_Right_Key : 8;
        uint64_t Keyboard_Key : 16;
        uint64_t Channel_4 : 11;
    } __attribute__((packed));

    // DR16遥控器键盘按键对应索引
    enum class KeyboardKeyIndex : uint8_t {
        KEY_W = 0,
        KEY_S,
        KEY_A,
        KEY_D,
        KEY_SHIFT,
        KEY_CTRL,
        KEY_Q,
        KEY_E,
        KEY_R,
        KEY_F,
        KEY_G,
        KEY_Z,
        KEY_X,
        KEY_C,
        KEY_V,
        KEY_B,
        KEY_TOTAL_NUMBER // 键盘按键枚举值总数
    };

private:
    DR16OriginalRxData m_rxFrames[3];
    // DR16遥控器解码数据
    fp32 m_rightStickX;
    fp32 m_rightStickY;
    fp32 m_leftStickX;
    fp32 m_leftStickY;
    fp32 m_scrollWheel;
    SwitchStatus3Pos m_rightSwitchStatus;
    SwitchStatus3Pos m_lastRightSwitchStatus; // 上一次右拨杆状态
    SwitchEvent3Pos m_rightSwitchEvent;
    SwitchStatus3Pos m_leftSwitchStatus;
    SwitchStatus3Pos m_lastLeftSwitchStatus; // 上一次左拨杆状态
    SwitchEvent3Pos m_leftSwitchEvent;
    fp32 m_mouseXSpeed;
    fp32 m_mouseYSpeed;
    fp32 m_mouseWheelSpeed;
    KeyStatus m_mouseLeftKeyStatus;
    KeyStatus m_lastMouseLeftKeyStatus; // 上一次鼠标左键状态
    KeyEvent m_mouseLeftKeyEvent;
    KeyStatus m_mouseRightKeyStatus;
    KeyStatus m_lastMouseRightKeyStatus; // 上一次鼠标右键状态
    KeyEvent m_mouseRightKeyEvent;
    KeyStatus m_keyboardKeyStatus[static_cast<uint8_t>(KeyboardKeyIndex::KEY_TOTAL_NUMBER)];
    KeyStatus m_lastKeyboardKeyStatus[static_cast<uint8_t>(KeyboardKeyIndex::KEY_TOTAL_NUMBER)]; // 上一次键盘按键状态
    KeyEvent m_keyboardKeyEvent[static_cast<uint8_t>(KeyboardKeyIndex::KEY_TOTAL_NUMBER)];

    bool isRxFrameValid(const uint8_t *data) const override;
    void decodeControlData(const DR16OriginalRxData &rxData);
    void updateEvent(const DR16OriginalRxData &rxData);
    void clearEvent();

public:
    DR16RemoteControl(fp32 stickDeadZone = 0.0f);

    void receiveRxDataFromISR(const uint8_t *data, uint16_t length, UART_RxEvent_t event) override;
    void updateRxData() override;

    fp32 getRightStickX()
    {
        return applyStickDeadZone(m_rightStickX);
    }
    fp32 getRightStickY()
    {
        return applyStickDeadZone(m_rightStickY);
    }
    fp32 getLeftStickX()
    {
        return applyStickDeadZone(m_leftStickX);
    }
    fp32 getLeftStickY()
    {
        return applyStickDeadZone(m_leftStickY);
    }
    fp32 getScrollWheel()
    {
        return m_scrollWheel;
    }
    SwitchStatus3Pos getRightSwitchStatus()
    {
        return m_rightSwitchStatus;
    }
    SwitchEvent3Pos getRightSwitchEvent()
    {
        return m_rightSwitchEvent;
    }
    SwitchStatus3Pos getLeftSwitchStatus()
    {
        return m_leftSwitchStatus;
    }
    SwitchEvent3Pos getLeftSwitchEvent()
    {
        return m_leftSwitchEvent;
    }
    fp32 getMouseX()
    {
        return m_mouseXSpeed;
    }
    fp32 getMouseY()
    {
        return m_mouseYSpeed;
    }
    fp32 getMouseWheel()
    {
        return m_mouseWheelSpeed;
    }
    KeyStatus getMouseLeftKeyStatus()
    {
        return m_mouseLeftKeyStatus;
    }
    KeyEvent getMouseLeftKeyEvent()
    {
        return m_mouseLeftKeyEvent;
    }
    KeyStatus getMouseRightKeyStatus()
    {
        return m_mouseRightKeyStatus;
    }
    KeyEvent getMouseRightKeyEvent()
    {
        return m_mouseRightKeyEvent;
    }
    KeyStatus getKeyboardKeyStatus(KeyboardKeyIndex keyIndex)
    {
        return m_keyboardKeyStatus[static_cast<uint8_t>(keyIndex)];
    }
    KeyEvent getKeyboardKeyEvent(KeyboardKeyIndex keyIndex)
    {
        return m_keyboardKeyEvent[static_cast<uint8_t>(keyIndex)];
    }
};

/**
 * @brief 天地飞ET08A遥控器类 (W.BUS)
 * @note 详细使用文档见Documentation/Device/ET08ARemoteControl.md
 * @note 每个控制周期开头调用一次updateRxData, getter仅返回本轮缓存
 * @note 接收UART/DMA的中断优先级数值须不小于configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY
 */
class ET08ARemoteControl : public RemoteControl
{
public:
    // W.BUS完整帧为25字节(含flags); F4库的同名结构体为24字节
    struct ET08ARawPacket {
        uint8_t startByte;
        uint8_t data[22];
        uint8_t flags; // bit0/1: CH17/18, bit2: 丢帧, bit3: 失控保护
        uint8_t stopByte;
    } __attribute__((packed));

    // ET08A通道索引枚举
    enum class ET08AChannelIndex : uint8_t {
        CH_1 = 0,
        CH_2,
        CH_3,
        CH_4,
        CH_5,
        CH_6,
        CH_7,
        CH_8,
        CH_NONE = 0xFF
    };

    // ET08A配置结构体
    struct Config {
        ET08AChannelIndex rightStickJ1X = ET08AChannelIndex::CH_1;
        ET08AChannelIndex rightStickJ2Y = ET08AChannelIndex::CH_3;
        ET08AChannelIndex leftStickJ3Y  = ET08AChannelIndex::CH_2;
        ET08AChannelIndex leftStickJ4X  = ET08AChannelIndex::CH_4;
        ET08AChannelIndex switchSA      = ET08AChannelIndex::CH_NONE;
        ET08AChannelIndex switchSB      = ET08AChannelIndex::CH_NONE;
        ET08AChannelIndex switchSC      = ET08AChannelIndex::CH_NONE;
        ET08AChannelIndex switchSD      = ET08AChannelIndex::CH_NONE;
        ET08AChannelIndex switchSASB    = ET08AChannelIndex::CH_5; // 复用SA和SB通道,SB微调
        ET08AChannelIndex switchSCSD    = ET08AChannelIndex::CH_6; // 复用SC和SD通道,SC微调
        ET08AChannelIndex knobLD        = ET08AChannelIndex::CH_7;
        ET08AChannelIndex knobRD        = ET08AChannelIndex::CH_8;
        ET08AChannelIndex trimmerT1     = ET08AChannelIndex::CH_NONE;
        ET08AChannelIndex trimmerT2     = ET08AChannelIndex::CH_NONE;
        ET08AChannelIndex trimmerT3     = ET08AChannelIndex::CH_NONE;
        ET08AChannelIndex trimmerT4     = ET08AChannelIndex::CH_NONE;
    };

private:
    struct ET08AProtocolData {
        uint16_t rightStickX;
        uint16_t rightStickY;
        uint16_t leftStickX;
        uint16_t leftStickY;
        uint16_t switchSA;
        uint16_t switchSB;
        uint16_t switchSC;
        uint16_t switchSD;
        uint16_t switchSASB;
        uint16_t switchSCSD;
        uint16_t knobLD;
        uint16_t knobRD;
        uint16_t trimmerT1;
        uint16_t trimmerT2;
        uint16_t trimmerT3;
        uint16_t trimmerT4;
    };

    ET08ARawPacket m_rxFrames[3];
    ET08AProtocolData m_protocolData; // 解码后的协议数据缓存
    Config m_config;                  // 通道配置结构体

    // ET08A 特有控件数据
    SwitchStatus2Pos m_switchSA, m_switchSD; // SA和SD只有UP和DOWN两个挡位
    SwitchStatus2Pos m_lastSwitchSA, m_lastSwitchSD;
    SwitchEvent2Pos m_eventSA, m_eventSD;
    SwitchStatus3Pos m_switchSB, m_switchSC;
    SwitchStatus3Pos m_lastSwitchSB, m_lastSwitchSC;
    SwitchEvent3Pos m_eventSB, m_eventSC;
    fp32 m_knobLD, m_knobRD;
    fp32 m_trimmerT1, m_trimmerT2, m_trimmerT3, m_trimmerT4;

    // 标准遥控器数据
    fp32 m_rightStickX, m_rightStickY;
    fp32 m_leftStickX, m_leftStickY;
    SwitchStatus3Pos m_rightSwitchStatus;
    SwitchEvent3Pos m_rightSwitchEvent;
    SwitchStatus3Pos m_leftSwitchStatus;
    SwitchEvent3Pos m_leftSwitchEvent;

    bool isRxFrameValid(const uint8_t *data) const override;
    void updateConnectionStatus(const ET08ARawPacket &rxData);
    void decodeControlData(const ET08ARawPacket &rxData);
    void updateEvent();
    void clearEvent();
    void parseET08AProtocol(const ET08ARawPacket &packet, ET08AProtocolData &out) const;

public:
    ET08ARemoteControl(Config config, fp32 stickDeadZone = 0.0f);
    ET08ARemoteControl(fp32 stickDeadZone = 0.0f);

    void receiveRxDataFromISR(const uint8_t *data, uint16_t length, UART_RxEvent_t event) override;
    void updateRxData() override;

    void setConfig(const Config &config)
    {
        m_config             = config; // 下一有效帧使用新映射; F4库可重新解包当前帧应用映射
        m_isEventInitialized = false;
    }
    const Config &getConfig() const
    {
        return m_config;
    }
    SwitchStatus2Pos getSwitchSA()
    {
        return m_switchSA;
    }
    SwitchStatus3Pos getSwitchSB()
    {
        return m_switchSB;
    }
    SwitchStatus3Pos getSwitchSC()
    {
        return m_switchSC;
    }
    SwitchStatus2Pos getSwitchSD()
    {
        return m_switchSD;
    }
    SwitchEvent2Pos getSwitchEventSA()
    {
        return m_eventSA;
    }
    SwitchEvent3Pos getSwitchEventSB()
    {
        return m_eventSB;
    }
    SwitchEvent3Pos getSwitchEventSC()
    {
        return m_eventSC;
    }
    SwitchEvent2Pos getSwitchEventSD()
    {
        return m_eventSD;
    }
    fp32 getKnobLD()
    {
        return m_knobLD;
    }
    fp32 getKnobRD()
    {
        return m_knobRD;
    }
    fp32 getTrimmerT1()
    {
        return m_trimmerT1;
    }
    fp32 getTrimmerT2()
    {
        return m_trimmerT2;
    }
    fp32 getTrimmerT3()
    {
        return m_trimmerT3;
    }
    fp32 getTrimmerT4()
    {
        return m_trimmerT4;
    }
    // 标准获取函数
    fp32 getRightStickX()
    {
        return applyStickDeadZone(m_rightStickX);
    }
    fp32 getRightStickY()
    {
        return applyStickDeadZone(m_rightStickY);
    }
    fp32 getLeftStickX()
    {
        return applyStickDeadZone(m_leftStickX);
    }
    fp32 getLeftStickY()
    {
        return applyStickDeadZone(m_leftStickY);
    }
};

/**
 * @brief 大疆VT13图传遥控器类，用于解码图传模块配套的遥控器数据
 * @note 使用前需确保receiveRxDataFromISR方法在对应UART接收中断服务函数中被调用
 * @note 该遥控器使用的UART通常与裁判系统图传链路共用，回调中依次调用VT13和Referee的接收函数即可
 */
class VT13RemoteControl : public RemoteControl
{
public:
    // VT13图传遥控器帧常量定义
    static constexpr uint8_t FRAME_SIZE = 21;
    static constexpr uint8_t SOF_1      = 0xA9;
    static constexpr uint8_t SOF_2      = 0x53;

    // VT13图传遥控器原始数据结构体
    struct VT13OriginalRxData {
        uint8_t sof1;
        uint8_t sof2;
        uint64_t Channel_0 : 11;  // 右摇杆水平
        uint64_t Channel_1 : 11;  // 右摇杆垂直
        uint64_t Channel_2 : 11;  // 左摇杆垂直
        uint64_t Channel_3 : 11;  // 左摇杆水平
        uint64_t Mode_Switch : 2; // 档位切换 C:0 N:1 S:2
        uint64_t Pause : 1;       // 暂停键
        uint64_t Fn_1 : 1;        // 自定义按键(左)
        uint64_t Fn_2 : 1;        // 自定义按键(右)
        uint64_t Wheel : 11;      // 拨轮
        uint64_t Trigger : 1;     // 扳机键
        int16_t Mouse_X;
        int16_t Mouse_Y;
        int16_t Mouse_Z;
        uint8_t Mouse_Left : 2;
        uint8_t Mouse_Right : 2;
        uint8_t Mouse_Middle : 2;
        uint16_t Keyboard_Key;
        uint16_t crc16;
    } __attribute__((packed));

    // VT13遥控器键盘按键对应索引
    enum class KeyboardKeyIndex : uint8_t {
        KEY_W = 0,
        KEY_S,
        KEY_A,
        KEY_D,
        KEY_SHIFT,
        KEY_CTRL,
        KEY_Q,
        KEY_E,
        KEY_R,
        KEY_F,
        KEY_G,
        KEY_Z,
        KEY_X,
        KEY_C,
        KEY_V,
        KEY_B,
        KEY_TOTAL_NUMBER // 键盘按键枚举值总数
    };

private:
    VT13OriginalRxData m_rxFrames[3];
    // 摇杆和拨轮解码数据
    fp32 m_rightStickX;
    fp32 m_rightStickY;
    fp32 m_leftStickX;
    fp32 m_leftStickY;
    fp32 m_scrollWheel;
    // 档位开关
    SwitchStatus3Pos m_modeSwitchStatus;
    SwitchStatus3Pos m_lastModeSwitchStatus;
    SwitchEvent3Pos m_modeSwitchEvent;
    // 按键状态
    KeyStatus m_pauseKeyStatus;
    KeyStatus m_lastPauseKeyStatus;
    KeyEvent m_pauseKeyEvent;
    KeyStatus m_fn1KeyStatus;
    KeyStatus m_lastFn1KeyStatus;
    KeyEvent m_fn1KeyEvent;
    KeyStatus m_fn2KeyStatus;
    KeyStatus m_lastFn2KeyStatus;
    KeyEvent m_fn2KeyEvent;
    KeyStatus m_triggerKeyStatus;
    KeyStatus m_lastTriggerKeyStatus;
    KeyEvent m_triggerKeyEvent;
    // 鼠标数据
    fp32 m_mouseXSpeed;
    fp32 m_mouseYSpeed;
    fp32 m_mouseWheelSpeed;
    KeyStatus m_mouseLeftKeyStatus;
    KeyStatus m_lastMouseLeftKeyStatus;
    KeyEvent m_mouseLeftKeyEvent;
    KeyStatus m_mouseRightKeyStatus;
    KeyStatus m_lastMouseRightKeyStatus;
    KeyEvent m_mouseRightKeyEvent;
    KeyStatus m_mouseMiddleKeyStatus;
    KeyStatus m_lastMouseMiddleKeyStatus;
    KeyEvent m_mouseMiddleKeyEvent;
    // 键盘按键
    KeyStatus m_keyboardKeyStatus[static_cast<uint8_t>(KeyboardKeyIndex::KEY_TOTAL_NUMBER)];
    KeyStatus m_lastKeyboardKeyStatus[static_cast<uint8_t>(KeyboardKeyIndex::KEY_TOTAL_NUMBER)];
    KeyEvent m_keyboardKeyEvent[static_cast<uint8_t>(KeyboardKeyIndex::KEY_TOTAL_NUMBER)];

public:
    VT13RemoteControl(fp32 stickDeadZone = 0.0f);

    void receiveRxDataFromISR(const uint8_t *data, uint16_t length, UART_RxEvent_t event) override;
    void updateRxData() override;

    fp32 getRightStickX()
    {
        return applyStickDeadZone(m_rightStickX);
    }
    fp32 getRightStickY()
    {
        return applyStickDeadZone(m_rightStickY);
    }
    fp32 getLeftStickX()
    {
        return applyStickDeadZone(m_leftStickX);
    }
    fp32 getLeftStickY()
    {
        return applyStickDeadZone(m_leftStickY);
    }
    fp32 getScrollWheel()
    {
        return m_scrollWheel;
    }
    SwitchStatus3Pos getModeSwitchStatus()
    {
        return m_modeSwitchStatus;
    }
    SwitchEvent3Pos getModeSwitchEvent()
    {
        return m_modeSwitchEvent;
    }
    KeyStatus getPauseKeyStatus()
    {
        return m_pauseKeyStatus;
    }
    KeyEvent getPauseKeyEvent()
    {
        return m_pauseKeyEvent;
    }
    KeyStatus getFn1KeyStatus()
    {
        return m_fn1KeyStatus;
    }
    KeyEvent getFn1KeyEvent()
    {
        return m_fn1KeyEvent;
    }
    KeyStatus getFn2KeyStatus()
    {
        return m_fn2KeyStatus;
    }
    KeyEvent getFn2KeyEvent()
    {
        return m_fn2KeyEvent;
    }
    KeyStatus getTriggerKeyStatus()
    {
        return m_triggerKeyStatus;
    }
    KeyEvent getTriggerKeyEvent()
    {
        return m_triggerKeyEvent;
    }
    fp32 getMouseX()
    {
        return m_mouseXSpeed;
    }
    fp32 getMouseY()
    {
        return m_mouseYSpeed;
    }
    fp32 getMouseWheel()
    {
        return m_mouseWheelSpeed;
    }
    KeyStatus getMouseLeftKeyStatus()
    {
        return m_mouseLeftKeyStatus;
    }
    KeyEvent getMouseLeftKeyEvent()
    {
        return m_mouseLeftKeyEvent;
    }
    KeyStatus getMouseRightKeyStatus()
    {
        return m_mouseRightKeyStatus;
    }
    KeyEvent getMouseRightKeyEvent()
    {
        return m_mouseRightKeyEvent;
    }
    KeyStatus getMouseMiddleKeyStatus()
    {
        return m_mouseMiddleKeyStatus;
    }
    KeyEvent getMouseMiddleKeyEvent()
    {
        return m_mouseMiddleKeyEvent;
    }
    KeyStatus getKeyboardKeyStatus(KeyboardKeyIndex keyIndex)
    {
        return m_keyboardKeyStatus[static_cast<uint8_t>(keyIndex)];
    }
    KeyEvent getKeyboardKeyEvent(KeyboardKeyIndex keyIndex)
    {
        return m_keyboardKeyEvent[static_cast<uint8_t>(keyIndex)];
    }

private:
    bool isRxFrameValid(const uint8_t *data) const override;
    void decodeControlData(const VT13OriginalRxData &rxData);
    void updateEvent(const VT13OriginalRxData &rxData);
    void clearEvent();
    static SwitchStatus3Pos mapModeSwitch(uint8_t raw); // 档位原始值到SwitchStatus3Pos的映射
};

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

/* Defines -------------------------------------------------------------------*/
