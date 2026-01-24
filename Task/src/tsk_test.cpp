/**
 ******************************************************************************
 * @file           : tsk_test.cpp
 * @brief          : 测试任务 - 六电机控制
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 GMaster
 * All rights reserved.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "dvc_motor.hpp"
#include "drv_can.h"

/* Define --------------------------------------------------------------------*/
// PID控制器参数（所有电机共用，可根据需要为每个电机单独配置）
SimplePID::PIDParam pidParam = {
    0.0f, // Kp
    0.0f, // Ki
    0.0f, // Kd
    0.0f, // outputLimit
    0.0f  // intergralLimit
};
SimplePID pid1(SimplePID::PID_POSITION, pidParam);
SimplePID pid2(SimplePID::PID_POSITION, pidParam);
SimplePID pid3(SimplePID::PID_POSITION, pidParam);
SimplePID pid4(SimplePID::PID_POSITION, pidParam);
SimplePID pid5(SimplePID::PID_POSITION, pidParam);
SimplePID pid6(SimplePID::PID_POSITION, pidParam);

// CAN1电机定义：1个GM6020、2个DM4310
MotorGM6020 motor_6020(1, &pid1);                            // GM6020，ID=1
MotorDM4310 motor_dm4310_1(1, 0, 3.1415926f, 40, 15, &pid2); // DM4310，ID=2
MotorDM4310 motor_dm4310_2(2, 3, 3.1415926f, 40, 15, &pid3); // DM4310，ID=3

// CAN2电机定义：3个M3508
MotorM3508 motor_3508_1(1, &pid4); // M3508，ID=1
MotorM3508 motor_3508_2(2, &pid5); // M3508，ID=2
MotorM3508 motor_3508_3(3, &pid6); // M3508，ID=3

/* Variables -----------------------------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/
extern "C" void can1RxCallback(can_rx_message_t *pRxMsg);
extern "C" void can2RxCallback(can_rx_message_t *pRxMsg);
inline void transmitMotorsControlData();

/* User code -----------------------------------------------------------------*/

/**
 * @brief 测试任务
 * @param argument 任务参数
 */
extern "C" void test_task(void *argument)
{
    CAN_Init(&hfdcan1, can1RxCallback);                // 初始化CAN1
    CAN_Init(&hfdcan2, can2RxCallback);                // 初始化CAN2
    TickType_t taskLastWakeTime = xTaskGetTickCount(); // 获取任务开始时间
    while (1) {
        // 所有电机输出无力控制（开环控制值为0）
        motor_6020.openloopControl(0.0f);
        motor_dm4310_1.openloopControl(0.0f);
        motor_dm4310_2.openloopControl(0.0f);
        motor_3508_1.openloopControl(0.0f);
        motor_3508_2.openloopControl(0.0f);
        motor_3508_3.openloopControl(0.0f);

        // 发送控制数据
        transmitMotorsControlData();

        vTaskDelayUntil(&taskLastWakeTime, 1); // 确保任务以定周期1ms运行
    }
}

/**
 * @brief CAN1接收中断回调函数
 * @param pRxMsg CAN接收消息指针
 */
extern "C" void can1RxCallback(can_rx_message_t *pRxMsg)
{
    // 解析CAN1上的三个电机反馈数据
    motor_6020.decodeCanRxMessageFromISR(pRxMsg);
    motor_dm4310_1.decodeCanRxMessageFromISR(pRxMsg);
    motor_dm4310_2.decodeCanRxMessageFromISR(pRxMsg);
}

/**
 * @brief CAN2接收中断回调函数
 * @param pRxMsg CAN接收消息指针
 */
extern "C" void can2RxCallback(can_rx_message_t *pRxMsg)
{
    // 解析CAN2上的三个M3508电机反馈数据
    motor_3508_1.decodeCanRxMessageFromISR(pRxMsg);
    motor_3508_2.decodeCanRxMessageFromISR(pRxMsg);
    motor_3508_3.decodeCanRxMessageFromISR(pRxMsg);
}

/**
 * @brief 发送电机控制数据
 * @note M3508电机使用一控四的方式，三个电机的数据合并到一个CAN包中
 * @note GM6020和DM4310需要分别发送
 */
inline void transmitMotorsControlData()
{
    // CAN1: 发送GM6020控制数据
    CAN_Send_Data(&hfdcan1,
                  (FDCAN_TxHeaderTypeDef *)motor_6020.getMotorControlHeader(),
                  (uint8_t *)motor_6020.getMotorControlData());

    // CAN1: 发送DM4310-1控制数据
    CAN_Send_Data(&hfdcan1,
                  (FDCAN_TxHeaderTypeDef *)motor_dm4310_1.getMotorControlHeader(),
                  (uint8_t *)motor_dm4310_1.getMotorControlData());

    // CAN1: 发送DM4310-2控制数据
    CAN_Send_Data(&hfdcan1,
                  (FDCAN_TxHeaderTypeDef *)motor_dm4310_2.getMotorControlHeader(),
                  (uint8_t *)motor_dm4310_2.getMotorControlData());

    // CAN2: 合并三个M3508电机的控制数据到一个CAN包（一控四方式）
    // M3508电机使用0x200控制ID，可以将ID 1-4的电机数据合并到一个包中
    MotorGM6020 combinedMotor = motor_3508_1 + motor_3508_2 + motor_3508_3;
    CAN_Send_Data(&hfdcan2,
                  (FDCAN_TxHeaderTypeDef *)combinedMotor.getMotorControlHeader(),
                  (uint8_t *)combinedMotor.getMotorControlData());
}