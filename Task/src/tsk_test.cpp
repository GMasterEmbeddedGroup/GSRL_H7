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
#include "dvc_remotecontrol.hpp"
#include "dvc_imu.hpp"
#include "drv_uart.h"

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

// 遥控器定义（使用 UART5 接收 DR16 遥控器数据）
DR16RemoteControl dr16Remote(0.0f); // 0.0f = 摇杆死区

// BMI088 IMU 定义
Mahony ahrs{};
BMI088::CalibrationInfo cali = {
    {0.0f, 0.0f, 0.0f},                         // gyroOffset
    {0.0f, 0.0f, 0.0f},                         // accelOffset
    {0.0f, 0.0f, 0.0f},                         // magnetOffset
    {GSRLMath::Matrix33f::MatrixType::IDENTITY} // installSpinMatrix
};
BMI088 imu(&ahrs,
           {&hspi2, ACCEL_CS_GPIO_Port, ACCEL_CS_Pin},
           {&hspi2, GYRO_CS_GPIO_Port, GYRO_CS_Pin},
           cali);

/* Variables -----------------------------------------------------------------*/
// 用于调试查看的变量
GSRLMath::Vector3f eulerAngle; // 欧拉角 (roll, pitch, yaw)
GSRLMath::Vector3f accelData;  // 加速度计数据
GSRLMath::Vector3f gyroData;   // 陀螺仪数据
const fp32 *quaternion;        // 四元数指针

/* Function prototypes -------------------------------------------------------*/
extern "C" void can1RxCallback(can_rx_message_t *pRxMsg);
extern "C" void can2RxCallback(can_rx_message_t *pRxMsg);
extern "C" void uart5RxCallback(uint8_t *pData, uint16_t Size); // UART5 接收回调（DR16遥控器）
inline void transmitMotorsControlData();

/* User code -----------------------------------------------------------------*/

/**
 * @brief 测试任务
 * @param argument 任务参数
 */
extern "C" void test_task(void *argument)
{
    // 初始化 BMI088
    (void)imu.init();

    CAN_Init(&hfdcan1, can1RxCallback);
    CAN_Init(&hfdcan2, can2RxCallback);
    UART_Init(&huart5, uart5RxCallback, 36);

    TickType_t taskLastWakeTime = xTaskGetTickCount();
    while (1) {
        // BMI088 数据读取和姿态解算
        imu.solveAttitude(); // 读取传感器数据并更新 AHRS

        // 获取欧拉角 (roll, pitch, yaw) 单位:度
        eulerAngle = imu.getEulerAngle();

        // 获取加速度计和陀螺仪数据
        accelData = imu.getAccel();
        gyroData  = imu.getGyro();

        // 获取四元数 (w, x, y, z)
        quaternion = imu.getQuaternion();

        motor_6020.openloopControl(0.0f);
        motor_dm4310_1.openloopControl(0.0f);
        motor_dm4310_2.openloopControl(0.0f);
        motor_3508_1.openloopControl(0.0f);
        motor_3508_2.openloopControl(0.0f);
        motor_3508_3.openloopControl(0.0f);

        transmitMotorsControlData();

        dr16Remote.decodeRxData();
        dr16Remote.updateEvent();

        vTaskDelayUntil(&taskLastWakeTime, 1);
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
 * @brief UART5接收中断回调函数（DR16遥控器）
 * @param pData 接收数据指针
 * @param Size 接收数据长度
 */
extern "C" void uart5RxCallback(uint8_t *pData, uint16_t Size)
{
    // 将接收到的DR16遥控器数据传递给遥控器对象
    dr16Remote.receiveRxDataFromISR(pData);
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
    // 新 API getMergedControlData 只支持成对合并, 三合一需手工拼接第三段
    uint8_t mergedFrame[8];

    memcpy(mergedFrame, motor_3508_1.getMergedControlData(motor_3508_2), 8);
    const uint8_t *data3  = motor_3508_3.getMotorControlData();
    uint8_t off3          = (uint8_t)(((motor_3508_3.getDjiMotorID() - 1) & 0x3) * 2);
    mergedFrame[off3]     = data3[off3];
    mergedFrame[off3 + 1] = data3[off3 + 1];

    CAN_Send_Data(&hfdcan2,
                  (FDCAN_TxHeaderTypeDef *)motor_3508_1.getMotorControlHeader(),
                  mergedFrame);
}