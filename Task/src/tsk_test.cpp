/**
 ******************************************************************************
 * @file           : tsk_test.cpp
 * @brief          : ET08A最小调试任务
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 GMaster
 * All rights reserved.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "dvc_buzzer.hpp"
#include "dvc_remotecontrol.hpp"
#include "drv_uart.h"


/* Define --------------------------------------------------------------------*/
ET08ARemoteControl et08aRemote; // ET08A遥控器对象



/* Variables -----------------------------------------------------------------*/


// ET08A 遥控器调试变量（在调试器 Watch 中观察）
volatile bool et08aConnected      = false;
volatile fp32 et08aRightStickX    = 0.0f;
volatile fp32 et08aRightStickY    = 0.0f;
volatile fp32 et08aLeftStickX     = 0.0f;
volatile fp32 et08aLeftStickY     = 0.0f;
volatile int32_t et08aSwitchSA    = 0;
volatile int32_t et08aSwitchSB    = 0;
volatile int32_t et08aSwitchSC    = 0;
volatile int32_t et08aSwitchSD    = 0;

/* Function prototypes -------------------------------------------------------*/

extern "C" void uart5RxCallback(uint8_t *pData, uint16_t Size); // UART5 接收回调（ET08A遥控器）


/* User code -----------------------------------------------------------------*/

/**
 * @brief 测试任务
 * @param argument 任务参数
 */
extern "C" void test_task(void *argument)
{
    UART_Init(&huart5, uart5RxCallback, 24);

    static Buzzer testBuzzer(&htim12, TIM_CHANNEL_2, 84000000U, false);
    testBuzzer.setFrequency(1000U, 0.25f);
    vTaskDelay(pdMS_TO_TICKS(200));
    testBuzzer.setFrequency(0U);

    TickType_t taskLastWakeTime = xTaskGetTickCount();
    while (1) {
        
        et08aRemote.updateEvent();
        et08aConnected   = et08aRemote.isConnected();
        et08aRightStickX = et08aRemote.getRightStickX();
        et08aRightStickY = et08aRemote.getRightStickY();
        et08aLeftStickX  = et08aRemote.getLeftStickX();
        et08aLeftStickY  = et08aRemote.getLeftStickY();
        et08aSwitchSA    = static_cast<int32_t>(et08aRemote.getSwitchSA());
        et08aSwitchSB    = static_cast<int32_t>(et08aRemote.getSwitchSB());
        et08aSwitchSC    = static_cast<int32_t>(et08aRemote.getSwitchSC());
        et08aSwitchSD    = static_cast<int32_t>(et08aRemote.getSwitchSD());

        vTaskDelayUntil(&taskLastWakeTime, 1);
    }
}


/**
 * @brief UART5接收中断回调函数（ET08A遥控器）
 * @param pData 接收数据指针
 * @param Size 接收数据长度
 */
extern "C" void uart5RxCallback(uint8_t *pData, uint16_t Size)
{
   
        et08aRemote.receiveRxDataFromISR(pData);
        
    
}



  