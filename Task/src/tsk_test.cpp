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
#include "dvc_ws2812.hpp"
#include "dvc_remotecontrol.hpp"
#include "drv_uart.h"


/* Define --------------------------------------------------------------------*/




/* Variables -----------------------------------------------------------------*/




/* Function prototypes -------------------------------------------------------*/



/* User code -----------------------------------------------------------------*/

/**
 * @brief 测试任务
 * @param argument 任务参数
 */
extern "C" void test_task(void *argument)
{
    static Ws2812Spi ws2812(&hspi6);
    ws2812.init();

    while (1) {
        // 在这里直接填 red, green, blue 数值
        ws2812.setColor(255, 0, 0); // 红色
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}




  