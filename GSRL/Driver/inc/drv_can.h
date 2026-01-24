/**
 ******************************************************************************
 * @file           : drv_can.h
 * @brief          : Header for drv_can.c file.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 GMaster
 * All rights reserved.
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DRV_CAN_H
#define __DRV_CAN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gsrl_common.h"
#include "fdcan.h"
#include "cmsis_os.h"

/* Exported types ------------------------------------------------------------*/

/**
 * @brief FDCAN接收消息结构体
 */
typedef struct
{
    FDCAN_RxHeaderTypeDef header;
    uint8_t data[8];
} can_rx_message_t;

/**
 * @brief CAN通信接收处理回调函数类型定义
 * @param pRxMsg 接收数据结构体指针
 */
typedef void (*CAN_Call_Back)(can_rx_message_t *pRxMsg);

/**
 * @brief FDCAN通信处理结构体
 */
typedef struct
{
    FDCAN_HandleTypeDef *hfdcan;
    CAN_Call_Back rxCallbackFunction;
} CAN_Manage_Object_t;

/* Exported constants --------------------------------------------------------*/
extern osMessageQId canRxQueueHandle;
extern CAN_Manage_Object_t s_can_manage_objects[3]; // FDCAN管理对象

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
void CAN_Init(FDCAN_HandleTypeDef *hfdcan, CAN_Call_Back rxCallbackFunction);
HAL_StatusTypeDef CAN_Send_Data(FDCAN_HandleTypeDef *hfdcan, FDCAN_TxHeaderTypeDef *pTxHeader, uint8_t *pTxData);

/* Defines -----------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* __DRV_CAN_H */
