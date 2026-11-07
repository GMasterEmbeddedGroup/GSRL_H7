/**
 ******************************************************************************
 * @file           : drv_uart.h
 * @brief          : header file for drv_uart.c
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 GMaster
 * All rights reserved.
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DRV_UART_H
#define __DRV_UART_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gsrl_common.h"
#include "usart.h"

/* Exported macro ------------------------------------------------------------*/
#define UART_BUFFER_SIZE 512 // UART接收缓冲区最大长度

/* Exported types ------------------------------------------------------------*/

typedef enum
{
    UART_RX_EVENT_DATA,
    UART_RX_EVENT_IDLE,
    UART_RX_EVENT_ERROR
} UART_RxEvent_t;

/**
 * @brief UART通信接收处理回调函数类型定义
 * @param pRxData 接收数据指针
 * @param rxDataLength 本段接收数据的字节数, IDLE通知允许为0
 * @param event 接收事件, ERROR时数据指针为NULL且长度为0
 * @note F4库(popchutty/GSRL)回调为(data, length); 本接口为(data, length, event)
 * @note 只交付尚未交付的接收字节, 一次回调不一定是一帧; 设备接收入口须转发event
 * @note 数据仅在回调期间有效; 同一串口的UART/DMA中断须使用相同抢占优先级
 * @note Circular接收保留HT/TC中断, 响应及取走数据的耗时须小于半缓冲区填充时间
 */
typedef void (*UART_Call_Back)(uint8_t *pRxData, uint16_t rxDataLength, UART_RxEvent_t event);

/**
 * @brief UART通信处理结构体
 */
typedef struct
{
    UART_HandleTypeDef *huart;
    uint8_t rxBuffer[UART_BUFFER_SIZE];
    uint16_t rxDataLimit;
    uint16_t rxReadPosition;
    UART_Call_Back rxCallbackFunction;
} UART_Manage_Object_t;

/* Exported variables --------------------------------------------------------*/
extern UART_Manage_Object_t s_uart_manage_objects[8]; // UART管理对象

/* Exported functions prototypes ---------------------------------------------*/
void UART_Init(UART_HandleTypeDef *huart, UART_Call_Back rxCallbackFunction, uint16_t rxDataLimit);
void UART_Reinit(UART_HandleTypeDef *huart);
HAL_StatusTypeDef UART_Send_Data(UART_HandleTypeDef *huart, uint8_t *pTxData, uint16_t txDataLength);

/* Defines -------------------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* __DRV_UART_H */
