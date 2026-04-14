/**
 ******************************************************************************
 * @file           : drv_can.c
 * @brief          : CAN driver
 *                   Contains CAN initialization related functions,
 *                   CAN receive interrupt callback function
 *                   Use queue to save received data when using FreeRTOS,
 *                   otherwise use global variables
 *                   CAN驱动
 *                   包含CAN初始化相关函数，CAN接收中断回调函数
 *                   使用FreeRTOS时使用队列保存接收数据，否则使用全局变量
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 GMaster
 * All rights reserved.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "drv_can.h"
#include "queue.h"

/* Typedef -----------------------------------------------------------*/

/* Define ------------------------------------------------------------*/
osMessageQId canRxQueueHandle;

/* Macro -------------------------------------------------------------*/

/* Variables ---------------------------------------------------------*/
CAN_Manage_Object_t s_can_manage_objects[3] = {0};   // FDCAN管理对象
static bool_t is_queue_initialized          = false; // 队列初始化标志

/**
 * @brief FDCAN管理实例数组
 * @note 根据board_config.h中的宏定义配置决定使用的FDCAN实例
 */
static FDCAN_GlobalTypeDef *const fdcanInstances[3] = {
#ifdef USE_FDCAN1
    FDCAN1,
#endif
#ifdef USE_FDCAN2
    FDCAN2,
#endif
#ifdef USE_FDCAN3
    FDCAN3
#endif
};

/* Function prototypes -----------------------------------------------*/
static void can_all_pass_filter_init(FDCAN_HandleTypeDef *hfdcan);

/* User code ---------------------------------------------------------*/

/**
 * @brief 获取FDCAN对象
 * @param hfdcan FDCAN句柄
 * @return FDCAN管理对象指针
 */
static CAN_Manage_Object_t *CAN_Get_Object(FDCAN_HandleTypeDef *hfdcan)
{
    for (int i = 0; i < 3; i++) {
        if (hfdcan->Instance == fdcanInstances[i]) {
            return &s_can_manage_objects[i];
        }
    }
    return NULL;
}

/**
 * @brief 初始化FDCAN，注册回调函数，并启动FDCAN
 * @param hfdcan FDCAN句柄
 * @param rxCallbackFunction 处理回调函数
 */
void CAN_Init(FDCAN_HandleTypeDef *hfdcan, CAN_Call_Back rxCallbackFunction)
{
    // 找到对应的FDCAN管理对象并设置参数
    CAN_Manage_Object_t *can_obj = CAN_Get_Object(hfdcan);
    if (can_obj == NULL) return;

    can_obj->hfdcan             = hfdcan;
    can_obj->rxCallbackFunction = rxCallbackFunction;

    // 初始化滤波器
    can_all_pass_filter_init(hfdcan);

    // 启动FDCAN
    HAL_FDCAN_Start(hfdcan);
    HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

    // 确保队列只初始化一次
    if (is_queue_initialized == false) {
        canRxQueueHandle     = xQueueCreate(16, sizeof(can_rx_message_t));
        is_queue_initialized = true;
    }
}

/**
 * @brief FDCAN发送消息
 * @param hfdcan FDCAN句柄
 * @param pTxHeader 发送帧头指针
 * @param pTxData 发送数据指针
 * @return HAL status
 */
HAL_StatusTypeDef CAN_Send_Data(FDCAN_HandleTypeDef *hfdcan, FDCAN_TxHeaderTypeDef *pTxHeader, uint8_t *pTxData)
{
    return HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, pTxHeader, pTxData);
}

/**
 * @brief 初始化FDCAN标准帧全通过滤器（只接收标准帧）
 *        Initialize FDCAN standard frame all-pass filter (standard frames only)
 * @param hfdcan FDCAN句柄
 * @retval None
 */
static void can_all_pass_filter_init(FDCAN_HandleTypeDef *hfdcan)
{
    FDCAN_FilterTypeDef sFilterConfig = {0};

    // 配置标准ID全通过滤器（只接收标准帧）
    sFilterConfig.IdType       = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex  = 0;                       // 使用第0个滤波器
    sFilterConfig.FilterType   = FDCAN_FILTER_MASK;       // 掩码滤波器
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; // 接收到FIFO0
    sFilterConfig.FilterID1    = 0x000;                   // 起始ID：0
    sFilterConfig.FilterID2    = 0x000;                   // Mask 值 (0x0 表示全通)

    // 配置标准ID滤波器
    if (HAL_FDCAN_ConfigFilter(hfdcan, &sFilterConfig) != HAL_OK) {
        Error_Handler();
    }

    // 配置全局过滤器
    if (HAL_FDCAN_ConfigGlobalFilter(hfdcan,
                                     FDCAN_REJECT,                   // 非匹配标准帧：拒绝
                                     FDCAN_REJECT,                   // 非匹配扩展帧：拒绝
                                     FDCAN_REJECT_REMOTE,            // 标准远程帧：拒绝
                                     FDCAN_REJECT_REMOTE) != HAL_OK) // 扩展远程帧：拒绝
    {
        Error_Handler();
    }
}

/**
 * @brief HAL库FDCAN中断回调函数
 *        HAL library FDCAN interrupt callback function
 * @param hfdcan FDCAN句柄
 * @param RxFifo0ITs FIFO0中断标志
 * @retval None
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    can_rx_message_t s_rx_msg;

    // 检查是否有新消息中断标志
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0) {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &s_rx_msg.header, s_rx_msg.data) == HAL_OK) {
            // 1. 使用队列保存接收数据（保持原有FreeRTOS机制）
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            if (xQueueIsQueueFullFromISR(canRxQueueHandle)) // 队列满,移除最早的数据
            {
                can_rx_message_t s_dummy_msg;
                xQueueReceiveFromISR(canRxQueueHandle, &s_dummy_msg, &xHigherPriorityTaskWoken);
            }
            // 写入新数据
            xQueueSendToBackFromISR(canRxQueueHandle, &s_rx_msg, &xHigherPriorityTaskWoken);

            // 2. 调用注册的回调函数（保持原有功能）
            CAN_Manage_Object_t *can_obj = CAN_Get_Object(hfdcan);
            if (can_obj != NULL && can_obj->rxCallbackFunction != NULL) {
                can_obj->rxCallbackFunction(&s_rx_msg);
            }

            // 重新发起调度
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}
