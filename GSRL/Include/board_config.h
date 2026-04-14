/**
 ******************************************************************************
 * @file           : board_config.h
 * @brief          : board configuration, used to configure the peripheral used
 *                   开发板配置信息，用于配置使用的外设组件
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 GMaster
 * All rights reserved.
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BOARD_CONFIG_H
#define __BOARD_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/* Exported macro ------------------------------------------------------------*/
/* System core ------------------------------*/
/* DMA */
/* GPIO */
/* NVIC */
/* RCC */
/* SYS */
/* Analog -----------------------------------*/
/* ADC */
/* DAC */
/* Timers -----------------------------------*/
/* RTC */
/* TIM */
/* Connectivity -----------------------------*/
/* FDCAN */
#define USE_FDCAN1
#define USE_FDCAN2
#define USE_FDCAN3
/* I2C */
/* SPI */
// #define USE_SPI1
#define USE_SPI2
/* UART */
#define USE_USART1
#define USE_UART5
#define USE_UART7
#define USE_UART10
/* USB */

#ifdef __cplusplus
}
#endif

#endif /* __BOARD_CONFIG_H */
