#ifndef __BSP_CAN_H__
#define __BSP_CAN_H__

/* 包含头文件 ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include <stdio.h>

/* 类型定义 ------------------------------------------------------------------*/
/* 宏定义 --------------------------------------------------------------------*/

#define CANx                            CAN1
#define CANx_CLK_ENABLE()               __HAL_RCC_CAN1_CLK_ENABLE()
#define CANx_FORCE_RESET()              __HAL_RCC_CAN1_FORCE_RESET()
#define CANx_RELEASE_RESET()            __HAL_RCC_CAN1_RELEASE_RESET()

#define CANx_GPIO_CLK_ENABLE()          {__HAL_RCC_GPIOI_CLK_ENABLE();__HAL_RCC_GPIOB_CLK_ENABLE();}
#define CANx_TX_GPIO_PORT               GPIOB
#define CANx_TX_PIN                     GPIO_PIN_9

#define CANx_RX_GPIO_PORT               GPIOI
#define CANx_RX_PIN                     GPIO_PIN_9

#define CANx_RX_IRQn                   CAN1_RX0_IRQn

/* 扩展变量 ------------------------------------------------------------------*/
extern CAN_HandleTypeDef hCAN;

/* 函数声明 ------------------------------------------------------------------*/
void MX_CAN_Init(void);

#endif /* __BSP_CAN_H__ */


/******************* (C) COPYRIGHT 2015-2020 硬石嵌入式开发团队 *****END OF FILE****/
