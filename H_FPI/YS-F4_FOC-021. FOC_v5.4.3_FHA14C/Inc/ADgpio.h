/**
  ******************************************************************************
  * File Name          : ADgpio.h
  * Description        : This file contains all the functions prototypes for 
  *                      the gpio  
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ADgpio_H
#define __ADgpio_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */
#define AD7606Cs_High()   HAL_GPIO_WritePin(CS__GPIO_Port, CS__Pin, GPIO_PIN_SET)
#define AD7606Cs_Low()    HAL_GPIO_WritePin(CS__GPIO_Port, CS__Pin, GPIO_PIN_RESET)
#define AD7606Rst_High()  HAL_GPIO_WritePin(RSET_GPIO_Port, RSET_Pin, GPIO_PIN_SET)
#define AD7606Rst_Low()   HAL_GPIO_WritePin(RSET_GPIO_Port, RSET_Pin, GPIO_PIN_RESET)
/* USER CODE END Private defines */

void MX_ADGPIO_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ pinoutConfig_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
