/**
  ******************************************************************************
  * File Name          : main.hpp
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_ll_crs.h"
#include "stm32f0xx_ll_rcc.h"
#include "stm32f0xx_ll_bus.h"
#include "stm32f0xx_ll_system.h"
#include "stm32f0xx_ll_exti.h"
#include "stm32f0xx_ll_cortex.h"
#include "stm32f0xx_ll_utils.h"
#include "stm32f0xx_ll_pwr.h"
#include "stm32f0xx_ll_dma.h"
#include "stm32f0xx_ll_usart.h"
#include "stm32f0xx.h"
#include "stm32f0xx_ll_gpio.h"

/* Includes ------------------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define LANE_0_Pin LL_GPIO_PIN_0
#define LANE_0_GPIO_Port GPIOA
#define LANE_1_Pin LL_GPIO_PIN_1
#define LANE_1_GPIO_Port GPIOA
#define LANE_2_Pin LL_GPIO_PIN_2
#define LANE_2_GPIO_Port GPIOA
#define LANE_3_Pin LL_GPIO_PIN_3
#define LANE_3_GPIO_Port GPIOA
#define LANE_4_Pin LL_GPIO_PIN_4
#define LANE_4_GPIO_Port GPIOA
#define LANE_5_Pin LL_GPIO_PIN_5
#define LANE_5_GPIO_Port GPIOA
#define LANE_6_Pin LL_GPIO_PIN_6
#define LANE_6_GPIO_Port GPIOA
#define LANE_7_Pin LL_GPIO_PIN_7
#define LANE_7_GPIO_Port GPIOA
#define I_LANE_0_Pin LL_GPIO_PIN_0
#define I_LANE_0_GPIO_Port GPIOB
#define I_LANE_1_Pin LL_GPIO_PIN_1
#define I_LANE_1_GPIO_Port GPIOB
#define LANE_8_Pin LL_GPIO_PIN_8
#define LANE_8_GPIO_Port GPIOA
#define LANE_9_Pin LL_GPIO_PIN_9
#define LANE_9_GPIO_Port GPIOA
#define LED_1_Pin LL_GPIO_PIN_10
#define LED_1_GPIO_Port GPIOA
#define LED_2_Pin LL_GPIO_PIN_15
#define LED_2_GPIO_Port GPIOA
#define I_LANE_2_Pin LL_GPIO_PIN_3
#define I_LANE_2_GPIO_Port GPIOB
#define I_LANE_3_Pin LL_GPIO_PIN_4
#define I_LANE_3_GPIO_Port GPIOB
#define I_LANE_4_Pin LL_GPIO_PIN_5
#define I_LANE_4_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
