/** 
  ******************************************************************************
  * @file    x_nucleo_ihm01a1_stm32f0xx.h
  * @author  IPC Rennes
  * @version V1.7.0
  * @date    August 11th, 2016  
  * @brief   Header for BSP driver for x-nucleo-ihm01a1 Nucleo extension board 
  *  (based on L6474)
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************  
  */ 
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef X_NUCLEO_IHM01A1_STM32F0XX_H
#define X_NUCLEO_IHM01A1_STM32F0XX_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_nucleo.h"
   
/** @addtogroup BSP
  * @{
  */   
   
/** @addtogroup X_NUCLEO_IHM01A1_STM32F0XX
  * @{   
  */   
   
/* Exported Constants --------------------------------------------------------*/
   
/** @defgroup IHM01A1_Exported_Constants IHM01A1 Exported Constants
  * @{
  */   
   
/******************************************************************************/
/* USE_STM32F0XX_NUCLEO                                                       */
/******************************************************************************/

 /** @defgroup Constants_For_STM32F0XX_NUCLEO   Constants For STM32F0XX NUCLEO
* @{
*/   
/// Interrupt line used for L6474 FLAG
#define EXTI_MCU_LINE_IRQn           (EXTI4_15_IRQn)

/// Timer used for PWM1
#define BSP_MOTOR_CONTROL_BOARD_TIMER_PWM1      (TIM14)//(TIM3)

/// Timer used for PWM2
#define BSP_MOTOR_CONTROL_BOARD_TIMER_PWM2      (TIM14)

/// Timer used for PWM3
#define BSP_MOTOR_CONTROL_BOARD_TIMER_PWM3      (TIM15)

/// Channel Timer used for PWM1
#define BSP_MOTOR_CONTROL_BOARD_CHAN_TIMER_PWM1      (TIM_CHANNEL_1)//(TIM_CHANNEL_2)

/// Channel Timer used for PWM2
#define BSP_MOTOR_CONTROL_BOARD_CHAN_TIMER_PWM2      (TIM_CHANNEL_1)

/// Channel Timer used for PWM3
#define BSP_MOTOR_CONTROL_BOARD_CHAN_TIMER_PWM3      (TIM_CHANNEL_1)

/// HAL Active Channel Timer used for PWM1
#define BSP_MOTOR_CONTROL_BOARD_HAL_ACT_CHAN_TIMER_PWM1      (HAL_TIM_ACTIVE_CHANNEL_1)//(HAL_TIM_ACTIVE_CHANNEL_2)

/// HAL Active Channel Timer used for PWM2
#define BSP_MOTOR_CONTROL_BOARD_HAL_ACT_CHAN_TIMER_PWM2      (HAL_TIM_ACTIVE_CHANNEL_1)

/// HAL Active Channel Timer used for PWM3
#define BSP_MOTOR_CONTROL_BOARD_HAL_ACT_CHAN_TIMER_PWM3      (HAL_TIM_ACTIVE_CHANNEL_1)

/// Timer Clock Enable for PWM1
#define __BSP_MOTOR_CONTROL_BOARD_TIMER_PWM1_CLCK_ENABLE()    __HAL_RCC_TIM14_CLK_ENABLE()//__HAL_RCC_TIM3_CLK_ENABLE()

/// Timer Clock Enable for PWM2
#define __BSP_MOTOR_CONTROL_BOARD_TIMER_PWM2_CLCK_ENABLE()    __HAL_RCC_TIM14_CLK_ENABLE()

/// Timer Clock Enable for PWM1
#define __BSP_MOTOR_CONTROL_BOARD_TIMER_PWM3_CLCK_ENABLE()    __HAL_RCC_TIM15_CLK_ENABLE()

/// Timer Clock Disable for PWM1
#define __BSP_MOTOR_CONTROL_BOARD_TIMER_PWM1_CLCK_DISABLE()    __HAL_RCC_TIM14_CLK_DISABLE()//__HAL_RCC_TIM3_CLK_DISABLE()

/// Timer Clock Disable for PWM2
#define __BSP_MOTOR_CONTROL_BOARD_TIMER_PWM2_CLCK_DISABLE()    __HAL_RCC_TIM14_CLK_DISABLE()

/// Timer Clock Disable for PWM3
#define __BSP_MOTOR_CONTROL_BOARD_TIMER_PWM3_CLCK_DISABLE()    __HAL_RCC_TIM15_CLK_DISABLE()

/// PWM1 global interrupt
#define BSP_MOTOR_CONTROL_BOARD_PWM1_IRQn   (TIM14_IRQn)//(TIM3_IRQn)

/// PWM2 global interrupt
#define BSP_MOTOR_CONTROL_BOARD_PWM2_IRQn   (TIM14_IRQn)

/// PWM3 global interrupt
#define BSP_MOTOR_CONTROL_BOARD_PWM3_IRQn   (TIM15_IRQn)

/// PWM1 GPIO alternate function 
#define BSP_MOTOR_CONTROL_BOARD_AFx_TIMx_PWM1  (GPIO_AF1_TIM3)

#ifndef BSP_MOTOR_CONTROL_BOARD_USE_SPI2
/// SPI SCK AF
#define SPIx_SCK_AF    (NUCLEO_SPIx_MISO_MOSI_AF)
#else /* #ifndef BSP_MOTOR_CONTROL_BOARD_USE_SPI2 */
/// SPI SCK AF
#define SPIx_SCK_AF    (GPIO_AF0_SPI2)
#endif /* #ifndef BSP_MOTOR_CONTROL_BOARD_USE_SPI2 */

/// PWM1 frequency rescaler (1 for HW PWM, 2 for SW PWM)
#define BSP_MOTOR_CONTROL_BOARD_PWM1_FREQ_RESCALER    (2)//(1)
/// PWM2 frequency rescaler (1 for HW PWM, 2 for SW PWM)
#define BSP_MOTOR_CONTROL_BOARD_PWM2_FREQ_RESCALER    (2)   
/// PWM3 frequency rescaler (1 for HW PWM, 2 for SW PWM)   
#define BSP_MOTOR_CONTROL_BOARD_PWM3_FREQ_RESCALER    (2)   
   
 /**
* @}
*/

/******************************************************************************/
/* Independent plateform definitions                                          */
/******************************************************************************/

   /** @defgroup Constants_For_All_Nucleo_Platforms Constants For All Nucleo Platforms
* @{
*/   

/// GPIO Pin used for the L6474 flag pin
#define BSP_MOTOR_CONTROL_BOARD_FLAG_PIN   (GPIO_PIN_10)
/// GPIO port used for the L6474 flag pin
#define BSP_MOTOR_CONTROL_BOARD_FLAG_PORT   (GPIOA)

/// GPIO Pin used for the L6474 step clock pin of device 0
#define BSP_MOTOR_CONTROL_BOARD_PWM_1_PIN  (GPIO_PIN_7)
/// GPIO Port used for the L6474 step clock pin of device 0
#define BSP_MOTOR_CONTROL_BOARD_PWM_1_PORT  (GPIOC)

/// GPIO Pin used for the L6474 step clock pin of device 1
#define BSP_MOTOR_CONTROL_BOARD_PWM_2_PIN  (GPIO_PIN_3)
/// GPIO port used for the L6474 step clock pin of device 1
#define BSP_MOTOR_CONTROL_BOARD_PWM_2_PORT  (GPIOB)

/// GPIO Pin used for the L6474 step clock pin of device 2
#define BSP_MOTOR_CONTROL_BOARD_PWM_3_PIN   (GPIO_PIN_10)
/// GPIO port used for the L6474 step clock pin of device 2
#define BSP_MOTOR_CONTROL_BOARD_PWM_3_PORT  (GPIOB)

/// GPIO Pin used for the L6474 direction pin of device 0
#define BSP_MOTOR_CONTROL_BOARD_DIR_1_PIN  (GPIO_PIN_8)
/// GPIO port used for the L6474 direction pin of device 0
#define BSP_MOTOR_CONTROL_BOARD_DIR_1_PORT  (GPIOA)

/// GPIO Pin used for the L6474 direction pin of device 1
#define BSP_MOTOR_CONTROL_BOARD_DIR_2_PIN   (GPIO_PIN_5)
/// GPIO port used for the L6474 direction pin of device 1
#define BSP_MOTOR_CONTROL_BOARD_DIR_2_PORT  (GPIOB)

/// GPIO Pin used for the L6474 direction pin of device 2
#define BSP_MOTOR_CONTROL_BOARD_DIR_3_PIN   (GPIO_PIN_4)
/// GPIO port used for the L6474 direction pin of device 2
#define BSP_MOTOR_CONTROL_BOARD_DIR_3_PORT  (GPIOB)

/// GPIO Pin used for the L6474 reset pin
#define BSP_MOTOR_CONTROL_BOARD_RESET_PIN  (GPIO_PIN_9)
/// GPIO port used for the L6474 reset pin
#define BSP_MOTOR_CONTROL_BOARD_RESET_PORT (GPIOA)

/// GPIO Pin used for the L6474 SPI chip select pin
#define BSP_MOTOR_CONTROL_BOARD_CS_PIN  (GPIO_PIN_6)
/// GPIO port used for the L6474 SPI chip select  pin
#define BSP_MOTOR_CONTROL_BOARD_CS_PORT (GPIOB)

/* Definition for SPIx clock resources */

#ifndef BSP_MOTOR_CONTROL_BOARD_USE_SPI2
/* Default SPI is SPI1 */

/// Used SPI
#define SPIx                             (SPI1)

/// SPI clock enable
#define SPIx_CLK_ENABLE()                __HAL_RCC_SPI1_CLK_ENABLE()

/// SPI SCK enable
#define SPIx_SCK_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOA_CLK_ENABLE()

/// SPI MISO enable
#define SPIx_MISO_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE() 

/// SPI MOSI enable
#define SPIx_MOSI_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE() 

/// SPI Force reset
#define SPIx_FORCE_RESET()               __HAL_RCC_SPI1_FORCE_RESET()

/// SPI Release reset
#define SPIx_RELEASE_RESET()             __HAL_RCC_SPI1_RELEASE_RESET()

/// SPI SCK pin
#define SPIx_SCK_PIN                     (GPIO_PIN_5)

/// SPI SCK port
#define SPIx_SCK_GPIO_PORT               (GPIOA)


/// SPI MISO pin 
#define SPIx_MISO_PIN                    (GPIO_PIN_6)

/// SPI MISO port
#define SPIx_MISO_GPIO_PORT              (GPIOA)

/// SPI MOSI pin
#define SPIx_MOSI_PIN                    (GPIO_PIN_7)

/// SPI MOSI port
#define SPIx_MOSI_GPIO_PORT              (GPIOA)

#else  /* USE SPI2 */

/// Used SPI
#define SPIx                             (SPI2)

/// SPI clock enable
#define SPIx_CLK_ENABLE()                __HAL_RCC_SPI2_CLK_ENABLE()

/// SPI SCK enable
#define SPIx_SCK_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOB_CLK_ENABLE()

/// SPI MISO enable
#define SPIx_MISO_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE() 

/// SPI MOSI enable
#define SPIx_MOSI_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE() 

/// SPI Force reset
#define SPIx_FORCE_RESET()               __SPI2_FORCE_RESET()

/// SPI Release reset
#define SPIx_RELEASE_RESET()             __SPI2_RELEASE_RESET()

/// SPI SCK pin
#define SPIx_SCK_PIN                     (GPIO_PIN_13)

/// SPI SCK port
#define SPIx_SCK_GPIO_PORT               (GPIOB)

/// SPI MISO pin 
#define SPIx_MISO_PIN                    (GPIO_PIN_14)

/// SPI MISO port
#define SPIx_MISO_GPIO_PORT              (GPIOB)

/// SPI MISO AF 
#define SPIx_MISO_AF                     (SPIx_SCK_AF)
  
/// SPI MOSI pin
#define SPIx_MOSI_PIN                    (GPIO_PIN_15)

/// SPI MOSI port
#define SPIx_MOSI_GPIO_PORT              (GPIOB)

#endif
   
/// SPI MISO AF 
#define SPIx_MISO_AF                     (SPIx_SCK_AF)

/// SPI MOSI AF
#define SPIx_MOSI_AF                     (SPIx_SCK_AF)
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* X_NUCLEO_IHM01A1_STM32F0XX_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
