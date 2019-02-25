/**
  ******************************************************************************
  * @file    CAN/CAN_Networking/Inc/main.h
  * @author  MCD Application Team
  * @version V1.5.0
  * @date    29-April-2015
  * @brief   Header for main.c module
  ******************************************************************************
  * @attention
  *
  *
  *
  ******************************************************************************
  */
// Pumpenparameter extern
// #define Drehrichtung 1
// Drehrichtung HP2: 1
// Drehrichtung HP1:0
//Variation der Endwerte

/*
#define Oszi 1
#define Oszi_UT_Toleranz 6000
#define Oszi_OT_Toleranz 6000
#define Oszi_Schrittweite 1000
*/





/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"
#include "stm32373c_eval.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
//#define debug_print       1
/* Definition for TIMx clock resources */
#define TIMx                           TIM3
#define TIMx_CLK_ENABLE                __HAL_RCC_TIM3_CLK_ENABLE
/* Definition for TIMx's NVIC */
#define TIMx_IRQn                      TIM3_IRQn
//#define TIMx_IRQHandler                TIM3_IRQHandler

#define TIMx4                           TIM4
#define TIMx4_CLK_ENABLE                __HAL_RCC_TIM4_CLK_ENABLE
/* Definition for TIMx's NVIC */
#define TIMx4_IRQn                      TIM4_IRQn


#define TIMx5                           TIM5
#define TIMx5_CLK_ENABLE                __HAL_RCC_TIM5_CLK_ENABLE
/* Definition for TIMx's NVIC */
#define TIMx5_IRQn                      TIM5_IRQn
//#define TIMx_IRQHandler                TIM3_IRQHandler
/* User can use this section to tailor CANx instance used and associated
   resources */
/* Definition for CANx clock resources */
#define CANx                            CAN
#define CANx_CLK_ENABLE()               __HAL_RCC_CAN1_CLK_ENABLE()
#define CANx_GPIO_CLK_ENABLE()          __HAL_RCC_GPIOB_CLK_ENABLE()

#define CANx_FORCE_RESET()              __HAL_RCC_CAN1_FORCE_RESET()
#define CANx_RELEASE_RESET()            __HAL_RCC_CAN1_RELEASE_RESET()
#define rs_485_dir_pin                     GPIO_PIN_7
#define rs_485_dir_port                    GPIOF
#define rs_485_pon_clock_enabel()       __HAL_RCC_GPIOF_CLK_ENABLE()

#define STM32F373     1

#ifdef STM32F373
    #define CANx_TX_PIN                    GPIO_PIN_9
    #define CANx_TX_GPIO_PORT              GPIOB
    #define CANx_TX_AF                     9
    #define CANx_RX_PIN                    GPIO_PIN_8
    #define CANx_RX_GPIO_PORT              GPIOB
    #define CANx_RX_AF                     9
    /* Definition for USARTx's NVIC */
    #define CANx_RX_IRQn                   CAN_RX0_IRQn
    #define CANx_RX_IRQHandler             CAN_RX0_IRQHandler

    #define USARTx                           USART2
    #define USARTx_CLK_ENABLE()              __HAL_RCC_USART2_CLK_ENABLE();
    #define USARTx_RX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOD_CLK_ENABLE()
    #define USARTx_TX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOD_CLK_ENABLE()

    #define USARTx_FORCE_RESET()             __HAL_RCC_USART2_FORCE_RESET()
    #define USARTx_RELEASE_RESET()           __HAL_RCC_USART2_RELEASE_RESET()

    /* Definition for USARTx Pins */
    #define USARTx_TX_PIN                    GPIO_PIN_5
    #define USARTx_TX_GPIO_PORT              GPIOD
    #define USARTx_TX_AF                     GPIO_AF7_USART2
    #define USARTx_RX_PIN                    GPIO_PIN_6
    #define USARTx_RX_GPIO_PORT              GPIOD
    #define USARTx_RX_AF                     GPIO_AF7_USART2

    /* Definition for USARTx's NVIC */
    #define USARTx_IRQn                      USART2_IRQn
    #define USARTx_IRQHandler                USART2_IRQHandler

    /* Power mode related macros */
    #define USARTx_RCC_CONFIG(__USARTxCLKSource__)   __HAL_RCC_USART2_CONFIG(__USARTxCLKSource__)
    #define RCC_USARTxCLKSOURCE_HSI                  RCC_USART2CLKSOURCE_HSI

    /* Size of Trasmission buffer */
    #define TXBUFFERSIZE                      (512)
    /* Size of Reception buffer */
    #define RXBUFFERSIZE                      1024
    /* Definition for SPIx clock resources */
    #define SPIx                             SPI1
    #define SPIx_CLK_ENABLE()                __HAL_RCC_SPI3_CLK_ENABLE()
    #define DMAx_CLK_ENABLE()                __HAL_RCC_DMA1_CLK_ENABLE()
    #define SPIx_SCK_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOA_CLK_ENABLE()
    #define SPIx_MISO_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()
    #define SPIx_MOSI_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()

    #define SPIx_FORCE_RESET()               __HAL_RCC_SPI1_FORCE_RESET()
    #define SPIx_RELEASE_RESET()             __HAL_RCC_SPI1_RELEASE_RESET()

    /* Definition for SPIx Pins */
    #define SPIx_SCK_PIN                     GPIO_PIN_5
    #define SPIx_SCK_GPIO_PORT               GPIOA
    #define SPIx_SCK_AF                      GPIO_AF5_SPI1
    #define SPIx_MISO_PIN                    GPIO_PIN_6
    #define SPIx_MISO_GPIO_PORT              GPIOA
    #define SPIx_MISO_AF                     GPIO_AF5_SPI1
    #define SPIx_MOSI_PIN                    GPIO_PIN_7
    #define SPIx_MOSI_GPIO_PORT              GPIOC
    #define SPIx_MOSI_AF                     GPIO_AF5_SPI1
    #define SPIx_CS_PIN                      GPIO_PIN_11
    #define SPIx_CS_PORT                     GPIOA

    #define SPI2_SCK_PIN                     GPIO_PIN_8
    #define SPI2_SCK_GPIO_PORT               GPIOA
    #define SPI2_SCK_AF                      GPIO_AF5_SPI1
    #define SPI2_MISO_PIN                    GPIO_PIN_10
    #define SPI2_MISO_GPIO_PORT              GPIOA
    #define SPI2_MISO_AF                     GPIO_AF5_SPI1
    #define SPI2_MOSI_PIN                    GPIO_PIN_9
    #define SPI2_MOSI_GPIO_PORT              GPIOC
    #define SPI2_MOSI_AF                     GPIO_AF5_SPI1
    #define SPI2_CS_PIN                      GPIO_PIN_12
    #define SPI_CS_PORT                      GPIOA

/* Definition for SPIx's DMA */
    #define SPIx_TX_DMA_CHANNEL              DMA1_Channel3
    #define SPIx_DMA_TX_IRQn                 DMA1_Channel3_IRQn
    #define SPIx_RX_DMA_CHANNEL              DMA1_Channel2
    #define SPIx_DMA_RX_IRQn                 DMA1_Channel2_IRQn

    /* Definition for SPIx's NVIC */
    #define SPIx_DMA_TX_IRQHandler           DMA1_Channel3_IRQHandler
    #define SPIx_DMA_RX_IRQHandler           DMA1_Channel2_IRQHandler

    /* Size of buffer */
    #define BUFFERSIZE                       (3)

  /* Exported macro ------------------------------------------------------------*/
  #define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))
  /* Exported functions ------------------------------------------------------- */


#endif

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
