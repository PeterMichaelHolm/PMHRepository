/**
  ******************************************************************************
  * @file    CAN/CAN_Networking/Src/stm32f3xx_it.c
  * @author  MCD Application Team
  * @version V1.5.0
  * @date    29-April-2015
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f3xx_it.h"

/** @addtogroup STM32F3xx_HAL_Examples
  * @{
  */

/** @addtogroup CAN_Networking
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern CAN_HandleTypeDef    CanHandle;
extern UART_HandleTypeDef UartHandle;
extern void uart_rx_tx_irq(void);
extern SPI_HandleTypeDef SpiHandle;
extern TIM_HandleTypeDef    TimHandle;
extern TIM_HandleTypeDef    Tim4Handle;
extern void TIM4_Handler(void);
extern void TIM5_Handler(void);
extern void timer3_int(void);
extern int timer3;
extern int motor_soll_tackt;
extern int motor_ist_tackt;
extern int motor_freigabe;
extern int motor_position2;

/* Private function prototypes -----------------------------------------------*/
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef *CanHandle);

extern void BSP_LED_Toggle(Led_TypeDef Led);
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
//int i;
  /* Go to infinite loop when Hard Fault exception occurs */
 //i=0;
  while (1)
  { 
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
  HAL_IncTick();
}

/******************************************************************************/
/*                 STM32F3xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f3xx.s).                                               */
/******************************************************************************/

/**
* @brief  This function handles CAN1 RX0 interrupt request.
* @param  None
* @retval None
*/

void CANx_RX_IRQHandler(void)
{
//int i;
//i++;
  HAL_CAN_IRQHandler(&CanHandle);
// HAL_CAN_RxCpltCallback(&CanHandle);
//  printf("rx\n");
}


/**

  * @brief  This function handles TIM interrupt request.

  * @param  None

  * @retval None

  */

void TIM3_IRQHandler(void)

{
 HAL_TIM_IRQHandler(&TimHandle);
 // TIM3_IRQHandler();
  timer3++;
  timer3_int();


}

void TIM4_IRQHandler(void) {

  TIM4_Handler();
}


void TIM5_IRQHandler(void) {
//  TIM5_Handler();
}


void USARTx_IRQHandler(void)

{

//  HAL_UART_IRQHandler(&UartHandle);
 uart_rx_tx_irq(); 
 //HAL_UART_IRQHandler(&UartHandle);
}






void SPIx_DMA_RX_IRQHandler(void)
{
    HAL_DMA_IRQHandler(SpiHandle.hdmarx);
}

/**
 * @brief  This function handles DMA Tx interrupt request.
 * @param  None
 * @retval None
 */
void SPIx_DMA_TX_IRQHandler(void)
{
    HAL_DMA_IRQHandler(SpiHandle.hdmatx);
}

/**
 * @}
  */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
