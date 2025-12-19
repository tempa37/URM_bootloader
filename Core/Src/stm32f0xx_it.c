/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f0xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define buf_size_rx             100
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
extern void MY_UARTEx_RxEventCallback(uint16_t Size);
extern uint8_t receive_buf[buf_size_rx];
/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M0 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVC_IRQn 0 */

  /* USER CODE END SVC_IRQn 0 */
  /* USER CODE BEGIN SVC_IRQn 1 */

  /* USER CODE END SVC_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */

  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f0xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line 4 to 15 interrupts.
  */
void EXTI4_15_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_15_IRQn 0 */

  /* USER CODE END EXTI4_15_IRQn 0 */
  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_11) != RESET)
  {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_11);
    /* USER CODE BEGIN LL_EXTI_LINE_11 */

    /* USER CODE END LL_EXTI_LINE_11 */
  }
  /* USER CODE BEGIN EXTI4_15_IRQn 1 */

  /* USER CODE END EXTI4_15_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel 2 and 3 interrupts.
  */
void DMA1_Channel2_3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel2_3_IRQn 0 */

  /* USER CODE END DMA1_Channel2_3_IRQn 0 */

  /* USER CODE BEGIN DMA1_Channel2_3_IRQn 1 */

  /* USER CODE END DMA1_Channel2_3_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
    /* --- IDLE: конец приёма пакета ------------------------------------- */
    if (LL_USART_IsActiveFlag_IDLE(USART1))
    {
        LL_USART_ClearFlag_IDLE(USART1);       /* 1. сброс флага IDLE      */
        (void)USART1->ISR;                     /* 2. чтение ISR + RDR      */
        (void)USART1->RDR;                     /*    (по errata ST)        */

        /* 3. останавливаем DMA‑канал, узнаём, сколько приняли */
        LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_3);
        uint16_t rx_len = buf_size_rx - LL_DMA_GetDataLength(DMA1, LL_DMA_CHANNEL_3);

        /* 4. обработай приём здесь */
       // MY_UARTEx_RxEventCallback(receive_buf, rx_len);    /* <- твоя функция */
        MY_UARTEx_RxEventCallback(rx_len);
        /* 5. снова ждём новый пакет */
        LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_3, buf_size_rx);
        LL_DMA_EnableChannel (DMA1, LL_DMA_CHANNEL_3);
    }

    /* --- Обработка ошибок ------------------------------------------------ */
    if (LL_USART_IsActiveFlag_FE (USART1) ||   /* Framing error   */
        LL_USART_IsActiveFlag_NE (USART1) ||   /* Noise error     */
        LL_USART_IsActiveFlag_ORE(USART1))     /* Overrun error   */
    {
        /* читаем RDR, чтобы сбросить ошибку, и перезапускаем приём */
        (void)USART1->RDR;
        LL_USART_ClearFlag_FE (USART1);
        LL_USART_ClearFlag_NE (USART1);
        LL_USART_ClearFlag_ORE(USART1);
                          
    }
}
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
