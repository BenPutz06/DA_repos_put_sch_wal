/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f3xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32f3xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

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

extern uint8_t TimeTicker;


/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
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
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
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
	
	TickCounter++;


  /* USER CODE END SysTick_IRQn 0 */

  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F3xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f3xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles RCC global interrupt.
  */
void RCC_IRQHandler(void)
{
  /* USER CODE BEGIN RCC_IRQn 0 */

  /* USER CODE END RCC_IRQn 0 */
  /* USER CODE BEGIN RCC_IRQn 1 */

  /* USER CODE END RCC_IRQn 1 */
}



/**
  * @brief This function handles DMA1 channel3 global interrupt.
	*         Used for TIM6 OVFLW 
  */
void DMA1_Channel3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel3_IRQn 0 */
	/* Check whether DMA transfer complete caused the DMA interruption */
  if(LL_DMA_IsActiveFlag_TC3(DMA1) == 1)
  {
    /* Clear flag DMA transfer complete */
    LL_DMA_ClearFlag_TC3(DMA1);
    
    /* Call interruption treatment function */
    CounterReadTransferComplete_Callback();
  }

 
}



/**
  * @brief This function handles DMA1 channel4 global interrupt.
	*         Used for UART1 Tx  
  */
void DMA1_Channel4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel3_IRQn 0 */
	/* Check whether DMA transfer complete caused the DMA interruption */
  if(LL_DMA_IsActiveFlag_TC4(DMA1) == 1)
  {
    /* Clear flag DMA transfer complete */
    LL_DMA_ClearFlag_TC4(DMA1);
		
    
    /* Call interruption treatment function */
    RF_UART_TXTransferComplete_Callback();
  }

  /* USER CODE END DMA1_Channel3_IRQn 0 */

  /* USER CODE BEGIN DMA1_Channel3_IRQn 1 */

  /* USER CODE END DMA1_Channel3_IRQn 1 */
}


/**
  * @brief This function handles DMA1 channel4 global interrupt.
	*         Used for UART1 Rx  
  */
void DMA1_Channel5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel3_IRQn 0 */
	/* Check whether DMA transfer complete caused the DMA interruption */
  if(LL_DMA_IsActiveFlag_TC5(DMA1) == 1)
  {
    
    LL_DMA_ClearFlag_TC5(DMA1);							// Clear flag DMA transfer complete
    
    /* Call interruption treatment function */
		CheckUART_RF_RX(0);
		//DMA_Tester1++;
  }
	
	if (LL_DMA_IsActiveFlag_HT5(DMA1) == 1)
	{
		LL_DMA_ClearFlag_HT5(DMA1);							// Clear flag DMA Half Transfer complete 
			CheckUART_RF_RX(0);	
			DMA_Tester2++;		
	}

  /* USER CODE END DMA1_Channel3_IRQn 0 */

  /* USER CODE BEGIN DMA1_Channel3_IRQn 1 */

  /* USER CODE END DMA1_Channel3_IRQn 1 */
}

	


void UART_RF_IRQHandler (void)
{
	
  /* Check RXNE flag value in ISR register */
  if(LL_USART_IsActiveFlag_RXNE(UART_RF) && LL_USART_IsEnabledIT_RXNE(UART_RF))
  {
    /* RXNE flag will be cleared by reading of RDR register (done in call) */
    /* Call function in charge of handling Character reception */
    CheckUART_RF_RX(0);
		DMA_Tester3++;
  }
  else if (LL_USART_IsEnabledIT_IDLE(UART_RF) && LL_USART_IsActiveFlag_IDLE(UART_RF))
//
	{
		LL_USART_ClearFlag_IDLE(UART_RF);

				CheckUART_RF_RX(1);
				DMA_Tester4++;
		
	}
//  {
    /* Call Error function */
//    Error_SPS_Callback();
// }
	else if (LL_USART_IsEnabledIT_TC(UART_RF) && LL_USART_IsActiveFlag_TC(UART_RF))
	{	
//			LL_USART_EnableDirectionRx(UART_SPS);
	}

}



void UART_PC_IRQHandler (void)
{
	
  /* Check RXNE flag value in ISR register */
  if(LL_USART_IsActiveFlag_RXNE(UART_PC) && LL_USART_IsEnabledIT_RXNE(UART_PC))
  {
    /* RXNE flag will be cleared by reading of RDR register (done in call) */
    /* Call function in charge of handling Character reception */
    USART_PC_CharReception_Callback();
  }
  else
  {
    /* Call Error function */
    Error_PC_Callback();
  }

}

/**
* @brief  This function handles TIM2 update interrupt.
* @param  None
* @retval None
*/



/**
* @brief  This function handles TIM2 update interrupt.
* @param  None
* @retval None
*/
void TIM2_IRQHandler(void)
{
  /* Check whether update interrupt is pending */
  if(LL_TIM_IsActiveFlag_UPDATE(TIM2) == 1)
  {
    /* Clear the update interrupt flag*/
    LL_TIM_ClearFlag_UPDATE(TIM2);
  }
  
	fCountOvflw = 0x01;    					//set overflow Bit
  /* TIM2 update interrupt processing */
 // TimerUpdate_Callback();
}


/**
* @brief  This function handles TIM7 update interrupt.
*         Trigger-Time is 10Hz
* @param  None
* @retval None
*/
void TIM7_IRQHandler(void)
{
	
if(LL_TIM_IsActiveFlag_UPDATE(TIM7) == 1)
  {
    /* Clear the update interrupt flag*/
    LL_TIM_ClearFlag_UPDATE(TIM7);
	  //LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_0);
  }
  
  /* TIM2 update interrupt processing */
  //LL_GPIO_TogglePin(GPIOA, LL_GPIO_PIN_12);
	
	 LED_BLINKING_Update(0);
	--LED_Blinking_Timer;
	
	if ((--RFTimeOutCounter) <= 0) RFTimeOutCounter = 0;
	
	if (RxRFPack.fDataReceiving > 0)
	{
		 if ((RFRcvTimeOutTimer--) <= 0)
		 {
			 RFRcvErrorReset();
			 
		 }

	 }
	/*	 if (LED_Power.Counter == 0)
	 {
			LED_POWER_RED();
	 };
	 else if (LED_Power.Counter == 1)
	 LED_Power.Counter++;
	 if LED_POWER.Counter ==	
	 else if (LED_Power.Status == 2) LED_POWER_GREEN();

*/ 
	
	//startADC = 0x01;
	//LL_ADC_REG_StartConversionSWStart(ADC1);
}



/**
  * Brief   This function handles I2C1 (Master) interrupt request.
  * Param   None
  * Retval  None
  */
void I2C1_EV_IRQHandler(void)
{
  /* Check RXNE flag value in ISR register */
  if(LL_I2C_IsActiveFlag_RXNE(I2C1))
  {
    /* Call function Master Reception Callback */
    Read_EEPROM_Reception_Callback();
  }
  /* Check STOP flag value in ISR register */
  else if(LL_I2C_IsActiveFlag_STOP(I2C1))
  {
    /* End of Transfer */
    LL_I2C_ClearFlag_STOP(I2C1);

    /* Call function Master Complete Callback */
		READ_EEPROM_Complete_Callback();
  }
  else
  {
    /* Call Error function */
    Error_I2C_Callback();
  }
}

/**
  * Brief   This function handles I2C1 (Master) error interrupt request.
  * Param   None
  * Retval  None
  */
void I2C1_ER_IRQHandler(void)
{
  /* Call Error function */
  Error_I2C_Callback();
}





/**
  * @brief  This function handles ADC2 interrupt request.
  * @param  None
  * @retval None
  */
void ADC1_2_IRQHandler(void)
{
  // Check whether ADC group regular overrun caused the ADC interruption 
  if(LL_ADC_IsActiveFlag_OVR(ADC1) != 0)
  {

    LL_ADC_ClearFlag_OVR(ADC1);								    // Clear flag ADC group regular overrun 
    
    AdcGrpRegularOverrunError_Callback();			    // Call interruption treatment function 

  }
}

/**
  * @brief  This function handles DMA1 interrupt request.
  * @param  None
  * @retval None
  */
void DMA1_Channel1_IRQHandler(void)
{
  // Check whether DMA transfer complete caused the DMA interruption */
  if(LL_DMA_IsActiveFlag_TC1(DMA1) == 1)
  {
    /* Clear flag DMA transfer complete */
    LL_DMA_ClearFlag_TC1(DMA1);
    
    /* Call interruption treatment function */
    AdcDmaTransferComplete_Callback();
  }
  

  
  /* Check whether DMA transfer error caused the DMA interruption */
  if(LL_DMA_IsActiveFlag_TE1(DMA1) == 1)
  {
    /* Clear flag DMA transfer error */
    LL_DMA_ClearFlag_TE1(DMA1);
    
    /* Call interruption treatment function */
    AdcDmaTransferError_Callback();
  }
}

