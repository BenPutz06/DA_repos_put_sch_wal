/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Private variables ---------------------------------------------------------*/

LL_GPIO_InitTypeDef gpio_initstruct;



uint8_t data[8] = {0x01, 0x03, 0x02, 0x00, 0xC8, 0xB9, 0xD2};

int8_t  TickerCounter;




/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);




/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
	{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/
	

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* System interrupt init*/
  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* Peripheral interrupt init*/
  /* RCC_IRQn interrupt configuration */
  NVIC_SetPriority(RCC_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(RCC_IRQn);

  /* USER CODE BEGIN Init */

	

	

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
	
	SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;

 
	GPIO_Init();
	
	
  DMA_Init();
	
	TIM1_Init();
	
	EnablePropValve;
	EnablePWMDisplay;
 
 
	Configure_I2C_Master();
	

	TIM6_Init();						//Counter DMA Transfer Trigger

	TIM2_Init();						//Counter Input	
	
	TIM7_Init();						//zeitliche Prozessabläufe (LED blinken)
	


if (USE_TIMER1_CLOCK == 1)
{
	TIM1_Init();
}
else
{
	UART_RF_Init();
}


	  UART_PC_Init();


	ParamAddress();
	
	Daten_Init();
	

 
 Assign_Functions();


	UART_PC_RX_Start();
  /* USER CODE END 2 */
	
	UART_RF_RX_Start();
	
	Configure_ADC(); 
	Activate_ADC();





// Handle_I2C_Master(EEPROM_ADRESS, &I2CTxBuffer[2], I2CTxBuffer[1], 2);
	
	Parameter_Init();
	
	Update_Peripherals();
	
	
  LED_Power_Green.Status = 3;
	LED_Power_Green.Blinking = LED_BLINK_FAST;
	LED_Power_Green.Counter = 0;
	LED_Power_Green.BlinkCounts = -1;
	


	State.PgmFlow = BOOT;

	
	TickerCounter = -100;

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if (TickCounter >= ProgrammPeriod )			//10ms period
		{
			
			ProgramState();
			Check_LED_State();
			TickCounter = 0;
//			Get_JumperConfig();
		
			if (((++LED_Blinky) % 50) == 0)			//1s period
			{
				ADC_StartConversion();
//				LL_GPIO_TogglePin(LED_Out_GPIO_Port, LED_Out_Pin);


				
				TickerCounter++;
				
				Read_Sensor_Switches();
				

					
			}
		}

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
 * 3. This file configures the system clock as follows:
  *=============================================================================
  *                         Supported STM32F3xx device
  *-----------------------------------------------------------------------------
  *        System Clock source                    | HSE
  *-----------------------------------------------------------------------------
  *        SYSCLK(Hz)                             | 32000000
  *-----------------------------------------------------------------------------
  *        HCLK(Hz)                               | 8000000
  *-----------------------------------------------------------------------------
  *        PLL Multiplier                         | 4
  *-----------------------------------------------------------------------------
  *        AHB Prescaler                          | 1
  *-----------------------------------------------------------------------------
  *        APB2 Prescaler                         | 1
  *-----------------------------------------------------------------------------
  *        APB1 Prescaler                         | 2
  *-----------------------------------------------------------------------------
  *        USB Clock                              | DISABLE
  *-----------------------------------------------------------------------------
  *=============================================================================
  ******************************************************************************


  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_1)
  {
  }
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {

  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_1, LL_RCC_PLL_MUL_4);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
//  LL_Init10msTick(32000000);
  LL_Init1msTick(32000000);	
  LL_SetSystemCoreClock(32000000);
	LL_RCC_SetI2CClockSource(LL_RCC_I2C1_CLKSOURCE_SYSCLK);
  LL_RCC_SetTIMClockSource(LL_RCC_TIM1_CLKSOURCE_PCLK2);
  LL_RCC_SetADCClockSource(LL_RCC_ADC12_CLKSRC_PLL_DIV_1);
}






/**
  * @brief  ADC group regular overrun interruption callback
  * @note   This function is executed when ADC group regular
  *         overrun error occurs.
  * @retval None
  */
void AdcGrpRegularOverrunError_Callback(void)
{
  /* Note: Disable ADC interruption that caused this error before entering in */
  /*       infinite loop below.                                               */
  
  /* Disable ADC group regular overrun interruption */
  LL_ADC_DisableIT_OVR(ADC1);
  
  /* Error from ADC */

}

/**
  * @brief  DMA transfer error callback
  * @note   This function is executed when the transfer error interrupt
  *         is generated during DMA transfer
  * @retval None
  */
void AdcDmaTransferError_Callback()
{
  /* Error detected during DMA transfer */
  // TODO
}


/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
