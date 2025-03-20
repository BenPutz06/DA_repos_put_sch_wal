/******************** (C) COPYRIGHT Benjamin Putz ********************
* File Name          : Periph_init.c
* Author             : Benjamim Putz
* Date First Issued  : 20/12/2024
* Description        : Initialize Peripherie of STM32
*
********************************************************************************
* History:
* 
*******************************************************************************
*
********************************************************************************
* Detailed Description
* 
*******************************************************************************
*******************************************************************************/


/* Includes ------------------------------------------------------------------*/

#include "Main.h"

//#include "Parameter.h"
//#include "Declarations.h"
#include "Periph_Init.h"
#include "stm32f3xx_ll_dac.h"
#include "stm32f3xx_ll_usart.h"
//#include "Functions.h"
//#include "Periph_Funct.h"
//#include "misc.h"

//#include "stm32l1xx_lcd.h"


/* Define --------------------------------------------------------------------*/

#define ADC1_DR_ADDRESS    ((uint32_t)0x40012458)


/**
  * Enable DMA controller clock
  */
void DMA_Init(void)
{

  /* Init with LL driver */
  /* DMA controller clock enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

  /* DMA interrupt init */ 
	//  UART1 TX DMA Init DMA1_Channel4_IRQn interrupt configuration 

  NVIC_SetPriority(DMA1_Channel4_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),5, 0));
  NVIC_EnableIRQ(DMA1_Channel4_IRQn);

	//  UART1 RX DMA Init DMA1_Channel5_IRQn interrupt configuration 	
  NVIC_SetPriority(DMA1_Channel5_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),4, 0));			//a bit lower Priority	
  NVIC_EnableIRQ(DMA1_Channel5_IRQn);

	

  /* (3) Configure the DMA functional parameters for transmission */
  LL_DMA_ConfigTransfer(DMA1, LL_DMA_CHANNEL_4, 
                        LL_DMA_DIRECTION_MEMORY_TO_PERIPH | 
                        LL_DMA_PRIORITY_HIGH              | 
                        LL_DMA_MODE_NORMAL                | 
                        LL_DMA_PERIPH_NOINCREMENT         | 
                        LL_DMA_MEMORY_INCREMENT           | 
                        LL_DMA_PDATAALIGN_BYTE            | 
                        LL_DMA_MDATAALIGN_BYTE);
  LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_4,
                         (uint32_t) UART_RF_TX_DMA_Buffer,
                         LL_USART_DMA_GetRegAddr(UART_RF, LL_USART_DMA_REG_DATA_TRANSMIT),
                         LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_CHANNEL_4));
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_4, TXRF_BUFFER_SIZE);										//size defined in Parameter.h

  /* (4) Configure the DMA functional parameters for reception */
  LL_DMA_ConfigTransfer(DMA1, LL_DMA_CHANNEL_5, 
                        LL_DMA_DIRECTION_PERIPH_TO_MEMORY | 
                        LL_DMA_PRIORITY_HIGH              | 
                        LL_DMA_MODE_CIRCULAR              | 
                        LL_DMA_PERIPH_NOINCREMENT         | 
                        LL_DMA_MEMORY_INCREMENT           | 
                        LL_DMA_PDATAALIGN_BYTE            | 
                        LL_DMA_MDATAALIGN_BYTE);
  LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_5,
                         LL_USART_DMA_GetRegAddr(UART_RF, LL_USART_DMA_REG_DATA_RECEIVE),
                         (uint32_t)UART_RF_RX_DMA_Buffer,
                         LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_CHANNEL_5));
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_5, RXRF_BUFFER_SIZE);											// size defined in Parameter.h

  /* (5) Enable DMA transfer complete/error interrupts  */
  LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_4);
  LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_4);
  LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_5);
//	LL_DMA_EnableIT_HT(DMA1, LL_DMA_CHANNEL_5);
  LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_5);
	
										 

  // Configure the NVIC to handle TIM2 update interrupt 
	//  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_3);


	
  NVIC_SetPriority(TIM2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),3, 0));
  NVIC_EnableIRQ(TIM2_IRQn);
	
  NVIC_SetPriority(TIM7_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),8, 0));
  NVIC_EnableIRQ(TIM7_IRQn);



 /* DMA interrupt init */ 
	/* TIM6 DMA Init DMA1_Channel3_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Channel3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),4, 0));
  NVIC_EnableIRQ(DMA1_Channel3_IRQn);

	
  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_3, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PRIORITY_MEDIUM);

  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MODE_CIRCULAR);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PERIPH_NOINCREMENT);

//	LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MEMORY_NOINCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PDATAALIGN_WORD);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MDATAALIGN_WORD);

  LL_SYSCFG_SetRemapDMA_TIM(LL_SYSCFG_TIM6_RMP_DMA1_CH3);
	
	LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_3, (uint32_t)&TIM2->CNT,
                         (uint32_t)&DMA_CntBuffer, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
  
  // Set DMA transfer size  Circuit Buffer   - only 1 Data
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_3, 1);
											 
  // Enable DMA transfer interruption: transfer complete
  LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_3);
										 
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_3);



	// configures DMA for transfer of data from ADC1 CH1

  /* Configure NVIC to enable DMA interruptions - lower Priority*/
  NVIC_SetPriority(DMA1_Channel1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),8,0)); /* DMA IRQ lower priority than ADC IRQ */
  NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  
  
  /* Configure the DMA transfer */
  /*  - :        */
  /*    DMA unlimited requests.                                               */
  /*  - D                      */
  /*  - D.                        */
  /*  - DMA transfer from ADC by half-word to match with ADC configuration:   */
  /*    ADC resolution 12 bits.                                               */
  /*  - DMA transfer to memory by half-word to match with ADC conversion data */
  /*    buffer variable type: half-word.                                      */
  LL_DMA_ConfigTransfer(DMA1,
                        LL_DMA_CHANNEL_1,
                        LL_DMA_DIRECTION_PERIPH_TO_MEMORY |
                        LL_DMA_MODE_CIRCULAR              |							// DMA transfer in circular mode to match with ADC configuration
                        LL_DMA_PERIPH_NOINCREMENT         |							// DMA transfer from ADC without address increment.
                        LL_DMA_MEMORY_INCREMENT           |							// DMA transfer to memory with address increment
                        LL_DMA_PDATAALIGN_HALFWORD        |
                        LL_DMA_MDATAALIGN_HALFWORD        |
                        LL_DMA_PRIORITY_HIGH               );
  
  // Set DMA transfer addresses of source and destination */
  LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_1,LL_ADC_DMA_GetRegAddr(ADC1, LL_ADC_DMA_REG_REGULAR_DATA), 
																(uint32_t)&PumpPressRawBuff[0], LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
  
  // Set DMA transfer size 
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, ADC_CONVERTED_DATA_BUFFER_SIZE);
  
  // Enable DMA transfer interruption: transfer complete */
  LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);
  
//   Enable DMA transfer interruption: half transfer 
//  LL_DMA_EnableIT_HT(DMA1, LL_DMA_CHANNEL_1);
  
  // Enable DMA transfer interruption: transfer error */
  LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_1);
  
  // Enable the DMA transfer 
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
}


/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
void GPIO_Init(void)
{

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
	
	

  /**/
  LL_GPIO_ResetOutputPin(LED_Out_GPIO_Port, LED_Out_Pin);

  /**/
  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTC, LL_SYSCFG_EXTI_LINE13);

  /**/
  LL_GPIO_SetPinPull(B1_GPIO_Port, B1_Pin, LL_GPIO_PULL_NO);

  /**/
  LL_GPIO_SetPinMode(B1_GPIO_Port, B1_Pin, LL_GPIO_MODE_INPUT);

 
  //----------------------
	//   Output Ports
	//----------------------
	

  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;

////Schubboden
	GPIO_InitStruct.Pin = FEEDER_FWD_Pin;
	LL_GPIO_Init(FEEDER_FWD_Port, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = FEEDER_REW_Pin;
	LL_GPIO_Init(FEEDER_REW_Port, &GPIO_InitStruct);
	
//Proportional Ventile
	GPIO_InitStruct.Pin = PROP_VALVE_DIS_Pin;
	LL_GPIO_Init(PROP_VALVE_DIS_Port, &GPIO_InitStruct);
	

// LED Anzeige ProportionalVentile
	GPIO_InitStruct.Pin = PWM_DISPLAY_DIS_Pin;
	LL_GPIO_Init(PWM_DISPLAY_DIS_Port, &GPIO_InitStruct);

	
	// LED

  GPIO_InitStruct.Pin = LED_Out_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED_Out_GPIO_Port, &GPIO_InitStruct);



	//----------------------
	//   Input Ports
	//----------------------

  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;

	//Config Jumper Input GPIOs 0,1,2 on PortB
	GPIO_InitStruct.Pin = LL_GPIO_PIN_14 | LL_GPIO_PIN_15;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	//Schubboden Fehler 
	GPIO_InitStruct.Pin =  FEEDER_ERR_FWD_Pin;
	LL_GPIO_Init(FEEDER_ERR_FWD_Port, &GPIO_InitStruct);
	GPIO_InitStruct.Pin =  FEEDER_ERR_REW_Pin;
	LL_GPIO_Init(FEEDER_ERR_REW_Port, &GPIO_InitStruct);

	//Zellenrad Fehler
	GPIO_InitStruct.Pin = ROTARY_ERROR_Pin;
	LL_GPIO_Init(ROTARY_ERROR_Port, &GPIO_InitStruct);

	//Zubringerschnecke Fehler
	GPIO_InitStruct.Pin = SCREW_ERROR_Pin;
	LL_GPIO_Init(SCREW_ERROR_Port, &GPIO_InitStruct);
	
}

/**
  * @brief TIM1 Initialization Function
	*	@note 	TIM1 is Clockdriver, PWM Output for 
	*					TIM1.CH2 (PA9)... Rotary Valve ..Prop Ventil - Zellenradschleuse
	*					TIM1.CH3 (PA10)... Screw Valve  ..Prop Ventil - Zubringerschnecke
*					TIM1 Clock is 32MHz, PWM 50:50 for Prop Ventil idle
*						both Valve had same clock : 1kHz
*
  * @param None
  * @retval None
  */
void TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};
//  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};
//  LL_TIM_BDTR_InitTypeDef TIM_BDTRInitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};


  //*************************
  // Time base configuration 
	// PWM frequence for Proportional Valve = 1kHz
	// TIM1 Clock frequence is 32MHz
  //**************************
  
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);										// Peripheral clock enable 

  TIM_InitStruct.Prescaler = (32000000/(PWM_FREQUENCY*PWM_RESOLUTION))-1;																		// Frequenz = 400kHz ... Grundfrequenz
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = PWM_RESOLUTION-1;																		//Frequenz = 1khz  .. Aufl�sung des PWMS
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  TIM_InitStruct.RepetitionCounter = 0;
  LL_TIM_Init(TIM1, &TIM_InitStruct);

  //*********************************
  // Output waveform configuration : Set output channel 2& 3 in PWM1 mode
  /*********************************/

  LL_TIM_OC_SetMode(TIM1,  LL_TIM_CHANNEL_CH2,  LL_TIM_OCMODE_PWM1);
  LL_TIM_OC_SetMode(TIM1,  LL_TIM_CHANNEL_CH3,  LL_TIM_OCMODE_PWM1);

  // TIM1 channel 2&3 configuration:  	
  LL_TIM_OC_ConfigOutput(TIM1, LL_TIM_CHANNEL_CH2, LL_TIM_OCPOLARITY_HIGH | LL_TIM_OCIDLESTATE_HIGH); 
	LL_TIM_OC_ConfigOutput(TIM1, LL_TIM_CHANNEL_CH3, LL_TIM_OCPOLARITY_HIGH | LL_TIM_OCIDLESTATE_HIGH); 

  // Set PWM duty cycle  for Zellenradschleuse
  LL_TIM_OC_SetCompareCH2(TIM1, ((LL_TIM_GetAutoReload(TIM1) - 1)/2));

	// Set PWM duty cycle  for Zubringerschnecke
  LL_TIM_OC_SetCompareCH3(TIM1, ((LL_TIM_GetAutoReload(TIM1) - 1)/2));
  
  // Enable register preload for TIM1 channel 2&3 
  LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH2);
	LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH3);

	
	// TIM1 GPIO Configuration  PA9   ------> TIM1_CH2
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);

	GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	// TIM1 GPIO Configuration  PA10   ------> TIM1_CH3

	GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
	//other Init-Value same than CH2
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);


  /**********************************/
  /* Start output signal generation */
  /**********************************/
  /* Enable TIM1 channel 2&3 */
  LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2);
	LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3);
  
  /* Enable TIM1 outputs */
  LL_TIM_EnableAllOutputs(TIM1);
  
  /* Enable counter */
  LL_TIM_EnableCounter(TIM1);

}

void TIM2_Init(void)
{



	//************************
  // GPIO AF configuration 
	//  PA5   ------> TIM2_ETR
  //***********************
  // Enable the peripheral clock of GPIOs */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);

  /* GPIO TIM1_CH1 configuration */
  LL_GPIO_SetPinMode(ROTARY_SPEED_Port, ROTARY_SPEED_Pin, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetPinPull(ROTARY_SPEED_Port, ROTARY_SPEED_Pin, LL_GPIO_PULL_DOWN);
  LL_GPIO_SetPinSpeed(ROTARY_SPEED_Port, ROTARY_SPEED_Pin, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetAFPin_0_7(ROTARY_SPEED_Port, ROTARY_SPEED_Pin, LL_GPIO_AF_1);

  
  //****************************************
  // TIM2 Config external clocking
  //*****************************************
  // Enable the timer peripheral clock 
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);
	
	LL_TIM_SetPrescaler(TIM2, 0);
	
	LL_TIM_SetAutoReload(TIM2,INPUT_CNT_ARR-1);
// 1ms
  
	LL_TIM_ConfigETR(TIM2, LL_TIM_ETR_POLARITY_NONINVERTED, LL_TIM_ETR_PRESCALER_DIV1, LL_TIM_ETR_FILTER_FDIV1);
 
	LL_TIM_SetClockSource(TIM2, LL_TIM_CLOCKSOURCE_EXT_MODE2); 

	//Filter setzen
	
	LL_TIM_EnableIT_UPDATE(TIM2);

  LL_TIM_EnableCounter(TIM2);								// Enable counter 

  LL_TIM_GenerateEvent_UPDATE(TIM2);				//Force update generation

}

/**
  * @brief TIM6 Initialization Function
	*	@note 	TIM6 triggers DMA to transfer TIM2 Cnt Value (Impulse IN)
*					to Variable
*					DMA1 --> CH3 is triggerd
  * @param None
  * @retval None
  */
void TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM6);



  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  TIM_InitStruct.Prescaler = 32000-1;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 100-1;
  LL_TIM_Init(TIM6, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM6);
  LL_TIM_SetTriggerOutput(TIM6, LL_TIM_TRGO_UPDATE);
  LL_TIM_DisableMasterSlaveMode(TIM6);
  /* USER CODE BEGIN TIM6_Init 2 */
 	LL_TIM_EnableDMAReq_UPDATE(TIM6);
	

	LL_TIM_EnableCounter(TIM6);
  /* USER CODE END TIM6_Init 2 */
}


/**
  * @brief TIM6 Initialization Function
	*	@note 	TIM6 triggers DMA to transfer TIM2 Cnt Value (Impulse IN)
*					to Variable
*					DMA1 --> CH3 is triggerd
  * @param None
  * @retval None
  */
void TIM7_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM7);



  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  TIM_InitStruct.Prescaler = 32000-1;												//1Khz durch Prescaler
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 100-1;												//10Hz durch Autoreload
  LL_TIM_Init(TIM7, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM7);
  LL_TIM_SetTriggerOutput(TIM6, LL_TIM_TRGO_UPDATE);
  LL_TIM_DisableMasterSlaveMode(TIM7);
  /* USER CODE BEGIN TIM6_Init 2 */
 	LL_TIM_EnableIT_UPDATE(TIM7);
	

	LL_TIM_EnableCounter(TIM7);
  /* USER CODE END TIM6_Init 2 */
}

/**
  * @brief  This function configures WWDG
	*         TimeOut: APB1 Clock is 16MHz
	*
	*				tWWDG = tPCLK1 � 4096 � 2^WDGTB[1:0] � (T[6:0] + 1) (ms)
	*				so rollover will be around 260ms = 1/16000*4096*2^3*127

	*       

  * @param  None
  * @retval None
  */
void Configure_WWDG(void)
{
  /* Enable the peripheral clock of DBG register (uncomment for debug purpose) */
  LL_DBGMCU_APB1_GRP1_FreezePeriph(LL_DBGMCU_APB1_GRP1_WWDG_STOP); 
  
  /* Enable the peripheral clock WWDG */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_WWDG);

  /* Configure WWDG */
  /* (1) set prescaler to have a rollover each about ~2s */
  /* (2) set window value to same value (~2s) as downcounter in order to ba able to refresh the WWDG almost immediately */
  /* (3) Refresh WWDG before activate it */
  /* (4) Activate WWDG */
  LL_WWDG_SetPrescaler(WWDG, LL_WWDG_PRESCALER_8); /* (1) */
  LL_WWDG_SetWindow(WWDG,0x7E);                    /* (2) */
  LL_WWDG_SetCounter(WWDG, 0X7E);                  /* (3) */
  LL_WWDG_Enable(WWDG);                            /* (4) */
}


/**
  * @brief  This function check if the system has resumed from WWDG reset
  * @param  None
  * @retval None
  */
void Check_WWDG_Reset(void)
{
  if (LL_RCC_IsActiveFlag_WWDGRST())
  {
    /* clear WWDG reset flag */
    LL_RCC_ClearResetFlags();
		
		Get_JumperConfig();

    /* turn Led on and wait for user event to perform example again */
//    LED_On();
    
    while(JumperConfig != 7)
    {
			Get_JumperConfig();
    }

    /* Reset KeyPressed value */
//    KeyPressed = 0;
  }
}


/**
  * @brief USART RF  Initialization Function
  * @param None
  * @retval None
  */
void UART_RF_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */
	
	

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
	
	


	// Port Clock enable
  UART_PC_GPIO_CLK_ENABLE();
  
  GPIO_InitStruct.Pin = UART_RF_TX_PIN|UART_RF_RX_PIN;   //for both RX & TX Pin
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = UART_RF_SET_GPIO_AF;
  LL_GPIO_Init(UART_RF_GPIO_PORT, &GPIO_InitStruct);

	
  // Peripheral clock enable
  RCC->APB2ENR |= (1<<14);
	
  USART_InitStruct.BaudRate = (UART_RF_Baudrate * 100);
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_2;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  //USART_InitStruct.HardwareFlowControl = LL_UASRT_HWCONTROL_DE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(UART_RF, &USART_InitStruct);
	
	//UART_RF->CR1 |=
//	UART_RF->CR3 |= (1<<14);     //Enable DE Signal for RS485 communication
//	UART_RF->CR3 |= (1<<3);			//Set Half Duplex Mode

	UART_RF->CR1 |= (1<<4);			//Enable IDLE IR Enable Function

	LL_USART_SetBaudRate(UART_RF, 16000000, LL_USART_OVERSAMPLING_16, (UART_RF_Baudrate * 100));
	
	

	
  //LL_USART_DisableIT_CTS(USART2);
  //LL_USART_ConfigAsyncMode(USART2);
  //LL_USART_Enable(USART2);

  /* (2) NVIC Configuration for USART interrupts */
  /*  - Set priority for USARTx_IRQn */
  /*  - Enable USARTx_IRQn */
  NVIC_SetPriority(UART_RF_IRQn, 0);  
  NVIC_EnableIRQ(UART_RF_IRQn);


  LL_USART_Enable(UART_RF);		  // (5) Enable USART 
	
	  /* Polling USART initialisation */
  while((!(LL_USART_IsActiveFlag_TEACK(UART_RF))) || (!(LL_USART_IsActiveFlag_REACK(UART_RF))))
  { 
  }


}

/**
  * @brief  This function initiates RX transfer
  * @param  None
  * @retval None
  */
void UART_RF_RX_Start(void)
{

	/* Clear Overrun flag, in case characters have already been sent to USART */
  LL_USART_ClearFlag_ORE(UART_RF);
	
  /* Enable DMA RX Interrupt */
  LL_USART_EnableDMAReq_RX(UART_RF);

  /* Enable DMA Channel Rx */
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_5);

  /* Enable DMA Channel Tx */
 // LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_4);
	
	
}

/**
  * @brief  This function initiates RX transfer
  * @param  None
  * @retval None
  */
void UART_RF_RX_Stop(void)
{
	
	
 /* Clear Overrun flag, in case characters have already been sent to USART */
  LL_USART_ClearFlag_ORE(UART_RF);

//		LL_USART_EnableIT_IDLE (UART_RF);
	
  /* Enable RXNE and Error interrupts */
  LL_USART_DisableIT_RXNE(UART_RF);
	LL_USART_DisableIT_RTO(UART_RF);
  LL_USART_DisableIT_ERROR(UART_RF);	
	
	
}



/**
  * @brief RF Modul Initialization Function
	*   	
  * @param None
  * @retval Status 1..ok   0..Modul NOK
  */


uint8_t RF_MOD_RX_Init()
{
	

	
	UART_RF_TX_DMA_Buffer[0] = 65;										// "A"
	UART_RF_TX_DMA_Buffer[1] = 84;										// "T"
	UART_RF_TX_DMA_Buffer[2] = 82;										// "R"
	UART_RF_TX_DMA_Buffer[3] = RF_CHANNEL;						// ..see at Parameter.h
	UART_RF_TX_DMA_Buffer[4] = RF_DATAPACKET_SIZE;		// ..see at Parameter.h
	UART_RF_TX_DMA_Buffer[5] = 3;								// Status
//	
	RF_UART_TXData(&UART_RF_TX_DMA_Buffer[0],5,&UART_RF_TX_DMA_Buffer[5]);	
	
	return 1;
	
	
}

/**
  * @brief RF Modul Read Configuration
	*   	
  * @param None
  * @retval Status 1..ok   0..Modul NOK
  */


uint8_t RF_MOD_Read_Config()
{

	UART_RF_TX_DMA_Buffer[0] = 65;							// "A"
	UART_RF_TX_DMA_Buffer[1] = 84;							// "T"
	UART_RF_TX_DMA_Buffer[2] = 63;							// "?"
	UART_RF_TX_DMA_Buffer[3] = 3;								// Status
	
	RF_UART_TXData(&UART_RF_TX_DMA_Buffer[0],3,&UART_RF_TX_DMA_Buffer[3]);
	
}


/**
  * @brief USART PC  Initialization Function
  * @param None
  * @retval None
  */
void UART_PC_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
	

	// Port Clock enable
  UART_PC_GPIO_CLK_ENABLE();

  
  GPIO_InitStruct.Pin = UART_PC_TX_PIN|UART_PC_RX_PIN;   //for both RX & TX Pin
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = UART_PC_SET_GPIO_AF;
  LL_GPIO_Init(UART_PC_GPIO_PORT, &GPIO_InitStruct);

  // Peripheral clock enable
  UART_PC_CLK_ENABLE();
	
  USART_InitStruct.BaudRate = UART_PC_Baudrate;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(UART_PC, &USART_InitStruct);
	
	LL_USART_SetBaudRate(UART_PC, 16000000, LL_USART_OVERSAMPLING_16, UART_PC_Baudrate);
	
  //LL_USART_DisableIT_CTS(USART2);
  //LL_USART_ConfigAsyncMode(USART2);
  //LL_USART_Enable(USART2);

  /* (2) NVIC Configuration for USART interrupts */
  /*  - Set priority for USARTx_IRQn */
  /*  - Enable USARTx_IRQn */
  NVIC_SetPriority(UART_PC_IRQn, 0);  
  NVIC_EnableIRQ(UART_PC_IRQn);


  LL_USART_Enable(UART_PC);		  // (5) Enable USART 
	
	  /* Polling USART initialisation */
  while((!(LL_USART_IsActiveFlag_TEACK(UART_PC))) || (!(LL_USART_IsActiveFlag_REACK(UART_PC))))
  { 
  }

}



/**
  * @brief  This function initiates RX transfer
  * @param  None
  * @retval None
  */
void UART_PC_RX_Start(void)
{

  /* Print user info on PC com port */
  //PrintInfo(aTextInfoStart, sizeof(aTextInfoStart));

 	
 /* Clear Overrun flag, in case characters have already been sent to USART */
  LL_USART_ClearFlag_ORE(UART_PC);

  /* Enable RXNE and Error interrupts */
  LL_USART_EnableIT_RXNE(UART_PC);
  LL_USART_EnableIT_ERROR(UART_PC);	
	
}

/**
  * @brief  This function configures I2C1 in Master mode.
  * @note   This function is used to :
  *         -1- Enables GPIO clock and configures the I2C1 pins.
  *         -2- Enable the I2C1 peripheral clock and I2C1 clock source.
  *         -3- Configure NVIC for I2C1.
  *         -4- Configure I2C1 functional parameters.
  *         -5- Enable I2C1.
  *         -6- Enable I2C1 transfer complete/error interrupts.
  * @note   Peripheral configuration is minimal configuration from reset values.
  *         Thus, some useless LL unitary functions calls below are provided as
  *         commented examples - setting is default configuration from reset.
	*					I2C Clock Source = HSI (8MHz), 100ns Rise, 10ns Fall, 100kHz I2C Speed
	*					=> I2C_TIMING value
	*					Timing calculated by I2C_Timing_Configuration_v1.0.1.xls ore CubeMX
  * @param  None
  * @retval None
  */
void Configure_I2C_Master(void)
{
  // (1) Enables GPIO clock and configures the I2C1 pins **********************/
  //    (SCL on PB.6, SDA on PB.7)                     **********************/


  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);									  // Enable the peripheral clock of GPIOB 

  // Configure SCL Pin as : Alternate function, High Speed, Open drain, Pull up 
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_8, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_8_15(GPIOB, LL_GPIO_PIN_8, LL_GPIO_AF_4);
  LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_8, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_8, LL_GPIO_OUTPUT_OPENDRAIN);
  LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_8, LL_GPIO_PULL_UP);

  // Configure SDA Pin as : Alternate function, High Speed, Open drain, Pull up 
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_9, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_8_15(GPIOB, LL_GPIO_PIN_9, LL_GPIO_AF_4);
  LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_9, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_9, LL_GPIO_OUTPUT_OPENDRAIN);
  LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_9, LL_GPIO_PULL_UP);

  // Enable the peripheral clock for I2C1 
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);

  // Set I2C1 clock source as HSI 
  LL_RCC_SetI2CClockSource(LL_RCC_I2C1_CLKSOURCE_HSI);

  /* (3) Configure NVIC for I2C1 **********************************************/

  /* Configure Event IT:
   *  - Set priority for I2C1_EV_IRQn
   *  - Enable I2C1_EV_IRQn
   */
  NVIC_SetPriority(I2C1_EV_IRQn, 0);  
  NVIC_EnableIRQ(I2C1_EV_IRQn);

  /* Configure Error IT:
   *  - Set priority for I2C1_ER_IRQn
   *  - Enable I2C1_ER_IRQn
   */
  NVIC_SetPriority(I2C1_ER_IRQn, 0);  
  NVIC_EnableIRQ(I2C1_ER_IRQn);

  /* (4) Configure I2C1 functional parameters *********************************/

  /* Disable I2C1 prior modifying configuration registers */
  LL_I2C_Disable(I2C1);

  /* Configure the SDA setup, hold time and the SCL high, low period */
  /* (uint32_t)0x00C0216C = I2C_TIMING*/
  LL_I2C_SetTiming(I2C1, I2C_TIMING);



  /* (5) Enable I2C1 **********************************************************/
  LL_I2C_Enable(I2C1);

 
//  LL_I2C_EnableIT_RX(I2C1);
//  LL_I2C_EnableIT_NACK(I2C1);
		LL_I2C_EnableIT_ERR(I2C1);
//  LL_I2C_EnableIT_STOP(I2C1);
}

/**
  * @brief  Configure ADC (ADC instance: ADC1) and GPIO used by ADC channels.
  * @note   In case re-use of this function outside of this example:
  *         This function includes checks of ADC hardware constraints before
  *         executing some configuration functions.
  *         - In this example, all these checks are not necessary but are
  *           implemented anyway to show the best practice usages
  *           corresponding to reference manual procedure.
  *           (On some STM32 series, setting of ADC features are not
  *           conditioned to ADC state. However, in order to be compliant with
  *           other STM32 series and to show the best practice usages,
  *           ADC state is checked anyway with same constraints).
  *           Software can be optimized by removing some of these checks,
  *           if they are not relevant considering previous settings and actions
  *           in user application.
  *         - If ADC is not in the appropriate state to modify some parameters,
  *           the setting of these parameters is bypassed without error
  *           reporting:
  *           it can be the expected behavior in case of recall of this 
  *           function to update only a few parameters (which update fullfills
  *           the ADC state).
  *           Otherwise, it is up to the user to set the appropriate error 
  *           reporting in user application.
  * @note   Peripheral configuration is minimal configuration from reset values.
  *         Thus, some useless LL unitary functions calls below are provided as
  *         commented examples - setting is default configuration from reset.
  * @param  None
  * @retval None
  */
void Configure_ADC(void)
{

  
		//ADC1 channel 2 is mapped on GPIO pin PA.0 */ 
  
  // Enable GPIO Clock 
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  
  // Configure GPIO in analog mode to be used as ADC input 
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_0, LL_GPIO_MODE_ANALOG);
  
  /*## Configuration of NVIC #################################################*/
  /* Configure NVIC to enable ADC1 interruptions */
  NVIC_SetPriority(ADC1_2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),7,0)); /* ADC IRQ one higher priority than DMA ADC IRQ */
  NVIC_EnableIRQ(ADC1_2_IRQn);
  
  // Configuration of ADC 
  
  // Enable ADC clock (core clock) 
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_ADC12);
  
  if(__LL_ADC_IS_ENABLED_ALL_COMMON_INSTANCE() == 0)
  {
    // Set ADC clock (conversion clock) common to several ADC instances */
    LL_ADC_SetCommonClock(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_CLOCK_SYNC_PCLK_DIV2);
  }
  
  // Common Configuration of ADC
  //       ADC must be disabled.                                              */
	if (LL_ADC_IsEnabled(ADC1) == 0)
  {
    
    // Set ADC data resolution 
     LL_ADC_SetResolution(ADC1, LL_ADC_RESOLUTION_8B);
    
    // Set ADC conversion data alignment 
     LL_ADC_SetResolution(ADC1, LL_ADC_DATA_ALIGN_RIGHT);
    
    // Set ADC low power mode 
     LL_ADC_SetLowPowerMode(ADC1, LL_ADC_LP_MODE_NONE);
    
    /* Set ADC selected offset number: channel and offset level */
    // LL_ADC_SetOffset(ADC2, LL_ADC_OFFSET_1, LL_ADC_CHANNEL_1, 0x000);
    
  }
  
  // Configuration of ADC hierarchical scope: ADC group regular 
  //       ADC must be disabled or enabled without conversion on going  on group regular.                                                  */

  if ((LL_ADC_IsEnabled(ADC1) == 0)               ||
      (LL_ADC_REG_IsConversionOngoing(ADC1) == 0)   )
  {
    // Set ADC group regular trigger source 
    LL_ADC_REG_SetTriggerSource(ADC1, LL_ADC_REG_TRIG_SOFTWARE);
    
    // Set ADC group regular continuous mode 
    LL_ADC_REG_SetContinuousMode(ADC1, LL_ADC_REG_CONV_SINGLE);
    
    // Set ADC group regular conversion data transfer */
    LL_ADC_REG_SetDMATransfer(ADC1, LL_ADC_REG_DMA_TRANSFER_UNLIMITED);
    
    // Set ADC group regular overrun behavior */
    LL_ADC_REG_SetOverrun(ADC1, LL_ADC_REG_OVR_DATA_OVERWRITTEN);
    
    // Set ADC group regular sequencer length and scan direction */
    LL_ADC_REG_SetSequencerLength(ADC1, LL_ADC_REG_SEQ_SCAN_DISABLE);
    
    // Set ADC group regular sequencer discontinuous mode */
     LL_ADC_REG_SetSequencerDiscont(ADC1, LL_ADC_REG_SEQ_DISCONT_DISABLE);
    
    // Set ADC group regular sequence: channel on the selected sequence rank. */
    LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_1);
  }
 
  // Channel Configuration 
  //       ADC must be disabled or enabled without conversion on going   on either groups regular or injected.                              */
  
	if ((LL_ADC_IsEnabled(ADC1) == 0)                    ||
      ((LL_ADC_REG_IsConversionOngoing(ADC1) == 0) &&
       (LL_ADC_INJ_IsConversionOngoing(ADC1) == 0)   )   )
  {

		LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_1, LL_ADC_SAMPLINGTIME_19CYCLES_5);
    
    // Set mode single-ended                                                  */
    LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_1, LL_ADC_SINGLE_ENDED);
  }
  
  // Configuration of ADC interruptions 
  // Enable interruption ADC group regular overrun 
  LL_ADC_EnableIT_OVR(ADC1);
  
}


/**
  * @brief  Perform ADC activation procedure to make it ready to convert
  *         (ADC instance: ADC2).
  * @note   Operations:
  *         - ADC instance
  *           - Enable internal voltage regulator
  *           - Run ADC self calibration
  *           - Enable ADC
  *         - ADC group regular
  *           none: ADC conversion start-stop to be performed
  *                 after this function
  *         - ADC group injected
  *           none: ADC conversion start-stop to be performed
  *                 after this function
  * @param  None
  * @retval None
  */
void Activate_ADC(void)
{
  __IO uint32_t wait_loop_index = 0;
  #if (USE_TIMEOUT == 1)
  uint32_t Timeout = 0; /* Variable used for timeout management */
  #endif /* USE_TIMEOUT */
  

  if (LL_ADC_IsEnabled(ADC1) == 0)
  {
    // Enable ADC internal voltage regulator */
    LL_ADC_EnableInternalRegulator(ADC1);
    
    wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US * (SystemCoreClock / (100000 * 2))) / 10);
    while(wait_loop_index != 0)
    {
      wait_loop_index--;
    }
    
    // Run ADC self calibration 
    LL_ADC_StartCalibration(ADC1, LL_ADC_SINGLE_ENDED);
    
    // Poll for ADC effectively calibrated 
    #if (USE_TIMEOUT == 1)
    Timeout = ADC_CALIBRATION_TIMEOUT_MS;
    #endif /* USE_TIMEOUT */
    
    while (LL_ADC_IsCalibrationOnGoing(ADC1) != 0)
    {
    #if (USE_TIMEOUT == 1)
      /* Check Systick counter flag to decrement the time-out value */
      if (LL_SYSTICK_IsActiveCounterFlag())
      {
        if(Timeout-- == 0)
        {
        /* Time-out occurred. Set LED to blinking mode */
        LED_Blinking(LED_BLINK_ERROR);
        }
      }
    #endif /* USE_TIMEOUT */
    }
    
    /* Delay between ADC end of calibration and ADC enable.                   */
    /* Note: Variable divided by 2 to compensate partially                    */
    /*       CPU processing cycles (depends on compilation optimization).     */
    wait_loop_index = (ADC_DELAY_CALIB_ENABLE_CPU_CYCLES >> 1);
    while(wait_loop_index != 0)
    {
      wait_loop_index--;
    }
    
    /* Enable ADC */
    LL_ADC_Enable(ADC1);
    
    /* Poll for ADC ready to convert */
    #if (USE_TIMEOUT == 1)
    Timeout = ADC_ENABLE_TIMEOUT_MS;
    #endif /* USE_TIMEOUT */
    
    while (LL_ADC_IsActiveFlag_ADRDY(ADC1) == 0)
    {
    #if (USE_TIMEOUT == 1)
      /* Check Systick counter flag to decrement the time-out value */
      if (LL_SYSTICK_IsActiveCounterFlag())
      {
        if(Timeout-- == 0)
        {
        /* Time-out occurred. Set LED to blinking mode */
        LED_Blinking(LED_BLINK_ERROR);
        }
      }
    #endif /* USE_TIMEOUT */
    }
    
  }
  
   
}