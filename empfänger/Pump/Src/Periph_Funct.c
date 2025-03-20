/******************** (C) COPYRIGHT Benjamin Putz ********************
* File Name          : Functions.c
* Author             : Benjamim Putz
* Date First Issued  : 20/12/2024
* Description        : some functions
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


#include "stm32f3xx_ll_i2c.h"

#include "string.h"
#include "stdlib.h"


/**
  * @brief  Function to Read Jumper configuration
  *         
  * @param  None
  * @retval None
  */
	
	
void Get_JumperConfig(void)
{
	
	JumperConfig =  (uint8_t) LL_GPIO_ReadInputPort(GPIOB);				//Read Bit0..Bit7

	JumperConfig &= 0x07;																					//Select Bit0..2
	
	
}





/**
  * @brief  Function called to update PWM Signal (Speed) of Zellenradschleuse
  *         
  * @param  In: Speed: Speed in [%] ... 
	*								positive:   Speed in forward
	*								negative:		Speed in reverse	
  * @retval None
  */

void PWM_RotarySpeed_Update(int8_t Speed)
{
	
  int32_t P;    // Pulse duration 
  uint32_t PC;    // PulseChange duration
	uint32_t N;		 // PWM signal neutral
	uint32_t		MaxFWDSpeedRange = 0;					//maximaler Regelbereich f�r Vorw�rtsrichtung
	uint32_t		MaxREWSpeedRange = 0;					//maximaler Regelbereich f�r R�ckw�rtsrichtung
  
	MaxFWDSpeedRange = (PWM_RESOLUTION * (PWM_DUTY_MAX - PWM_DUTY_NEUTRAL)) / 100; 
	
	MaxREWSpeedRange = (PWM_RESOLUTION * (PWM_DUTY_NEUTRAL - PWM_DUTY_MIN)) / 100;
	
	N = (LL_TIM_GetAutoReload(TIM1) + 1) >> 1;     //div 2 Neutralstellung ist 50:50 PWM 
	
	PC = 0;
	P = N;						//if Speed = 0 .. nothing happen
	
	if (Speed > 0)     //positiv -> Rotary in FWD
	{
		if (Speed > 100) Speed = 100;
		PC = (MaxFWDSpeedRange * Speed ) / 100;
		P = N + PC;						//Calc new CC Value
	}

  else if (Speed < 0)	//negative -> Rotary in REW
	{
		if (Speed < -100) Speed = -100;
		Speed *= (-1);													// Absolutbetrag	
		PC = (MaxREWSpeedRange * Speed) / 100;  //PC do subtract, because Speed is negative
		P = N - PC;						//Calc new CC Value		
	}

  LL_TIM_OC_SetCompareCH2(TIM1, P);		// Pulse duration is determined by the value of the compare register.       
	
}

/**
  * @brief  Function called to update PWM Signal (Speed) of Zubringerschnecke
  *         
  * @param  In: Speed: Speed in [%] ... 
	*								positive:   Speed in forward
	*								negative:		Speed in reverse	
  * @retval None
  */

void PWM_ScrewSpeed_Update(int8_t Speed)
{
	
  int32_t P;    // Pulse duration 
  uint32_t PC;    // PulseChange duration
	uint32_t N;		 // PWM signal neutral
	uint32_t		MaxFWDSpeedRange = 0;					//maximaler Regelbereich f�r Vorw�rtsrichtung
	uint32_t		MaxREWSpeedRange = 0;					//maximaler Regelbereich f�r R�ckw�rtsrichtung
  
	MaxFWDSpeedRange = (PWM_RESOLUTION * (PWM_DUTY_MAX - PWM_DUTY_NEUTRAL)) / 100; 
	
	MaxREWSpeedRange = (PWM_RESOLUTION * (PWM_DUTY_NEUTRAL - PWM_DUTY_MIN)) / 100;
	
	N = (LL_TIM_GetAutoReload(TIM1) + 1) >> 1;     //div 2 Neutralstellung ist 50:50 PWM 
	
	PC = 0;	
	P = N;				//if Speed = 0 .. nothing happen
	
	if (Speed > 0)     //positiv -> Rotary in FWD
	{
		if (Speed > 100) Speed = 100;
		PC = (MaxFWDSpeedRange * Speed ) / 100;
		P = N + PC;						//Calc new CC Value
	}

  else if (Speed < 0)	//negative -> Rotary in REW
	{
		if (Speed < -100) Speed = -100;
		Speed *= (-1);													// Absolutbetrag	
		PC = (MaxREWSpeedRange * Speed) / 100;  //PC do subtract, because Speed is negative
		P = N - PC;						//Calc new CC Value		
	}

  LL_TIM_OC_SetCompareCH3(TIM1, P);		// Pulse duration is determined by the value of the compare register.       
	
}




/**
  * @brief  Function calls Update Functions 
  *         
  * @param  In: DValue: Int Time [ms] ... Bereich: 100 - 1000 
  * @retval None
  */


void Update_Peripherals(void)
{
	
	PWM_ScrewSpeed_Update(*(Screw.Speed));
	PWM_RotarySpeed_Update(*(Rotary.Speed));
	
}


/**
  * @brief  Function Set whole Peripherie to init State
  *         
  * @param  None 
  * @retval None
  */

void Reset_Peripherie(void)
{
	
		*Feeder.SetState = 0x00;
	
		
	
		DisableFEEDERForward;
		DisableFEEDERRewind;
	
	
	
		*Screw.Speed = 0;
		*Rotary.Speed = 0;
	
		Update_Peripherals();
	
}



/**
  * @brief  Read Counter Value - Called from DMA1 - CH3 IRRoutine
  * @param  None
  * @retval None
  */


void CounterReadTransferComplete_Callback()
{
  uint8_t tmp_index = 0;
static	uint32_t Cnt_Sum = 0;
	uint16_t result;
//	uint8_t fOvflwDone = 0;
  
  /* Computation of ADC conversions raw data to physical values               */
  /* using LL ADC driver helper macro.                                        */
  /* Management of the 2nd half of the buffer */
//  for (tmp_index = 0; tmp_index < DMA_CntCycle; tmp_index++)
//  {
//    Cnt_Sum +=  DMA_CntBuffer[tmp_index];
//  }
		if (fCountOvflw)							//overflow of Input Timer happens
		{
//			fCountOvflw = 0;					// reset Flag
			
			Cnt_Sum = INPUT_CNT_ARR - Counter_Init_Value;
			Cnt_Sum += DMA_CntBuffer[0];
			fCountOvflw = 0;
		}
		else
		{
			Cnt_Sum = DMA_CntBuffer[0]- Counter_Init_Value;
			
		}
		
		Counter_Init_Value = DMA_CntBuffer[0];
		
		if (IntTimeCounter == IntTimeCntMax)
		{
			result = CounterSumValue + Cnt_Sum;
			result *= 60;															// Einheit U/min
			result >>= 4;															// dividiert durch 16 (16 Z�hne (impulse) pro Umdrehung
			*Rotary.ActSpeed = (uint8_t) result; 
			(*CountIncrement)++;								//increase Measurecounter
			CounterSumValue = 0;
			IntTimeCounter = 1;
		
		}
		else 
		{
			CounterSumValue  += Cnt_Sum;
			IntTimeCounter++;
			
		}
		
	

}



/**
  * @brief  Function - Start ADC Conversion 
  *         called from main-routine
	*					Read Values DMA transfered to 
  * @param  None 
  * @retval None
  */

void ADC_StartConversion()
{

  /* Update status variable of DMA transfer before performing the first       */
  /* ADC conversion start.                                                    */
		if (PFlags.Pump_Pressure_UpdateReq == 2)
  {
    /* Update status variable of DMA transfer */
    PFlags.Pump_Pressure_UpdateReq = 0;
  }

  if ((LL_ADC_IsEnabled(ADC1) == 1)               &&
      (LL_ADC_IsDisableOngoing(ADC1) == 0)        &&
      (LL_ADC_REG_IsConversionOngoing(ADC1) == 0)   )
  {
    LL_ADC_REG_StartConversion(ADC1);
  }
  else
  {
    /* Error: ADC conversion start could not be performed */
    //  TODO 
  }

}


/**
  * @brief  DMA transfer complete callback
  * @note   This function is executed when the transfer complete interrupt
  *         is generated
  * @retval None
  */
void AdcDmaTransferComplete_Callback()
{
  uint32_t tmp_index = 0;
  
	PumpPress_ADCValue = 0;
  /* Computation of ADC conversions raw data to physical values               */
  /* using LL ADC driver helper macro.                                        */
  /* Management of the 2nd half of the buffer */
  for (tmp_index = 0; tmp_index < ADC_CONVERTED_DATA_BUFFER_SIZE; tmp_index++)
  {
    PumpPress_ADCValue +=  __LL_ADC_CALC_DATA_TO_VOLTAGE(VDDA_APPLI, PumpPressRawBuff[tmp_index], LL_ADC_RESOLUTION_12B);
  }
  
	PumpPress_ADCValue /= ADC_CONVERTED_DATA_BUFFER_SIZE;
	*Pump.Pressure = PumpPress_ADCValue;
	
	
  // Update status variable of DMA transfer 
  PFlags.Pump_Pressure_UpdateReq = 1;
  
  /* Set LED depending on DMA transfer status */
  /* - Turn-on if DMA transfer is completed */
  /* - Turn-off if DMA transfer is not completed */
//  LED_On();

}



/**
  * @brief  Function called from USART IRQ Handler when RXNE flag is set
  *         Function is in charge of reading character received on USART RX line.
  * @param  None
  * @retval None
  */
void USART_PC_CharReception_Callback(void)
{
	uint8_t *ptemp;
	uint8_t dmyChar;

	

	
		// Read Received character. RXNE flag is cleared by reading of RDR register 
		dmyChar = LL_USART_ReceiveData8(UART_PC);	

		if (RxPCPack.fAckReceiving == 1) 											//waiting for Acknowledge
		{
			if (dmyChar == ACKP)	//&& (!RxPCPack.DataReceiving))				//test if Acknowledge is 
			{
					 RxPCPack.pBufferReception[RxPCPack.dRxReceptionChars++] = dmyChar;
					 if (RxPCPack.dRxReceptionChars == 3)
					 {
							RxPCPack.pBufferReception[RxPCPack.dRxReceptionChars] = '\0'; 
						 	ptemp =   RxPCPack.pBufferProcess;
							RxPCPack.pBufferProcess = RxPCPack.pBufferReception;
							RxPCPack.pBufferReception = ptemp;
							RxPCPack.dRxProcessChars = RxPCPack.dRxReceptionChars;
							RxPCPack.dRxReceptionChars = 0;
							RxPCPack.fDataReceiving = 0;
							// Set Acknowledge valid 
							RxPCPack.fAckReceiving = 2;  //Ack is ok
					 }
			 }
			
		}
		else if ((dmyChar == STX) && (!RxPCPack.fDataReceiving))
		{
				RxPCPack.fDataReceiving = 1;
		}
		else if (RxPCPack.fDataReceiving)
		{
			if (dmyChar == ETX)  //check if  End of Datapacket received
			{

				/* Swap buffers for next bytes to be received */
				ptemp =   RxPCPack.pBufferProcess;
				RxPCPack.pBufferProcess = RxPCPack.pBufferReception;
				RxPCPack.pBufferReception = ptemp;
				RxPCPack.dRxProcessChars = RxPCPack.dRxReceptionChars;
				RxPCPack.dRxReceptionChars = 0;
				RxPCPack.fDataReceiving = 0;
				// Set Buffer swap indication 
				RxPCPack.fBufferReady = 1;								// received packet is ok

			}
			else 
			{
				RxPCPack.pBufferReception[RxPCPack.dRxReceptionChars++] = dmyChar;
			}
	
  }
}



/**
  * @brief  Send Txt information message on USART Tx line (to PC Com port).
  * @param  None
  * @retval None
  */
void SendUartTx(USART_TypeDef *USARTTyp, uint8_t *String, uint32_t Size, uint8_t *State)
{
  uint32_t index = 0;
  uint8_t *pchar = String;
	
	*State = 2;
	
  
  /* Send characters one per one, until last char to be sent */
  for (index = 0; index < Size; index++)
  {
    /* Wait for TXE flag to be raised */
    while (!LL_USART_IsActiveFlag_TXE(USARTTyp))
    {
    };

    /* Write character in Transmit Data register.
       TXE flag is cleared by writing data in TDR register */
    LL_USART_TransmitData8(USARTTyp, *pchar++);
  }

  /* Wait for TC flag to be raised for last char */
  while (!LL_USART_IsActiveFlag_TC(USARTTyp))
  {
  };
	

	
	*State = 1;
	
	
	
}



/**
  * @brief  Send Txt information message over RS485 to SPS (to UART_SPS).
  * @param  None
  * @retval None
  */
void RF_UART_TXData(uint8_t *DMA_Buffer, uint32_t Size, uint8_t *State)
{
  uint32_t index = 0;
  uint8_t *pchar = DMA_Buffer;
	
	*State = 2;

	if (PFlags.RF_TX_Acitve) 
	{
		*State = 0;
	}
		
	else
	{
	
		LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_4,
                         (uint32_t) DMA_Buffer,
                         LL_USART_DMA_GetRegAddr(UART_RF, LL_USART_DMA_REG_DATA_TRANSMIT),
                         LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_CHANNEL_4));
		LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_4, Size);										
	

		LL_USART_EnableDMAReq_TX(UART_RF);
		LL_DMA_ClearFlag_TE4(DMA1);
		LL_USART_ClearFlag_TC(UART_RF);
		LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_4);
	
		PFlags.RF_TX_Acitve = 1;
		*State = 1;
	}
	
	
}



void RFRcvErrorReset(void)
{
	
	
	RxRFPack.dRxReceptionChars = 0;
	RxRFPack.fBufferReady	= 0;
	RxRFPack.fDataReceiving = 0;
	RxRFPack.fDataValid = 0;
	
	
}





/**
	* @brief  This function convert the Speed Information from the user: 
	*						Speed Value and direction into +/- Speed Value
	*						z.B Schleuse vorw�rts, Geschwindigketi: 33% 
	*						ValveDirection = 6 (Fwd), SpeedValue = 33 wir konvertiert
	*						in Speed: +33
	*						ValveDirection = 7 (Rew), SpeedValue = 33 wir konvertiert
	*						in Speed: -33
	* @param  None
  * @retval None
  */

int8_t SpeedUserToValveConvert(uint8_t ValveDirection, uint8_t SpeedValue)
{
	
	int8_t	ValveSpeed;
	
	if (SpeedValue > 100) SpeedValue = 100;
	
	switch (ValveDirection)
	{
		case 6:				//Forward direction
			
			ValveSpeed = SpeedValue;
		
		break;
		
		case 7: 			//Rewind direction
			
			ValveSpeed = SpeedValue * (-1);

		break;
		
		default:
			
			ValveSpeed = 0;
		break;
		
	}	

	return ValveSpeed;
	
}




/**
  * @brief  This function Updates a peripheral via a function call
	*					The function is stored in a list: UPDATEFCTLIST
	*					This list is based on a pointer structure
	*					Entry is found by the SRAM-Adress of the variable 
  * @param  SRAM_Adress of Variable, Speed..new Value for funcion
  * @retval None
  */


uint8_t UpdateValuePeriph(uint8_t SRAM_Adress, int8_t Speed)
{

	struct UPDATEFCTLISTTyp *srchList;
 	srchList = UpdateFctList;           //load head of Funciont List
 
 
	while (srchList != NULL)
	{
		if (srchList->Src == SRAM_Adress)									//durchsuchen der Liste bis passende SRAM-Adresse gefunden
		{
				(fpUpdateParam(*srchList->Dest,Speed));	//Wert gefunden, erfolgreich
				return 1;
		}
		srchList = srchList->pNext;
 
 }

	return 0;  //Element not found

}	
// 
// 
//---------------------------------------------------------------------------------- 
//
//    LED MANIPULATION
//
//----------------------------------------------------------------------------------



void LED_POWER_GREEN_OFF(void)
{
	LL_GPIO_SetOutputPin(LED_Out_GPIO_Port, LED_Out_Pin);
}

void LED_POWER_GREEN_ON(void)
{
	LL_GPIO_ResetOutputPin(LED_Out_GPIO_Port, LED_Out_Pin);

}



/**
  * @brief  Send Txt information message over RS485 to SPS (to UART_SPS).
  * @param  None
  * @retval None
  */

void LED_Manipulate(struct LED_LIGHT* Led, void (*LED_ON_Fct)(), void (*LED_OFF_Fct)())
{
	
	switch (Led->Status)
	{
		case	0:						//LED OFF
			LED_OFF_Fct();
		break;
		
		case 1:							//LED permanent ON
			LED_ON_Fct();
		break;
		
		case 3:							//LED is Blinking, actual State is OFF
			if (Led->BlinkCounts == 0)
			{
				Led->Status = 0;
			}
			else
			{
				if ((Led->Counter) <=0)											//Counter von Intervall abgelaufen 
				{																										
						Led->Counter = Led->Blinking;						//Lade Blinkzeit	
						LED_ON_Fct();														//schalte LED wieder ein
						Led->Status = 4;		
				}
				else (--Led->Counter);
			}
			break;
		case 4:						//LED is blinking, actual State is ON
			if ((Led->Counter) <=0)
			{
					Led->Counter = Led->Blinking;
					LED_OFF_Fct();
					if (Led->BlinkCounts < 0)					//continous blinking
						Led->Status = 3;				
					else if (Led->BlinkCounts <= 1)   // Blinkanzahl  
						Led->Status = 0;
					else if (Led->BlinkCounts > 1)
					{
						Led->BlinkCounts--;
						Led->Status = 3;		
					}
					else 			
						Led->Status = 3;
			}
			else (--Led->Counter);
		break;

			default:
				break;
					
	}
		
}


//*************************************************
// oPutz 02.23
//
//  Update of the leds
// Force: always update of the LEDs, regardless of their status
//
//*************************************************


void LED_BLINKING_Update(uint8_t Force)
{

	
	if (LED_Power_Green.Status > 0) LED_Manipulate(&LED_Power_Green, LED_POWER_GREEN_ON, LED_POWER_GREEN_OFF);

	
}



void Copy_LED_Struct(struct LED_LIGHT *Code, struct LED_LIGHT *Ledtype)
{
	
	Ledtype->BlinkCounts = Code->BlinkCounts;
	Ledtype->Blinking = Code->Blinking;
	Ledtype->Counter = Code->Counter;
	Ledtype->Status = Code->Status;
	
	
}




//************************************************************************************************************
//
// For Configuration Blinking Code, the original State have to be saved
//************************************************************************************************************


void BACKUP_LED_STATUS(struct LED_LIGHT *Led)
{
	
	Led->Status_Backup = Led->Status;
	Led->Status = 0;
	Led->Blinking_Backup = Led->Blinking;
	Led->Blinking = 0;
	Led->BlinkingCounts_Backup = Led->BlinkCounts;
	Led->BlinkCounts = 0;
		
}


void RESTORE_LED_STATUS(struct LED_LIGHT *Led)
{
	
	Led->Status = Led->Status_Backup;
	Led->Blinking = Led->Blinking_Backup;
	Led->BlinkCounts = Led->BlinkingCounts_Backup;
}



//************************************************************************************************************
//
// Generates the Blinkcode for ERROR Configuration 
//
// Der Blinkcode besteht aus 7 Zust�nden:
// Zustand 1 (Case 0): alle Leds l�schen	Zustand 2: kurzes Blinken f�r n�chste (Zustand 3) Anzeige  
// Zustand 3: Led_NF_Att zeigt Zustand NF-Attenuator an: ON = Led leuchtet   Zustand 4: kurzes Blinken f�r n�chste (Zustand 5) Anzeige
// Zustand 5: Led_Power_Green zeigt Zustand TP-Filter an: 0x blinken: 500kHz, 1x 100kHz 2x 10kHz  Zustand 6: LED-NF_Att kurzes Blinken f�r n�chsten Zustand
// Zustand 7: LEd_Power_Red zeigt Zustand HP-Filter an: 0x blinken:DC , 1x blinken 12Hz, 2x blinken 300Hz
// Zustadn 8 (Case 7): kurzes Blinken: LED_NF-Att gibt Ende der Anzeige an
//
//************************************************************************************************************

void BLINKCODE_ERROR_Generate(struct LED_LIGHT *Code, uint8_t State)
{
	
	 //if (LED_BLINKING_State == 2)
	switch (State)
	{
		case 0:  //clear all necessary Leds


			LED_POWER_GREEN_OFF();
			
			// save current state of LEDs
			BACKUP_LED_STATUS(&LED_Power_Green);
			break;
		case 1:
			Copy_LED_Struct(Code+State, &LED_Power_Green);
			LED_Manipulate(&LED_Power_Green, LED_POWER_GREEN_ON, LED_POWER_GREEN_OFF);
//			printf("Case 0 \n");
			break;
		case 2:
			Copy_LED_Struct(Code+State, &LED_Power_Green);
			LED_Manipulate(&LED_Power_Green, LED_POWER_GREEN_ON, LED_POWER_GREEN_OFF);
//			printf("Case 1 \n");
			break;
		case 3:
			Copy_LED_Struct(Code+State, &LED_Power_Green);
			LED_Manipulate(&LED_Power_Green, LED_POWER_GREEN_ON, LED_POWER_GREEN_OFF);
//			printf("Case 2  \n");
			break;
		case 4:
			Copy_LED_Struct(Code+State, &LED_Power_Green);
			LED_Manipulate(&LED_Power_Green, LED_POWER_GREEN_ON, LED_POWER_GREEN_OFF);
//			printf("Case 3  \n");
			break;		
		case 5:
			Copy_LED_Struct(Code+State, &LED_Power_Green);
			LED_Manipulate(&LED_Power_Green, LED_POWER_GREEN_ON, LED_POWER_GREEN_OFF);
//			printf("Case 4  \n");
			break;		
		case 6:
			Copy_LED_Struct(Code+State, &LED_Power_Green);
			LED_Manipulate(&LED_Power_Green, LED_POWER_GREEN_ON, LED_POWER_GREEN_OFF);
//			printf("Case 5  \n");
			break;		
		case 7:
			Copy_LED_Struct(Code+State, &LED_Power_Green);
			LED_Manipulate(&LED_Power_Green, LED_POWER_GREEN_ON, LED_POWER_GREEN_OFF);
//			printf("Case 0 \n");
			break;		
		
	
	}
	
}







//----------------------------------------------------------------------------------
//  is Called to accept a function as a parameter
//
//
//----------------------------------------------------------------------------------

void fpUpdateParam(void (*Updateer)(int8_t), int8_t Wert)
 {
		
	 Updateer(Wert);
	   
 }////----------------------------------------------------------------------------------
////
////
////
////----------------------------------------------------------------------------------

/**
  * @brief  Function called to Read Input Port from Sensors
  * @param  None
  * @retval None
  */
 
 void Read_Sensor_Switches(void)
{
	
	

	
}




/**
  * @brief  Function called in case of error detected in USART IT Handler
  * @param  None
  * @retval None
  */
void Error_PC_Callback(void)
{
  __IO uint32_t isr_reg;

  /* Disable USARTx_IRQn */
  NVIC_DisableIRQ(UART_PC_IRQn);
  
  /* Error handling example :
    - Read USART ISR register to identify flag that leads to IT raising
    - Perform corresponding error handling treatment according to flag
  */
  isr_reg = LL_USART_ReadReg(UART_PC, ISR);
  if (isr_reg & LL_USART_ISR_NE)
  {
    /* case Noise Error flag is raised : Clear NF Flag */
    LL_USART_ClearFlag_NE(UART_PC);
  }
  else
  {
    /* Unexpected IT source : Set LED to Blinking mode to indicate error occurs */
    //LED_Blinking(LED_BLINK_ERROR);
  }
}







/**
  * @brief  This Function handle Master events to perform a transmission process
  * @note  This function is composed in different steps :
  *        -1- Initiate a Start condition to the Slave device
  *        -2- Loop until end of transfer received (STOP flag raised)
  *             -2.1- Transmit data (TXIS flag raised)
  *        -3- Clear pending flags, Data consistency are checking into Slave process
  * @param  None
  * @retval None
  */
void LL_I2C_Write(void)
{
	
	uint8_t Cnt = 0;
  /* (1) Initiate a Start condition to the Slave device ***********************/

  /* Master Generate Start condition for a write request :              */
  /*    - to the Slave with a 7-Bit SLAVE_OWN_ADDRESS                   */
  /*    - with a auto stop condition generation when transmit all bytes */
//  LL_I2C_HandleTransfer(I2C1, EEPROM_ADRESS, LL_I2C_ADDRSLAVE_7BIT, I2CTxBufferCnt, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);
	  LL_I2C_HandleTransfer(I2C1, EEPROM_ADRESS, LL_I2C_ADDRSLAVE_7BIT, 0x05, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);

  /* (2) Loop until end of transfer received (STOP flag raised) ***************/

#if (USE_TIMEOUT == 1)
  Timeout = I2C_SEND_TIMEOUT_TXIS_MS;
#endif /* USE_TIMEOUT */

  /* Loop until STOP flag is raised  */
//  while(!LL_I2C_IsActiveFlag_STOP(I2C1))
//	{
//	 while(Cnt < I2CTxBufferCnt)
	 while(Cnt < 0x05)
  {
    /* (2.1) Transmit data (TXIS flag raised) *********************************/

    /* Check TXIS flag value in ISR register */
		while(LL_I2C_IsActiveFlag_TXIS(I2C1) == RESET)
		{
		}
    if(LL_I2C_IsActiveFlag_TXIS(I2C1))
    {
      /* Write data in Transmit Data register.
      TXIS flag is cleared by writing data in TXDR register */
      LL_I2C_TransmitData8(I2C1, (I2CTxBuffer[Cnt++]));
		}
#if (USE_TIMEOUT == 1)
      Timeout = I2C_SEND_TIMEOUT_TXIS_MS;
#endif /* USE_TIMEOUT */
    }
		 while(LL_I2C_IsActiveFlag_TC(I2C1))
		 {
			 
		 }
		 
//	 }
#if (USE_TIMEOUT == 1)
    /* Check Systick counter flag to decrement the time-out value */
    if (LL_SYSTICK_IsActiveCounterFlag()) 
    {
      if(Timeout-- == 0)
      {
        /* Time-out occurred. Set LED2 to blinking mode */
        LED_Blinking(LED_BLINK_SLOW);
      }
    }
#endif /* USE_TIMEOUT */
  

  /* (3) Clear pending flags, Data consistency are checking into Slave process */

  /* End of I2C_SlaveReceiver_MasterTransmitter Process */
  LL_I2C_ClearFlag_STOP(I2C1);

  /* Turn LED2 On:
   *  - Expected bytes have been sent
   *  - Master Tx sequence completed successfully
   */

}

uint8_t Handle_I2C_Master(uint8_t DevAddr, uint8_t* pBuffer, uint8_t WriteAddr, uint16_t NumbToWrite)
{
	
	static uint8_t Cnt = 0;

	
	// Set I2C Transfermodus: Anzahl zu �bertragenden Bytes : NumbToWrite + 1 f�r �bertagung der Schreibadresse 
	  LL_I2C_HandleTransfer(I2C1, DevAddr, LL_I2C_ADDRSLAVE_7BIT, NumbToWrite+1, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);

		EEDataPack.EETimeOut = sEE_LONG_TIMEOUT;	
		while(LL_I2C_IsActiveFlag_TXIS(I2C1) == RESET)			// Check TXIS flag value in ISR register 
		{
			if((EEDataPack.EETimeOut--) == 0) return (sEE_TIMEOUT_UserCallback());	
		}
		if(LL_I2C_IsActiveFlag_TXIS(I2C1))
    {
      // Write WriteAddr to data register - TXIS flag is cleared by writing data in TXDR register
      LL_I2C_TransmitData8(I2C1, WriteAddr);
		}

	while(Cnt < NumbToWrite)				  										// Loop until end of transfer received 
  {
     
		EEDataPack.EETimeOut = sEE_LONG_TIMEOUT;	
		while(LL_I2C_IsActiveFlag_TXIS(I2C1) == RESET)			// Check TXIS flag value in ISR register 
		{
			if((EEDataPack.EETimeOut--) == 0) return (sEE_TIMEOUT_UserCallback());	
		}
		if(LL_I2C_IsActiveFlag_TXIS(I2C1))
    {
      // Write data in Transmit Data register - TXIS flag is cleared by writing data in TXDR register
      LL_I2C_TransmitData8(I2C1, *pBuffer);
			pBuffer++;
			Cnt++;
		}
  }
	
	EEDataPack.EETimeOut = sEE_LONG_TIMEOUT;	
	while(LL_I2C_IsActiveFlag_TC(I2C1))
	{
			if((EEDataPack.EETimeOut--) == 0) return (sEE_TIMEOUT_UserCallback());			 
	}

  LL_I2C_ClearFlag_STOP(I2C1);		  // Clear pending flags

	return 0;
	
}

/**
  * @brief  This Function handle Master events to perform a transmission process
  * @note  This function is composed in different steps :
  *        -1- Initiate a Start condition to the Slave device
  *        -2- Loop until end of transfer received (STOP flag raised)
  *             -2.1- Transmit data (TXIS flag raised)
  *        -3- Clear pending flags, Data consistency are checking into Slave process
  * @param  None
  * @retval None
  */

uint8_t EE_WriteBuffer(uint8_t DevAddr, uint8_t* pBuffer, uint8_t WriteAddr, uint16_t NumbToWrite)
{

uint8_t Buffer[NumbToWrite + 2];
static uint8_t Cnt = 0;


		Cnt = 0;
	
	Buffer[0] = WriteAddr;							//First Byte is Memory Adress
	while ((++Cnt) <= NumbToWrite)			//so copy Address & Data to (a new) Buffer
	{
		Buffer[Cnt] = *pBuffer++;
	}
	Cnt = 0;
	
		I2C1->CR2 &= ~(1<<10);						//Clear RD_WRN Bit (Master request Write Transfer
		I2C1->ICR |= (1<<5);							//Clear STOP Flag ... �brig von ReadFunction


	
  /* (1) Initiate a Start condition to the Slave device ***********************/

  /* Master Generate Start condition for a write request :              */
  /*    - to the Slave with a 7-Bit SLAVE_OWN_ADDRESS                   */
  /*    - with a auto stop condition generation when transmit all bytes */
	// NumbToWrite + 1 .. Write Adress also transferred
  LL_I2C_HandleTransfer(I2C1, DevAddr, LL_I2C_ADDRSLAVE_7BIT, (NumbToWrite+1), LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);

  /* (2) Loop until end of transfer received (STOP flag raised) ***************/

	

	
	
	
#if (USE_TIMEOUT == 1)
  Timeout = I2C_SEND_TIMEOUT_TXIS_MS;
#endif /* USE_TIMEOUT */

	
  /* Loop until STOP flag is raised  */
  while(!LL_I2C_IsActiveFlag_STOP(I2C1))
  {
    /* (2.1) Transmit data (TXIS flag raised) *********************************/

    /* Check TXIS flag value in ISR register */
    if(LL_I2C_IsActiveFlag_TXIS(I2C1))
    {
      /* Write data in Transmit Data register.
      TXIS flag is cleared by writing data in TXDR register */
      //LL_I2C_TransmitData8(I2C1, (*pBuffer++));
			LL_I2C_TransmitData8(I2C1, Buffer[(Cnt++)]);

#if (USE_TIMEOUT == 1)
      Timeout = I2C_SEND_TIMEOUT_TXIS_MS;
#endif /* USE_TIMEOUT */
    }

#if (USE_TIMEOUT == 1)
    /* Check Systick counter flag to decrement the time-out value */
    if (LL_SYSTICK_IsActiveCounterFlag()) 
    {
      if(Timeout-- == 0)
      {
        /* Time-out occurred. Set LED2 to blinking mode */
        LED_Blinking(LED_BLINK_SLOW);
      }
    }
#endif /* USE_TIMEOUT */
  }

  /* (3) Clear pending flags, Data consistency are checking into Slave process */

  /* End of I2C_SlaveReceiver_MasterTransmitter Process */
  LL_I2C_ClearFlag_STOP(I2C1);

  /* Turn LED2 On:
   *  - Expected bytes have been sent
   *  - Master Tx sequence completed successfully
   */

	
	return 0;
}

uint8_t EE_ReadBuffer(uint8_t DevAddr, uint8_t* pBuffer, uint8_t ReadAddr, uint16_t ByteToRead)
{

   uint8_t  Cnt = 0;

	I2CRxBufferCnt = 0;
  /* (1) Initiate a Start condition to the Slave device ***********************/

  // Master Generate Start condition for a read request:   - to the Slave with a 7-Bit SLAVE_OWN_ADDRESS
  //																											 - with no auto stop condition generation when receive 1 byte
   
		EEDataPack.EERxDataCnt = ByteToRead;
	
		EEDataPack.EERxRcvFlag = 0;
	
		LL_I2C_HandleTransfer(I2C1, 0xA0, LL_I2C_ADDRSLAVE_7BIT, 1,LL_I2C_MODE_SOFTEND	, LL_I2C_GENERATE_START_WRITE);

		EEDataPack.EETimeOut = sEE_LONG_TIMEOUT;	
		while(!(LL_I2C_IsActiveFlag_TXIS(EE_I2C)))
		{
			if((EEDataPack.EETimeOut--) == 0) return (sEE_TIMEOUT_UserCallback());
		}
	    // Write data in Transmit Data register.    TXIS flag is cleared by writing data in TXDR register */
//      LL_I2C_TransmitData8(I2C1, ReadAddr);
		      LL_I2C_TransmitData8(I2C1, ReadAddr);
		//}
    
		EEDataPack.EETimeOut = sEE_LONG_TIMEOUT;	
		while(LL_I2C_IsActiveFlag_TC(I2C1))
		 {
				if((EEDataPack.EETimeOut--) == 0) return (sEE_TIMEOUT_UserCallback());
		 }
		 
		LL_I2C_HandleTransfer(I2C1, 0xA0, LL_I2C_ADDRSLAVE_7BIT, ByteToRead,LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_READ);
	
		while (Cnt < ByteToRead)
		{
			EEDataPack.EETimeOut = sEE_LONG_TIMEOUT;	
			while (!(LL_I2C_IsActiveFlag_RXNE(EE_I2C)))
			{
				if((EEDataPack.EETimeOut--) == 0) return (sEE_TIMEOUT_UserCallback());
			}
			*pBuffer = LL_I2C_ReceiveData8(I2C1);
			pBuffer++;
			Cnt++;
			 
		 }
			 

		 
	EEDataPack.EERxRcvFlag = 0;
		
		return sEE_OK;
	
}



/**
  * @brief  Function called from I2C IRQ Handler when RXNE flag is set
  *         Function is in charge of reading byte received on I2C lines.
  * @param  None
  * @retval None
  */
void Read_EEPROM_Reception_Callback(void)
{


	  DummyByte = LL_I2C_ReceiveData8(I2C1);
		I2CRxBuffer[I2CRxBufferCnt++] = DummyByte;
	
	
}

/**
  * @brief  Function called from I2C IRQ Handler when STOP flag is set
  *         Function is in charge of checking data received,
  *         LED2 is On if data are correct.
  * @param  None
  * @retval None
  */
void READ_EEPROM_Complete_Callback(void)
{
//  /* Read Received character.
//  RXNE flag is cleared by reading of RXDR register */
	EEDataPack.EEBufferCounter = 0;
	EEDataPack.EERxRcvFlag = 1;

	 if(DummyByte == 0xFF)
  {
    /* Turn LED2 On:
     *  - Expected byte has been received
     *  - Master Rx sequence completed successfully
     */
    DummyByte = 0;
  }
  else
  {
    /* Call Error function */
    DummyByte = 1;
  }
	
	
}






/**
  * @brief  Function called in case of error detected in I2C IT Handler
  * @param  None
  * @retval None
  */
void Error_I2C_Callback(void)
{
#ifdef SLAVE_BOARD
  /* Disable I2C1_EV_IRQn */
  NVIC_DisableIRQ(I2C1_EV_IRQn);
#else
  /* Disable I2C1_EV_IRQn */
  NVIC_DisableIRQ(I2C1_EV_IRQn);
#endif /* SLAVE_BOARD */

  /* Disable I2C1_ER_IRQn */
#ifdef SLAVE_BOARD
  NVIC_DisableIRQ(I2C1_ER_IRQn);
#else
  /* Disable I2C1_ER_IRQn */
  NVIC_DisableIRQ(I2C1_ER_IRQn);
#endif /* SLAVE_BOARD */

  /* Unexpected event : Set LED2 to Blinking mode to indicate error occurs */

}


uint8_t sEE_TIMEOUT_UserCallback(void)
{
	/* just reset to avoid stall of the program */
	NVIC_SystemReset();
		/* Block communication and all processes -> leads to watchdog reset */
		while (1)
		{   
		}
	return 0x00;
}











