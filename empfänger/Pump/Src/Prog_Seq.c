/**
  ******************************************************************************
  * @file    Prog_Seq.c
  * @author  Benjamin Putz
  * @brief   File contains all functions for programm sequence
  *          
  *          
  ******************************************************************************
  * @attention
  *
  * All rights reserved.</center></h2>
  *
  * 
  *
  ******************************************************************************
  */
	
	
	/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "Functions.h"
#include "Communicate.h"
#include "Declarations.h"
#include "stm32f3xx_ll_rcc.h"
#include "stdlib.h"

#include "Periph_Funct.h"


/**
  * @brief  This function is the State machine for Roller Device (Dosiervorrichtung)
  * @note   This function is used to :
  *         
	* @note   
  *         
  *         
  * @param  None
  * @retval None
  */

void Feeder_Task()
{

	 switch (*(Feeder.SetState))
	 {
		 case 0x06:								//Schubboden vorw�rts
			DisableFEEDERRewind;
		  EnableFEEDERForward;
		 
		 break;
		 
		 case 0x07:								//Schubboden r�ckw�rts
			DisableFEEDERForward;
			EnableFEEDERRewind;
		 
		 break;
		 
		 default:
			DisableFEEDERForward;
			DisableFEEDERRewind;
		 
	 }
	 *Feeder.ActState = *Feeder.SetState;							//Aktualisieren des Zustandes
	 

	
}

	
	/**
  * @brief  This function is the main routine (State machine)
  * @note   This function is used to :
  *         
	* @note   
  *         
  *         
  * @param  None
  * @retval None
  */
	
	void ProgramState()
{
	
		if ((State.Status & 0x10) == 0x10)  //test if change in PSW
		{
			
			switch ((*PSW) & 0x70)
			{	

				case 0x00:

					(State.Status) &= 0xEF; 			//Reset Status Flag
					State.PgmFlow = IDLE;
				
//					UART_SPS->CR2 |= (1<<23);			//Enable Receiver Timeout Function
//					UART_SPS->RTOR = 35;					//Wartezeit bis IRQ aktiv 3.5Zeichen x 10Bit (8bit Daten+start+stop)
//					LL_USART_ClearFlag_RTO(UART_SPS);
//				
//				 UART_SPS_RX_Start();
//				
				break;
		
				case 0x10:

					(State.Status) &= 0xEF; 			//Reset Status Flag
					State.PgmFlow = MEASURE;
					
				break;
				
				case 0x20:
					(State.Status) &= 0xEF;
					State.PgmFlow = CONFIG_KALIB;
				
//				 UART_SPS_RX_Stop();
//				
//					UART_SPS->CR2 &= ~(1<<23);			//Disable Receiver Timeout Function
//					LL_USART_ClearFlag_RTO(UART_SPS);
//					

				break;
				case 0x40:					
				break;
			}
							
		}

		switch (State.PgmFlow)
	 {
		 
		 case IDLE:
			

			CheckRFTransferStatus();	 
			CheckPCTransferStatus();
		 
		 Feeder_Task();
		 
		 if ((RFRcvTimeOutTimer--) < 1)
		 {
			 Reset_Peripherie();
			 RFRcvTimeOutTimer = 2;
			 	LED_Power_Green.Blinking = LED_BLINK_FAST;
		 }
		 
	 
		  
		 break;


		 case MEASURE:
			 
			CheckPCTransferStatus();	

//			if (sPCTransfer != RxTxIdle)
//			{


			if ((ProcessTick % 10) == 0)
			{


			}
			ProcessTick++;
			
			
		 break;
		 
		 case CONFIG_KALIB:										//Writing Configuration from PC to Device and EEPROM
				
			CheckPCTransferStatus();	
		  if (EEDataPack.EETxSendFlag > 1)
			{
				EEDataPack.EETxSendFlag--;				//Waiting for next WriteCyle to EEPROM, Sendflag is Timervariable 
			}
			else if (EEDataPack.EETxSendFlag == 1)		//Data available for EEPROM Write
			{
				if (EEDataPack.EETxDataCnt > EEPROM_MaxDataPage)
				{
					if (EE_WriteBuffer(EEPROM_ADRESS,EEDataPack.pEETxBuffer,EEDataPack.EERxTxAddress,EEPROM_MaxDataPage) == 0)
//					if (Handle_I2C_Master(EEPROM_ADRESS,EEDataPack.pEETxBuffer,EEDataPack.EERxTxAddress,EEPROM_MaxDataPage) == 0)
					{
						EEDataPack.EERxTxAddress += EEPROM_MaxDataPage;    //increase Write Adress for next Write cycle
						EEDataPack.pEETxBuffer += EEPROM_MaxDataPage; 
						// EEDataPack.pEETxBuffer is incrementet in Function
						EEDataPack.EETxDataCnt -= EEPROM_MaxDataPage;			//decrease DataCnt Value
						if (ProgrammPeriod < 10) 
						{
							EEDataPack.EETxSendFlag = (10 / ProgrammPeriod) + 1;			//berechne n�tige Zeit f�r n�chsten EEPROM Write
						}
						else 
						{
							EEDataPack.EETxSendFlag = 1;
						}
						
					}						
					else
					{
						while (1);
					}
					
				}
				
				else 
				{

					if (EE_WriteBuffer(EEPROM_ADRESS,EEDataPack.pEETxBuffer,EEDataPack.EERxTxAddress,EEDataPack.EETxDataCnt) == 0)
					{
						EEDataPack.EETxSendFlag = 0;
					}
					else
					{
						//
						while (1);
					}
				}
			}	
		
		
		break;

 
		case BOOT:
			

			  CheckRFTransferStatus();	
				CheckPCTransferStatus();

				if ((RFModData.StartTimer--) <= 0)
				{		
							RF_MOD_RX_Init();
							RFModData.StartTimer = 0;
							State.PgmFlow = IDLE;
				}
					


		break;
		 
		case STOP:
			
					CheckPCTransferStatus();
			
		break;
		
			 
		 default:
			 
		 break;
		 
	 }
 }

 
 
	void Check_LED_State()
	{		
	 	if (LED_Blinking_Timer <= 0)
	{
		LED_Blinking_Timer = 10;
		//printf("da\n");

    switch (LED_BLINKING_State)
		{

			case 0:
				
			break;
			
			case 1:							//Update LED in Normal Mode
				
				LED_BLINKING_Update(0);

				if ((LED_BLINKING_Task & 0x01) == 0x01)
				{
					LED_BLINKING_State = 2;
				}
				else if ((LED_BLINKING_Task & 0x02) == 0x02)
				{
					LED_BLINKING_State = 3;
				}
					
			break;

			case 2:							//Update LED in NF Config Mode
				if(Error_BlinkCode_State == 8)     //Blinkcode finished
					{  
						Error_BlinkCode_State = 0;
						LED_BLINKING_Task = LED_BLINKING_Task & 0xFE; 		//clr Bit 0 NF Blinking code Task


						RESTORE_LED_STATUS(&LED_Power_Green);

						LED_BLINKING_State = 1;
					}
				else 
					{
						BLINKCODE_ERROR_Generate(&Error_Config_Code[0],Error_BlinkCode_State);
						Error_BlinkCode_State++;
					}
					
				break;
					

			
			break;
		}
	
	}
	
}