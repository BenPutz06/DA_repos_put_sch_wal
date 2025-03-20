/**
	****************** (C) COPYRIGHT Otmar Putz ********************
	* @file    Examples_LL/USART/USART_Communication_Rx_IT_Continuous/Src/main.c
  * @author  Otmar Putz
  * @brief   Declaration of all variables
	* File Name          : Declaration.c
	* Date First Issued  : 25/01/2024
	
*
********************************************************************************
* History:
* 
*******************************************************************************/


/* Define to prevent recursive inclusion ------------------------------------ */



/* Includes ------------------------------------------------------------------*/

//#include "stm32l1xx.h"
#include "Main.h"
#include "Parameter.h"
//#include "DataManage.h"
#include "Declarations.h"
#include "Periph_Funct.h"
#include "stdlib.h"

/* Exported defines ----------------------------------------------------------*/

/********************************************************************************
*
* File Name          : Declaration.h
*
********************************************************************************/






//uint16_t TickCounter;


//uint16_t		LED_Blinky;											//Led Counter for Blinking frequency


///**
//  * @brief RX buffers for storing received data
//  */
//uint8_t aRXPCBufferA[RXPC_BUFFER_SIZE];
//uint8_t aRXPCBufferB[RXPC_BUFFER_SIZE];



//struct DATARxTyp RxPCPack; 										// Datenstruktur Empfangsdaten von PC	
//DATATxTyp TxPCPack;											// Datenstruktur Sendedaten an PC

//  uint8_t *PSW;												// beinhaltet Zustandsbits für Programmablauf
//																			// Bit0...only Read..	          // Bit1...only Read.. unused
//																			// Bit2...only Read..									// Bit3...only Read..
//																			// Bit4...R/W											// Bit5...R/W..1 = Measure-State
//																			// Bit6...R/W 1= Config State			// Bit7...R/W
//	
//	uint8_t *ErrorByte;									//contains the error bits
//	

//// Bit: 0...	
//// Bit: 1...
//// Bit 4: 
//// Bit 5: 
//// Bit 6: Request accepted, Sending in progress
//// Bit 7: Wr-Request sending MeasValue to the PC 


//uint16_t				ProcessTick; 					//zählt die Prozessdurchläufe


//TRANSFERSTATES		sPCTransfer; 				// Statemachine for Datatransfer via UART

//PCDATAPACKTyp			PCRxDataPack;				//	Datenpaket empfangene Daten von PC
//PCDATAPACKTyp			PCTxDataPack;				//  Datenpaket für Sendedaten an PC



uint32_t aTIM6Val[4];
//uint8_t		TimeTicker = 0;

//uint16_t 			*MeasValue;
//uint32_t			TIM1_Clock;
//uint16_t			TIM1_PWM_Width = TIM1_Reloader / 2;
//CLOCKType			ClockLLC;
//DACType 			DAC1Data;

//uint16_t aTIM6Val[TIM6_DATA_BUFFER_SIZE];
//uint16_t uhTIM2Val;

///* Variables for ADC conversion data */
//uint16_t aADCxConvertedData[ADC_CONVERTED_DATA_BUFFER_SIZE]; 		// ADC group regular conversion data */
//uint16_t uhADCxConvertedData = VAR_CONVERTED_DATA_INIT_VALUE; 	// ADC group regular conversion data */ 
//uint16_t uhADCxConvertedData_Voltage_mVolt = 0;  								// Value of voltage calculated from ADC conversion data (unit: mV) */

//// Variable to report status of DMA transfer of ADC group regular conversions 
////  0: DMA transfer is not completed                                          
////  1: DMA transfer is completed                                              
////  2: DMA transfer has not yet been started yet (initial state)              
//uint8_t ubDmaTransferStatus = 2; // Variable set into DMA interruption callback 

//PFlagsTyp PFlags;									//Programm Flags



//STATETyp State;								//Statemachine Programmablauf



//// verkettete Liste, welche die Speicheradresse der Variablen und die 
//// dazugehörige Funktion (zum Aktualisieren) 
//// beinhaltet
//struct UpdateFctListTyp *UpdateFctList;	



//__IO uint8_t TestCounter = 0;

//__IO uint8_t UpdateOpVar;				//Operation Variable have to be updated




////void(*fDACUpdate[2])(uint16_t);
////	fDACUpdate[0] = Update_DAC_ULD;
////	fDACUpdate[1] = Update_DAC_LLD;


