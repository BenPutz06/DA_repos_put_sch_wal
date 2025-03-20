/******************** (C) COPYRIGHT Otmar Putz ********************
* File Name          : Declaration.h
* Author             : Otmar Putz
* Date First Issued  : 25/01/2024
* Description        : Header Variablen declarations current project 
*
********************************************************************************
* History:
* 
*******************************************************************************/


/* Define to prevent recursive inclusion ------------------------------------ */


#ifndef __DECLARATIONS_H
#define __DECLARATIONS_H

/* Includes ------------------------------------------------------------------*/

//#include "stm32l1xx.h"
#include "Main.h"
//#include "Parameter.h"
#include "DataManage.h"

/* Exported defines ----------------------------------------------------------*/

/********************************************************************************
*
* File Name          : Declaration.h
*
********************************************************************************/


//extern uint16_t		LED_Blinky;								// LED counter for blinking  frequency

///**
//  * @brief RX buffers for storing received data
//  */
////extern uint8_t aRXPCBufferA[RXPC_BUFFER_SIZE];
////extern uint8_t aRXPCBufferB[RXPC_BUFFER_SIZE];

//typedef struct DATARxTyp RxPCPack; 										// Datenstruktur Empfangsdaten von PC	
//extern DATATxTyp TxPCPack;


//extern uint8_t 					*PSW;
//extern 	uint8_t 			*ErrorByte;									//contains the error bits

//extern	uint16_t				ProcessTick; 							//zählt die Prozessdurchläufe

////extern PFlagsTyp PFlags;

////extern TRANSFERSTATES 	sPCTransfer;							// Statemachine for Datatransfer to PC via UART

////extern PCDATAPACKTyp		PCRxDataPack;
////extern PCDATAPACKTyp		PCTxDataPack;


//extern struct UpdateFctListTyp *UpdateFctList;

extern uint32_t 	aTIM6Val[4];
//extern uint8_t		TimeTicker;

//extern uint16_t 			*MeasValue;
//extern uint32_t				TIM1_Clock;
//extern uint16_t				TIM1_PWM_Width;
////extern CLOCKType			ClockLLC;
////extern DACType 				DAC1Data;

////extern uint16_t aTIM6Val[TIM6_DATA_BUFFER_SIZE];
////extern uint16_t uhTIM2Val;

////extern uint16_t aADCxConvertedData[ADC_CONVERTED_DATA_BUFFER_SIZE]; 		// ADC group regular conversion data */
////extern uint16_t uhADCxConvertedData; 								// ADC group regular conversion data */ 
//extern uint16_t uhADCxConvertedData_Voltage_mVolt;  // Value of voltage calculated from ADC conversion data (unit: mV) */

//extern uint8_t ubDmaTransferStatus;									// Variable set into DMA interruption callback 



////extern STATETyp State;





#endif
