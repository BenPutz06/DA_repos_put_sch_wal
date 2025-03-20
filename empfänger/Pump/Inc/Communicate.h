/******************** (C) COPYRIGHT Otmar Putz ********************
* File Name          : Communicate.h
* Author             : Otmar Putz
* Date First Issued  : 06/11/2017
* Description        : Header File Communicate.c
*
********************************************************************************
* History:
* 
*******************************************************************************/


/* Define to prevent recursive inclusion ------------------------------------ */

#ifndef __COMMUNICATE_H
#define __COMMUNICATE_H

/* Includes ------------------------------------------------------------------*/
#include "Main.h"
#include "DataManage.h"


/* Exported defines ----------------------------------------------------------*/

/********************************************************************************
*
* File Name          : Functions.h
*
********************************************************************************/


void ProgramState(void);

void CheckPCTransferStatus(void);

void SendPCPackager(void);																			//Sendstring for PC Communication

void GetPCDataProcessing(void);

void CheckRFTransferStatus(void);

void RF_UART_TXTransferComplete_Callback(void);

void CheckUART_RF_RX (uint8_t TriggerType);

void RFDataCollecting(uint8_t RX_Buffer[], uint8_t Position, uint8_t TriggerType);

void RxRFFindStart(uint8_t ** RXString, uint8_t *StrLength);

void GetRFDataProcessing(void);

void SendSPSPackager(void);





#endif
