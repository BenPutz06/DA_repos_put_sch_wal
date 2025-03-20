/******************** (C) COPYRIGHT Otmar Putz ********************
* File Name          : Communicate.c
* Author             : Otmar Putz
* Date First Issued  : 05.11.2017
* Description        : Functions for the Communication with SSM1+ and internal I2C
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
#include "Functions.h"
#include "Communicate.h"
#include "stdlib.h"
#include "string.h"




/**
	* @brief  SendPCPackager: Convert all Data from PCTxDataPack to one TxPack Sendstring for PC communication
	*					including STX, ETX, CRC
	*								// Read Request from Master
			// Adress = 0xFF  =  Cmd for Read Request
			// Cnt = Amount of Data to send
			// Data = Data to Send
  * @param  None, Information is stored in PCTXDataPack
	*		 Write Request from Programm
	*		 .Adress = Mem Location from Value to be written
	*		 .Cnt = Amount of Data to send
	*		 .Data = Data to Send
  * @retval None
  */


void SendPCPackager()
{

	uint8_t	LoopCnt = 0;
	uint8_t bCRC;
	uint8_t SendByte[PCTxDataPack.Cnt];
	uint8_t *pStart;


		if (TxPCPack.TxdSucc == 0)  				//letzte Übertragung erfolgreich
			{

			TxPCPack.pBufferSend = (uint8_t *) malloc(sizeof(uint8_t)*(11+PCTxDataPack.Cnt *2)); 	// 11 Zeichen (Grundstruktur) + Datenanzahl*2 
			pStart = TxPCPack.pBufferSend;
			*TxPCPack.pBufferSend = STX;
			TxPCPack.pBufferSend++;
		  strncpy((char *) TxPCPack.pBufferSend, ID3, 3);
			TxPCPack.pBufferSend += 3;
		  BytesToHexString((char *) TxPCPack.pBufferSend,&PCTxDataPack.Adress,1);
			TxPCPack.pBufferSend += 2;
			BytesToHexString((char *) TxPCPack.pBufferSend,&PCTxDataPack.Cnt,1);
			TxPCPack.pBufferSend += 2;
		
			do
			{
				 //SendByte[LoopCnt] = *((uint8_t *) SRAM_PARAM_OFFSET + PCTxDataPack.Data[0]+LoopCnt);
				SendByte[LoopCnt] = PCTxDataPack.Data[LoopCnt];
			
			} while ((++LoopCnt) < PCTxDataPack.Cnt);

			BytesToHexString((char *) TxPCPack.pBufferSend,SendByte, LoopCnt);
			TxPCPack.pBufferSend += (LoopCnt*2);
			
			bCRC = CalcBCC(&SendByte[0],LoopCnt);
			
			BytesToHexString((char *) TxPCPack.pBufferSend,&bCRC,1);
		
			TxPCPack.pBufferSend += 2;

			*TxPCPack.pBufferSend = ETX;
			TxPCPack.Size = 1+3+2+2+(LoopCnt*2)+2+1;

			TxPCPack.pBufferSend = pStart;
			TxPCPack.AckNeed = PCTxDataPack.AckReq;
			
			sPCTransfer = SendData;									//Neuer Zustand für Statemachine
			
			free(PCTxDataPack.Data);
			
			PCTxDataPack.State = 0;								// Datenübernahme abgeschlossen, Datentransfer kann starten
			PFlags.WrReq  = 0x00;
			PFlags.WrSend = 0x01;
//			*PFlags = (*PFlags & ~0x0080);				// Clr Bit 7	 Data write request
//			*PFlags = (*PFlags | 0x0040);					// Set Bit 6   Data write in Progress
			
			}


}

/**
	* @brief  ParsePCDataReceived(): Empfangenes Paket auf gültige (fehlerfreie)
	*					Übertragung testen, wenn ok 
	*					RxPCPack.fDataValid = 1 -  Freigabe für 


			// 
			// 
			// 
  * @param  None, Information is stored in PCTXDataPack
	*		 
	*		 
  * @retval None
  */



uint8_t ParsePCDataReceived()
{
	
//	uint8_t LoopCounter;
//	uint8_t DmyByte;
	char DmyTxt[128];
	uint8_t *pDmyTxtSrc;
//	char *pDmyTxtDest;
	

		 RxPCPack.fDataValid = 0;								//new Data - Reset Valid
	
	//Read first the ID (3chars)
	strncpy(&DmyTxt[0],(char *) RxPCPack.pBufferProcess,3);
	DmyTxt[3] = '\0';
	if (strcmp(&DmyTxt[0],ID3) != 0)
	{
		return 0;  //wrong ID3
	}
		 
		 pDmyTxtSrc = (RxPCPack.pBufferProcess + 3);

		//Read Write Adress	
		 strncpy(&DmyTxt[0], (char *) pDmyTxtSrc,2);
		 DmyTxt[2] = '\0';
		 if (HexStringToBytes(&DmyTxt[0], &PCRxDataPack.Adress) == 0)
		 {
				//Error wrong character
			 return 0;
		 }
		 
		 pDmyTxtSrc = (RxPCPack.pBufferProcess + 5);

		 //Read Data Cnt 	
		 strncpy(&DmyTxt[0], (char *) pDmyTxtSrc,2);
		 DmyTxt[2] = '\0';
		 if (HexStringToBytes(&DmyTxt[0], &PCRxDataPack.Cnt) == 0)
		 {
				//Error wrong character
			 return 0;
		 }
		 pDmyTxtSrc = (RxPCPack.pBufferProcess + 7);

		 //Read Data, Cnt have to multiplied by 2 because each Value consists of two signs
		 //z.B. Wert: 0x2A entspricht Übertragung von zwei Zeichen: "2" und "A"
		 strncpy(&DmyTxt[0], (char *) pDmyTxtSrc,(PCRxDataPack.Cnt * 2));
		 DmyTxt[PCRxDataPack.Cnt * 2] = '\0';
		 PCRxDataPack.Data = (uint8_t *) malloc (sizeof(uint8_t)*PCRxDataPack.Cnt);
		 if (HexStringToBytes(&DmyTxt[0], PCRxDataPack.Data) == 0)
		 {
				//Error wrong character
			 return 0;
		 }
		 pDmyTxtSrc = (RxPCPack.pBufferProcess + (PCRxDataPack.Cnt * 2) +7);

		 //Read Data 	
		 strncpy(&DmyTxt[0], (char *) pDmyTxtSrc,2);
		 DmyTxt[2] = '\0';
		 if (HexStringToBytes(&DmyTxt[0], &PCRxDataPack.CRC_Data) == 0)
		 {
				//Error wrong character
			 return 0;
		 }
		 
		 if (PCRxDataPack.CRC_Data != CalcBCC(PCRxDataPack.Data,PCRxDataPack.Cnt))
		 {
			 
			 return 0; //BCC Error
			 
		 }
		 RxPCPack.fDataValid = 1;
		 return 1;
	
}

/**
	* @brief  CheckPCTransferStatus(): PC Datenübertragung gemanaged
	*					Zustände von sPCTransfer 
	*								
			// 
			// 
			// 
  * @param  None, Information is stored in PCTXDataPack
	*		 
	*		 
  * @retval None
  */


void CheckPCTransferStatus()
{

	
	   switch (sPCTransfer)
		 {
			 
			 case RxTxIdle:
				 
			 if (RxPCPack.fBufferReady == 1)  //Set in Fkt: "USART_PC_CharReception_Callback" when paket ok
					{
						sPCTransfer = GetData;
						
					}
					else if (TxPCPack.DataPending)
					{
						sPCTransfer = SendData;
					}
					
										
					if (PFlags.WrReq == 0x01) //&& (TxPCPack.DataPending == 0))
					{
								SendPCPackager();
					}
			 
			 break;
			 
			 case GetData:
				 
				 if (PCRxDataPack.State == 0)
				 {
						if (RxPCPack.fBufferReady == 1)
						{	
							ParsePCDataReceived();
							RxPCPack.fBufferReady = 0;
						}

						if (RxPCPack.fDataValid)
						{
								GetPCDataProcessing();
						}
						else //Data is invalid
						{

								RxPCPack.fDataValid = 0;
								sPCTransfer = ErrorCall;
							
								//Error Invalid Data
						}
						
				 }
				 if (PCRxDataPack.State == 1)
				 {
						free(PCRxDataPack.Data);
					  PCRxDataPack.State = 0;					// ready for next
				 }
			 
			 
			 break;
			 
			 case SendData:
				 
							if (TxPCPack.TxdSucc == 0)
							{
								
								SendUartTx(UART_PC,TxPCPack.pBufferSend,TxPCPack.Size,&TxPCPack.TxdSucc);
								if (TxPCPack.TxdSucc == 1)	//successfull
								{
									TxPCPack.TxdSucc = 0;
									if (TxPCPack.AckNeed)
									{
											RxPCPack.fAckReceiving = 1;
											RxPCPack.sAckTimeOut.TimeOutAct 	= 1;
											RxPCPack.sAckTimeOut.TimeOutCnt	= 0;
											RxPCPack.sAckTimeOut.TimeOutMax	= 20;
											sPCTransfer = GetAck;
									}
									else if (TxPCPack.AckNeed == 0)
											sPCTransfer = RxTxIdle;
							}
								else if (TxPCPack.TxdSucc != 1)
								{
									//Error UART Data Write
									sPCTransfer = ErrorCall;
								}
								TxPCPack.DataPending = 0;							// Data Transmission finished
								free(TxPCPack.pBufferSend);	
							}
			 
			 break;
			 
			 case GetAck:
				 
					if (RxPCPack.fAckReceiving == 2)
					{
						if (strncmp((char *)RxPCPack.pBufferProcess,ACK,3) == 0)				//Compare received string with ACK
						{
							RxPCPack.fAckReceiving = 0;
							if (PFlags.WrSend == 1)		//Test if Wr Sending Flag is set (Wr Request from Programm)
							{
								PFlags.WrSend = 0;      //Clr Wr Sending Flag
								PCTxDataPack.State	= 1;
							}
							sPCTransfer = RxTxIdle;
						}
						else 
						{
							//Fehler Acknowledge	
							sPCTransfer = ErrorCall;
						}
					}
				  else if (RxPCPack.fAckReceiving == 1)  //waiting for Ack
					 {
							if (RxPCPack.sAckTimeOut.TimeOutAct)
							{
								if ((++RxPCPack.sAckTimeOut.TimeOutCnt) > (RxPCPack.sAckTimeOut.TimeOutMax))
								{
									
									RxPCPack.fAckReceiving = 0;
									RxPCPack.sAckTimeOut.TimeOutAct = 0;
									PCTxDataPack.State = 0;
									PFlags.WrSend = 0;
									sPCTransfer = RxTxIdle;
									*PSW &= 0xEF;								// Clear MeasureBit
									State.Status |= 0x10;				// Set Statusänderungsbit damit PSW geändert
									
								}
							}
					 }
						
				
				 
				
			 break;
			 
			 case SendAck:
				 TxPCPack.pBufferSend = (uint8_t *) malloc(sizeof(uint8_t)*3);
				 strncpy((char *)TxPCPack.pBufferSend,ACK, 3);
				 SendUartTx(UART_PC,TxPCPack.pBufferSend,3,&TxPCPack.TxdSucc);
			   free(TxPCPack.pBufferSend);
			   if (TxPCPack.TxdSucc == 1)
				 {
						if (PCRxDataPack.Update == 3)
						{
							EEDataPack.EETxSendFlag = 1;
						}
						TxPCPack.TxdSucc = 0;
					  sPCTransfer = RxTxIdle;
				 }
				 
			 
			 break;
				 
			 case ErrorCall:
				
					//Ausgabe Error
			 
			 break;
			 
			 default:
				break;
			 
			 
		 }

	
}







/**
	* @brief  GetPCDataProcessing(): 	empfangen Daten werden  analysiert
	*					
			// 
			// 
			// 
  * @param  None, Information is stored in PCRxDataPack.Data
	*		 //   
	*		 
  * @retval None
  */



void GetPCDataProcessing(void)
{
	
	static uint8_t	LoopCnt = 0, AdressCnt = 0;
	uint8_t bCRC, dmyData;
	uint8_t SendByte[RXPC_BUFFER_SIZE];
	uint8_t  SendByteCounter = 0;	
	uint8_t *pStart;
	uint32_t	StartSetStateAdress, StartActStateAdress, StartSpeedAdress, StartOPStateAdress; 
	int8_t  ValveSpeed = 0;
	
	
	PCRxDataPack.State = 2;
	
	switch (PCRxDataPack.Adress)
	{
		case PgmStatus:
			
			LoopCnt = 0;
			
			if (PCRxDataPack.Cnt == 0x01) 		//check if only one Byte is sent
			{
				if (PCRxDataPack.Data[0] == PCRxDataPack.CRC_Data)		//Data and CRC have to be equal (only one Byte)
				{
						if (((*PSW) & 0xF0) != (PCRxDataPack.Data[0] & 0xF0))		//check if PSW is changed
						{
							State.Status |= 0x10;								
 						}
						*PSW = (PCRxDataPack.Data[0] & 0xF0);						//Bit0..Bit3 is ReadOnly
						PCRxDataPack.State = 1;													// Datenübernahme erfolgreich
						sPCTransfer = SendAck;
					
				}
				
			}
		
		break;
			
		case DevStatus:

			AdressCnt = 0;
	
			TxPCPack.pBufferSend = (uint8_t *) malloc(sizeof(uint8_t)*(1+3+(1+1+DevStatusDataSize+1)*2+1+1)); 
		//STX+ID+(Frame-ID + DataCnt + Data + CRC)*2..weil Hexdarstellung + ETX + 1..saveByte :-)
			pStart = TxPCPack.pBufferSend;
			*TxPCPack.pBufferSend = STX;
			TxPCPack.pBufferSend++;
		  strncpy((char *) TxPCPack.pBufferSend, ID3, 3);
			TxPCPack.pBufferSend += 3;
		  dmyData = SRAM_DSW;																									//Kennung für Device Status Frame
		  BytesToHexString((char *) TxPCPack.pBufferSend,&dmyData,1);
			TxPCPack.pBufferSend += 2;
			dmyData = DevStatusDataSize;
			BytesToHexString((char *) TxPCPack.pBufferSend,&dmyData,1);
			TxPCPack.pBufferSend += 2;	

			if (PCRxDataPack.Cnt == DevStatusDataSize)				//Check datasize
			
			{
				StartSetStateAdress = (SRAM_PARAM_OFFSET + SRAM_FeederSetState);
				StartActStateAdress = (SRAM_PARAM_OFFSET + SRAM_FeederActState);
				StartSpeedAdress = 		(SRAM_PARAM_OFFSET + SRAM_ScrewSpeed);
				
				LoopCnt = 0;
				do
				{
					*((uint8_t *) StartSetStateAdress +LoopCnt) = PCRxDataPack.Data[AdressCnt++];
					SendByte[SendByteCounter++] = *((uint8_t *) StartActStateAdress +LoopCnt);
//					SendByte[SendByteCounter++] = *((uint8_t *) SRAM_PARAM_OFFSET + SRAM_ScrewActState +LoopCnt);					
					LoopCnt+=1;
			
				} while ((LoopCnt) < OPERATIONAL_DATA_SIZE);	
		
				LoopCnt = 0;

				do
				{
					*((uint8_t *) SRAM_PARAM_OFFSET + SRAM_ScrewSpeed +LoopCnt) = PCRxDataPack.Data[AdressCnt];

					SendByte[SendByteCounter++] = *((uint8_t *) StartSpeedAdress+LoopCnt) + 1;
					//convert speed info from SetState and Speed (Direction and speed) into signed speed (+/- Speed)
					ValveSpeed = SpeedUserToValveConvert(*((uint8_t *) SRAM_PARAM_OFFSET + SRAM_ScrewSetState+LoopCnt), ConvertByteArrToUint(&PCRxDataPack.Data[AdressCnt++]));
					UpdateValuePeriph((SRAM_ScrewSpeed + LoopCnt), ValveSpeed);
					LoopCnt+=1;
			
				} while ((LoopCnt) < PERIPHERAL_SPEED_SIZE);		

				LoopCnt = 0;
				do
				{
					//*((uint8_t *) SRAM_PARAM_OFFSET + SRAM_PumpDevState +LoopCnt) = PCRxDataPack.Data[AdressCnt++];    	// this Parameters are only Transmitted
																																																								//	for the "user" read only	
					 SendByte[SendByteCounter++] = *((uint8_t *) SRAM_PARAM_OFFSET + SRAM_RotaryActSpeed +LoopCnt);
					//UpdateValuePeriph((PCRxDataPack.Adress + LoopCnt), ConvertByteArrToUint(&PCRxDataPack.Data[LoopCnt]));
					LoopCnt+=1;
			
				} while ((LoopCnt) < PERIPHERAL_OP_STATE_SIZE);	
				
				LoopCnt = 0;
				do													//Read and Write Reserve Variable from DataStream
				{
					*((uint8_t *) SRAM_PARAM_OFFSET + SRAM_ReserveVar1 +LoopCnt) = PCRxDataPack.Data[AdressCnt++];
					SendByte[SendByteCounter++] = *((uint8_t *) SRAM_PARAM_OFFSET + SRAM_ReserveVar1 +LoopCnt);
//					SendByte[SendByteCounter++] = *((uint8_t *) SRAM_PARAM_OFFSET + SRAM_ScrewActState +LoopCnt);					
					LoopCnt+=1;
			
				} while ((LoopCnt) < RESERVE_VAR_SIZE);				

				BytesToHexString((char *) TxPCPack.pBufferSend,SendByte, SendByteCounter);
				TxPCPack.pBufferSend += (SendByteCounter * 2);																			//increase pointer (memadress)

				bCRC = CalcBCC(&SendByte[0],SendByteCounter);
				BytesToHexString((char *) TxPCPack.pBufferSend,&bCRC,1);
				
				TxPCPack.pBufferSend += 2;

				*TxPCPack.pBufferSend = ETX;
			TxPCPack.Size = 1+3+2+2+(SendByteCounter*2)+2+1;

			TxPCPack.pBufferSend = pStart;
			TxPCPack.AckNeed = 0;
			
			sPCTransfer = SendData;
			
			PCRxDataPack.State = 1;								// Datenübernahme erfolgreich			
				
			}

		break;
		
		case RdReq:
			
			LoopCnt = 0;
			
			if (TxPCPack.TxdSucc == 0)  				//letzte Übertragung erfolgreich
			{
			// Read Request from Master
			// Adress = 0xFF  =  Cmd for Read Request
			// Cnt = Amount of Data to send
			// Data = Startadress of requested Data (always 1 Byte)
			TxPCPack.pBufferSend = (uint8_t *) malloc(sizeof(uint8_t)*(11+PCRxDataPack.Data[1]*2)); 	// 11 Zeichen (Grundstruktur) + Datenanzahl*2 
			pStart = TxPCPack.pBufferSend;
			*TxPCPack.pBufferSend = STX;
			TxPCPack.pBufferSend++;
		  strncpy((char *) TxPCPack.pBufferSend, ID3, 3);
			TxPCPack.pBufferSend += 3;
		  BytesToHexString((char *) TxPCPack.pBufferSend,&PCRxDataPack.Data[0],1);
			TxPCPack.pBufferSend += 2;
			BytesToHexString((char *) TxPCPack.pBufferSend,&PCRxDataPack.Data[1],1);
			TxPCPack.pBufferSend += 2;
		
			do
			{
				 SendByte[LoopCnt] = *((uint8_t *) SRAM_PARAM_OFFSET + PCRxDataPack.Data[0]+LoopCnt);
			
			} while ((++LoopCnt) < PCRxDataPack.Data[1]);

			BytesToHexString((char *) TxPCPack.pBufferSend,SendByte, LoopCnt);
			TxPCPack.pBufferSend += (LoopCnt*2);
			
			bCRC = CalcBCC(&SendByte[0],LoopCnt);
			
			BytesToHexString((char *) TxPCPack.pBufferSend,&bCRC,1);
		
			TxPCPack.pBufferSend += 2;

			*TxPCPack.pBufferSend = ETX;
			TxPCPack.Size = 1+3+2+2+(LoopCnt*2)+2+1;

			TxPCPack.pBufferSend = pStart;
			TxPCPack.AckNeed = 0;
			
			sPCTransfer = SendData;
			
			PCRxDataPack.State = 1;								// Datenübernahme erfolgreich
			}
		
		break;
		
		default:
			
			if (State.PgmFlow == CONFIG_KALIB)				//Konfigurationsdaten werden geschickt
			{
				LoopCnt = 0;
				I2CTxBufferCnt = 0;
				EEDataPack.EERxTxAddress = EE_Store_address[PCRxDataPack.Adress];
				
				do
				{
					*((uint8_t *) SRAM_PARAM_OFFSET + PCRxDataPack.Adress+LoopCnt) = PCRxDataPack.Data[LoopCnt];
					//*((uint8_t *) SRAM_PARAM_OFFSET + PCRxDataPack.Adress+1) = PCRxDataPack.Data[1];
					I2CTxBuffer[(I2CTxBufferCnt++)] = PCRxDataPack.Data[LoopCnt];
					
			
				} while ((++LoopCnt) < PCRxDataPack.Cnt);				
				
//				I2CTxBufferCnt--;					//letzte Inkrementation wieder abgezogen
				EEDataPack.EETxDataCnt = I2CTxBufferCnt;
				EEDataPack.pEETxBuffer	= &I2CTxBuffer[0];
				EEDataPack.EETxSendFlag = 0;
				PCRxDataPack.State = 1;							//Datenstruktur_Defaultwerte übernommen 
//				PCTxDataPack.Update = 3;
				PCRxDataPack.Update = 3;						// EEPROM Update necessary
				sPCTransfer = SendAck;
				
			}
			
			else
			{
				LoopCnt = 0;
				do
				{
					*((uint8_t *) SRAM_PARAM_OFFSET + PCRxDataPack.Adress+LoopCnt) = PCRxDataPack.Data[LoopCnt];
					*((uint8_t *) SRAM_PARAM_OFFSET + PCRxDataPack.Adress+LoopCnt+1) = PCRxDataPack.Data[LoopCnt+1];

					if ((PCRxDataPack.Adress + LoopCnt) >= SRAM_PERIPH_VAR_START)
					{
							UpdateValuePeriph((PCRxDataPack.Adress + LoopCnt), ConvertByteArrToUint(&PCRxDataPack.Data[LoopCnt]));
					}
					else PCRxDataPack.Update = 0;

					LoopCnt+=2;
	
			
				} while ((LoopCnt) < PCRxDataPack.Cnt);
			
			
				PCRxDataPack.State = 1;							//Datenstruktur_Defaultwerte übernommen 
				sPCTransfer = SendAck;
			
			}
			
		break;
		
	}
}



/**
	* @brief  CheckPCTransferStatus(): PC Datenübertragung gemanaged
	*					Zustände von sPCTransfer 
	*								
			// 
			// 
			// 
  * @param  None, Information is stored in PCTXDataPack
	*		 
	*		 
  * @retval None
  */


void CheckRFTransferStatus()
{

	uint8_t	LoopCnt = 0;
	uint8_t Status[2];
//	uint8_t SendByte[PCRxDataPack.Cnt];

static	uint8_t pStart;
uint8_t *pStartData;
//static	uint8_t	Offset_Address = 0;	
//static	uint8_t	TableAddress = 0;	
//static  uint8_t EEStoreAddress = 0;
	
	   switch (sRFTransfer)
		 {
			 
			 case RxTxIdle:
				 
					if (RxRFPack.fBufferReady == 1)     // Data from Transmitter (Sender) received
					{
						RxRFFindStart(&RxRFPack.pBufferProcess, &(RxRFPack.dRxProcessChars));
						if (RxRFPack.fBufferReady > 1)
						{
							RxRFPack.fDataValid = 1;
							GetRFDataProcessing();
						}
						
						RxRFPack.fBufferReady = 0;
						
					}


					if (RxRFPack.fBufferReady == 2)					// Config Data from ZETAPLUS Modul received
					{
						//Test ob letztes Empfangenes Zeichen (=Empfangskanal) übereinstimmt
						if (RxRFPack.pBufferReception[RxRFPack.dRxReceptionChars] == RF_CHANNEL)
						{
							RxRFPack.fDataValid = 1;
							RxRFPack.fBufferReady = 0;
							
						}
						
					}
										
					if (PFlags.WrReq == 0x01) //&& (TxPCPack.DataPending == 0))
					{
								SendPCPackager();
					}
			 
			 break;
			 


			 case ProcessData:
				 

			 
			 
			 break;
			 
			 case SendData:
				 
								
							if (TxRFPack.DataPending == 2)
							{
								if (PFlags.RF_TX_Acitve == 0)    // Transmit (DMA) erfolgreich			
								{
									TxPCPack.DataPending = 0;							// Data Transmission finished
	//								free(TxPCPack.pBufferSend);	

									sRFTransfer = RxTxIdle;
								}
//								else if (TxPCPack.TxdSucc != 1)
//								{
//									//Error UART Data Write
//									sPCTransfer = ErrorCall;
//								}

							}
							else if (TxRFPack.DataPending == 0)
							{
								RF_UART_TXData(TxRFPack.pBufferSend, TxRFPack.Size, &Status[0]);
								TxRFPack.DataPending = 1;
							}



							
			 break;
					 
			 case ErrorCall:
				
					//Ausgabe Error
			 
			 break;
			 
			 default:
				break;
			 
			 
		 }

	
	 }


/**
  * @brief  End of RF Modul TX Transfer
  *         called ISR
	*					Reset TX Busy Flag
	*					Disable, TX-DMA Channel and  UART DMA Req 
  * @param  None 
  * @retval None
  */	 
	 
	 
void RF_UART_TXTransferComplete_Callback(void)
{
	
				LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_4);
				LL_USART_DisableDMAReq_TX(UART_RF);
	
				PFlags.RF_TX_Acitve = 0;
				TxRFPack.DataPending = 2;													//Übertragung erfolgreich
	
}	 
	 
	 
	 
void CheckUART_RF_RX (uint8_t TriggerType)
	{
    /*
     * Set old position variable as static.
     *
     * Linker should (with default C configuration) set this variable to `0`.
     * It is used to keep latest read start position,
     * transforming this function to not being reentrant or thread-safe
     */
    static size_t old_pos;
    size_t pos;

    /* Calculate current position in buffer and check for new data available */
    pos =  RXRF_BUFFER_SIZE- LL_DMA_GetDataLength(DMA1, LL_DMA_CHANNEL_5);
    if (pos != old_pos) {                       /* Check change in received data */
        if (pos > old_pos) {                    /* Current position is over previous one */
            /*
             * Processing is done in "linear" mode.
             *
             * Application processing is fast with single data block,
             * length is simply calculated by subtracting pointers
             *
             * [   0   ]
             * [   1   ] <- old_pos |------------------------------------|
             * [   2   ]            |                                    |
             * [   3   ]            | Single block (len = pos - old_pos) |
             * [   4   ]            |                                    |
             * [   5   ]            |------------------------------------|
             * [   6   ] <- pos
             * [   7   ]
             * [ N - 1 ]
             */
            RFDataCollecting(&UART_RF_RX_DMA_Buffer[old_pos], pos - old_pos,TriggerType);
        } else {
            /*
             * Processing is done in "overflow" mode..
             *
             * Application must process data twice,
             * since there are 2 linear memory blocks to handle
             *
             * [   0   ]            |---------------------------------|
             * [   1   ]            | Second block (len = pos)        |
             * [   2   ]            |---------------------------------|
             * [   3   ] <- pos
             * [   4   ] <- old_pos |---------------------------------|
             * [   5   ]            |                                 |
             * [   6   ]            | First block (len = N - old_pos) |
             * [   7   ]            |                                 |
             * [ N - 1 ]            |---------------------------------|
             */
            RFDataCollecting(&UART_RF_RX_DMA_Buffer[old_pos], RXRF_BUFFER_SIZE - old_pos,TriggerType);
            if (pos > 0) {
                RFDataCollecting(&UART_RF_RX_DMA_Buffer[0], pos,TriggerType);
            }
        }
        old_pos = pos;                          /* Save current position as old for next transfers */
    }
		else RFDataCollecting(&UART_RF_RX_DMA_Buffer[old_pos], pos - old_pos,TriggerType);
}
	 
	 
	 

/**
	* @brief  GetRFDataCollecting(): empfangen Daten werden in die Systemvariablen 
	*					umkopiert
	*								
			// 
			// 
			// 
  * @param  RX_Buffer (startpointer of RX_Buffer),  BufferCnt .. amount of elements
	*		 //   
	*		 
  * @retval None
  */



void RFDataCollecting(uint8_t* RX_Buffer, uint8_t BufferCnt, uint8_t TriggerType)
{
	
//	uint8_t	LoopCnt = 0;
//	uint8_t bCRC;
//	uint8_t SendByte[PCRxDataPack.Cnt];
	uint8_t *pStart,*pRcvBuff;
//static	uint8_t	Offset_Address = 0;
	static 	uint8_t	BYTE_CRC;
	static  uint8_t PayLoadCnt;
	uint8_t	Counter = 0;
	uint8_t	DownCounter;


	pStart = pRxBuff[RxRFActBufferID];
	
	while (Counter < BufferCnt)
	{
		
	   pStart[RxRFPack.dRxReceptionChars++] = RX_Buffer[Counter];
		 Counter++;	
		
	}
	
  if (TriggerType == 1)																				//IDLE Line detected - end of transmission
	{
		RxRFPack.pBufferProcess = pStart;
		RxRFPack.dRxProcessChars = RxRFPack.dRxReceptionChars;
		RxRFPack.dRxReceptionChars = 0;
		
		if (RxRFActBufferID == 0) RxRFActBufferID = 0x01;										//swap Rx Buffer
		else RxRFActBufferID = 0x00;	
    DMA_Tester1  ++;	
		
		if (DMA_Tester1 == 0x48)
		{
			RxRFPack.fDataValid = 2;
		}

		if (RxRFPack.dRxProcessChars > 5)													// test length of received String, 5 chars minimum (=header) 
		{																													// if smaller, Data not valid
			 RxRFPack.fBufferReady= 1;
			
		}
			
	}
		


	
	
	
	
}



void NextRxRFSearch(uint8_t ** RXString, uint8_t *StrLength)
{


	++(*RXString);
	(*StrLength)--;
	if ((*StrLength) > 0) RxRFFindStart(RXString, StrLength);			
	
}

/**
	* @brief  RxRFFindStart: search for Header in received Datastring
	*								

* @param  RXString: is a adresspointer, so the HeaderData is removed from received string
	*       and the actual (new) adress is returned (so the remaining data include only userdata
	*		 		StrLength: datasize: is as the RXString, reduced and returned   
	*		 
  * @retval None
  */



void RxRFFindStart(uint8_t ** RXString, uint8_t *StrLength)
{
	
				switch (RxRFPack.fBufferReady)
				{
					case 1:
									
						if (*RXString[0] == 0x23) 							// "#"
						{
							RxRFPack.fBufferReady = 2;
						}
	
						NextRxRFSearch(RXString, StrLength);
	
					break;
								
					case 2:
						if (*RXString[0] == 0x52) 							// "R"	...Data is from RemoteControl
						{

							if ((*RXString)[3] == STX)						//test if 4. Databyte is the Startbyte
							{
								RxRFPack.fBufferReady = 4;
								(*RXString) += 4;														// Set Datapointer to first Databyte of user Data
								(*StrLength) -= 4;													// reduce Datasize 
							}
							else
							{
								RxRFPack.fBufferReady = 0;
								NextRxRFSearch(RXString, StrLength);					// no valid start charakter
							}
						}
						else if (*RXString[0] == 0x3F)					//"?"			...Data is from RF Modul (=ConfigData)
						{	
							RxRFPack.fBufferReady = 3;
//							++RXString;
//							(*StrLength)--;
//							RxRFFindStart(RXString, StrLength)
						}
						else
						{
							RxRFPack.fBufferReady = 0;
							NextRxRFSearch(RXString, StrLength);					// no valid start charakter
						}
					break;
						
					default:
						
					break;
						
							
					}
				
	
}



void RFDataCopy(void)
{
	
	
//	   switch (RxRFPack.fBufferReady)
//		 {
//			 
//			 case 0:         //
//				 RxRFPack.
			 
//			 break;
	
	
//	switch (RxRFPack.fDataReceiving)
//	{
//		
//		case 0:      //searching 
//			do
//			{
//						RxRFPack.pBufferReception[RxRFPack.dRxReceptionChars] = *RX_Buffer;
//						if ((*RX_Buffer)
//				
//				
//				
//			}while (DownCounter > 0);
//		
//		
//		
//	
//	

//		if (0)
//	{		
//	do
//	{
//		RFDataPacket[Counter] = (*pStart);
//		pStart++;
//		Counter++;
//		
//	} while (Counter < BufferCnt);
//	RFDataPacket[Counter] = BufferCnt;
//	

//	
////	if (BufferCnt == RF_RCV_MODULDATA_SIZE)
////	{


//	
//	while ((BufferCnt--) > 0)								//searching for StartByte
//	{
//		switch (RxRFPack.fDataReceiving)
//		{
//			case 0:																//searching for first Byte Startpattern: "#"
//				if (*RX_Buffer == RFStartPattern)
//				{
//					RFRcvTimeOutTimer = 20;
//					RxRFPack.fDataReceiving = 4;				//1st Byte of Startpattern ok
//					RxRFPack.dRxReceptionChars = 0;			// Reset 
//				}	
//				
//			break;
//				
//			case 1:	

//					if (RxRFPack.dRxReceptionChars == RF_PAYLOAD_SIZE)					//check if all Payload Data is received
//					{
//						
//						BYTE_CRC = CalcBCC(&RxRFPack.pBufferReception[0],RF_PAYLOAD_SIZE);
//						if (BYTE_CRC == (*RX_Buffer)) 							//Last Byte is CRC, index begins with 0, 
//						{						
//								RX_Buffer++;
//							  if (*RX_Buffer == ETX)
//								{
//									RxRFPack.fBufferReady = 1;
//									RxRFPack.fDataReceiving = 0;
//									RxRFPack.fDataValid = 1;
//								}
//						}
//						else RxRFPack.fDataValid = 0;
//					}
//					else 
//					{
//						RxRFPack.pBufferReception[RxRFPack.dRxReceptionChars] = *RX_Buffer;
//						++RxRFPack.dRxReceptionChars;
//					}
//					
//			break;
//			
//			case 2:
//				if (*RX_Buffer > RF_MIN_RSSI_MIN)
//				{
//					RxRFPack.fDataReceiving = 6;		//Startpattern ok 
//				}
//				
//			break;
//			
//			case 3:																	
//				if (*RX_Buffer == RF_DATAPACKET_SIZE)				
//				{
//					RxRFPack.fDataReceiving = 2;		// Datengröße ist  ok
//				}
//			
//			break;
//				
//			case 4:																	
//				if (*RX_Buffer == RFRxDataPattern)
//				{
//					RxRFPack.fDataReceiving = 3;		// 2nd Byte of Startpattern ok
//				}
//				else if (*RX_Buffer == RFRxConfigPattern)
//				{
//						RxRFPack.fDataReceiving = 5;		//Read Configuration		
//				}
//				
//			break;
//					
//			case 5:
//			
//				RxRFPack.pBufferReception[RxRFPack.dRxReceptionChars] = *RX_Buffer;
//			
//				if (RxRFPack.dRxReceptionChars == 7)
//				{
//						RxRFPack.fBufferReady = 2;
//						RxRFPack.fDataReceiving = 0;					
//					
//				}
//				else (++RxRFPack.dRxReceptionChars);
//			
//			break;
//				
//			case 6:						//Check STX of Payload
//					if (*RX_Buffer == STX)					//check if start
//					{
//						 RxRFPack.fDataReceiving = 1;
//					}
//			break;
//					
//				
//			default:
//				
//			break;
//			
//		}
//				
//		++RX_Buffer;
//		
//	}
//	
//	
//	//SPSRxDataPack.State = 2;
	

}


/**
	* @brief  GetRFDataProcessing(): empfangen Daten werden in die Systemvariablen 
	*					umkopiert
	*					Es werden immer alle Systemvariablen (Zustände) übertragen
	*								
			// 
			// 
			// 
  * @param  None, Information is stored in PCRxDataPack.Data
	*		 //   
	*		 
  * @retval None
  */



void GetRFDataProcessing(void)
{
	
	static uint8_t	LoopCnt = 0;
	uint8_t bCRC;
	uint8_t SendByte[RF_DATAPACKET_SIZE+6];			//PayloadSize+STX+ETX+CRC+ 2 Byte Reserve
	uint8_t *pStart;
	uint32_t	StartSetStateAdress, StartActStateAdress, StartSpeedAdress, StartOPStateAdress; 
	uint8_t  SendByteCounter = 0, AdressCnt = 0;	
	int8_t  ValveSpeed = 0;	
	uint8_t * pPayLoadStart;
	
	//check CRC
	
	//when ok - umkompieren der Systemvariablen
	
	//Buffer wieder "löschen"  - Startadresse von Buffer wieder 0

			RxRFPack.dRxProcessChars = 0;
	
////			TxRFPack.pBufferSend = (uint8_t *) malloc(sizeof(uint8_t)*(RF_DATAPACKET_SIZE+5+1));   //STX+Payload + CRC + ETX + 
////																																				//..+5="ATS+RF-Kanal+Paketgröße ..+1=..saveByte :-)
			pStart = TxRFPack.pBufferSend;

			*TxRFPack.pBufferSend = 65;			//..A
			TxRFPack.pBufferSend++;
			*TxRFPack.pBufferSend = 84;			//..T
			TxRFPack.pBufferSend++;
			*TxRFPack.pBufferSend = 83;			//..S
			TxRFPack.pBufferSend++;
			*TxRFPack.pBufferSend = RF_CHANNEL;
			TxRFPack.pBufferSend++;
			*TxRFPack.pBufferSend = RF_DATAPACKET_SIZE;
			TxRFPack.pBufferSend++;
			*TxRFPack.pBufferSend = STX;
			TxRFPack.pBufferSend++;
			
			pPayLoadStart = TxRFPack.pBufferSend;															//For CRC Berechnung

			if (RxRFPack.fDataValid)	
			{
				StartSetStateAdress = (SRAM_PARAM_OFFSET + SRAM_FeederSetState);
				StartActStateAdress = (SRAM_PARAM_OFFSET + SRAM_FeederActState);
				StartSpeedAdress = 		(SRAM_PARAM_OFFSET + SRAM_ScrewSpeed);
				
				LoopCnt = 0;
				do
				{
					*((uint8_t *) StartSetStateAdress +LoopCnt) = RxRFPack.pBufferProcess [AdressCnt++];
					//SendByte[SendByteCounter++] = *((uint8_t *) StartActStateAdress + LoopCnt);
					*TxRFPack.pBufferSend = *((uint8_t *) StartActStateAdress + LoopCnt);
					TxRFPack.pBufferSend++;					
//					SendByte[SendByteCounter++] = *((uint8_t *) SRAM_PARAM_OFFSET + SRAM_ScrewActState +LoopCnt);					
					LoopCnt+=1;
			
				} while ((LoopCnt) < OPERATIONAL_DATA_SIZE);	
		
				LoopCnt = 0;

				do
				{
					*((uint8_t *) SRAM_PARAM_OFFSET + SRAM_ScrewSpeed +LoopCnt) = RxRFPack.pBufferProcess[AdressCnt];

					//SendByte[SendByteCounter++] = *((uint8_t *) StartSpeedAdress+LoopCnt) + 1;
					*TxRFPack.pBufferSend = *((uint8_t *) StartSpeedAdress + LoopCnt);
					TxRFPack.pBufferSend++;					
					//convert speed info from SetState and Speed (Direction and speed) into signed speed (+/- Speed)
					ValveSpeed = SpeedUserToValveConvert(*((uint8_t *) SRAM_PARAM_OFFSET + SRAM_ScrewSetState+LoopCnt), ConvertByteArrToUint(&RxRFPack.pBufferProcess[AdressCnt++]));
					UpdateValuePeriph((SRAM_ScrewSpeed + LoopCnt), ValveSpeed);
					*((uint8_t *) SRAM_PARAM_OFFSET + SRAM_ScrewActState +LoopCnt) = *((uint8_t *) SRAM_PARAM_OFFSET + SRAM_ScrewSetState +LoopCnt);
					LoopCnt+=1;
			
				} while ((LoopCnt) < PERIPHERAL_SPEED_SIZE);		

				LoopCnt = 0;
				do
				{
					//*((uint8_t *) SRAM_PARAM_OFFSET + SRAM_PumpDevState +LoopCnt) = PCRxDataPack.Data[AdressCnt++];    	// this Parameters are only Transmitted
																																																								//	for the "user" read only	
					// SendByte[SendByteCounter++] = *((uint8_t *) SRAM_PARAM_OFFSET + SRAM_RotaryActSpeed +LoopCnt);
					*TxRFPack.pBufferSend = *((uint8_t *) SRAM_PARAM_OFFSET + SRAM_RotaryActSpeed +LoopCnt);
					TxRFPack.pBufferSend++;						
					//UpdateValuePeriph((PCRxDataPack.Adress + LoopCnt), ConvertByteArrToUint(&PCRxDataPack.Data[LoopCnt]));
					LoopCnt+=1;
			
				} while ((LoopCnt) < PERIPHERAL_OP_STATE_SIZE);	
				
				LoopCnt = 0;
				do													//Read and Write Reserve Variable from DataStream
				{
					*((uint8_t *) SRAM_PARAM_OFFSET + SRAM_ReserveVar1 +LoopCnt) = RxRFPack.pBufferProcess[AdressCnt++];
					//SendByte[SendByteCounter++] = *((uint8_t *) SRAM_PARAM_OFFSET + SRAM_ReserveVar1 +LoopCnt);
					*TxRFPack.pBufferSend = *((uint8_t *) SRAM_PARAM_OFFSET + SRAM_ReserveVar1 +LoopCnt);
					TxRFPack.pBufferSend++;					
					LoopCnt+=1;
			
				} while ((LoopCnt) < RESERVE_VAR_SIZE);				

				bCRC = CalcBCC(pPayLoadStart,RF_PAYLOAD_SIZE);

				*TxRFPack.pBufferSend = bCRC;
				TxRFPack.pBufferSend++;					
				
				*TxRFPack.pBufferSend = ETX;

				TxRFPack.Size = RF_MODULDATA_SIZE;

				TxRFPack.pBufferSend = pStart;
				TxRFPack.AckNeed = 0;
			
				TxRFPack.DataPending = 0;
				sRFTransfer = SendData;
				
//				free(TxPCPack.pBufferSend);
				RxRFPack.fDataValid = 0;
				
				RFRcvTimeOutTimer = RFRcvMaxTimeOut;
				LED_Power_Green.Blinking = LED_BLINK_SLOW;
			
				//RFRxDataPack.State = 1;								// Datenübernahme erfolgreich			
				
			}		
	
}


void BackupGetRFDataProcessing(void)
{
	
	static uint8_t	LoopCnt = 0;
	uint8_t bCRC;
	uint8_t SendByte[RF_DATAPACKET_SIZE+6];			//PayloadSize+STX+ETX+CRC+ 2 Byte Reserve
	uint8_t *pStart;
	uint32_t	StartSetStateAdress, StartActStateAdress, StartSpeedAdress, StartOPStateAdress; 
	uint8_t  SendByteCounter = 0, AdressCnt = 0;	
	int8_t  ValveSpeed = 0;	
	uint8_t * pPayLoadStart;
	
	//check CRC
	
	//when ok - umkompieren der Systemvariablen
	
	//Buffer wieder "löschen"  - Startadresse von Buffer wieder 0
			RxRFPack.dRxProcessChars = 0;
	
			TxRFPack.pBufferSend = (uint8_t *) malloc(sizeof(uint8_t)*(RF_DATAPACKET_SIZE+5+1));   //STX+Payload + CRC + ETX + 
																																				//..+5="ATS+RF-Kanal+Paketgröße ..+1=..saveByte :-)
			pStart = TxRFPack.pBufferSend;

			*TxRFPack.pBufferSend = 65;			//..A
			TxRFPack.pBufferSend++;
			*TxRFPack.pBufferSend = 84;			//..T
			TxRFPack.pBufferSend++;
			*TxRFPack.pBufferSend = 83;			//..S
			TxRFPack.pBufferSend++;
			*TxRFPack.pBufferSend = RF_CHANNEL;
			TxRFPack.pBufferSend++;
			*TxRFPack.pBufferSend = RF_DATAPACKET_SIZE;
			TxRFPack.pBufferSend++;
			*TxRFPack.pBufferSend = STX;
			TxRFPack.pBufferSend++;
			
			pPayLoadStart = TxRFPack.pBufferSend;															//For CRC Berechnung
//		  dmyData = SRAM_DSW;																									//Kennung für Device Status Frame
//		  BytesToHexString((char *) TxPCPack.pBufferSend,&dmyData,1);
//			TxPCPack.pBufferSend += 2;
//			dmyData = DevStatusDataSize;
//			BytesToHexString((char *) TxPCPack.pBufferSend,&dmyData,1);
//			TxPCPack.pBufferSend += 2;	

//			if (RxRFPack.dRxReceptionChars == (RF_PAYLOAD_SIZE + 1))				//Check Nutzdatengröße + CRC
			if (RxRFPack.fDataValid)	
			{
				StartSetStateAdress = (SRAM_PARAM_OFFSET + SRAM_FeederSetState);
				StartActStateAdress = (SRAM_PARAM_OFFSET + SRAM_FeederActState);
				StartSpeedAdress = 		(SRAM_PARAM_OFFSET + SRAM_ScrewSpeed);
				
				LoopCnt = 0;
				do
				{
					*((uint8_t *) StartSetStateAdress +LoopCnt) = RxRFPack.pBufferProcess [AdressCnt++];
					//SendByte[SendByteCounter++] = *((uint8_t *) StartActStateAdress + LoopCnt);
					*TxRFPack.pBufferSend = *((uint8_t *) StartActStateAdress + LoopCnt);
					TxRFPack.pBufferSend++;					
//					SendByte[SendByteCounter++] = *((uint8_t *) SRAM_PARAM_OFFSET + SRAM_ScrewActState +LoopCnt);					
					LoopCnt+=1;
			
				} while ((LoopCnt) < OPERATIONAL_DATA_SIZE);	
		
				LoopCnt = 0;

				do
				{
					*((uint8_t *) SRAM_PARAM_OFFSET + SRAM_ScrewSpeed +LoopCnt) = RxRFPack.pBufferProcess[AdressCnt];

					//SendByte[SendByteCounter++] = *((uint8_t *) StartSpeedAdress+LoopCnt) + 1;
					*TxRFPack.pBufferSend = *((uint8_t *) StartSpeedAdress + LoopCnt);
					TxRFPack.pBufferSend++;					
					//convert speed info from SetState and Speed (Direction and speed) into signed speed (+/- Speed)
					ValveSpeed = SpeedUserToValveConvert(*((uint8_t *) SRAM_PARAM_OFFSET + SRAM_ScrewSetState+LoopCnt), ConvertByteArrToUint(&RxRFPack.pBufferReception[AdressCnt++]));
					UpdateValuePeriph((SRAM_ScrewSpeed + LoopCnt), ValveSpeed);
					LoopCnt+=1;
			
				} while ((LoopCnt) < PERIPHERAL_SPEED_SIZE);		

				LoopCnt = 0;
				do
				{
					//*((uint8_t *) SRAM_PARAM_OFFSET + SRAM_PumpDevState +LoopCnt) = PCRxDataPack.Data[AdressCnt++];    	// this Parameters are only Transmitted
																																																								//	for the "user" read only	
					// SendByte[SendByteCounter++] = *((uint8_t *) SRAM_PARAM_OFFSET + SRAM_RotaryActSpeed +LoopCnt);
					*TxRFPack.pBufferSend = *((uint8_t *) SRAM_PARAM_OFFSET + SRAM_RotaryActSpeed +LoopCnt);
					TxRFPack.pBufferSend++;						
					//UpdateValuePeriph((PCRxDataPack.Adress + LoopCnt), ConvertByteArrToUint(&PCRxDataPack.Data[LoopCnt]));
					LoopCnt+=1;
			
				} while ((LoopCnt) < PERIPHERAL_OP_STATE_SIZE);	
				
				LoopCnt = 0;
				do													//Read and Write Reserve Variable from DataStream
				{
					*((uint8_t *) SRAM_PARAM_OFFSET + SRAM_ReserveVar1 +LoopCnt) = RxRFPack.pBufferProcess[AdressCnt++];
					//SendByte[SendByteCounter++] = *((uint8_t *) SRAM_PARAM_OFFSET + SRAM_ReserveVar1 +LoopCnt);
					*TxRFPack.pBufferSend = *((uint8_t *) SRAM_PARAM_OFFSET + SRAM_ReserveVar1 +LoopCnt);
					TxRFPack.pBufferSend++;					
					LoopCnt+=1;
			
				} while ((LoopCnt) < RESERVE_VAR_SIZE);				

				bCRC = CalcBCC(pPayLoadStart,RF_PAYLOAD_SIZE);

				*TxRFPack.pBufferSend = bCRC;
				TxRFPack.pBufferSend++;					
				
				*TxRFPack.pBufferSend = ETX;

				TxRFPack.Size = RF_MODULDATA_SIZE;

				TxRFPack.pBufferSend = pStart;
				TxRFPack.AckNeed = 0;
			
				TxRFPack.DataPending = 0;
				sRFTransfer = SendData;
				
				free(TxPCPack.pBufferSend);
				RxRFPack.fDataValid = 0;
			
				//RFRxDataPack.State = 1;								// Datenübernahme erfolgreich			
				
			}		
	
}




/**
	* @brief  SendPCPackager: Convert all Data from PCTxDataPack to one TxPack Sendstring for PC communication
	*					including STX, ETX, CRC
	*								// Read Request from Master
			// Adress = 0xFF  =  Cmd for Read Request
			// Cnt = Amount of Data to send
			// Data = Data to Send
  * @param  None, Information is stored in PCTXDataPack
	*		 Write Request from Programm
	*		 .Adress = Mem Location from Value to be written
	*		 .Cnt = Amount of Data to send
	*		 .Data = Data to Send
  * @retval None
  */


void SendSPSPackager(void)
{

static	uint8_t	LoopCnt = 0;
	uint8_t PayLoadCnt = 0;
	uint8_t DataCnt; 
//	uint8_t SendByte[DataCnt];
static	uint8_t *pStart;
	uint16_t	MOD_CRC;

	
	



}
