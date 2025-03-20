/******************** (C) COPYRIGHT Otmar Putz ********************
* File Name          : Functions.h
* Author             : Otmar Putz
* Date First Issued  : 01/04/2010
* Description        : Header File Periph_init.c
*
********************************************************************************
* History:
* 
*******************************************************************************/


/* Define to prevent recursive inclusion ------------------------------------ */
#ifndef __PERIPH_FUNCT_H
#define __PERIPH_FUNCT_H

/* Includes ------------------------------------------------------------------*/
#include "Main.h"
#include "DataManage.h"


/* Exported defines ----------------------------------------------------------*/

/********************************************************************************
*
* File Name          : Functions.h
*
********************************************************************************/


#define sEE_OK                    0
#define sEE_FAIL                  1 


//extern DATARxTyp RxPCPack;

void Get_JumperConfig(void);

void PWM_ScrewSpeed_Update(int8_t Speed);
void PWM_RotarySpeed_Update(int8_t Speed);

void Update_Peripherals(void);

void Reset_Peripherie(void);

void INT_TIME_Update(uint16_t TValue);

void CounterReadTransferComplete_Callback(void);

void ADC_StartConversion(void);
void AdcDmaTransferComplete_Callback(void);


//void Start_PC_Reception(void);

void USART_PC_CharReception_Callback(void);



void SendUartTx(USART_TypeDef *USARTTyp, uint8_t *String, uint32_t Size, uint8_t *State);
void RF_UART_TXData(uint8_t *DMA_Buffer, uint32_t Size, uint8_t *State);

void RFRcvErrorReset(void);




void CounterReadTransferComplete_Callback(void);


//void TIM2TransferComplete_Callback(void);

int8_t SpeedUserToValveConvert(uint8_t ValveDirection, uint8_t SpeedValue);

uint8_t UpdateValuePeriph(uint8_t SRAM_Adress, int8_t Speed);
void fpUpdateParam(void (*Updateer)(int8_t), int8_t Wert);

void Read_Sensor_Switches(void);

//LEDs
void LED_BLINKING_Update(uint8_t Force);
void Copy_LED_Struct(struct LED_LIGHT *Code, struct LED_LIGHT *Ledtype);
void BACKUP_LED_STATUS(struct LED_LIGHT *Led);
void RESTORE_LED_STATUS(struct LED_LIGHT *Led);
void BLINKCODE_ERROR_Generate(struct LED_LIGHT *Code, uint8_t State);

void 	EXTI_Start_Callback(void);
void 	EXTI_End_Callback(void);
void 	EXTI_Scatter_Callback(void);


void Error_PC_Callback(void);

void LL_I2C_Write(void);

uint8_t EE_WriteBuffer(uint8_t DevAddr, uint8_t* pBuffer, uint8_t WriteAddr, uint16_t NumbToWrite);

uint8_t Handle_I2C_Master(uint8_t DevAddr, uint8_t* pBuffer, uint8_t WriteAddr, uint16_t NumbToWrite);

uint8_t EE_ReadBuffer(uint8_t DevAddr, uint8_t* pBuffer, uint8_t ReadAddr, uint16_t ByteToRead);

uint8_t sEE_ReadBuffer(uint8_t DevAddr, uint8_t* pBuffer, uint16_t ReadAddr, uint16_t ByteToRead);
void Read_EEPROM_Reception_Callback(void);
void READ_EEPROM_Complete_Callback(void);

uint8_t sEE_TIMEOUT_UserCallback(void);

#endif

