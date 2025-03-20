/******************** (C) COPYRIGHT Otmar Putz ********************
* File Name          : Periph_init.h
* Author             : Otmar Putz
* Date First Issued  : 01/04/2010
* Description        : Header File Periph_init.c
*
********************************************************************************
* History:
* 
*******************************************************************************/


/* Define to prevent recursive inclusion ------------------------------------ */

/* Includes ------------------------------------------------------------------*/



/* Exported defines ----------------------------------------------------------*/

/********************************************************************************
*
* File Name          : GPIO_init.h
*
********************************************************************************/




/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PERIPH_INIT_H
#define __PERIPH_INIT_H


#include "main.h"
//#include "Parameter.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/




/* Exported functions ------------------------------------------------------- */



//void Main_Init (void);


//void SystemClock_Config(void);


void GPIO_Init(void);
void DMA_Init(void);

void EXTI_Init(void);

void TIM1_Init(void);						//PWM Ausgänge
void TIM2_Init(void);						//Impulszähler Zellenradschleuse	
void TIM6_Init(void);						//Taktgeber auslesen Werte TIM2
void TIM7_Init(void);						//TimeBase for Programmabläufe z.B LED blinken

void Configure_WWDG(void);
void Check_WWDG_Reset(void);


void Configure_ADC(void); 
void Activate_ADC(void);

void UART_RF_Init(void);
void UART_RF_RX_Start(void);
void UART_RF_RX_Stop(void);

uint8_t RF_MOD_RX_Init();
uint8_t RF_MOD_Read_Config();

void UART_PC_Init(void);
void UART_PC_RX_Start(void);

void Configure_I2C_Master(void);


void Error_I2C_Callback(void);
//void GPIO_Configuration(void);

//void DMA_Configuration(void);

//void Configure_DMA1(void);

//void Configure_DAC(void);

//void Activate_DAC(void);

void Configure_ADC(void);

void Activate_ADC(void);

//void Configure_USART(void);

//void Configure_TIM1(void);

//void Configure_TIM2(void);

//void TIM6_Init(void);


#endif
