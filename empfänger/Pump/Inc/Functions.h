/**
  ******************************************************************************
  * @file    Functions.h
  * @author  Otmar Putz  Seibersdorf Laboratories
  * @brief   Header File for Functions.c 
  *          which contains common used  functions for programm sequence
  *          
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2024 Seibersdorf Labor
  * All rights reserved.</center></h2>
  *
  * 
  *
  ******************************************************************************
  */
	
	/* Includes ------------------------------------------------------------------*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FUNCTIONS_H
#define __FUNCTIONS_H
	



#include "DataManage.h"


uint16_t ConvertByteArrToUint(uint8_t ArrByte[2]);

uint8_t CalcBCC(uint8_t* U8,uint8_t Cnt);
void pCalcBCC(uint8_t* pU8, uint8_t Cnt);
void ppCalcBCC(uint8_t* pU8, uint8_t Cnt, uint8_t *BCC);


uint8_t HexStringToBytes(char s[], uint8_t *bData);
uint8_t BytesToHexString(char *s, uint8_t *bData, uint8_t DataCount);

void Initialize_Text(char* Desti, char* Source);

void Schreib_I2C(void);

void SET_Error_BlinkCode(uint8_t ErrorTyp[]);

#endif






///**
//  * @brief ADC2 Initialization Function
//  * @param None
//  * @retval None
//  */
//static void ADC1_Init(void)
//{


//  LL_ADC_InitTypeDef ADC_InitStruct = {0};
//  LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};

//  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

//  /* Peripheral clock enable */
//  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_ADC12);

//  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
//  /**ADC2 GPIO Configuration
//  PA1   ------> ADC1_IN2
//  */
//  GPIO_InitStruct.Pin = LL_GPIO_PIN_1;
//  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
//  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);



//  /** Common config
//  */
//  ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
//  ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
//  ADC_InitStruct.LowPowerMode = LL_ADC_LP_MODE_NONE;
//  LL_ADC_Init(ADC1, &ADC_InitStruct);
//  ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
//  ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_DISABLE;
//  ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
//  ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_SINGLE;
//  ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_LIMITED;
//  ADC_REG_InitStruct.Overrun = LL_ADC_REG_OVR_DATA_OVERWRITTEN;
//  LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);
//  LL_ADC_SetCommonClock(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_CLOCK_ASYNC_DIV1);

//  /* Enable ADC internal voltage regulator */
//  LL_ADC_EnableInternalRegulator(ADC1);
//  /* Delay for ADC internal voltage regulator stabilization. */
//  /* Compute number of CPU cycles to wait for, from delay in us. */
//  /* Note: Variable divided by 2 to compensate partially */
//  /* CPU processing cycles (depends on compilation optimization). */
//  /* Note: If system core clock frequency is below 200kHz, wait time */
//  /* is only a few CPU processing cycles. */
//  uint32_t wait_loop_index;
//  wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US * (SystemCoreClock / (100000 * 2))) / 10);
//  while(wait_loop_index != 0)
//  {
//    wait_loop_index--;
//  }

//  /** Configure Regular Channel
//  */
//  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_VREFINT);
//  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_VREFINT, LL_ADC_SAMPLINGTIME_1CYCLE_5);
//  LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_VREFINT, LL_ADC_SINGLE_ENDED);
//  LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_PATH_INTERNAL_VREFINT);
//  /* USER CODE BEGIN ADC2_Init 2 */

//  /* USER CODE END ADC2_Init 2 */

//}
