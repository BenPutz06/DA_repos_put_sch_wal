/**
  ******************************************************************************
  * @file    Prog_Seq.h
  * @author  Otmar Putz  Seibersdorf Laboratories
  * @brief   Header File for Prog_Seq.c 
  *          
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
	
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PROG_SEQ_H
#define __PROG_SEQ_H
	

	
	/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "Functions.h"
#include "Communicate.h"
#include "Declarations.h"
#include "stm32f3xx_ll_rcc.h"
#include "stdlib.h"

#include "Periph_Funct.h"


	void ProgramState(void);
	void Check_LED_State(void);





#endif

