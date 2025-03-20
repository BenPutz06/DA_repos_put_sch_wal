 /******************** (C) COPYRIGHT Benjamin Putz ********************
* File Name          : Parameter.h
* Author             : Benjamin Putz
* Date First Issued  : 01/02/2024
* Description        : Parameter File for current project
*
********************************************************************************
* History:
* 
*******************************************************************************/


/* Define to prevent recursive inclusion ------------------------------------ */

/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef __PARAMETER_H
#define __PARAMETER_H


/* Includes ------------------------------------------------------------------*/

#include "Main.h"
#include "Periph_Init.h"
//#include "DataManage.h"
/* Exported defines ----------------------------------------------------------*/


#define USE_VCP_CONNECTION	1
#define USE_NUCLEO  				0				//0..Pump Board, 1..NucleoBoard 2..Spreader
#define USE_TIMER1_CLOCK		0				//0..PortA.10 used for SPS-Uart, 1..PortA.10 Timer1 - ClockOut


/********************************************************************************
*
* File Name          : Parameter.h
*
********************************************************************************/





//----------------------------------------------------------------------------------
//   Device INIT Variables
//----------------------------------------------------------------------------------


	#define		ProgrammPeriod								10							//Zyklus Programmdurchlauf [ms], 

	#define 	EEProm_ID											"P0"					
	#define 	DetektTypText									"PumpPutz"
	#define		DetektSerienNr								"Pump-001"
	#define		ProdDatum											"24052024"
	#define   FW_Version										"FW_V0001"
	#define		HW_Version										"HW_V0002"

	#define   DatumIni											"24052024"



//----------------------------------------------------------------------------------
//   System INIT Variables
//----------------------------------------------------------------------------------

#define 	JMP_CFG_DEFAULT_PARAM					0x06						// Default Parameter geladen, nicht vom EEPROM

#define		RFRcvMaxTimeOut								250							//2.5sek ist 2000/ProgrammPeriod = 2000ms/SysTick(1ms)/Programmperiod						





	//----------------------------------------------------------------------------------
	//   RF Modul: ZETAPLUS-433
	//----------------------------------------------------------------------------------
	//

#define RF_CHANNEL											2								// RF Modul Sende/Empfangskanal
#define RF_MODULDATA_SIZE								18							//RF_DATAPACKET_SIZE + ATS + RF-CHANNEL + DATASIZE
#define RF_RCV_MODULDATA_SIZE						17							//RF_DATAPACKE_SIZE + #R + Datasize + Signalstärke
#define RF_DATAPACKET_SIZE							13					// RF Modul Datenpaketgröße
#define RF_PAYLOAD_SIZE									10								//Nutzdaten d.h ohne STX, ETX, CRC
#define RF_MIN_RSSI_MIN									32						// minimale RF Signalstärke 

#define RF_STARTUP_TIME									200/ProgrammPeriod				//200ms  Wartezeit bis Modul konfiguriert

//#define		RFModul_CH_INI								2									
//#define		RFModul_DATA_SIZE							13								
#define		RFStartPattern								0x23 						// = "#" ... Start Pattern when Receiving Data from RF Modul
#define		RFRxDataPattern								0x52							// = "R" ... Pattern for Receiving Data
#define 	RFRxConfigPattern							0x3F							// = "?" ....Pattern for Receiving Configuration


//------------------------
//Proportional Ventile
//------------------------


#define		PWM_FREQUENCY									1000								// PWM Grundfrequenz [Hz]   = Timer Prescale * AutoReload
#define		PWM_RESOLUTION								400									// PWM Auflösung [Einzelwerte] = Timer AutoReload
#define		PWM_DUTY_MAX									85									// maximaler elaubter Dutycycle [%]
#define 	PWM_DUTY_MIN									20									// minimaler erlaubter Dutycyle [%] 
#define 	PWM_DUTY_NEUTRAL							50									// Neutralstellung PropVentil

//-----------
//Kratzboden
//-----------

#define		DAC_SCRAPPER_CORR								102									// Korrekturfaktor für DAC -> FU Kratzboden conversion [%]													
#define 	DAC_SCRAPPER_MAX_V							255								// maximaler Werte (Value) Wertebereich: 0-255



#define	ROLLER_STAR_RUN_TIMER_INI					5000								//5000ms Wartezeit von Stern auf Dreieck
#define	ROLLER_DELTA_RUN_TIMER_INI				100									//Warten bis von Stern auf Dreiecklauf (Hauptschütz) schaltet
																																	//Wichtig: 	ROLLER_DELTA_RUN_TIMER_INI > ROLLER_DELTA_START_TIMER_INI sein!!				
#define	ROLLER_DELTA_START_TIMER_INI			50									//Warten bis Stern Dreieckralais schaltet


#define 	INT_TIME_INIT									1000								// Integration Time for Counts [ms]
#define 	INPUT_CNT_ARR									0xFFFF									// überlaufwert Input Counter (max. Counterwert)




#define TIM1_Reloader											220
#define TIM1_Prescaler										1
//#define TIM1_PWM_Width									TIM1_Reloader / 2 

	//----------------------------------------------------------------------------------
	//DACs & ADCs
	//----------------------------------------------------------------------------------
  // Full-scale digital value with a resolution of 12 bits (voltage range     
  // determined by analog voltage references Vref+ and Vref-,                 
  #define DIGITAL_SCALE_12BITS             (__LL_DAC_DIGITAL_SCALE(LL_DAC_RESOLUTION_12B))
	#define DIGITAL_SCALE_8BITS								(__LL_DAC_DIGITAL_SCALE(LL_DAC_RESOLUTION_8B))
	
	// Definitions of environment analog values 
  // Value of analog reference voltage (Vref+), connected to analog voltage   
  // supply Vdda (unit: mV).                                                  
  #define VDDA_APPLI                       ((uint32_t)3300)	

	#define ADC_CONVERTED_DATA_BUFFER_SIZE		8
  #define ADC_DELAY_CALIB_ENABLE_CPU_CYCLES  (LL_ADC_DELAY_CALIB_ENABLE_ADC_CYCLES * 32)



//	#define HV_Init										1000						//1000mV for starting
//	

	//----------------------------------------------------------------------------------
	// GPIO Ports
	//----------------------------------------------------------------------------------

#if (USE_NUCLEO == 1)

#define LED_Out_Pin LL_GPIO_PIN_5
#define LED_Out_GPIO_Port GPIOA

#else 


#define LED_Out_Pin LL_GPIO_PIN_8
#define LED_Out_GPIO_Port GPIOA

#endif


//-----------------------
// LEDs
//-----------------------

#define LED_TIMER_UPDATETIME			10							


#define LED_BLINK_OFF	0
#define LED_BLINK_FAST  1
#define LED_BLINK_SLOW  10
#define LED_BLINK_ERROR 20

//-----------------------
// NOT AUS 
//-----------------------



#define	ALL_FC_STOP_Pin				LL_GPIO_PIN_5					// FU (beide) Stop (Not Stop)
#define ALL_FC_STOP_Port			GPIOA





//-----------------------
// Schubboden
//-----------------------


#define	FEEDER_FWD_Pin					LL_GPIO_PIN_0					//Schubboden Vorwärtsrichtung
#define FEEDER_FWD_Port					GPIOB
#define	FEEDER_REW_Pin					LL_GPIO_PIN_1					//Schubboden Rückwärts
#define FEEDER_REW_Port					GPIOB
#define	FEEDER_ERR_FWD_Pin			LL_GPIO_PIN_2					//Schubboden Vorwärts Error
#define FEEDER_ERR_FWD_Port			GPIOB
#define	FEEDER_ERR_REW_Pin			LL_GPIO_PIN_10				//Schubboden Rückwärts Error
#define FEEDER_ERR_REW_Port			GPIOB


#define	EnableFEEDERForward			LL_GPIO_SetOutputPin(FEEDER_FWD_Port, FEEDER_FWD_Pin)
#define DisableFEEDERForward		LL_GPIO_ResetOutputPin(FEEDER_FWD_Port, FEEDER_FWD_Pin)
#define EnableFEEDERRewind			LL_GPIO_SetOutputPin(FEEDER_REW_Port, FEEDER_REW_Pin)
#define DisableFEEDERRewind			LL_GPIO_ResetOutputPin(FEEDER_REW_Port, FEEDER_REW_Pin)

//-----------------------
// Zellenradschleuse
//-----------------------

#define ROTARY_SPEED_Pin				LL_GPIO_PIN_5					//Zellenradschleuse Impulseingang
#define ROTARY_SPEED_Port				GPIOA

#define	ROTARY_PWM_Pin					LL_GPIO_PIN_9					//Zellenradschleuse PWM Ausgang
#define ROTARY_PWM_Port 				GPIOA

#define ROTARY_ERROR_Pin				LL_GPIO_PIN_12			  //Zellenradschleuse Error PIN
#define ROTARY_ERROR_Port				GPIOB


//-----------------------------
// Zubringerschnecke
//-----------------------------

#define	SCREW_PWM_Pin						LL_GPIO_PIN_10				//ZubringerSchnecke PWM Ausgang
#define SCREW_PWM_Port 					GPIOA

#define SCREW_ERROR_Pin					LL_GPIO_PIN_13			  //ZubringerSchnecke Error PIN
#define SCREW_ERROR_Port				GPIOB


#define PROP_VALVE_DIS_Pin			LL_GPIO_PIN_11				//Proportionalventile Disable PIN
#define PROP_VALVE_DIS_Port			GPIOA

#define EnablePropValve					LL_GPIO_SetOutputPin(PROP_VALVE_DIS_Port,PROP_VALVE_DIS_Pin)
#define DisablePropValve				LL_GPIO_ResetOutputPin(PROP_VALVE_DIS_Port,PROP_VALVE_DIS_Pin)


#define PWM_DISPLAY_DIS_Pin			LL_GPIO_PIN_12				//PWM LED Anzeige disable PIN (Low = Enable)
#define PWM_DISPLAY_DIS_Port		GPIOA


#define EnablePWMDisplay				LL_GPIO_ResetOutputPin(PWM_DISPLAY_DIS_Port,PWM_DISPLAY_DIS_Pin)
#define DisablePWMDisplay				LL_GPIO_SetOutputPin(PWM_DISPLAY_DIS_Port,PWM_DISPLAY_DIS_Pin)




//-----------------------------
// Pumpendruck
//-----------------------------


#define PUMP_PRESSURE_Pin				LL_GPIO_PIN_0						// Pumpendruck - analoger Eingang
#define PUMP_PRESSURE_Port			GPIOA




#define CONFIG_IN1_Pin					LL_GPIO_PIN_14				//Configurationspin 1
#define CONFIG_IN1_Port					GPIOB
#define	CONFIG_IN2_Pin					LL_GPIO_PIN_15				//Configurationspin 2
#define CONFIG_IN2_Port		GPIOB










	//----------------------------------------------------------------------------------
	// Timer Port
	//----------------------------------------------------------------------------------

#if (USE_NUCLEO == 1)

#define SIG_IN_Pin 				LL_GPIO_PIN_0
#define SIG_IN_GPIO_Port 	GPIOA

#else 

#define SIG_IN_Pin 				LL_GPIO_PIN_5 
#define SIG_IN_GPIO_Port 	GPIOA

#endif

	//----------------------------------------------------------------------------------
	//I2C 
	//
	//----------------------------------------------------------------------------------

//	#define I2C_TIMING								0x00201D2B 					
	#define I2C_TIMING									0x00201D7B
	#define	EEPROM_ADRESS								0xA0
	#define EE_I2C											I2C1
	#define EEPROM_MaxDataPage					0x10						//Size Page Write Buffer

	#define sEE_FLAG_TIMEOUT         		((uint32_t) 0x1000)
	#define sEE_LONG_TIMEOUT         		((uint32_t)(10 * sEE_FLAG_TIMEOUT))
	#define	sEE_DataSize								 64																	//max Anzahl zu lesender/schreibender Zeichen


	//----------------------------------------------------------------------------------
	//UARTSs
	//----------------------------------------------------------------------------------


	//----------------------------------------------------------------------------------
	//   RF UART Port
	//----------------------------------------------------------------------------------


	#define 	UART_RF										USART1
	#define 	UART_RF_Baudrate						192       //entspricht 57600 Baud, Wert wird bei der Zuweisung: *100 

	// USART1 instance is used. (TX on PA.09, RX on PA.10)
	#define UART_RF_CLK_ENABLE()           	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1)

	#define UART_RF_IRQn                   	USART1_IRQn
	#define UART_RF_IRQHandler             	USART1_IRQHandler

	#define UART_RF_GPIO_CLK_ENABLE()      	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB)   // Enable the peripheral clock of GPIOB
	#define UART_RF_TX_PIN                 	LL_GPIO_PIN_6
	#define UART_RF_RX_PIN                 	LL_GPIO_PIN_7

	#define UART_RF_GPIO_PORT           		GPIOB
	#define UART_RF_SET_GPIO_AF       		 	LL_GPIO_AF_7



	//----------------------------------------------------------------------------------
	//   RF Port (UART1)
	//----------------------------------------------------------------------------------
	//

// UART Transmission & Receiver Max Buffer 
	#define RXRF_BUFFER_SIZE  							32				//max. Datenanzahl 32 Zeichen für DMA Datasize
	#define TXRF_BUFFER_SIZE								32				//max. 16 Zeichen 	für DMA Datasize




	//----------------------------------------------------------------------------------
	//   PC Port
	//----------------------------------------------------------------------------------
	//

	#define 	STX					0xEA   //0x23				//					0xEA   		//dez: 234 // = "#";
	#define 	ETX					0xF6   //0x2A				//					0xF6 			//dez: 246 // = "*";
	#define   ACKP													0x21 		// = "!"
	#define 	ID3														"HPW"							//Kennung der Sonde: HackschnitzelPumpWagen
	#define   ACK														"!!!"
	#define	  PgmStatus											0x00							//PSW is received
	#define		DevStatus											0x01							//Device Status Word received
	#define		RdReq													0xFF							//Read Request is received

//----------------------------------------------------------------------------------
//   Device Transmission Variables
//----------------------------------------------------------------------------------

	#define		DevStatusDataSize							0x0A							//Datengröße vom StatusFrame
																														//Byte0-3: Status Antrieb, Kratzboden, Dosierwalze, Zusatz
																														//Byte4-5: Speed Maschine, Speed Kratzboden
																														//Byte6-9: Status Schalter (Start, End, ...)
	
	#define    TestString										"#HPW6003123EF6DA*"  //Kennung HackschnitzelPumpWagen


	

	// UART Transmission Buffer 
	#define RXPC_BUFFER_SIZE  							70								//max. Datenanzahl 64 Zeichen, + CRC + Adresse + Anzahl + STX(#) + ETX(*) + 3 Zeichen Kennung 

	
	 #if (USE_NUCLEO == 2)

	//USART2 GPIO Configuration
  //PB10   ------> USART2_TX
  //PB11  ------> USART2_RX
	
	#define 	UART_PC											USART3
	#define 	UART_PC_Baudrate						19200

	// USART3 instance is used. (TX on PB.10, RX on PB.11)
	//	(please ensure that USART communication between the target MCU and ST-LINK MCU is properly enabled 
	// on HW board in order to support Virtual Com Port) */
	#define UART_PC_CLK_ENABLE()           LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3)
// No Clk source to be selected for USART2 on this device : PCLK1 
	#define UART_PC_IRQn                   USART3_IRQn
	#define UART_PC_IRQHandler             USART3_IRQHandler

	#define UART_PC_GPIO_CLK_ENABLE()      LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB)   /* Enable the peripheral clock of GPIOA */
	#define UART_PC_TX_PIN                 LL_GPIO_PIN_10
//	#define UART_PC_TX_GPIO_PORT           GPIOA
//	#define UART_PC_SET_TX_GPIO_AF()       LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_2, LL_GPIO_AF_7)
	#define UART_PC_RX_PIN                 LL_GPIO_PIN_11
	#define UART_PC_GPIO_PORT           	GPIOB
	#define UART_PC_SET_GPIO_AF       		 LL_GPIO_AF_7		
	

 #else    //USE_NUCLEO == 0 oder ==1
 
 
 //USART2 GPIO Configuration
  //PA2   ------> USART2_TX
  //PA3   ------> USART2_RX
	
	#define 	UART_PC											USART2
	#define 	UART_PC_Baudrate						19200

	// USART2 instance is used. (TX on PA.02, RX on PA.03)
	//	(please ensure that USART communication between the target MCU and ST-LINK MCU is properly enabled 
	// on HW board in order to support Virtual Com Port) */
	#define UART_PC_CLK_ENABLE()           LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2)
// No Clk source to be selected for USART2 on this device : PCLK1 
	#define UART_PC_IRQn                   USART2_IRQn
	#define UART_PC_IRQHandler             USART2_IRQHandler

	#define UART_PC_GPIO_CLK_ENABLE()      LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA)   /* Enable the peripheral clock of GPIOA */
	#define UART_PC_TX_PIN                 LL_GPIO_PIN_2
//	#define UART_PC_TX_GPIO_PORT           GPIOA
//	#define UART_PC_SET_TX_GPIO_AF()       LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_2, LL_GPIO_AF_7)
	#define UART_PC_RX_PIN                 LL_GPIO_PIN_3
	#define UART_PC_GPIO_PORT           	GPIOA
	#define UART_PC_SET_GPIO_AF       		 LL_GPIO_AF_7
	
	
	#endif 					// USE_NUCLEO

#endif
