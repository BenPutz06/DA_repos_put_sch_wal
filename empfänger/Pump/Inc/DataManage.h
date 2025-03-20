/*
	Dieses File enthält Variablen und Datenstrukturen, die die Einstellungen für den Betrieb des SSM1+ beinhalten,
	und Funktionen für die Verwaltung und Manipulation dieser Daten.
	In mw_Datenverwaltung.c  wird die Datenstruktur angelegt und in mw_Datenverwaltung.h als extern deklariert, somit haben alle CFiles die
	#include mw_Datenverwaltung.h enthalten Zugriff auf die Datenstrukturvariable

	Nomenklatur:




*/
//#pragma once

#ifndef __DATAMANAGE_H
#define __DATAMANAGE_H

#include "Main.h"

//#include "stm32l1xx.h"

// #include "Parameter.h"

//#include "Functions.h"		//für Aktivierung/Deaktivierung der Interrupts				



/* EEPROM address assignment */

// EEPROM 24AA025E48  1024Bit nutzbarer Speicher => Adressraum 0x00 - 0x80
// Beachte: Bei diesem EEPROM wird ein 16Byte Page Write angewendet. Dh. es werden bis zu 16Byte
//          auf einmal übertragen. Wichtig dabei ist, das beim Schreiben nicht über die Page hinaus (also auf die nächste Page)
//					geschrieben wird. Deshalb wird bei allen folgenden Variablenblöcken eine Page-Startadresse (vielfaches von 16)
//					verwendet

#define 	E2Prom_Kennung													0x007E				//Kennung = letzten 2 Byte im EEPROM

// device identification data 

#define		E2Prom_SondenText												0x0010				//Test Sondenkennung	.."NDS 07    "  ->8Byte
#define 	E2Prom_Seriennummer											0x0018				//Die Seriennummer hat 8 Stellen (8Bytes)
#define 	E2Prom_ProdDat													0x0020				//Produktionsdatum bzw. Programmierdatum (8Bytes) TTMMJJJJ 
#define		E2Prom_FW_Ver														0x0028				//Die Firmware Version hat 8 Stellen
#define		E2Prom_HW_Ver														0x0030				//Die Hardware Version hat 8 Stellen


// Device Modul parameter

#define 	E2Prom_RF_BAUD														0x0040				//2 Bytes 
#define		E2Prom_RF_CHANNEL															0x0042				//1 Byte   0-7  siehe Lookup Table

// device operation data 


#define 	E2Prom_LC_LEVEL													0x0050				//2 Bytes 
#define 	E2Prom_HC_LEVEL 												0x0052				//2 Bytes 
#define		E2Prom_OVL_LEVEL												0x0054				//2 Bytes

#define 	E2Prom_DAC_LLD													0x0060				//2 Bytes
#define 	E2Prom_MOD_HV														0x0062				//2 Bytes
#define 	E2Prom_DAC_ULD													0x0064				//2 Bytes

#define 	E2Prom_INT_TIME													0x0066				//2 Bytes




//// Adressen der Variablen im SRAM
//// Entspricht der Adresse für die SSM1 Kommunikation
//// Die Adressen sind in Gruppen aufgeteilt
//// Wichtig: nach jeder Variable MUSS eine freie Speicherstelle sein, hier wird der BCC abgespeichert


	#define SRAM_PARAM_OFFSET						0x20002000							//Basisadresse
	#define SRAM_PERIPH_VAR_START				0x80										//Startadresse Variablen für Peripheriebereich
																															//d.h. für diese Variablen wird Fkt: UpdateValuePeriph(...) aufgerufen
	#define SRAM_EEPROM_VAR_START				0x40										//Startadresse Variablen die im EEPROM gesichert werden

		//--- Geräteeinstellungen ---

////#define 	E2Prom_AutoPowerOff 									500
////#define 	E2Prom_PIN 														505

////------- ProgrammStatusWort
#define		SRAM_PSW																0x00				//Programmablauf (StateMachine), Zustände, Einstellungen : 1 Byte
#define 	SRAM_DSW																0x01				//Device Status Word, Zustände der einzelnene Funktionen
#define		SRAM_ErrorByte													0x03

//------ Lesebereich Geräteeinstellungen (DeviceConfig)
//------   Kennungen: Read Only (R)
#define		SRAM_SondenText													0x06				//Test Sondenkennung	.."iDLxxxxx"  ->8Byte
#define		SRAM_Serial															0x0E				//Die Seriennummer hat 8 Stellen (8Bytes)
#define		SRAM_ProdDat														0x16 				//Produktionsdatum bzw. Programmierdatum (8Bytes) TTMMJJJJ 
#define 	SRAM_FW_Ver															0x1E				//Die Firmware hat 8 Stellen
#define		SRAM_HW_Ver															0x26				//Software ID, Checksum (4 Byte)

#define   SRAM_ROM_ID															0x2E				//gelesene EEProm Versions ID
#define		SRAM_ROM_ID_ACT													0x30				//aktuelle EEPROM Versions ID

//-------- Sondenparameter


////---- Betriebsvariablen	

#define			SRAM_RF_Baud														0x40				//aktueller Baudrate RF Modul Transfer
#define			SRAM_RF_CHANNEL													0x42				//aktueller Sende/Empfangskanal


////---- Betriebsvariablen	

#define			MODBUS



//



//----- Lesebereich Messmodus: Adressbereich von 0x40 bis 0x70 kann im Messmodus direkt gelesen werden


//-------------  Peripheral Variable --------------------
//-------
//-------  MSB is set at this variables i.e. Adressrange 0x80 - 0xFE
//-------  0xFF is reserved for Read Request

#define		PERIPHERAL_OP_STATE_SIZE									2					//digitale Peripheriezustände 1Bytes Switch Sum States, Rest Einzelschalter

#define		SRAM_RotaryActSpeed									0x71					//Zellenradschleuse aktuelle Drehzahl (U/min)
#define		SRAM_PumpPressure										0x72					//Gebläse Druck ADC Wert
#define		SRAM_PumpDevState										0x73					//Gebläse Ist Zustand


//--------------	Operational Variables with Function Call --------
//--------
//--------	following Variables need a function call after update
#define		OPERATIONAL_DATA_SIZE									3						//Anzahl (Bytes) an Betriebsvariablen

#define 	SRAM_FeederSetState									0x80					//Schubboden Soll Zustand
#define 	SRAM_ScrewSetState									0x81					//Zubringerschnecke Soll Zustand
#define 	SRAM_RotarySetState									0x82					//Zellenradschleuse Soll Zustand

#define 	SRAM_FeederActState									0x86					//Schubboden Ist Zustand
#define		SRAM_ScrewActState									0x87					//Zubringerschnecke Ist Zustand
#define 	SRAM_RotaryActState									0x88					//Zellenradschleuse Ist Zustand



//-------------  Peripheral Variable --------------------
//-------
//-------  Peripheral Variable: when they are changed also
//-------  a Function have to be called, so the varibles take affect
//-------  Starting Adress is: SRAM_PERIPH_VAR_START  for this variables Fct: UpdateValuePeriph(...) is called

#define		PERIPHERAL_SPEED_SIZE								2							//Peripherie Analogwerte (DAC Werte)

#define		SRAM_ScrewSpeed											0x90					//PWM Wert [5] (Drehzahl) Zubringerschnecke
#define		SRAM_RotarySpeed										0x91					//PWM Wert (Drehzahl) Zellenradschleuse


//-------------  Reserve Variable --------------------
//-------
//-------  Reserve Variable: not used

#define		RESERVE_VAR_SIZE												3					//Reserve Bytes im Datenprotokoll

#define		SRAM_ReserveVar1										0x96			//Byte1
#define		SRAM_ReserveVar2										0x97			//Byte2
//#define		SRAM_ReserveVar3										0x98			//Byte3
#define 	SRAM_MeasCounter										0x98


//#define  	SRAM_DungHeap															0xAA				// Adresse für ungewollte Schreibzugriffe
//#define		SRAM_SensorType													0x200				//Sensortyp vom Gerät


typedef enum
{
	Stop 			= 0,
	Running 	= 1,
	UpSpeed		= 2,
	Switching = 3,
	DownSpeed = 4,
	Forward		= 6,
	Rewind		= 7,
} DEVSTATEMODE;

typedef struct
{
	uint8_t TimeOutAct;					// =1 wenn TimeOut ist aktiv
	uint16_t TimeOutCnt;				// aktueller Timeout countwert
	uint16_t TimeOutMax;				// max Timeoutcount bis Timer abläuft
		
} TIMEOUTTyp;

struct  LED_LIGHT
{
	uint8_t Counter;						//aktuelle Counter für die Blinkzeit
	uint8_t Blinking;						//gibt die Blinkzeit an. d.h wie lange ist Led ein bzw. aus (wieoft TIM7 überläuft und Status ändert)
	int8_t  BlinkCounts;				//Anzahl wie oft geblinkt. -1..Dauerblinken, 0..blinken aus, >0..Blinkanzahl 
	uint8_t	Status;							//0..Led off 1..Led permanent ON 3..blinking Led off 4..blinking led On
	uint8_t Status_Backup;			//save the acutal State
	uint8_t Blinking_Backup;		//save the actual blinking time
	int8_t BlinkingCounts_Backup;  //actual Blinking counts

};


struct 	PCDATAPACKTyp									//Datenstruktur für PC UART Datenempfang 
{
	uint8_t Adress;
	uint8_t Cnt;
	uint8_t *Data;
	uint8_t CRC_Data;
	uint8_t	AckReq;				//= 1 wenn Acknowledge gewünscht
	uint8_t State;				//Status der Datenkonversion 0..idle, 1..ok, 2..busy
	uint8_t Update;				//=1: update function necessary to refresh data
};


struct RFMODDATAPACKTyp
{
		uint8_t 	Channel_ID;					// Kanaladresse vom Empfäner
		uint16_t	Baud;								// Baudrate Kommunikation 
		int16_t  StartTimer;					// Timer für Initialisierung			
		uint8_t 	State;							// Status der Datenkonversion 0..idle, 1..ok, 2..busy, 6..Error
		uint8_t 	Update;							//=1: update data =2:			=3: update EEPROM
};

struct DATARxTyp
{
	uint8_t 				*pBufferProcess;							//Buffer Daten for processing
	uint8_t 				*pBufferReception;						//Buffer Daten for receiving
	uint8_t     		dRxReceptionChars;						//Anzahl empfangener Daten
	uint8_t					dRxProcessChars;							//Anzahl Daten für Verarbeitung = Anzahl empfangender Daten
	uint32_t     		fBufferReady;									//Daten bereit zur Verarbeitung (Daten empfangen)								1..Nutzdaten,  2..Configdaten RF Modul
	uint8_t  				fDataReceiving; 							//1 STX received incoming Data is collected 
	uint8_t					fAckReceiving;								//0..idle, 1..ACK  receiving, 2..ACK ok
  uint8_t 				fDataValid;										//0..idle, 1..Data valid, 2..Waiting for Data
TIMEOUTTyp				sAckTimeOut;
};

struct	DATATxTyp
{
	uint8_t DataPending;						// =1:Daten zumm Senden bereit   //=2: Senden erfolgreich
	uint8_t *pBufferSend;
	uint8_t Size;
	uint8_t TxdSucc;								//gibt aktuellen Status der Tx Übertragung an 0..idle, 1..ok, 2..busy
	uint8_t AckNeed;								//0.. no Acknowledge needed (Rd Request) //1..waiting for Ack
};


struct SYSTIMERTyp
{
	uint32_t RollerStarRunTimer;									// Wartezeit von Stern auf Dreieck
	uint32_t RollerDeltaRunTimer; 									//Warten bis von Stern auf Dreiecklauf (Hauptschütz) schaltet
	uint32_t RollerDeltaStartTimer;									//Warten bis Stern Dreieckralais schaltet
};

struct	DEVSTATUSTyp
{
  DEVSTATEMODE	State;
	uint8_t		*SetState;
	uint8_t		*ActState;
	uint16_t	StartTimer;
	uint16_t	StopTimer;
	uint8_t		*Speed;
	uint8_t		*ActSpeed;
	uint8_t		Error;
};


struct SENSORSYSTEMTyp
{
	uint8_t * DeviceStatus;
	uint8_t	* Pressure;
	
};


struct EEDataTyp
{
		uint8_t		*pEERxBuffer;											// Datenempfangsbuffer
		uint8_t		*pEETxBuffer;											// Datensendebuffer
		uint8_t		EERxTxAddress;										// Lese- Schreibadresse 
		uint8_t		EERxDataCnt;											// Anzahl zu empfangener Zeichen (1-sEE_DataSize)
		uint8_t		EETxDataCnt;											// Anzahl zu sendender Zeichen (1-sEE_DataSize)
		uint8_t		EERxRcvFlag;											// 1..Daten empfangen, gesetzt von IRQn_Type
		uint8_t		EETxSendFlag;											// >0..Dataen zum Senden vorhanden , >1 Wartezeit für nächsten WriteZyklus
		uint8_t		EEBufferCounter;										// fortlaufender Datenzähler
		uint32_t	EETimeOut;												// EE-Prom Lese/Schreibe TimeOut
};
	
	
	


typedef enum 
{
	RxTxIdle = 0,
	ProcessData = 2,
	GetData = 3,
	SendData = 4,
	GetAck = 5,
	SendAck = 6,
	ErrorCall = 10,
	
} TRANSFERSTATES;



typedef enum 
{
	BOOT, 
	IDLE,
	MEASURE,
	CONFIG_KALIB,
	STOP,
} PGMSTATES;






struct STATETyp
{
	PGMSTATES		PgmFlow;
//	MEASSTATES	MeasFlow;
  uint8_t			Status;				//Statusbit für Programmablauf: Bit0=1 TxDataMeasure erfolgreich
														//Bit 4: =1 PSW has changed 
};


struct 	PFlagsTyp
{
	uint8_t WrReq;
	uint8_t	WrSend;
	uint8_t EE_WrReq;
	uint8_t RF_TX_Acitve;										// = 1..Data availabel for transmitting
	uint8_t Pump_Pressure_UpdateReq;
	
} ;


// 


// function pointer for updating the received values 
//
typedef void (*fpUpdateTyp)(int8_t);			// Funktion(-spointer): kein return-wert (void), 1. Paramter (16bit)

// verkettete Liste, welche die Speicheradresse der Variablen und die 
// dazugehörige Funktion (zum Aktualisieren) beinhaltet
struct UPDATEFCTLISTTyp
{
		struct UPDATEFCTLISTTyp * pNext;					//Zeiger auf nächstes Listenelement
	  uint8_t	 Src;															//SRAM Adresse des empfangenen Wertes
		fpUpdateTyp	 *Dest;												//Adresse der aufzurufenden Funktion
	
};


//-----------------------------------------
// Functions
//-----------------------------------------
//extern SSM1CommTyp COMM;

void ParamAddress(void);

void Assign_Functions(void);

void Daten_Init(void);

void Parameter_Init(void);

void Daten_Default(uint8_t ini);

uint8_t EEPROM_Load_Data(void);




//-----------------------------------------
// Variables for Programm Process
//-----------------------------------------  

extern uint8_t					*PSW;
extern uint8_t					*ErrorByte;
extern uint16_t 				TickCounter;			// SystemTimer Counter
extern uint16_t					ProcessTick;
extern struct PFlagsTyp PFlags;									//Programm Flags

extern struct 	STATETyp				State;

extern struct UPDATEFCTLISTTyp	*UpdateFctList;

//-----------------------------------------
// Variables for Programm Process
//-----------------------------------------  

extern uint8_t		JumperConfig; 						//Gerätekonfiguration anhand der Jumper

extern uint16_t		*RFModul_Channel;
extern uint16_t		*RFModul_Baud;
//extern uint8_t		RFRcvPattern[3];				//Receivining Start Pattern from RF Modul
extern int16_t	RFTimeOutCounter;				// Zähler für Zeitüberschreitung in RF Modul Kommunikation
extern int16_t		RFRcvTimeOutTimer;						// Zähler wenn Datenpaket nicht (innerhalt einer Zeit) gültig

extern uint8_t		EmergencyStop;

extern uint8_t RFPacketCounter;
extern uint8_t   RFDataPacket[RF_RCV_MODULDATA_SIZE + 2];

extern struct	RFMODDATAPACKTyp		RFModData;						//Datenstruktur für RF Modul

extern struct DEVSTATUSTyp Feeder;
extern struct DEVSTATUSTyp Rotary;
extern struct DEVSTATUSTyp Screw;


extern struct SENSORSYSTEMTyp Pump;

extern struct SYSTIMERTyp TimerSys;

extern uint8_t		UART_RF_RX_DMA_Buffer[RXRF_BUFFER_SIZE];		// DMA Buffer for receiving Data
extern uint8_t		UART_RF_TX_DMA_Buffer[TXRF_BUFFER_SIZE];		// DMA Buffer for transmitting

extern uint8_t		fCountOvflw;									//Flag: Overflow of Input counter - important for Countintegration
extern uint8_t		IntTimeCounter;
extern uint8_t		IntTimeCntMax;
extern uint8_t		*CountIncrement;									//fortlaufender Zähler für Messwerte
extern uint8_t		*CountRateState;
extern uint16_t		*IntTime;													//Integrationszeit für gezählte Impulse [ms]
extern uint8_t	  DMA_CntCycle;							//Größe Bufferzyklus dh. wieviel 100ms Werte für gemessene Counts
extern uint32_t		DMA_CntBuffer[12];
extern uint16_t		Counter_Init_Value;					//StartValue (Cnts) for difference Calculation from CounterValue

extern uint16_t		CounterSumValue;

extern uint16_t		*PumpPressValue;												//aktueller Hochspannungswert Modul Ausgang [V]
extern uint16_t		PumpPress_ADCValue;										//aktueller Hochspannungswert ADC Eingang [mV]
extern uint16_t		PumpPressRawBuff[9];									//gelesene ADC Werte

extern uint8_t 		SPSRcvBuffer[128];
extern uint8_t		SPSRcvCnt;


extern uint8_t		I2CRxBuffer[sEE_DataSize];
extern uint8_t		I2CTxBuffer[sEE_DataSize];
extern uint8_t		I2CTxBufferCnt;
extern uint8_t		I2CRxBufferCnt;

extern uint8_t		DummyByte;
extern uint8_t		DMA_Tester1;
extern uint8_t		DMA_Tester2;
extern uint8_t		DMA_Tester3;
extern uint8_t		DMA_Tester4;


//-----------------------------------------
// Variables for LEDs
//-----------------------------------------  

extern uint8_t LED_BLINKING_State;					//0..kein LED Update  1..LED Update Normalbetrieb  2..Konfig Blinkcode NF 3..Konfig Blinkcode HF
extern uint8_t LED_BLINKING_Task;						//0..Idle, //1..Update pending (Bit 0) //2..HAL Update pending (Bit 1) //3..NF & HAL Update pending

extern int16_t		LED_Blinking_Timer;
extern struct LED_LIGHT 	LED_Power_Green;

extern struct LED_LIGHT	Error_Config_Code[10];						// Angabe der verschiedenen möglichen Errorcodes 
extern uint8_t	Error_BlinkCode_State;				// aktueller Zustand des ausführendnen Errorcodes
extern uint8_t 	Error_BlinkCode_Typ[2]; 										//Definition des Blinkcodes (Ablauf)

//-----------------------------------------
// Variables for Communication
//-----------------------------------------  

extern struct		PCDATAPACKTyp			PCRxDataPack;					//geprüfte und separierte (Weiterverabeitung) Daten RX
extern struct 	PCDATAPACKTyp			PCTxDataPack;
//extern struct		SPSDATAPACKTyp		SPSRxDataPack;					//geprüfte und separierte (Weiterverabeitung) Daten RX
//extern struct 	SPSDATAPACKTyp		SPSTxDataPack;

extern struct 	DATARxTyp 			RxPCPack;
extern struct 	DATATxTyp				TxPCPack;

extern struct 	DATARxTyp 			RxRFPack;										//von UART empfangene Daten
extern struct 	DATATxTyp 			TxRFPack;										//für UART zu sendende Daten

extern uint8_t * RxRFBuffer1;													//Rx Buffer No1.. für Datenempfang von UART-DMA
extern uint8_t	* RxRFBuffer2;													//Rx Buffer No2.. für Datenempfang
extern uint8_t **pRxBuff;															//Pointer auf aktuellen Empfangsbuffer 1 oder 2
extern uint8_t RxRFActBufferID;												//aktuelle Buffer für DMA schreiben	


extern struct EEDataTyp	EEDataPack;										// Daten I2C EEProm

extern TRANSFERSTATES		sPCTransfer;
extern TRANSFERSTATES		sRFTransfer;

extern uint16_t		LED_Blinky;
extern uint16_t		WWDG_Counter;

extern uint32_t Test;
//-----------------------------------------
// Lookup tables
//----------------------------------------- 




static const uint8_t EE_Store_address[] = {
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, E2Prom_SondenText, 0x00, 0x00, 0x00, 
		0x00, 0x00, 0x00, 0x00, E2Prom_Seriennummer, 0x00, 0x00, 0x00, 0x00, 0x00, 
		0x00, 0x00, E2Prom_ProdDat, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
		E2Prom_FW_Ver, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, E2Prom_HW_Ver, 0x00, 
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, E2Prom_Kennung, 0x00, 0x00, 0x00, 	//40-49
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 											
		0x00, 0x00, 0x00, 0x00, E2Prom_RF_BAUD, 0x00, E2Prom_RF_CHANNEL, 0x00, 0x00, 0x00,			//60-69 			Modul Parameter
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 	
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 			//100-109
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
		E2Prom_LC_LEVEL, 0x00, E2Prom_HC_LEVEL, 0x00, E2Prom_OVL_LEVEL, 0x00, 0x00, 0x00, 0x00, 0x00, 			//130-139
		0x00, 0x00, 0x00, 0x00, E2Prom_DAC_LLD, 0x00, E2Prom_MOD_HV, 0x00,  E2Prom_DAC_ULD, 0x00,				//140-149 	
		E2Prom_INT_TIME, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,				//150-159 
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 			//160-169
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 	
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	
};

static const uint32_t Baudrate_table[] = {
		1200,2400,4800,9600,19200,38400,57600,115200
};

#endif

