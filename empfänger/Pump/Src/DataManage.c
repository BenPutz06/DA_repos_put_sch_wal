/*
	Dieses File enthält Variablen und Datenstrukturen, die die Einstellungen für den Betrieb beinhalten,
	und Funktionen für die Verwaltung und Manipulation dieser Daten.


*/


#include "Parameter.h"



#include "Periph_Funct.h"
#include "Declarations.h"
#include "DataManage.h"
#include "stdlib.h"
#include "string.h"


//-------------  System Variable --------------------
//-------







//-------------  Programm Operation Variable --------------------
//-------



 uint8_t *PSW;												// beinhaltet Zustandsbits für Programmablauf
																			// Bit0...only Read..	          // Bit1...only Read.. unused
																			// Bit2...only Read..									// Bit3...only Read..
																			// Bit4...R/W											// Bit5...R/W..1 = Measure-State
																			// Bit6...R/W 1= Config State			// Bit7...R/W
	
	uint8_t *ErrorByte;									//contains the error bits
																			// Bit: 0...										// Bit: 1...
																			// Bit 4: 											// Bit 5: 
																			// Bit 6: Request accepted, Sending in progress  // Bit 7: Wr-Request sending MeasValue to the PC 

uint16_t 				TickCounter = 0;			// SystemTimer Counter
uint16_t				TimerCounter = 0;			// Zählcounter für Betriebsvariablen (StartTimer, StopTimer)
uint16_t				ProcessTick; 					//zählt die Prozessdurchläufe

struct PFlagsTyp PFlags;							//Programm Flags

struct 	STATETyp State;


struct UPDATEFCTLISTTyp	*UpdateFctList;

struct	PCDATAPACKTyp			PCRxDataPack;					//geprüfte und separierte (Weiterverabeitung) Daten RX
struct 	PCDATAPACKTyp			PCTxDataPack;


//struct 	SPSDATAPACKTyp		SPSTxDataPack;

struct 	DATARxTyp RxPCPack;											//von UART empfangene Daten
struct 	DATATxTyp TxPCPack;											//für UART zu sendende Daten

struct 	DATARxTyp RxRFPack;											//von RF empfangene Daten
struct 	DATATxTyp TxRFPack;											//für RF zu sendende Daten

uint8_t * RxRFBuffer1;													//Rx Buffer No1.. für Datenempfang von UART-DMA
uint8_t	* RxRFBuffer2;													//Rx Buffer No2.. für Datenempfang
uint8_t **pRxBuff;															//Pointer auf aktuellen Empfangsbuffer 1 oder 2
uint8_t  RxRFActBufferID;												//aktuelle Buffer für DMA schreiben	


struct EEDataTyp	EEDataPack;										// Daten I2C EEProm


TRANSFERSTATES		sPCTransfer; 				// Statemachine for Datatransfer to PC
TRANSFERSTATES		sRFTransfer;				// Statemachine for Datatransfer to SPS

uint16_t		LED_Blinky = 0;											//Led Counter for Blinking frequency
uint16_t		WWDG_Counter = 0;										//Counter for Updating WWDG

uint32_t			Test = 0;


// 
//-------------  Device ID Variable --------------------

char *		DeviceText[9];									//Gerätebezeichnung
char *		DeviceSerial[9];								//Seriennummer
char *		DeviceProdDate[9];							//Produktionsdatum
char *		DeviceFWVersion[9];							//Version Firmware
char *		DeviceHWVersion[9];							//Verison Hardware

char *		EEPromID[2];										//gelesene EEPROM Versions ID
char *		EEPromID_ACT[2];								//aktuelle EEPROM Version


//-------------  Operation Variable --------------------
//-------
uint8_t		JumperConfig;											//Gerätekonfiguration anhand der Jumper

uint16_t	*RFModul_Channel;												//RF - Geräte Adresse (Modul ID)
uint16_t	*RFModul_Baud;											//RF Modul Baudrate
//uint8_t		RFRcvPattern[2]	= RFStartPattern;		//Receivining Start Pattern from RF Modul
int16_t		RFTimeOutCounter = 0;									// Zähler für Zeitüberschreitung in RF Modul Kommunikation

int16_t		RFRcvTimeOutTimer = RFRcvMaxTimeOut;			// Zähler wenn Datenpaket nicht (innerhalt einer Zeit) gültig

uint8_t		EmergencyStop;										//Not-Aus Schalter  ...aktiv = 0,   inaktiv (Betrieb erlaubt) = 1

uint8_t   RFPacketCounter;
uint8_t   RFDataPacket[RF_RCV_MODULDATA_SIZE + 2];

																																				//..+5="ATS+RF-Kanal+Paketgröße ..+1=..saveByte :-)
struct	RFMODDATAPACKTyp		RFModData;						//Datenstruktur für RF Modul


struct DEVSTATUSTyp Feeder;									//Gerätevariable Schubboden
struct DEVSTATUSTyp Rotary;									//Gerätevariable Zellenradschleuse
struct DEVSTATUSTyp Screw;									//Gerätevariable Zubringerschnecke


struct SENSORSYSTEMTyp Pump;							// Sensorik (alle Sensoren) vom Gerät

struct SYSTIMERTyp TimerSys;								// alle im System benötigten Timer - für Abläufe


uint8_t		UART_RF_RX_DMA_Buffer[RXRF_BUFFER_SIZE];		// DMA Buffer for receiving Data
uint8_t		UART_RF_TX_DMA_Buffer[TXRF_BUFFER_SIZE];		// DMA Buffer for transmitting


uint8_t		fCountOvflw = 0;									//Flag: Overflow of Input counter - important for Countintegration
uint8_t		IntTimeCounter = 1;
uint8_t		IntTimeCntMax = 0;
uint8_t		*CountIncrement;									//fortlaufender Zähler für Messwerte
uint8_t		*CountRateState;


uint16_t	*IntTime;													//Integrationszeit für gezählte Impulse [ms]

uint8_t	  DMA_CntCycle	= 10;								//Größe Bufferzyklus dh. wieviel 100ms Werte für gemessene Counts
uint32_t	DMA_CntBuffer[12] = {0,0,0,0,0,0,0,0,0,0,0,0};    //Buffer for readed Counts 
uint16_t	Counter_Init_Value = 0;				//StartValue (Cnts) for difference Calculation from CounterValue
uint16_t	CounterSumValue = 0;							//aktueller gemessener Drehzahlwert;

uint16_t	*PumpPressValue;													//aktueller Hochspannungswert Modul Ausgang [V]
uint16_t	PumpPress_ADCValue = 0;									//aktueller Hochspannungswert ADC Eingang [mV]
uint16_t	PumpPressRawBuff[ADC_CONVERTED_DATA_BUFFER_SIZE+1];										//gelesene ADC Werte

uint8_t 	SPSRcvBuffer[128];
uint8_t		SPSRcvCnt = 0;

uint8_t		I2CRxBuffer[sEE_DataSize];
uint8_t		I2CTxBuffer[sEE_DataSize];
uint8_t		I2CTxBufferCnt;
uint8_t		I2CRxBufferCnt;

uint8_t		DummyByte;
uint8_t		DMA_Tester1 = 0;												//schaut welcher DMA IR ausgelöst TC, HT oder Idle Line
uint8_t		DMA_Tester2= 0;												//schaut welcher DMA IR ausgelöst TC, HT oder Idle Line
uint8_t		DMA_Tester3=0;												//schaut welcher DMA IR ausgelöst TC, HT oder Idle Line
uint8_t		DMA_Tester4=0;												//schaut welcher DMA IR ausgelöst TC, HT oder Idle Line


int16_t	LED_Blinking_Timer;								// Zähler, wann LED-Status update wieder aufgerufen wird

uint8_t LED_BLINKING_State = 0x00;			//0..kein LED Update  1..LED Update Normalbetrieb  2..Konfig Blinkcode NF 3..Konfig Blinkcode HF
uint8_t LED_BLINKING_Task = 0x00;			//0..Idle, //1..NF Update pending (Bit 0) //2..HAL Update pending (Bit 1) //3..NF & HAL Update pending


struct LED_LIGHT LED_Power_Green;

struct LED_LIGHT	Error_Config_Code[10];						// Angabe der verschiedenen möglichen Errorcodes 
uint8_t	Error_BlinkCode_State = 0;				// aktueller Zustand des ausführendnen Errorcodes
uint8_t Error_BlinkCode_Typ[2]; 										//Definition des Blinkcodes (Ablauf)


void INIT_LEDs()
{
	
	LED_Power_Green.Status = 0;						//0..LED OFF  1..LED GREEN  2..LED RED
	LED_Power_Green.Counter = 0;
	LED_Power_Green.Blinking = 0;	
	
}




///**
//  * @brief  ParamAddress()
//	*		
//  * @param  None
//  * @retval None
//  */


void ParamAddress()
{
		
		uint8_t		i=0;

		PSW						 = (uint8_t *) (SRAM_PARAM_OFFSET  + (uint8_t) SRAM_PSW);
		/* assign programm status word in SRAM */
	
	// Zuweisung der Geräteparameter

		do						//Schreiben des SondenTypTextes in den Speicher
		{
				DeviceText[i] 			= (char *) (SRAM_PARAM_OFFSET + (uint8_t) SRAM_SondenText+ i );				
				DeviceSerial[i]			= (char *) (SRAM_PARAM_OFFSET + (uint8_t) SRAM_Serial + i);
				DeviceProdDate[i]		= (char *) (SRAM_PARAM_OFFSET + (uint8_t) SRAM_ProdDat + i);
				DeviceFWVersion[i]	= (char *) (SRAM_PARAM_OFFSET + (uint8_t) SRAM_FW_Ver + i);
				DeviceHWVersion[i]	= (char *) (SRAM_PARAM_OFFSET + (uint8_t) SRAM_HW_Ver + i);
		} while (i++ < 8);

		i= 0;

		EEPromID[0]							= (char *) (SRAM_PARAM_OFFSET + (uint8_t) SRAM_ROM_ID);
		EEPromID[1]							= (char *) (SRAM_PARAM_OFFSET + (uint8_t) SRAM_ROM_ID+1);
		EEPromID_ACT[0]							= (char *) (SRAM_PARAM_OFFSET + (uint8_t) SRAM_ROM_ID_ACT);
		EEPromID_ACT[1]							= (char *) (SRAM_PARAM_OFFSET + (uint8_t) SRAM_ROM_ID_ACT+1);

		
		CountIncrement				= (uint8_t *) (SRAM_PARAM_OFFSET + (uint16_t) SRAM_MeasCounter);

	
		
		RFModul_Channel						= (uint16_t *)  (SRAM_PARAM_OFFSET + (uint16_t) SRAM_RF_CHANNEL);
		RFModul_Baud							= (uint16_t *) (SRAM_PARAM_OFFSET + (uint16_t) SRAM_RF_Baud);
		/* assign errorbyte in SRAM */
		ErrorByte = (uint8_t *) (SRAM_PARAM_OFFSET + (uint8_t) SRAM_ErrorByte);
	
		Feeder.SetState				= (uint8_t *) (SRAM_PARAM_OFFSET + (uint8_t) SRAM_FeederSetState);
		Feeder.ActState				= (uint8_t *)	(SRAM_PARAM_OFFSET + (uint8_t) SRAM_FeederActState);
		
		Rotary.SetState		 	= (uint8_t *) (SRAM_PARAM_OFFSET + (uint8_t) SRAM_RotarySetState);
		Rotary.ActState			= (uint8_t *) (SRAM_PARAM_OFFSET + (uint8_t) SRAM_RotaryActState);
		Rotary.Speed				= (uint8_t *) (SRAM_PARAM_OFFSET + (uint8_t) SRAM_RotarySpeed);					
		Rotary.ActSpeed			= (uint8_t *)	(SRAM_PARAM_OFFSET + (uint8_t) SRAM_RotaryActSpeed);
		
		Screw.SetState		 	= (uint8_t *) (SRAM_PARAM_OFFSET + (uint8_t) SRAM_ScrewSetState);
		Screw.ActState			= (uint8_t *) (SRAM_PARAM_OFFSET + (uint8_t) SRAM_ScrewActState);
		Screw.Speed						= (uint8_t *) (SRAM_PARAM_OFFSET + (uint8_t) SRAM_ScrewSpeed);	

	
		Pump.DeviceStatus 	= (uint8_t *) (SRAM_PARAM_OFFSET + (uint8_t) SRAM_PumpDevState);
		Pump.Pressure		 		= (uint8_t *) (SRAM_PARAM_OFFSET + (uint8_t) SRAM_PumpPressure);

	
}



//---------------------------------------------------------------------------------------
//	Filling the pointer list, with the adresses of the variables and the corresponding
//	Function to update this variables
//
//---------------------------------------------------------------------------------------



void Assign_Functions()
{

	struct UPDATEFCTLISTTyp *dmyList;
	
	UpdateFctList = (struct UPDATEFCTLISTTyp *) malloc(sizeof (*UpdateFctList));	//neues Listenelement anlegen
	//Head of List
	dmyList = UpdateFctList;			
	

	dmyList->Src = SRAM_ScrewSpeed;																							// Speicherstelle des Wertes
	dmyList->Dest =(fpUpdateTyp *) malloc(sizeof(*dmyList->Dest));							// Speicher für Funktionspointer allokieren
	*dmyList->Dest = &PWM_ScrewSpeed_Update;																	// Adresse der Funktion in Liste aufnehmen	
	dmyList->pNext = (struct UPDATEFCTLISTTyp *) malloc(sizeof (*dmyList));  //neues Listenelement anlegen
	
	dmyList = dmyList->pNext;
	dmyList->Src = SRAM_RotarySpeed;																					// Speicherstelle des Wertes
	dmyList->Dest =(fpUpdateTyp *) malloc(sizeof(*dmyList->Dest));					// Speicher für Funktionspointer allokieren
	*dmyList->Dest = &PWM_RotarySpeed_Update;															// Adresse der Funktion in Liste aufnehmen

	dmyList->pNext = NULL;					//End of List


}





///**
//  * @brief  Daten_Init()
//	*			Initialisierung der Funktionen und Variablen
//	*		
//  * @param  None
//  * @retval None
//  */


void Daten_Init()
{

//	uint16_t LoopCounter = 0;
	


	RxRFBuffer1 = (uint8_t *) malloc (sizeof (uint8_t) * RXRF_BUFFER_SIZE);
	RxRFBuffer2	= (uint8_t *) malloc (sizeof (uint8_t) * RXRF_BUFFER_SIZE);
	pRxBuff = (uint8_t **) malloc (sizeof (uint8_t *) * 3);													// 2 Buffer adressen

	pRxBuff[0] = RxRFBuffer1;
	pRxBuff[1] = RxRFBuffer2;
	RxRFActBufferID = 0;

	
	RxRFPack.pBufferReception = RxRFBuffer1;					//Vorab - ini
	RxRFPack.pBufferProcess = RxRFBuffer2;
	RxRFPack.dRxProcessChars = 0;
	RxRFPack.dRxReceptionChars = 0;
	RxRFPack.fBufferReady	= 0;
	RxRFPack.fDataReceiving = 0;
	RxRFPack.fDataValid = 0;
	RxRFPack.fAckReceiving = 0;
	
	
	
	RxPCPack.pBufferProcess = (uint8_t *) malloc (sizeof (uint8_t) * RXPC_BUFFER_SIZE);
	RxPCPack.pBufferReception = (uint8_t *) malloc (sizeof (uint8_t) * RXPC_BUFFER_SIZE);
	RxPCPack.dRxProcessChars = 0;
	RxPCPack.dRxReceptionChars = 0;
	RxPCPack.fAckReceiving = 0;
	RxPCPack.fBufferReady = 0;
	RxPCPack.fDataReceiving = 0;
	RxPCPack.fDataValid	= 0;


	TxRFPack.pBufferSend = (uint8_t *) malloc(sizeof(uint8_t)*(RF_DATAPACKET_SIZE+5+1));   //STX+Payload + CRC + ETX + 



	sPCTransfer = RxTxIdle;	
	
	EEDataPack.EERxRcvFlag = 0;
	EEDataPack.EERxDataCnt = 0;
	EEDataPack.EEBufferCounter = 0;
	EEDataPack.EETimeOut = sEE_LONG_TIMEOUT;
	EEDataPack.pEERxBuffer = &I2CRxBuffer[0];  
	EEDataPack.pEETxBuffer = &I2CTxBuffer[0];  
	
	
	
	
//opi	ParamAddress();


	*PSW = 0x00;
	*ErrorByte = 0x00;	
	State.Status = 0x00;
//	State.PgmFlow = BOOT;
	PFlags.WrReq = 0x00;
	PFlags.WrSend = 0x00;
	PFlags.EE_WrReq = 0x00;
	PFlags.RF_TX_Acitve = 0x00;											//
	PFlags.Pump_Pressure_UpdateReq = 0x02;
	
	LED_Blinking_Timer = LED_TIMER_UPDATETIME;
	
	*EEPromID_ACT[0] = '\0';											// Init aktuelle EEPROM Version
	strcat (EEPromID_ACT[0], EEProm_ID);	

	

}

///**
//  * @brief  Daten_Default()
//	*			Systemvariablen werden mit Standardwerten belegt
//	*			Danach werden die aktuellen Werte aus dem EEPROM gelesen
//	*		
//  * @param  None
//  * @retval None
//  */

void Parameter_Init(void)
{
	

	

	uint8_t EEProm_State = 0;


Daten_Default(1);	
	

	
	
}



void Daten_Default(uint8_t ini)
{

	uint8_t i;
	
	static char Text[9];
	
	strcpy(&Text[0], DetektTypText);
	
	
  for (i = 0; i < 8; i++)
	{
		*DeviceText[i] = Text[i];
		
	}
	
	Initialize_Text(DeviceSerial[0], DetektSerienNr);
	

	*DeviceProdDate[0] = '\0';
	strcat (DeviceProdDate[0], ProdDatum);
	
	*DeviceFWVersion[0] = '\0';
	strcat (DeviceFWVersion[0], FW_Version);

	Initialize_Text(DeviceHWVersion[0], HW_Version);
	
	RFModData.Channel_ID = RF_CHANNEL;
	RFModData.State = 0;
	RFModData.StartTimer = RF_STARTUP_TIME;						//200ms StartUp Wait-Timer for PowerUp the RF Modul

	*RFModul_Channel 	= RF_CHANNEL;				//Modbus - Geräte Adresse (Modul ID)
	*RFModul_Baud 		= UART_RF_Baudrate;			//MODBUS Baudrate

	*Rotary.ActSpeed = 0;
	*CountIncrement = 0;												// Messzähler
	
 
	// Variablen für Counter Input

	DMA_CntCycle	= (*IntTime) / 100;						//Größe Bufferzyklus dh. wieviel 100ms Werte für gemessene Counts
	
	Counter_Init_Value = 0;
	fCountOvflw = 0;	
	IntTimeCntMax = INT_TIME_INIT / 100;			//Anzahl an summierenden Elementen


	EmergencyStop = 0;

	Screw.State 				= Stop;
	*Screw.ActState			= 0x00;
	*Screw.SetState 		= 0x00;
	*Screw.Speed				= 0x00;
	Screw.StartTimer		= 0x00;
	Screw.Error					= 0x00;
	
	Rotary.State			= Stop;
	*Rotary.ActState	= 0x00;
	*Rotary.SetState	= 0x00;
	*Rotary.Speed			= 0x00;
	Rotary.StartTimer	= 0x00;
	Rotary.Error			= 0x00;
	
	Feeder.State				= Stop;
	*Feeder.ActState		= 0x00;
	*Feeder.SetState		= 0x00;
	Feeder.StartTimer		= 0x00;
	Feeder.Error				= 0x00;
	
	
	*Pump.DeviceStatus 	= 0x00;
	*Pump.Pressure			= 0x00;
	
	
	
}



uint8_t EEPROM_Load_Data()
{
	uint8_t i = 0x00;

	uint16_t nrCells = 0x08;
	uint32_t  Buffer32[6];
	uint32_t calcCRC = 0x00000000;
	uint32_t eepromStatus = 0x00000000;
	nrCells = 0x08;
	
	char Test1, Test2;
	


// Load Device Configuration 
	
	nrCells = 0x02;
	eepromStatus += EE_ReadBuffer(EEPROM_ADRESS, ((uint8_t *) (SRAM_PARAM_OFFSET + (uint8_t) SRAM_ROM_ID)), E2Prom_Kennung, nrCells);

	
	if ((*EEPromID[0]) != (*EEPromID_ACT[0]) || (*EEPromID[1]) != (*EEPromID_ACT[1]))
	{
			return 2;								// EEPORM ID wrong  - jungfräulich?
	}
	

	
	
	nrCells = 0x08;
	eepromStatus += EE_ReadBuffer(EEPROM_ADRESS, ((uint8_t *) (SRAM_PARAM_OFFSET + (uint8_t) SRAM_SondenText)), E2Prom_SondenText, nrCells);

	nrCells = 0x08;
	eepromStatus += EE_ReadBuffer(EEPROM_ADRESS, ((uint8_t *) (SRAM_PARAM_OFFSET + (uint8_t) SRAM_Serial)), E2Prom_Seriennummer, nrCells);

	nrCells = 0x08;
	eepromStatus += EE_ReadBuffer(EEPROM_ADRESS, ((uint8_t *) (SRAM_PARAM_OFFSET + (uint8_t) SRAM_ProdDat)), E2Prom_ProdDat, nrCells);

	nrCells = 0x08;
	eepromStatus += EE_ReadBuffer(EEPROM_ADRESS, ((uint8_t *) (SRAM_PARAM_OFFSET + (uint8_t) SRAM_FW_Ver)), E2Prom_FW_Ver, nrCells);

	nrCells = 0x08;
	eepromStatus += EE_ReadBuffer(EEPROM_ADRESS, ((uint8_t *) (SRAM_PARAM_OFFSET + (uint8_t) SRAM_HW_Ver)), E2Prom_HW_Ver, nrCells);

// Load Modul Parameter

	nrCells = 0x02;
	eepromStatus += EE_ReadBuffer(EEPROM_ADRESS, ((uint8_t *) (SRAM_PARAM_OFFSET + (uint8_t) SRAM_RF_Baud)), E2Prom_RF_BAUD, nrCells);
	
	nrCells = 0x02;
	eepromStatus += EE_ReadBuffer(EEPROM_ADRESS, ((uint8_t *) (SRAM_PARAM_OFFSET + (uint8_t) SRAM_RF_CHANNEL)), E2Prom_RF_CHANNEL, nrCells);

// Load operational parameter


//	
	if (eepromStatus != 0)
	{
		return 1;						//Read Error of EEPROM
	}
	
	return 0;							//alles ok
		
}


