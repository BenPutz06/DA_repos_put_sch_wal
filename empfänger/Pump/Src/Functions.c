/**
  ******************************************************************************
  * @file    Functions.c
  * @author  Benjamin Putz  
  * @brief   File contains common used  functions for programm sequence
  *          
  *          
  ******************************************************************************
  * @attention
  *
  * All rights reserved.</center></h2>
  *
  * 
  *
  ******************************************************************************
  */
	
	/* Includes ------------------------------------------------------------------*/


#include "Functions.h"
#include "Periph_Funct.h"
#include "Parameter.h"
#include "stdlib.h"
#include "string.h"



/********************************************************************************************************
* Convert two Byte Array to 16bit unsigned int
* Param:  Byte Array
* RetVal: 16bit Val
*********************************************************************************************************/

uint16_t ConvertByteArrToUint(uint8_t ArrByte[2])
{
	uint16_t  dmy;
	
	dmy = ArrByte[1];
	dmy <<= 8;
	dmy += ArrByte[0];
	
	return dmy;
}

	
/********************************************************************************************************
* Calculate BCC (XOR) of UINT8 - Array
* Param:  None
* RetVal: None
*********************************************************************************************************/

uint8_t CalcBCC(uint8_t* U8,uint8_t Cnt)
{
	
	uint8_t iCnt = 0x01;
	uint8_t BCC = U8[0];
	
	while (--Cnt)
	{
		BCC ^= U8[iCnt++];
		
	}
	
	return BCC;
	
}


/********************************************************************************************************
* Calculate number of Cnt BCC (XOR) of  UINT8 - Array, the calculated BCC Value is stored after the last Byte in the next
*	Memory location.
* Param:  None
* RetVal: None
*********************************************************************************************************/


void pCalcBCC(uint8_t* pU8, uint8_t Cnt)
{
	
	uint8_t ptr;
	//ptr = malloc (sizeof(uint8_t));
	
	while (Cnt--)
	{
		ptr ^= *pU8; 
		(pU8)++;
	}
	*pU8 = ptr;
	
	//free(ptr);
	
}


/********************************************************************************************************
* Calculate number of Cnt BCC (XOR) of  UINT8 - Memory, the calculated BCC Value is returnd through BCC
*	variable
* Param:  None
* RetVal: None
*********************************************************************************************************/


void ppCalcBCC(uint8_t* pU8, uint8_t Cnt, uint8_t *BCC)
{
	
	uint8_t ptr;
	ptr = 0;
	uint8_t *pptr;
	
	//ptr = malloc (sizeof(uint8_t));
	pptr = pU8;
	
	while (Cnt--)
	{
		ptr ^= *pptr; 
		(pptr)++;
	}
	
	*BCC = ptr;
	//free(ptr);
		
}


uint8_t HexStringToBytes(char s[], uint8_t *bData)
	{
		uint8_t StrLen,LoopCnt;
		char *Sign;
		const char* HEX_CHARS = "0123456789ABCDEF";
		
		StrLen = strlen(s);
		
		LoopCnt = 0;
		
    if (StrLen == 0)
        bData[0] = 0;

    if ((StrLen + 0) % 2 != 0)
        return 0; //MessageBox.Show("Fehler");//throw new FormatException("ung�ltig");

    //byte[] bytes = new byte[(s.Length + 0) / 2];

    uint8_t state = 0; // 0 = expect first digit, 1 = expect second digit, 2 = expect hyphen
    int currentByte = 0;
    int x;
		uint8_t value = 0;

    do 
    {
       switch (state)
        {
          case 0:
						Sign = strchr(HEX_CHARS, s[LoopCnt]);
						x = (int) (Sign-HEX_CHARS);
              //x = HEX_CHARS[s[LoopCnt]];
						if (x == -1)
								return 0;
						value = x << 4;
						state = 1;
          break;
          case 1:
						Sign = strchr(HEX_CHARS, s[LoopCnt]);
						x = (int) (Sign-HEX_CHARS);
               //x = HEX_CHARS[s[LoopCnt]];
            if (x == -1)
                 return 0;
            bData[currentByte++] = (uint8_t)(value + x);
            state = 0;
          break;
         case 2:
 //             if (c != '-')
 //             throw new FormatException();
 //             state = 0;
         break;
			 }
    } while ((++LoopCnt) < StrLen);

    return 1;
}

uint8_t BytesToHexString(char *s, uint8_t *bData, uint8_t DataCount)
{
	
		char BYTE_CHARS[16] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};  
	
		uint8_t LoopCount = 0;
		uint8_t Value;
		uint8_t State = 0; 
		char Sign;
		*s = '\0';
		
	 
		do
		{
		   switch (State)
			 {
				 case 0:
					 Value = *bData;
					 Value >>= 4;
					 Sign = BYTE_CHARS[Value];
//					 *s[StringCount++] = *Sign;
				 		 *s = Sign;
							s++;
						State = 1;
				 
				 break;
				 
				 case 1:
						Value = *bData;
						bData++;
						Sign = BYTE_CHARS[Value & 0x0F];
//					 *s[StringCount++] = *Sign;
						*s = Sign;
						s++;
						State = 0;
						++LoopCount;
				 break;
				 
			 }
		 
		 
		} while (LoopCount < DataCount);
		
		return 1;
		
}

void Initialize_Text(char* Desti, char* Source)
{
	uint8_t i;
	
  for (i = 0; i < 8; i++)
	{
		Desti[i] = Source[i];
		
	}	
	
}

void Schreib_I2C(void)
{

	
		I2CTxBuffer[0] = 0x70;

	I2CTxBuffer[1] = 0x06;
	I2CTxBuffer[2] = 0x06;
	I2CTxBuffer[3] = 0x04;
	I2CTxBuffer[4] = 0x00;


 
	I2CTxBuffer[5] = 0x05;
	I2CTxBuffer[6] = 0x00;
	I2CTxBuffer[7] = 0x06;
	I2CTxBuffer[8] = 0x00;
//	I2CTxBuffer[5] = 0x46;


 Handle_I2C_Master(EEPROM_ADRESS, &I2CTxBuffer[1], I2CTxBuffer[0], 2);
	
	
}


void SET_Error_BlinkCode(uint8_t ErrorTyp[])
{
	
//Code[0] ist clearing the LEDs, is done in void BLINKCODE_Generate ()	

	//Code 1 is a LED flash of LED_NF_Att
	Error_Config_Code[1].Status = 3;
	Error_Config_Code[1].BlinkCounts = 1;
	Error_Config_Code[1].Blinking = 0;

	//Code 2 LED_NF_Att is powered  LED off..Atten off   LED On..Att On
//		if (ErrorTyp[0]) Error_Config_Code[2].Blinking = 7;
//		else Error_Config_Code[2].Blinking = 0;
		Error_Config_Code[2].Blinking = 7;
	  Error_Config_Code[2].Status = 3;
		Error_Config_Code[2].BlinkCounts = 5;

		//Code 3 is a LED flash of LED_NF_Att
	Error_Config_Code[3].Status = 3;
	Error_Config_Code[3].BlinkCounts = 1;
	Error_Config_Code[3].Blinking = 0;

//Code 4 LED_Power_Green is blinking for the selected TP Filter: 0..500kHz 1..100kHz 2..10kHz
		Error_Config_Code[4].BlinkCounts = ErrorTyp[0];				//set Blinkcode for Tiefpass
		Error_Config_Code[4].Blinking = 2;
		Error_Config_Code[4].Status = 3;

	//Code 5 is a LED flash of LED_NF_Att
	Error_Config_Code[5].Status = 3;
	Error_Config_Code[5].BlinkCounts = 1;
	Error_Config_Code[5].Blinking = 0;
	
	//Code 6 LED_Power_Red is blinking for the selected HP Filter: 0..DC,  1..12kHz  2..300Hz  3..not used
		Error_Config_Code[6].BlinkCounts = ErrorTyp[1];				//set Blinkcode for Highpass
		Error_Config_Code[6].Blinking = 2;
		Error_Config_Code[6].Status = 3;

	//Code 7 is a LED flash of LED_NF_Att
	Error_Config_Code[7].Status = 3;
	Error_Config_Code[7].BlinkCounts = 1;
	Error_Config_Code[7].Blinking = 0;

	
}