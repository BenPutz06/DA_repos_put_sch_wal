#include <Arduino.h>
#include <U8x8lib.h>

//#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>

#define SbFwdPin 2
#define SbRevPin 3
#define SkFwdPin 4
#define SkRevPin 5
#define SlFwdPin 6
#define SlRevPin 7
#define DBtn01Pin A4
#define DBtn02Pin A5
#define PwrBtnPin A6
#define PwrBtnIcPin A7
#define SkPotiPin A0
#define SlPotiPin A1
#define RFResetPin A3
#define NotAusPin 12

#define PacketLengthTx 13
#define PacketLengthRx 19


byte SendPrtkl[PacketLengthTx];
byte RecvPrtkl[PacketLengthRx];
byte Checksum = 0;
int timerSend = 0;
int connectionTimerWatchdog = 0;
int checksumIncoming = -1;

/////////////// Init for Display///////////////////////
U8X8_SSD1309_128X64_NONAME2_4W_SW_SPI u8x8(13, 11, 10, 9, 8);
const int PinBtnDisplayChange = 2;
int btnDisplayChangeState = 0;
int previousBtnDisplayChangeState = 0;
int displayPage = 2;
int animationCounter = 0;
bool connection = false;
bool connectionCheck = false;
bool emergencyStop = false;
int cellWheelTarget = 0;
int cellWheelDirection = 0;
int cellWheelActual = 0;
int cellWheelActualTemp = 0;
int augerSpeed = 0;
int augerDirection = 0;
double fanPressure = 0;
int pushOffDirection = 0;
byte connectionInputArray[11];

byte TempByteArray[] = { 114, 44, 0, 143, 35, 7, 0, 0, 0, 0, 15, 250, 0, 0, 0, 0, 42, 97, 0, 0, 29 };

/// Symbols:
uint8_t CellWheel[16] = { 0x81, 0x42, 0x42, 0x42, 0x81, 0x91, 0x91, 0x9f, 0xf9, 0x89, 0x89, 0x81, 0x42, 0x42, 0x42, 0x81 };
uint8_t RotationR[8] = { 0x3c, 0x42, 0x81, 0x81, 0x81, 0x85, 0x46, 0x06 };
uint8_t RotationL[8] = { 0x06, 0x46, 0x85, 0x81, 0x81, 0x81, 0x42, 0x3c };
uint8_t Auger[16] = { 0x1c, 0x1e, 0x1c, 0x38, 0x78, 0x38, 0x1c, 0x1e, 0x1c, 0x38, 0x78, 0x38, 0x1c, 0x1e, 0x1c, 0x00 };
uint8_t PushOff[16] = { 0x91, 0xa2, 0x88, 0x92, 0xc4, 0x91, 0xa4, 0xc8, 0xe0, 0xff, 0xbf, 0x98, 0x98, 0x98, 0x98, 0xff };
uint8_t Fan[16] = { 0x0e, 0x8c, 0xc8, 0xe0, 0x07, 0x13, 0x31, 0x70, 0x00, 0x92, 0x92, 0x49, 0x49, 0x92, 0x92, 0x49 };


//////////////////////////////////// Methoden für Interrrupts/////////////////////////////////////
unsigned long lastDebounceTime = 0; 
unsigned long debounceDelay = 150;
void DBtn01Falling() {
  unsigned long currentTime = millis();

  if (currentTime - lastDebounceTime > debounceDelay) {
    lastDebounceTime = currentTime;

    if (displayPage >= 3) {
      displayPage = 1;
    } else {
      displayPage = displayPage + 1;
    }
    pre();
  }
}
void DBtn02Falling() {
  // Display btn02
}
void PwrBtn() {
  digitalWrite(PwrBtnIcPin, HIGH);
}
void NotAus() {
  byte Sendbyte[PacketLengthTx] = { 234, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 246 };

  Serial1.write("ATS");           // Befehl an RF Modul zum Senden
  Serial1.write(2);               // Sende an Kanal 2
  Serial1.write(PacketLengthTx);  // 13 Bytes werden gesendet

  for (int j = 0; j < PacketLengthTx; j++) {
    Serial1.write(Sendbyte[j]);
  }

   if(emergencyStop != true){
     displayPage = 3;
     pre();
   }
  emergencyStop = true;
}

void pre(void) {
  u8x8.clear();
  u8x8.setFont(u8x8_font_chroma48medium8_r);

  if (connection == false) {
    u8x8.print("V-|");
  } else if (connection == true) {
    u8x8.print("V+|");
  } else {
    u8x8.print("V?|");
  }

  if (displayPage == 1) {
    u8x8.inverse();
    u8x8.print(" Verbind. ");
  } else if (displayPage == 2) {
    u8x8.inverse();
    u8x8.print(" Uebers. ");
  } else if (displayPage == 3){
    u8x8.inverse();
    u8x8.print(" Notaus  ");
  }



  u8x8.setFont(u8x8_font_chroma48medium8_r);
  u8x8.noInverse();
  u8x8.setCursor(12, 0);
  u8x8.print("|#");
  u8x8.print(displayPage);
  u8x8.print(" ");
  u8x8.setCursor(0, 1);

  u8x8.setFont(u8x8_font_chroma48medium8_r);
  u8x8.noInverse();
  u8x8.setCursor(0, 1);
  uint8_t tiles2[128] = { 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 0, 0, 0, 0, 0, 0, 0, 0 };
  u8x8.drawTile(0, 1, 16, tiles2);
}



void draw_bar(uint8_t c, uint8_t is_inverse) {
  uint8_t r;
  u8x8.setInverseFont(is_inverse);
  for (r = 0; r < u8x8.getRows(); r++) {
    u8x8.setCursor(c, r);
    u8x8.print(" ");
  }
}

void setup() {
  delay(1000);
  digitalWrite(PwrBtnIcPin, LOW);

  pinMode(SbFwdPin, INPUT);
  pinMode(SbRevPin, INPUT);
  pinMode(SkFwdPin, INPUT);
  pinMode(SkRevPin, INPUT);
  pinMode(SlFwdPin, INPUT);
  pinMode(SlRevPin, INPUT);
  pinMode(DBtn01Pin, INPUT);
  pinMode(DBtn02Pin, INPUT);
  pinMode(PwrBtnPin, INPUT);
  pinMode(NotAusPin, INPUT_PULLUP);
  pinMode(PwrBtnIcPin, OUTPUT);
  pinMode(SkPotiPin, INPUT);
  pinMode(SlPotiPin, INPUT);
  pinMode(RFResetPin, OUTPUT);

  u8x8.begin();
  pre();

  attachInterrupt(digitalPinToInterrupt(DBtn01Pin), DBtn01Falling, FALLING);
  // attachInterrupt(digitalPinToInterrupt(DBtn02Pin), DBtn02Falling, FALLING);
  attachInterrupt(digitalPinToInterrupt(PwrBtnPin), PwrBtn, RISING);
  attachInterrupt(digitalPinToInterrupt(NotAusPin), NotAus, RISING);


  Serial.begin(19200);
  Serial1.begin(19200);
  delay(1000);
  digitalWrite(RFResetPin, LOW);
  Serial1.write("ATR");
  Serial1.write(2);
  Serial1.write(PacketLengthTx);
  delay(100);
  Serial1.write("AT?");
  delay(10);
  if (Serial1.available() > 0) {
    Serial.println("Configuration of the RF Modul:");
  }
  while (Serial1.available() != 0) {
    Serial.print(Serial1.read());
  }

  if(digitalRead(NotAusPin) == HIGH) {
    NotAus();
  }

  send();
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial1.available() > 0) {
    delay(10);
    connectionTimerWatchdog = 0;
    empfangen();
  }
  if (timerSend >= 400) {
    if(emergencyStop == true && digitalRead(NotAusPin) == LOW){
      emergencyStop = false;
    }
    send();
    timerSend = 0;
    UpdateDisplay();
  }
  timerSend++;
  connectionCheck = connection;

  /////////////////////////////Display////////////////////////////

  if (animationCounter <= 1000) {
    animationCounter += 1;
  } else {
    animationCounter = 0;
  }

  connectionTimerWatchdog++;
  if (connectionTimerWatchdog > 2000 || checksumIncoming == -1) {
    connection = false;
    if(connectionTimerWatchdog > 2000) {
      checksumIncoming = -1;
    }
    if (connectionCheck != connection) {
      pre();
      displayPage = 1;
    }
  } else {
    connection = true;
    if (connectionCheck != connection) {
      pre();
    }
  }

delay(1);
}

////////////////////////////////Funkübertragung////////////////////////////////

void send() {
  //Serial.println();

  readPins();

  finishProtocol();

  Serial1.write("ATS");           // Befehl an RF Modul zum Senden
  Serial1.write(2);               // Sende an Kanal 2
  Serial1.write(PacketLengthTx);  // 13 Bytes werden gesendet

  //Serial.write("ATS");  // Befehl an RF Modul zum Senden
  //Serial.write(2);      // Sende an Kanal 2
  //Serial.write(PacketLengthTx);

  if(emergencyStop != true){
    for (int j = 0; j < PacketLengthTx; j++) {
      Serial1.write(SendPrtkl[j]);  // Byte für Byte versenden
      //Serial.print(SendPrtkl[j], DEC);  // Byte für Byte versenden
      //Serial.print("|");
    }
  }
}

void readPins() {
  SendPrtkl[4] = analogRead(SkPotiPin) / 10.23;  // durch 10.23 dividieren, damit ein Wert zw. 0-100 entsteht
  SendPrtkl[5] = analogRead(SlPotiPin) / 10.23;  // durch 10.23 dividieren, damit ein Wert zw. 0-100 entsteht

  augerSpeed = analogRead(SkPotiPin) / 10.2;
  cellWheelTarget = analogRead(SlPotiPin) / 10.2;

  if (digitalRead(SbFwdPin) == HIGH && digitalRead(SbRevPin) == HIGH) {
    SendPrtkl[1] = 0;
  } else if (digitalRead(SbFwdPin) == LOW) {
    SendPrtkl[1] = 6;
  } else if (digitalRead(SbRevPin) == LOW) {
    SendPrtkl[1] = 7;
  }

  if (digitalRead(SkFwdPin) == HIGH && digitalRead(SkRevPin) == HIGH) {
    SendPrtkl[2] = 0;
  } else if (digitalRead(SkFwdPin) == LOW) {
    SendPrtkl[2] = 6;
  } else if (digitalRead(SkRevPin) == LOW) {
    SendPrtkl[2] = 7;
  }

  if (digitalRead(SlFwdPin) == HIGH && digitalRead(SlRevPin) == HIGH) {
    SendPrtkl[3] = 0;
  } else if (digitalRead(SlFwdPin) == LOW) {
    SendPrtkl[3] = 6;
  } else if (digitalRead(SlRevPin) == LOW) {
    SendPrtkl[3] = 7;
  }
}

void finishProtocol() {
  SendPrtkl[0] = 234;
  SendPrtkl[6] = 0;
  SendPrtkl[7] = 0;
  SendPrtkl[8] = 0;
  SendPrtkl[9] = 0;
  SendPrtkl[10] = 0;

  Checksum = SendPrtkl[1];
  for (int i = 2; i < PacketLengthTx - 3; i++) {
    Checksum ^= SendPrtkl[i];
  }
  SendPrtkl[11] = Checksum;
  SendPrtkl[12] = 246;
}

void empfangen() {
  int i = 0;
  while (Serial1.available() != 0) {
    RecvPrtkl[i] = Serial1.read();
    //Serial.print(RecvPrtkl[i], DEC);
    i++;
  }
  for (int j = 0; j < i; j++) {
    Serial.print(RecvPrtkl[j], DEC);
    Serial.print("|");
  }
  Serial.println();

  int startIndex = -1;
  int endIndex = -1;

  for (int j = 0; j < i; j++) {
    if (RecvPrtkl[j] == 234) {
      startIndex = j + 1;
    }
    if (RecvPrtkl[j] == 246) {
      endIndex = j;
      break;
    }
  }

  if (startIndex != -1 && endIndex != -1 && (endIndex - startIndex) == 11) {
    for (int j = 0; j < 11; j++) {
      connectionInputArray[j] = RecvPrtkl[startIndex + j];
    }
    pushOffDirection = connectionInputArray[0];
    augerDirection = connectionInputArray[1];
    cellWheelDirection = connectionInputArray[2];
    fanPressure = double(connectionInputArray[6]) / 10;
    cellWheelActualTemp = connectionInputArray[5];
    checksumIncoming = connectionInputArray[10];
  }

    if (cellWheelActualTemp >= 1 && cellWheelActualTemp <= 128) {
      cellWheelActual = cellWheelActualTemp;// map(cellWheelActualTemp, 1, 128, -1, -100);
    } else if (cellWheelActualTemp >= 129 && cellWheelActualTemp <= 255) {
      cellWheelActual = cellWheelActualTemp; //map(cellWheelActualTemp, 129, 255, 1, 100);
    }
    else{
      cellWheelActual = 0;
    }

  UpdateDisplay();
}

void UpdateDisplay() {

  if (displayPage == 1) {
    if (connection == true) {
      u8x8.setFont(u8x8_font_7x14B_1x2_f);
      u8x8.drawString(2, 3, " Verbindung");
      u8x8.drawString(5, 5, " O.K.");
      u8x8.noInverse();
    } else {
      u8x8.setFont(u8x8_font_7x14B_1x2_f);
      if (animationCounter >= 500) {
        u8x8.inverse();
      } else {
        u8x8.noInverse();
      }

      u8x8.drawString(5, 3, "KEINE");
      u8x8.drawString(2, 5, "VERBINDUNG!");
      u8x8.noInverse();
    }
  } else if (displayPage == 2) {
    u8x8.setFont(u8x8_font_chroma48medium8_r);


    u8x8.drawTile(0, 2, 2, CellWheel);
    u8x8.setCursor(3, 2);
    u8x8.print(cellWheelTarget);
    if(cellWheelTarget < 10){
    u8x8.print("%  ");
    }
    else if(cellWheelTarget < 100){
    u8x8.print("% ");
    }
    else {
    u8x8.print("%");
    }
    u8x8.drawString(8, 2, "|");
    if (cellWheelDirection == 7) {
      u8x8.drawTile(7, 2, 1, RotationR);
    } else if (cellWheelDirection == 6) {
      int posCellWheelTarget = cellWheelTarget * (-1);
      u8x8.drawTile(7, 2, 1, RotationL);
    } else {
      u8x8.drawString(7, 2, "X");
    }

    u8x8.setCursor(10, 2);
    u8x8.print(cellWheelActual);
    if(cellWheelActual < 10){
      u8x8.print("Um  ");
    }
    else if(cellWheelActual < 100){
      u8x8.print("Um ");
    }
    else {
      u8x8.print("Um");
    }

    u8x8.drawTile(0, 4, 2, Auger);
    u8x8.setCursor(3, 4);

    u8x8.print(augerSpeed);
    if(augerSpeed < 10){
      u8x8.print("%  ");
    }
    else if(augerSpeed < 100){
      u8x8.print("% ");
    }
    else {
      u8x8.print("%");
    }

    if (augerDirection == 7) {
      u8x8.setFont(u8x8_font_open_iconic_arrow_1x1);
      u8x8.drawString(7, 4, "A B");
      u8x8.setFont(u8x8_font_chroma48medium8_r);
    } else if (augerDirection == 6) {
      u8x8.setFont(u8x8_font_open_iconic_arrow_1x1);
      u8x8.drawString(7, 4, "B A");
      u8x8.setFont(u8x8_font_chroma48medium8_r);
    } else if(augerDirection == 0) {
      u8x8.drawString(7, 4, " X ");
    } else {
      u8x8.drawString(7, 4, " X ");
    }

    u8x8.drawTile(0, 6, 2, PushOff);
    if (pushOffDirection == 7) {
      u8x8.setFont(u8x8_font_open_iconic_arrow_1x1);
      u8x8.drawString(3, 6, "A");
      u8x8.setFont(u8x8_font_chroma48medium8_r);
      u8x8.drawString(4, 6, "Z");
      u8x8.setFont(u8x8_font_open_iconic_arrow_1x1);
      u8x8.drawString(5, 6, " ");
    } else if (pushOffDirection == 6) {
      u8x8.setFont(u8x8_font_open_iconic_arrow_1x1);
      u8x8.drawString(3, 6, " ");
      u8x8.setFont(u8x8_font_chroma48medium8_r);
      u8x8.drawString(4, 6, "V");
      u8x8.setFont(u8x8_font_open_iconic_arrow_1x1);
      u8x8.drawString(5, 6, "B");
    } else if (pushOffDirection == 0) {
      u8x8.setFont(u8x8_font_open_iconic_arrow_1x1);
      u8x8.drawString(3, 6, "A");
      u8x8.setFont(u8x8_font_chroma48medium8_r);
      u8x8.drawString(4, 6, "X");
      u8x8.setFont(u8x8_font_open_iconic_arrow_1x1);
      u8x8.drawString(5, 6, "B");
    } else {
      if (animationCounter >= 500) {
        u8x8.inverse();
      } else {
        u8x8.noInverse();
      }
      u8x8.setFont(u8x8_font_open_iconic_arrow_1x1);
      u8x8.drawString(3, 6, "A");
      u8x8.setFont(u8x8_font_chroma48medium8_r);
      u8x8.drawString(4, 6, "?");
      u8x8.setFont(u8x8_font_open_iconic_arrow_1x1);
      u8x8.drawString(5, 6, "B");
    }
    u8x8.noInverse();
    u8x8.drawTile(7, 6, 2, Fan);
    u8x8.setFont(u8x8_font_chroma48medium8_r);
    u8x8.setCursor(10, 6);
    u8x8.print(fanPressure);
    u8x8.print("b");
  } else if(displayPage == 3){
    if (emergencyStop == true) {
    //if(digitalRead(NotAusPin) == HIGH) {
      u8x8.setFont(u8x8_font_7x14B_1x2_f);
      if (animationCounter >= 500) {
        u8x8.inverse();
      } else {
        u8x8.noInverse();
      }

      u8x8.drawString(5, 3, "NOTAUS");
      u8x8.drawString(5, 5, "AKTIV!");
      u8x8.noInverse();
    } else if(emergencyStop == false) {
    //} else if(digitalRead(NotAusPin) == LOW) {
      u8x8.setFont(u8x8_font_7x14B_1x2_f);
      u8x8.drawString(4, 3, " Notaus");
      u8x8.drawString(5, 5, " O.K. ");
      u8x8.noInverse();
    } else {
      u8x8.setFont(u8x8_font_7x14B_1x2_f);
      u8x8.drawString(4, 3, " Error");
      u8x8.drawString(5, 5, " 500");
      u8x8.noInverse();
    }
  }
}