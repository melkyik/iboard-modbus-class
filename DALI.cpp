#include "DALI.h"
// **************************** прием ответа от слейва *******************  


// #define DEBUG



byte DALIprotocol::DaliReciveCMD(){
  byte recive_byte;
  unsigned long pause_ms1;
     
 //delayMicroseconds(7* DALI_HALF_BIT_TIME);      // пауза между передачей и ответом 2,92 мс для алюминиевых блоков 11 для пластика 7

  pause_ms1 = millis();
  while (digitalRead(DALI_RX_PIN) ^ INV_READ){          // ждем падающий фронт в линии
     if (millis() - pause_ms1 > 30){              // если таймаут ожидания вышел
       recive_byte=0;
    //Serial.println ("  NO answer from device ");
       break;
     }
  }

  if ((!digitalRead(DALI_RX_PIN) ^ INV_READ )){              // считали низкий уровень в шине (активность)
    delayMicroseconds(DALI_HALF_BIT_TIME*3+200); // пропускаем стоповый бит и 3/4 следующего такта
    for (int i=0; i< 8; i++){      // цикл битового чтения
        pause_ms1 = millis();
      recive_byte  <<= 1;         // сдвигаем регистр
      if (digitalRead(DALI_RX_PIN)^ INV_READ) bitSet (recive_byte,0);  // вставляем новый бит, если он 1
      delayMicroseconds(DALI_HALF_BIT_TIME*2);      // задержка, равная периоду передачи бита

    }
    //Serial.print ("  Answer = ");
    //Serial.println (recive_byte, BIN);
  }

delay(DALI_TWO_PACKET_DELAY);
return recive_byte;
}



//---------------------- функция отправки команд в шину ---------------------------
// параметры -  address, commamd, TX, RX
// возвращаемое значение - значение принятое в ответ 
byte  DALIprotocol::DaliTransmitCMD(uint8_t Part1, uint8_t Part2)
{
  uint8_t DALI_CMD[] = { Part1, Part2 };
  
//****************** передача команды ***************************
  //Старт бит
  digitalWrite(DALI_TX_PIN, TX_LOW_LEVEL ^ INV_WRITE);
  delayMicroseconds(DALI_HALF_BIT_TIME);
  digitalWrite(DALI_TX_PIN, TX_HIGH_LEVEL ^ INV_WRITE);
  delayMicroseconds(DALI_HALF_BIT_TIME);
  //команда
  for (uint8_t CmdPart = 0; CmdPart < 2; CmdPart++)
  {
    for(int i = 7; i >= 0; i--)
    {
        if ((DALI_CMD[CmdPart] >> i) & 1){            // если бит =1
          digitalWrite(DALI_TX_PIN, TX_LOW_LEVEL ^ INV_WRITE);
          delayMicroseconds(DALI_HALF_BIT_TIME);
          digitalWrite(DALI_TX_PIN, TX_HIGH_LEVEL ^ INV_WRITE);
          delayMicroseconds(DALI_HALF_BIT_TIME);
        }
        else{                                         // если бит =0
          digitalWrite(DALI_TX_PIN, TX_HIGH_LEVEL ^ INV_WRITE);
          delayMicroseconds(DALI_HALF_BIT_TIME);
          digitalWrite(DALI_TX_PIN, TX_LOW_LEVEL ^ INV_WRITE);
          delayMicroseconds(DALI_HALF_BIT_TIME);
        }  
    }
  }

// стоп бит
  digitalWrite(DALI_TX_PIN, TX_HIGH_LEVEL^ INV_WRITE);     // стоп бит
  delayMicroseconds(4*DALI_HALF_BIT_TIME);      // стоп бит
}

// инициализация DALI
byte  DALIprotocol::DaliInit(word FirstAddr)
{
  
  int ShortAddr=FirstAddr;
  Serial.println("Initialization process...");
  Serial.println("Reset command");
  DaliTransmitCMD(RESET, 0x00);
  delay(2*DALI_TWO_PACKET_DELAY);
  DaliTransmitCMD(RESET, 0x00);
  delay(2*DALI_TWO_PACKET_DELAY);
  delay(100);

  Serial.println("Init command");  
  DaliTransmitCMD(INITIALISE, 0x00); 
  delay(DALI_TWO_PACKET_DELAY);
  DaliTransmitCMD(INITIALISE, 0x00);
  delay(DALI_TWO_PACKET_DELAY);
  DaliTransmitCMD(INITIALISE, 0x00);
  delay(DALI_TWO_PACKET_DELAY);
  delay(100);

  Serial.println("Randomise command");
  DaliTransmitCMD(RANDOMISE, 0x00);
  delay(DALI_TWO_PACKET_DELAY);
  DaliTransmitCMD(RANDOMISE, 0x00);
  delay(DALI_TWO_PACKET_DELAY);
  delay(100);

  int Step_Counter = 0;   // счетчик шагов при инициализации драйверов
   int StartShortAddr = ShortAddr;

  while(ShortAddr < 64)
  {
    long SearchAddr = 0xFFFFFF;
    bool Response = 0;
    long LowLimit = 0;
    long HighLimit = 0x1000000;

    Response = SearchAndCompare(SearchAddr);
    delay(DALI_TWO_PACKET_DELAY);
  
    if(Response>0)
    {
      Serial.println("Device detected, address searching...");
      
      if(!SearchAndCompare(SearchAddr - 1))
      {
        delay(DALI_TWO_PACKET_DELAY);
        SearchAndCompare(SearchAddr);
        delay(DALI_TWO_PACKET_DELAY);
        DaliTransmitCMD(PRG_SHORT_ADDR, ((ShortAddr << 1) | 1));
        delay(3*DALI_TWO_PACKET_DELAY);
        DaliTransmitCMD(WITHDRAW, 0x00);
        Serial.print("24-bit address found: 0x");
        Serial.print(SearchAddr, HEX);
        Serial.print("  Assigning short address: ");
        Serial.println(ShortAddr);
        break;
      }
    }
    else
    {
      Serial.println("No devices detected");
      break;
    }

    while(1)
    {
      SearchAddr = (long)((LowLimit + HighLimit) / 2);

      Response = SearchAndCompare(SearchAddr);
     // delay(DALI_TWO_PACKET_DELAY);
     //    Mb.MbsRun();   ///******************************* костыль, чтобы модбас не отваливался
      if (Response>0)
      {
        if ((SearchAddr == 0) || (!SearchAndCompare(SearchAddr - 1)))
          break;
        
        HighLimit = SearchAddr;
      }
      else
        LowLimit = SearchAddr;
    }

    delay(DALI_TWO_PACKET_DELAY);
    SearchAndCompare(SearchAddr);
    delay(DALI_TWO_PACKET_DELAY);
    DaliTransmitCMD(PRG_SHORT_ADDR, ((ShortAddr << 1) | 1));
    delay(5*DALI_TWO_PACKET_DELAY);
    DaliTransmitCMD(WITHDRAW, 0x00);
    delay(DALI_TWO_PACKET_DELAY);
    
    Serial.print("24-bit address found: 0x");
    Serial.println(SearchAddr, HEX);
    Serial.print("Assigning short address ");
    Serial.println(ShortAddr);

    //Mb.MbData[19] ++;
    ShortAddr++;

   // break; //только для одного модуля
   //Mb.MbsRun();   ///******************************* костыль, чтобы модбас не отваливался

  }

  delay(DALI_TWO_PACKET_DELAY);
  DaliTransmitCMD(TERMINATE, 0x00);
  delay(DALI_TWO_PACKET_DELAY);
  Serial.println("Init complete");

  return (ShortAddr-StartShortAddr);
}


bool DALIprotocol::SearchAndCompare(long SearchAddr)
    {
    bool Response = 0;
    uint8_t HighByte = SearchAddr >> 16;
    uint8_t MiddleByte = SearchAddr >> 8;
    uint8_t LowByte = SearchAddr;
  
    Serial.print (" Step # "); Serial.print(Step_Counter); Serial.print (" 24-bit address = "); Serial.println(SearchAddr, DEC);
    Step_Counter ++;

    for(uint8_t i = 0; i < 3; i++)
    {
      DaliTransmitCMD(SEARCHADDRH, HighByte);
      delay(DALI_TWO_PACKET_DELAY);
      DaliTransmitCMD(SEARCHADDRM, MiddleByte);
      delay(DALI_TWO_PACKET_DELAY);
      DaliTransmitCMD(SEARCHADDRL, LowByte);
      delay(DALI_TWO_PACKET_DELAY);
    }
    DaliTransmitCMD(COMPARE, 0x00);
    Response=DaliReciveCMD();
    return Response;
    }
//-------------------------------------------------
