//--************** DALI[0] шлюз для LED драйверов                    *************
//--************** Старт разработки 23.09.2021г.                  *************
//************** Версия для Iboard 1.2, релиз от 04.05.2022 (w5500+Atmega328P)     *************
// -------------- Реализован сброс адреса через 10 сек подтягивания к земле контакта D2 на кнопке модуля DALI[0] 
// -------------- Реализован выбор модуля MIKROE DALIClick 
// -------------- Работает только отправка и прием данных. Управление бродкастами/адресно. 
// -------------- Реализован функционал начальной настройки коротких адресов
// -------------- Пример работы с шиной и функция отправки отсюда https://habr.com/ru/post/410105/ 
// -------------- Реализовано управление 1 каналом DALI[0] полностью. 2-3 канал только бродкасты. модулей на плате нет
// -------------- Добавлена команда (300) установления мощности при обрыве связи и при подаче питания на драйвер
// -------------- 
// библиотека MgsModbus.h    http://myarduinoprojects.com/modbus.html
// пример                    https://www.youtube.com/watch?v=kiQawntjzy0
// Modbus functions 1, 2, 3, 4, 5, 6 15 and 16 are implemented. 
// The port 502 is used as standard. 
// Управление LED драйверами DALI[0]:
// Адрес  0 - (Read) счетчик секунд работы шлюза
// Адрес  1,2,3 - (Write)мощность LED на канале 1,2,3 (бродкаст) 
//            1 - выкл, 254 - макс. (на драйверах Арлайт эффективное регулирование в диапазоне 172-254)
//            После успешной передачи команды, в адрес выставляется переданное значение уставки
//            Запись значения 300 приводит к установке текущей мощности света в качестве дефолтной (используемой при подаче питания или обрыве связи) 
// Адрес  4,5,5 - (Read) мощность LED на канале 1,2,3 (обратная связь)
// 
// *************************************************************** выбор комнаты ******************************************
//#include <SPI.h>
#include <EEPROM.h>
#include <Ethernet.h>
#include "MgsModbus.h"
#include <Arduino.h>
#include <avr/boot.h>
#include "DALI.h"
//#include "DaliReciveCMD.ino"
//#include "DaliTransmitCMD.ino"
//#include "Dali_Init.ino"


//#define ROOM1       // Клубника комната #1 (вторая очередь, справа)
//#define ROOM2     // Клубника комната #2 (первая очередь, слева)



#define Capsum     // Клубника комната #2 (первая очередь, слева)
//#define LOCALIP   // 192, 168, 1, 10
// Ethernet settings (depending on MAC and Local network)

#ifdef ROOM1
  byte mac[] = {0x90, 0xA2, 0xDA, 0x0E, 0x94, 0xB7 };  // Клубника комната #1 (вторая очередь, справа)
  IPAddress ip(10, 10, 2, 68);  // Клубника комната #1 (вторая очередь, справа)
  IPAddress gateway(10, 10, 2, 1);
#endif  
#ifdef ROOM2
  byte mac[] = {0x90, 0xA2, 0xDA, 0x0E, 0x94, 0xB6 };  // Клубника комната #2 (первая очередь, слева)
  IPAddress ip(10, 10, 2, 67);  // Клубника комната #2 (первая очередь, слева)

  IPAddress gateway(10, 10, 2, 1);
#endif
#ifdef Capsum
  byte mac[] = {0x90, 0xA2, 0xDA, 0x0E, 0x94, 0xA1 };  // Клубника комната #2 (первая очередь, слева)
  IPAddress ip(10, 10, 2, 90);  // Проект Capsum, временный адрес
  IPAddress gateway(10, 10, 2, 1);
#endif
#ifdef LOCALIP
  byte mac[] = {0x90, 0xA2, 0xDA, 0x0E, 0x94, 0xB5 };  
  IPAddress ip(192, 168, 1, 10);
  IPAddress gateway(192, 168, 1, 1);
#endif

#define ADDR_IP            0 
#define MB_ADDR_IP         155 //смещение EEPROM адреса в модбасе ip 4 адреса (10.10.2.90)  4 адреса шлюза (10.10.2.1 ) 1 байт маски(24) и 2 байта контрольной суммы 

#ifdef ROOM2
  #define InversTX    // Инверсия сигнала необходима для модуля DALI_2_CLICK Клубника комната #2 (первая очередь, слева)
#endif
//#define resetbutton




#define DALI_RX_PIN_CH1   14  // канал DALI[0] 1 А0
#define DALI_TX_PIN_CH1   15  // канал DALI[0] 1 А1
#define DALI_RX_PIN_CH2   16   // канал DALI[0] 2 А2
#define DALI_TX_PIN_CH2   17  // канал DALI[0] 2 А3
#define DALI_RX_PIN_CH3   18  // канал DALI[0] 3 А4
#define DALI_TX_PIN_CH3   19  // канал DALI[0] 3 А5

#ifdef resetbutton
#define RESET_PIN         2   // кнопка ресет адреса
#endif

//----------------------------- Уровни сигналов ---------------------------
bool TX_HIGH_LEVEL = 1; 
bool TX_LOW_LEVEL = 0;
//----------------------------- Ethernet, DALI[0] & Modbus ---------------------------
MgsModbus Mb;
DALIprotocol DALI[3];
int inByte = 0; // incoming serial byte
int current_adr;                          // текущий короткий адрес 
IPAddress subnet(255, 255, 255, 0);
//uint8_t DALIPrevVals[DALI_CHNL_COUNT] = {0, 0, 0, 0};
//uint8_t LampState[DALI_CHNL_COUNT] = {0, 0, 0, 0};

int heartbeat; // пульс датчика - кол-во секунд работы 0-32 767
unsigned long last_Update_Time  = 1;     // Переменная для хранения времени последнего считывания с датчика
int PIN_LED=14;  // светодиод "передача"
////////////////////////////////////////////////////////////



uint8_t Channel_1_Power, Channel_2_Power, Channel_3_Power;  // уставка мощности каналов
int LED_Ch1_Power [64];

unsigned long ms;
unsigned long FeedBack_Timer;             // таймер опроса драйверов

unsigned long last_command_Time  = 1;     // Переменная для хранения времени последней команды
//unsigned long last_reset_Time  = 0;     // Переменная для хранения времени последней команды
  byte bufmask[4];  
void setup()
{
  
   // pinMode(RESET_PIN, INPUT);
  // serial setup
  Serial.begin(9600);
     pinMode (PIN_LED, OUTPUT);           // светодиод "передача"
   // initialize the ethernet device
  Mb.MbData[200];     // Holding Register 40001
  Serial.println("Modbus - Dali Gateway on Dali2click shield");  
  //читаем с флеша то что там есть, или при ошибке будут использованы инициализированные данные с заголовка
#ifdef resetbutton
         if (readipfromflash()) {//если благополучно считали EEprom
                        // Serial.println("EEPROM data is OK"); 
                         ip[0]=Mb.MbData[MB_ADDR_IP+0];
                         ip[1]=Mb.MbData[MB_ADDR_IP+1];
                          ip[2]=Mb.MbData[MB_ADDR_IP+2];
                          ip[3]=Mb.MbData[MB_ADDR_IP+3];
 
                          gateway[0]=Mb.MbData[MB_ADDR_IP+4];
                          gateway[1]=Mb.MbData[MB_ADDR_IP+5];
                          gateway[2]=Mb.MbData[MB_ADDR_IP+6];
                          gateway[3]=Mb.MbData[MB_ADDR_IP+7];
                          
                          mac[5]=Mb.MbData[MB_ADDR_IP+8];
                          
                          getmaskbyte(subnet,Mb.MbData[MB_ADDR_IP+9]);
                          }   else  readipfromram();
#endif 
  Ethernet.begin(mac, ip, gateway, subnet);   // start ethernet interface
  Serial.println("Ethernet interface started"); 

  // print your local IP address:
  Serial.print("My IP address: ");
  for (byte thisByte = 0; thisByte < 4; thisByte++) {
    // print the value of each byte of the IP address:
    Serial.print(Ethernet.localIP()[thisByte], DEC);
    Serial.print("."); 
  }
 
  Serial.println();

  // Fill MbData
//  Mb.SetBit(0,false);

//-----------------DALI[0]---------------
  pinMode(DALI_TX_PIN_CH1, OUTPUT);
  pinMode(DALI_TX_PIN_CH2, OUTPUT);
  pinMode(DALI_TX_PIN_CH3, OUTPUT);
  //ЗАЖИГАЕМ КАНАЛЫ В СИГНАЛ
  digitalWrite(DALI_TX_PIN_CH1, TX_HIGH_LEVEL);
  digitalWrite(DALI_TX_PIN_CH2, TX_HIGH_LEVEL);
  digitalWrite(DALI_TX_PIN_CH3, TX_HIGH_LEVEL);
  //обязательно задаем значения пинов для обьектов DALI
  DALI[0].DALI_RX_PIN=DALI_RX_PIN_CH1;
  DALI[0].DALI_RX_PIN=DALI_TX_PIN_CH1;
}
//------------------------функция расчет контрольной суммы----------------------
unsigned int CRC16_2(unsigned char *buf,  int len)
{ unsigned int crc = 0xFFFF;
  for (int pos = 0; pos < len; pos++)
  { crc ^= (unsigned int)buf[pos];    // XOR byte into least sig. byte of crc
  for (int i = 8; i != 0; i--) {    // Loop over each bit
    if ((crc & 0x0001) != 0) {      // If the LSB is set
      crc >>= 1;                    // Shift right and XOR 0xA001
      crc ^= 0xA001;
    }
    else                            // Else LSB is not set
      crc >>= 1;                    // Just shift right
    }
  }
  return crc;
}

//-----читаем текущий ип из памяти без контрольной суммы
int readipfromram() {
    Mb.MbData[MB_ADDR_IP+0]=ip[0];
    Mb.MbData[MB_ADDR_IP+1]=ip[1];
    Mb.MbData[MB_ADDR_IP+2]=ip[2];
    Mb.MbData[MB_ADDR_IP+3]=ip[3];
    Mb.MbData[MB_ADDR_IP+4]=gateway[0];
    Mb.MbData[MB_ADDR_IP+5]=gateway[1];
    Mb.MbData[MB_ADDR_IP+6]=gateway[2];
    Mb.MbData[MB_ADDR_IP+7]=gateway[3];
    Mb.MbData[MB_ADDR_IP+8]=mac[5]; 
    Mb.MbData[MB_ADDR_IP+9]=24;
}
//запись значений по умолчанию в модбас регистры
int setipdefault() {
    Mb.MbData[MB_ADDR_IP+0]=10;
    Mb.MbData[MB_ADDR_IP+1]=10;
    Mb.MbData[MB_ADDR_IP+2]=2;
    Mb.MbData[MB_ADDR_IP+3]=90;
    Mb.MbData[MB_ADDR_IP+4]=Mb.MbData[MB_ADDR_IP+0];
    Mb.MbData[MB_ADDR_IP+5]=Mb.MbData[MB_ADDR_IP+1];
    Mb.MbData[MB_ADDR_IP+6]=Mb.MbData[MB_ADDR_IP+2];
    Mb.MbData[MB_ADDR_IP+7]=1;
    Mb.MbData[MB_ADDR_IP+8]=222; 
    Mb.MbData[MB_ADDR_IP+9]=24;
}
//-------------переводим маску ip из цифры в массив 4 байт ---------------
void getmaskbyte(byte val ,byte *mask) 
{
  for (int i=0;i<constrain(val,0,32);i++)
      {
        mask[(i>7)+(i>15)+(i>23)]|= 128>>(i%8);//двигаем шифт вправо тк биты в массиве маски начинаются с самого старшего бита
      }
}
//----------------------------чтение из флеш-----------------------
int readipfromflash() 
{ byte buf[10]; //
  for (byte i=0;i<=9;i++)
   {
     Mb.MbData[MB_ADDR_IP+i]=EEPROM.read(ADDR_IP+i); 
      buf[i]=Mb.MbData[MB_ADDR_IP+i]; //читаем 10 байт с адресом и пишем сразу в регистры модбас
    } 
   Mb.MbData[MB_ADDR_IP+10]=EEPROM.read(ADDR_IP+11); //читаем  старший байт контрольной суммы 
   Mb.MbData[MB_ADDR_IP+10]<<=8;
   Mb.MbData[MB_ADDR_IP+10]|=EEPROM.read(ADDR_IP+10); //читаем  младший байт контрольной суммы 
 Serial.print("CRC:");
 Serial.println(Mb.MbData[MB_ADDR_IP+10], HEX);
//далее считаем контрольную сумму и сравниваем с тем что прочитан из EEPROM
return CRC16_2(buf,10)==Mb.MbData[MB_ADDR_IP+10]; //вернем результат сравнения контрольной суммы 1 если все хорошо
}
//----------------------------запись в флеш-----------------------
void writeiptoflash()
{
   byte buf[10];
   for (byte i=0;i<=9;i++){ //пишем 9 байт в буфер и сразу в EEPROM
   buf[i]=Mb.MbData[MB_ADDR_IP+i] ;
   EEPROM.write(ADDR_IP+i, buf[i]);
   }
   Mb.MbData[MB_ADDR_IP+10]=CRC16_2(buf,10);//вычисляем контрольную сумму буфера
   Serial.print("write CRC:");
   Serial.print(Mb.MbData[MB_ADDR_IP+10], HEX);
   EEPROM.write(ADDR_IP+10, Mb.MbData[MB_ADDR_IP+10]&0x00FF);//пишем младший байт;
   EEPROM.write(ADDR_IP+11, Mb.MbData[MB_ADDR_IP+10]>>8);//затем старший
} 

void loop()
{

 ms = millis();
 //механизм сброса---------------
#ifdef resetbutton
if ( !digitalRead(RESET_PIN)){  //если кнопка нормальнозамкнута и 5 секунд она отключена
    delay(5000);
    if (!digitalRead(RESET_PIN)){
    setipdefault();
    randomSeed(ms);// генерим рандомный мак
     Mb.MbData[MB_ADDR_IP+8]=random(255); 
    Serial.println( Mb.MbData[MB_ADDR_IP+8],HEX); 
    writeiptoflash();
     }
}
 #endif  
  if (ms < last_Update_Time) last_Update_Time=0;       // сброс при переполнении millis()
  if (ms - last_Update_Time > 1000 || last_Update_Time == 1){
        last_Update_Time = ms;
        heartbeat = heartbeat + 1;            // время работы датчика
        if (heartbeat == 32767) heartbeat = 0;
        Mb.MbData[0] = heartbeat;
  }

if (ms < last_command_Time) last_command_Time=0;       // сброс при переполнении millis()
 
//---------------- управление через регистр  1 - мощность LED на канале 1 в % (бродкаст) 1- выкл, 100 - макс
// 1 в регистр 1 - выключить канал 1 
 if (Mb.MbData[1] == 1 && (ms - last_command_Time) > 1000){
                     digitalWrite(PIN_LED, HIGH);   // выкл
                     Serial.println("Broadcast LED channel#1 OFF"); 
                     DALI[0].DaliTransmitCMD(BROADCAST_CMD, OFF);
                     delay(DALI_TWO_PACKET_DELAY);
                     Mb.MbData[4] = 1;          // транслировать выполненную команду
                     Mb.MbData[1] = 0;          // сбросить регистр 1
                     last_command_Time = ms;
                      }
// управление мощностью канала 1 
  if (Mb.MbData[1] >= 1 && Mb.MbData[1] <= 254 && (ms - last_command_Time) > 1000){ 
                      digitalWrite(PIN_LED, HIGH);   // выкл
                      Channel_1_Power = Mb.MbData[1];    // читаем уставку мощности
                      Serial.print("Broadcast LED channel#1 Power "); Serial.println(Channel_1_Power);                
            //          delay(2*DALI_TWO_PACKET_DELAY);
                      DALI[0].DaliTransmitCMD(BROADCAST_PWR, Channel_1_Power);
              //        delay(2*DALI_TWO_PACKET_DELAY);
                      Mb.MbData[4] = Channel_1_Power;     // транслировать выполненную команду
                      Mb.MbData[1] = 0;           // сбросить регистр 1
                      last_command_Time = ms;   
                      }
// Установка мощности по умолчанию (при включении) - для этого необходимо записать значение 300 в регистр 2
 if (Mb.MbData[1] == 300 && (ms - last_command_Time) > 1000){
                     digitalWrite(PIN_LED, HIGH);   // выкл
                     Serial.println("Broadcast LED channel#1 Set POWER ON LEVEL"); 
                     DALI[0].DaliTransmitCMD(BROADCAST_CMD, STORE_DTR0);
                     delay(DALI_TWO_PACKET_DELAY);
                     DALI[0].DaliTransmitCMD(BROADCAST_CMD, STORE_DTR0);
                     delay(DALI_TWO_PACKET_DELAY);
                     DALI[0].DaliTransmitCMD(BROADCAST_CMD,POWER_ON_LEVEL);
                     delay(DALI_TWO_PACKET_DELAY);
                     DALI[0].DaliTransmitCMD(BROADCAST_CMD,POWER_ON_LEVEL);
                     delay(DALI_TWO_PACKET_DELAY);
                     DALI[0].DaliTransmitCMD(BROADCAST_CMD,POWER_FAILURE_LEVEL);
                     delay(DALI_TWO_PACKET_DELAY);
                     DALI[0].DaliTransmitCMD(BROADCAST_CMD,POWER_FAILURE_LEVEL);
                     delay(DALI_TWO_PACKET_DELAY);
                     Mb.MbData[4] = 300;          // транслировать выполненную команду
                     Mb.MbData[1] = 0;          // сбросить регистр 1
                     last_command_Time = ms;
                      }
/*
//---------------- управление через регистр  2 - мощность LED на канале 2 в % (бродкаст) 1- выкл, 100 - макс
// 1 в регистр 2 - выключить канал 2 
 if (Mb.MbData[2] == 1 && (ms - last_command_Time) > 1000){
                     digitalWrite(PIN_LED, HIGH);   // выкл
                     DALI[0].DaliTransmitCMD(BROADCAST_CMD, OFF);
                     Serial.println("Broadcast LED channel#2 OFF"); 
                     delay(DALI_TWO_PACKET_DELAY);
                     Mb.MbData[5] = 1;          // транслировать выполненную команду
                     Mb.MbData[2] = 0;          // сбросить регистр 1
                     last_command_Time = ms;
                      }
// управление мощностью канала 2 
  if (Mb.MbData[2] >= 1 && Mb.MbData[2] <= 254 && (ms - last_command_Time) > 1000){       
                      digitalWrite(PIN_LED, HIGH);   // выкл
                      Channel_2_Power = Mb.MbData[2];    // читаем уставку по температуре
                      Serial.print("Broadcast LED channel#2 Power "); 
                      Serial.print(Channel_2_Power);                
                      Mb.MbData[5] = Channel_2_Power;     // транслировать выполненную команду
                      Serial.println(Channel_2_Power); 
                      DALI[0].DaliTransmitCMD(BROADCAST_PWR, Channel_2_Power);
                      delay(DALI_TWO_PACKET_DELAY);
                      Mb.MbData[2] = 0;           // сбросить регистр 1
                      last_command_Time = ms;
                      }
// Установка мощности по умолчанию (при включении) - для этого необходимо записать значение 200 в регистр 2
 if (Mb.MbData[2] == 300 && (ms - last_command_Time) > 1000){
                     digitalWrite(PIN_LED, HIGH);   // выкл
                     DALI[0].DaliTransmitCMD(BROADCAST_CMD, STORE_DTR0);
                     Serial.println("Broadcast LED channel#2 Set POWER ON LEVEL"); 
                     delay(DALI_TWO_PACKET_DELAY);
                     DALI[0].DaliTransmitCMD(BROADCAST_CMD,POWER_ON_LEVEL);
                     delay(DALI_TWO_PACKET_DELAY);
                     DALI[0].DaliTransmitCMD(BROADCAST_CMD,POWER_FAILURE_LEVEL);
                     delay(DALI_TWO_PACKET_DELAY);
                     Mb.MbData[5] = 300;          // транслировать выполненную команду
                     Mb.MbData[2] = 0;          // сбросить регистр 1
                     last_command_Time = ms;
                      }

//---------------- управление через регистр  3 - мощность LED на канале 3 в % (бродкаст) 1- выкл, 100 - макс
// 1 в регистр 3 - выключить канал 3 
 if (Mb.MbData[3] == 1 && (ms - last_command_Time) > 1000){
                     digitalWrite(PIN_LED, HIGH);   // выкл
                     DALI[0].DaliTransmitCMD(BROADCAST_CMD, OFF);
                     Serial.println("Broadcast LED channel#3 OFF"); 
                     delay(DALI_TWO_PACKET_DELAY);
                     Mb.MbData[6] = 1;          // транслировать выполненную команду
                     Mb.MbData[3] = 0;          // сбросить регистр 3
                     last_command_Time = ms;
                      }
// управление мощностью канала 3 
  if (Mb.MbData[3] >= 1 && Mb.MbData[3] <= 254 && (ms - last_command_Time) > 1000){       
                      digitalWrite(PIN_LED, HIGH);   // выкл
                      Channel_3_Power = Mb.MbData[3];    // читаем уставку по температуре
                      Serial.print("Broadcast LED channel#3 Power "); 
                      Serial.print(Channel_3_Power);                
                      Mb.MbData[6] = Channel_3_Power;     // транслировать выполненную команду
                      Serial.println(Channel_3_Power); 
                      DALI[0].DaliTransmitCMD(BROADCAST_PWR, Channel_3_Power);
                      delay(DALI_TWO_PACKET_DELAY);
                      Mb.MbData[3] = 0;           // сбросить регистр 3
                      last_command_Time = ms;
                      }
// Установка мощности по умолчанию (при включении) - для этого необходимо записать значение 300 в регистр 2
 if (Mb.MbData[3] == 300 && (ms - last_command_Time) > 1000){
                     digitalWrite(PIN_LED, HIGH);   // выкл
                     DALI[0].DaliTransmitCMD(BROADCAST_CMD, STORE_DTR0);
                     Serial.println("Broadcast LED channel#3 Set POWER ON LEVEL"); 
                     delay(DALI_TWO_PACKET_DELAY);
                     DALI[0].DaliTransmitCMD(BROADCAST_CMD,POWER_ON_LEVEL);
                     delay(DALI_TWO_PACKET_DELAY);
                     DALI[0].DaliTransmitCMD(BROADCAST_CMD,POWER_FAILURE_LEVEL);
                     delay(DALI_TWO_PACKET_DELAY);
                     Mb.MbData[6] = 300;          // транслировать выполненную команду
                     Mb.MbData[3] = 0;          // сбросить регистр 3
                     last_command_Time = ms;
                      }      

  */                               
//******************************************** Передача в шину ************************************************************-***************************
// Передача прямой команды в DALI[0] канал #1 * Передача команды в регистре 13 по адресу 12. Кол-во повторов в регистре 14. Старт выполнения по записи 1 в регистр 11. Ответ читаем из регистра 15.
 if (Mb.MbData[11] == 1 && (ms - last_command_Time) > 1000){
    int i=1;
    do {
      Serial.print  ("Address "); Serial.print  (Mb.MbData[12]); Serial.print  ("; Command "); Serial.print  (Mb.MbData[13]); Serial.print  ("; Repeat "); Serial.println  (i); 
      Mb.MbData[15] = DALI[0].DaliTransmitCMD(Mb.MbData[12], Mb.MbData[13]);
      delay(DALI_TWO_PACKET_DELAY);
      i++;
    } while (Mb.MbData[14] >= i);
   Mb.MbData[11] = 0;          // сбросить регистр 11
   last_command_Time = ms;
   }

//******************************************** ip адрес задача ************************************************************-***************************
//
if   (Mb.MbData[MB_ADDR_IP-1] == 1 && (ms - last_command_Time) > 1000){
writeiptoflash();
  Mb.MbData[MB_ADDR_IP-1] = 0;

}
  
//**************************Инициализация LED драйверов (назначение коротких адресов******************************************************************************************
 if (Mb.MbData[17] == 1 && (ms - last_command_Time) > 1000){
  Mb.MbData[19] = 0;
  DALI[0].DaliInit(Mb.MbData[18]);         // запускаем процесс поиска LED-драйверов
  Mb.MbData[17] = 0;                  // сбросить регистр 17
  Mb.MbsRun();                        //после долгого поиска может слететь модбас запустим заново
  last_command_Time = ms;
 }

//************************** установка мощности отдельно по LED драйверам 0-63 пакетом регистров 20-83 *****************************
for (int i=0; i<64; i++){
  if (Mb.MbData[20+i] > 0 && Mb.MbData[20+i] < 255){
    int adr = i << 1;
    int Pwr;
    Pwr = Mb.MbData[20+i];
    if (Pwr == 1) Pwr = 0;
    Serial.print ("LED # "); Serial.print (i); Serial.print (" Send power = "); Serial.println (Mb.MbData[20+i]);
    DALI[0].DaliTransmitCMD(adr, Pwr);
    delay(DALI_TWO_PACKET_DELAY);
    Mb.MbData[20+i] = 0;
  }
}
//************************** 

 if ((ms - last_command_Time) > 2000) digitalWrite(PIN_LED, LOW);  // выкл
                      

// ******** циклический опрос состояний драйверов раз в 50 мс по одному 
 if (millis() - FeedBack_Timer > 50){
      int adr = current_adr << 1;                                     // сдвигаем адрес на 1 бит (формат адреса в DALI[0] для отправки команды)
      DALI[0].DaliTransmitCMD(adr+1, 160);  // чтение текущей мощности с драйвера   
      LED_Ch1_Power [current_adr] = DALI[0].DaliReciveCMD();   // 
      Mb.MbData[90+current_adr] = LED_Ch1_Power [current_adr];
      Serial.print ("  LED # "); Serial.print (current_adr); Serial.print ("  Power = "); Serial.println (LED_Ch1_Power [current_adr]);
      current_adr ++;
      if (current_adr > 63) current_adr = 0;
      FeedBack_Timer = millis();
      delay(DALI_TWO_PACKET_DELAY);
     }

//  Mb.MbmRun();
  Mb.MbsRun();
  
}
