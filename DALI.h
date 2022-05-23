#include "Arduino.h"
#include "MgsModbus.h"



// Инверсия сигнала необходима для модуля DALI_2_CLICK Клубника комната #2 (первая очередь, слева)

// ----------------------------- DALI ------------------------


#define BROADCAST_CMD       0b11111111  // адрес бродкаста команды
#define BROADCAST_PWR       0b11111110  // адрес бродкаста установки мощности 0-254

#define OFF                 0b00000000   // выключить
#define DOWN                0b00000010
#define UP                  0b00000001
#define ON                  0b00001000
#define ON_254              0b11111110   // максимальная мощность

#define STORE_DTR0          0b00100001   // сохранить текущую мощность в регистр DTR0
#define POWER_ON_LEVEL      0b00101101   // установить мощность DTR0 как дефолт
#define POWER_FAILURE_LEVEL 0b00101100   // установить мощность DTR0 как мощность при ошибке связи
//#define LINEAR              0b11111110   // установить линейную шкалу мощности

#define RESET               0b00100000    // константы для процедуры инициализации новых БП
#define INITIALISE          0xA5
#define RANDOMISE           0xA7
#define SEARCHADDRH         0xB1
#define SEARCHADDRM         0xB3
#define SEARCHADDRL         0xB5
#define PRG_SHORT_ADDR      0xB7
#define COMPARE             0xA9
#define WITHDRAW            0xAB
#define TERMINATE           0xA1


//uint8_t ShortAddr = 1;             // Адрес, с которого начинается присвоение DALI адресов

#define DALI_CHNL_COUNT     4
#define LAMP_OFF_VALUE      0

#define DALI_HALF_BIT_TIME      416 //microseconds
#define DALI_TWO_PACKET_DELAY   10 //miliseconds




#ifndef DALI_h
#define DALI_h


class DALIprotocol
{
public:
bool INV_READ =           0; //инверсия уровня для чтения
bool INV_WRITE =          0; //инверсия уровня для записи
uint8_t DALI_RX_PIN;
uint8_t DALI_TX_PIN;
MgsModbus *mb;  //указатель для перезапуска модбас во время поиска
int LedsFound = 0;                        //найденые светильники
  // general
  byte DaliReciveCMD();
  byte DaliTransmitCMD(uint8_t Part1, uint8_t Part2);
  byte DaliInit(word FirstAddr,word inc);
private:
    bool SearchAndCompare(long SearchAddr);
    bool TX_HIGH_LEVEL = 1; 
    bool TX_LOW_LEVEL = 0;
    int Step_Counter;                         // счетчик шагов при инициализации драйверов
 

};

#endif
