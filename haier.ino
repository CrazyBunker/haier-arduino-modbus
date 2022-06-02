#include <EEPROM.h>
#include <SoftwareSerial.h>
#include <ModbusRtu.h>

#define MODE_OFFSET                 14
#define MODE_MSK                    0xF0
#define MODE_FAN                    0xC0

#define FAN_OFFSET                  14 
#define FAN_MSK                     0x0F
#define FAN_AUTO                    0x05
#define FAN_LOW                     0x03

#define LIGHT_OFFSET                16
#define TARGET_LIGHT_OFFSET         16
#define LIGHT_ON                    0x02
#define LIGHT_OFF                   0x00

#define HORIZONTAL_SWING_OFFSET     19
#define HORIZONTAL_SWING_AUTO       0x07
#define VERTICAL_SWING_OFFSET       13
#define VERTICAL_SWING_AUTO         0x0C

#define TEMPERATURE_OFFSET          12
#define TARGET_TEMPERATURE_OFFSET   22


#define COMMAND_OFFSET              2     
#define RESPONSE_POLL               0x2A

// Control commands
#define CTR_POWER_OFFSET            17
#define CTR_POWER_ON                0x01
#define CTR_POWER_OFF               0x00
  
#define POLY                        0xa001
#define CRC_OFFSET(message)         (2 + message[2])

// temperatures supported by AC system
#define MIN_SET_TEMPERATURE         16
#define MAX_SET_TEMPERATURE         30

#define SRate                       9600
#define ID                          3
#define RX                          11
#define TX                          10
#define TXEN                        4

/*
#define MODE_AUTO     0x00
#define MODE_DRY      0x40
#define MODE_COOL     0x20
#define MODE_HEAT     0x80
byte MODE[5] = {MODE_AUTO, MODE_COOL, MODE_HEAT, MODE_DRY, MODE_FAN};
#define FAN_MID         0x02
#define FAN_HIGH        0x01
#define FAN_AUTO        0x05

byte FAN[4] = {FAN_AUTO, FAN_HIGH, FAN_MID, FAN_LOW};
#define HORIZONTAL_SWING_CENTER     0x00
#define HORIZONTAL_SWING_MAX_LEFT     0x03
#define HORIZONTAL_SWING_LEFT       0x04
#define HORIZONTAL_SWING_RIGHT      0x05
#define HORIZONTAL_SWING_MAX_RIGHT    0x06
#define HORIZONTAL_SWING_AUTO       0x07

byte HORIZONTAL_SWING[6] = {HORIZONTAL_SWING_AUTO, HORIZONTAL_SWING_CENTER, HORIZONTAL_SWING_MAX_LEFT, HORIZONTAL_SWING_LEFT, HORIZONTAL_SWING_MAX_RIGHT, HORIZONTAL_SWING_RIGHT};
  
#define VERTICAL_SWING_HEALTH_UP      0x01
#define VERTICAL_SWING_MAX_UP     0x02
#define VERTICAL_SWING_HEALTH_DOWN    0x03
#define VERTICAL_SWING_UP       0x04
#define VERTICAL_SWING_CENTER       0x06
#define VERTICAL_SWING_DOWN       0x08
#define VERTICAL_SWING_MAX_DOWN       0x10
  

byte VERTICAL_SWING[7] = {VERTICAL_SWING_AUTO, VERTICAL_SWING_MAX_UP, VERTICAL_SWING_UP, VERTICAL_SWING_CENTER, VERTICAL_SWING_DOWN, VERTICAL_SWING_HEALTH_UP, VERTICAL_SWING_HEALTH_DOWN};

#define STATUS_DATA_OFFSET      22 // Purify/Quiet mode/OnOff/...
#define POWER_BIT       (0) 
#define PURIFY_BIT        (1) 
#define QUIET_BIT       (3) 
#define AUTO_FAN_MAX_BIT    (4)
  
#define SET_POINT_OFFSET    12  

// Another byte
#define SWING_OFFSET           27
    #define SWING_OFF           0
    #define SWING_VERTICAL      1
    #define SWING_HORIZONTAL    2
    #define SWING_BOTH

byte SWING[4] = {SWING_OFF, SWING_VERTICAL, SWING_HORIZONTAL, SWING_BOTH};

  #define LOCK            28
  #define LOCK_ON         80
  #define LOCK_OFF        00

// Updated read offset

#define FRESH             31
  #define FRESH_ON        1
  #define FRESH_OFF       0

// Updated read offset

*/



#define size_register 20
uint16_t au16data[size_register];   //Modbus holding register
unsigned long tempus;
byte register_comand[25] = {0xFF,0xFF,0x14,0x40,0x00,0x00,0x00,0x00,0x00,0x01,0x60,0x01,0x0B,0x0C,0x03,0x00,0x02,0x01,0x00,0x07,0x00,0x00,0xDA,0x36,0x49};
byte status_register[15] =  {0xFF,0xFF,0x0A,0x40,0x00,0x00,0x00,0x00,0x00,0x01,0x4D,0x01,0x99,0xB3,0xB4};
byte poweroff[17] = {0xFF,0xFF,0x0C,0x40,0x00,0x00,0x00,0x00,0x00,0x01,0x5D,0x01,0x00,0x00,0xAB,0x7D,0x3A};
byte wifi[17] =     {0xFF,0xFF,0x0C,0x40,0x00,0x00,0x00,0x00,0x00,0xF7,0x00,0x00,0x00,0x3A,0x7D,0x17,0x30};
int8_t state = 0;
byte temp;
bool blink = HIGH;
bool flag_status = false;

SoftwareSerial mySerial(RX, TX);
Modbus slave(ID, Serial, TXEN);

byte getChecksum(const byte * message, size_t size) {
    byte position = CRC_OFFSET(message);
    byte crc = 0;
    if (size < ( position)) {
      return 0;
    }
    for (int i = 2; i < position; i++)
        crc += message[i];
    return crc;
    
}


unsigned crc16(unsigned crc, unsigned char *buf, size_t len)
{
    while (len--) {
        crc ^= *buf++;
        crc = crc & 1 ? (crc >> 1) ^ POLY : crc >> 1;
        crc = crc & 1 ? (crc >> 1) ^ POLY : crc >> 1;
        crc = crc & 1 ? (crc >> 1) ^ POLY : crc >> 1;
        crc = crc & 1 ? (crc >> 1) ^ POLY : crc >> 1;
        crc = crc & 1 ? (crc >> 1) ^ POLY : crc >> 1;
        crc = crc & 1 ? (crc >> 1) ^ POLY : crc >> 1;
        crc = crc & 1 ? (crc >> 1) ^ POLY : crc >> 1;
        crc = crc & 1 ? (crc >> 1) ^ POLY : crc >> 1;
    }
    return crc;
}

void sendData(byte * message, byte size) {
    byte crc_offset = CRC_OFFSET(message);
    byte crc = getChecksum(message, size);
    word crc_16 = crc16(0, &(message[2]), crc_offset-2);
    
    // Updates the crc
    message[crc_offset] = crc;
    message[crc_offset+1] = (crc_16>>8)&0xFF;
    message[crc_offset+2] = crc_16&0xFF;

    mySerial.write(message, size);
}




void setup() {

  mySerial.begin(SRate);
  
  byte initialization_1[13] = {0xFF,0xFF,0x0A,0x0,0x0,0x0,0x0,0x0,0x00,0x61,0x00,0x07,0x72};
  byte initialization_2[13] = {0xFF,0xFF,0x08,0x40,0x0,0x0,0x0,0x0,0x0,0x70,0xB8,0x86,0x41};
  
  Serial.begin(SRate);
  pinMode(TXEN, OUTPUT);
    delay(1000);
    mySerial.write(initialization_1, sizeof(initialization_1));
    delay(1000);
    mySerial.write(initialization_2, sizeof(initialization_2));
    delay(1000);
    mySerial.write(wifi, sizeof(wifi));
    delay(1000);
    mySerial.write(status_register, sizeof(status_register));

    noInterrupts();                       // отключаем все прерывания
    TCCR1A = 0;
    TCCR1B = 0;
    TCCR1B |= (1 << CS10)|(1 << CS12);    // 1024 prescaler (коэффициент деления предделителя)
    TIMSK1 |= (1 << TOIE1);               // enable timer overflow interrupt ISR (разрешаем вызов процедуры обработки прерывания переполнения счетчика)
    interrupts();    
    

}

ISR(TIMER1_OVF_vect)                    // процедура обработки прерывания переполнения счетчика
{
    flag_status = true;
}

void loop() {  
  state = slave.poll( au16data, size_register );
  if (state > 4) {
    tempus = millis() + 50;
    digitalWrite(13, HIGH);
  }
  byte data[47];
  if (state == 8) {
     register_comand[MODE_OFFSET] &= ~MODE_MSK;
     register_comand[MODE_OFFSET] |= au16data[0];
     
     if (au16data[0] == MODE_FAN && au16data[1] == FAN_AUTO ) au16data[1] = FAN_LOW;
     register_comand[FAN_OFFSET] &= ~FAN_MSK;
     register_comand[FAN_OFFSET]  |= au16data[1];
     
     register_comand[HORIZONTAL_SWING_OFFSET] = au16data[2];
     
     register_comand[VERTICAL_SWING_OFFSET] = au16data[3];
     
     temp = au16data[4]- 16;
     if ( au16data[4] < MIN_SET_TEMPERATURE ) temp = 0x00;
     else if ( au16data[4] > MAX_SET_TEMPERATURE ) temp = 0x0E;
     register_comand[TEMPERATURE_OFFSET] = temp;
     
     if ( au16data[7] == 0) {
       register_comand[HORIZONTAL_SWING_OFFSET] = 0x00;
       register_comand[VERTICAL_SWING_OFFSET] = 0x00;
     }
     else if ( au16data[7] == 1 ) {
       register_comand[HORIZONTAL_SWING_OFFSET] = 0x00;
       register_comand[VERTICAL_SWING_OFFSET] = VERTICAL_SWING_AUTO;
     }
     else if ( au16data[7] == 2 ) {
       register_comand[HORIZONTAL_SWING_OFFSET] = HORIZONTAL_SWING_AUTO;
       register_comand[VERTICAL_SWING_OFFSET] = 0x00;
     } 
     else if ( au16data[7] == 3 ) {
       register_comand[HORIZONTAL_SWING_OFFSET] = HORIZONTAL_SWING_AUTO;
       register_comand[VERTICAL_SWING_OFFSET] = VERTICAL_SWING_AUTO;
     }
     if ( au16data[8] == 1 )      register_comand[LIGHT_OFFSET] = LIGHT_ON;
     else if ( au16data[8] == 0 ) register_comand[LIGHT_OFFSET] = LIGHT_OFF;
 
     if ( au16data[0] ==  0xF0) {     
       mySerial.write(poweroff, sizeof(poweroff));
     }
     else 
     {
       sendData(register_comand,sizeof(register_comand));
     }
  }
  if (millis() > tempus) digitalWrite(13, LOW );
  if (mySerial.available() > 0) {

    mySerial.readBytesUntil(0xFF,data, sizeof(data)-2);
    if(data[0] == 0xFF && data[1] == 0xFF ){
      // If is a status response
      if (data[COMMAND_OFFSET] == RESPONSE_POLL) {
        if (data[CTR_POWER_OFFSET] == 0x01) au16data[0] = data[MODE_OFFSET] & ~FAN_MSK;
        else au16data[0] = 0xF0;
        au16data[1] = data[FAN_OFFSET] & ~MODE_MSK;
        au16data[2] = data[HORIZONTAL_SWING_OFFSET];
        au16data[3] = data[VERTICAL_SWING_OFFSET];
        au16data[4] = data[TEMPERATURE_OFFSET]+16;
        au16data[5] = data[TARGET_TEMPERATURE_OFFSET]/2;
        au16data[6] = data[CTR_POWER_OFFSET];
        au16data[8] = data[TARGET_LIGHT_OFFSET]/2;
            
       }    
    }
  }
  if (flag_status) {
    mySerial.write(status_register, sizeof(status_register));
    flag_status = false;
  }
}
