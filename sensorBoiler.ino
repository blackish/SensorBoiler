//#include <LiquidCrystal.h>

//includes for RF
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"

//includes for Thermal Sensor
#include <DallasTemperature.h>
#include <OneWire.h>

//Includes for config
#include <EEPROM.h>


/*LCD defines*/
#define EBit       0b00001000
#define A0Bit      0b00000100
#define CLEAR_LCD  0b00100000
#define CURSOR_OFF 0b00001100
#define CURSOR_ON  0b00101100
#define LCD_LED_B  0b00000001

#define LCD_WIDTH 20
#define LCD_HEIGHT 4
#define LCDBUF 150

#define LCDBUF_0 20
#define LCDBUF_1 24
#define LCDBUF_2 150
#define LCDBUF_3 20

/*Hardware pins*/
//LCD pins
#define DS 10
#define SH_CP 8
#define ST_CP 9
//RF hardware pins
#define RF_CE 2
#define RF_CS 1
//Thermal pin
#define W1_TEMP 7
//speaker
#define SPEAKER 6
//relay
#define RELAY 3
//hardware buttons
#define LEFT_BUTTON 5
#define RIGHT_BUTTON 4
/*end of pins*/

/*Timers*/
//Maximum radio silence timeout
#define PING_TIME 6000
//Period for temperature read
#define THERM_TIME 60000
//LCD scroll period
#define SCROLL_TIME 200
//Menu press threshold
#define BUTTON_THR 100
/*end of timers*/

/*RF defines*/
//RF logic address
#define ADDR 0x0202
//MAX useful Payload
#define MAXPAYLOAD 29
//MAX send retries
#define MAXTRIES 5
//MAX buffer size
#define MAXBUF LCDBUF+2
//Packet timeout (millis)
#define PACKET_TIMEOUT 2205
//max packet size
#define MAXPACKET 32
//Capabilities
/*
1 - Thermap sensor, 2 - LCD display, 4 - Alarm,8 - Relay,16 - Speaker,5,6,7,8,9,10,11,12,13,14,15
*/
#define CAPS 0b00011011
/*end of RD defines*/


/*Object initialization*/
// Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins 9 & 10 
RF24 radio(RF_CE,RF_CS);
//Thermal object
OneWire w1 ( W1_TEMP );
DallasTemperature sensors ( &w1 );
unsigned long mills = 0;
/*end of object initialization*/


/*LCD globals*/
//char LCDStr[ 4 ][ LCDBUF ];
char LCDStr_0 [ LCDBUF_0 ];
char LCDStr_1 [ LCDBUF_1 ];
char LCDStr_2 [ LCDBUF_2 ];
char LCDStr_3 [ LCDBUF_3 ];

int LCDStrSize [ 4 ];
int LCDStrPos [ 4 ];
int LCD_LED;
int LCD_Cursor_Y;
int LCD_Cursor_X;
boolean LCDBusy;


//Thermal global
int tempC;
/*RF globals*/
//address[0] always towards master
const uint64_t addresses[2] = { 0x3102020101, 0x3102010302 };
//send address of master
const uint64_t sendAddress = 0x3101010202;
//Are we connected to master?
boolean isRFConnected;
//Receive buffer
char receiveBuffer [ MAXBUF ];
//Received len
int receiveLen;
//Received CMD
unsigned int cmd;
//global buffer
char globalBuff [ MAXPACKET ];

//Relay status
int relayStatus = LOW;
byte relayLoTemp = 20;
byte relayHiTemp = 21;
byte relayReceived = 255;
byte fixTemp = 255;

/*Menu status and buttons*/
boolean rightButtonPressed = false;
boolean leftButtonPressed = false;
boolean doubleButtonPressed = false;
boolean leftButtonRead = 1;
boolean rightButtonRead = 1;
int menuActive = 0;

//Alarm status
boolean alarmActive = false;

/*Timers*/
//RF timer
unsigned long lastSend;
//Thermal timer
unsigned long tempTimer;
//LCD scroll timer
unsigned long lastScroll;
//Packet timer
unsigned long packetTimer;
//Button timers
unsigned long buttonTimer;
//Cook timer
unsigned long cookTimerMillis = 0;
int cookTimer = 0;
unsigned long alarmDisplayTimer = 0;
/*end timers*/


void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);
  printf_begin();

  // Setup and configure rf radio
  radio.begin();                          // Start up the radio
  radio.setAutoAck(1);                    // Ensure autoACK is enabled
  radio.setRetries(15,15);                // Max delay between retries & number of retries
  radio.setCRCLength( RF24_CRC_16 ) ;
  radio.enableDynamicPayloads();
  radio.setPALevel( RF24_PA_MAX ) ;
  radio.setChannel ( 0x4c );
  radio.setDataRate ( RF24_1MBPS );
  radio.maskIRQ ( 1, 1, 0 );
  radio.openReadingPipe(1,addresses[0]);
  radio.openWritingPipe ( sendAddress );
  radio.startListening();                 // Start listening
//  attachInterrupt ( 2, radioProceed, FALLING );

//Setup thermal sensor
  sensors.begin ();

//Init timers
  lastSend = 0;
  tempTimer = 0;
  lastScroll = 0;
  buttonTimer = 0;
  
//init RF globals
  isRFConnected = false;
  receiveLen = 0;
  
//init thermal global
  tempC = 255;

//init LCD globals  
  pinMode ( DS, OUTPUT );
  pinMode ( SH_CP, OUTPUT );
  pinMode ( ST_CP, OUTPUT );
  pinMode ( RELAY, OUTPUT );
  pinMode ( LEFT_BUTTON, INPUT_PULLUP );
  pinMode ( RIGHT_BUTTON, INPUT_PULLUP );
//  pinMode ( LED, OUTPUT );
//  digitalWrite ( LED, HIGH );
  LCD_Cursor_Y = 0;
  LCD_Cursor_X = 0;
  /*LCDStr_0[ 0 ] = 'A';
  LCDStr_1[ 0 ] = 'B';
  LCDStr_2[ 0 ] = 'C';
  LCDStr_3[ 0 ] = 'D';*/
  LCDStrSize [ 0 ] = 0;
  LCDStrSize [ 1 ] = 0;
  LCDStrSize [ 2 ] = 0;
  LCDStrSize [ 3 ] = 0;
  LCDStrPos [ 0 ] = -1;
  LCDStrPos [ 1 ] = -1;
  LCDStrPos [ 2 ] = -1;
  LCDStrPos [ 3 ] = -1;
  LCDBusy = false;

  LCD_LED = 1;
  LCDBusy = false;

/*LCD init*/
  delayMicroseconds ( 20 );
  digitalWrite ( ST_CP, LOW );
  shiftOut ( DS, SH_CP, MSBFIRST, 0b00111000 );
  digitalWrite ( ST_CP, HIGH );
  digitalWrite ( ST_CP, LOW );
  delayMicroseconds ( 1 );
  shiftOut ( DS, SH_CP, MSBFIRST, 0b00110000 );
  digitalWrite ( ST_CP, HIGH );
  digitalWrite ( ST_CP, LOW );
  delayMicroseconds ( 50 );
  shiftOut ( DS, SH_CP, MSBFIRST, 0b00111000 );
  digitalWrite ( ST_CP, HIGH );
  digitalWrite ( ST_CP, LOW );
  delayMicroseconds ( 1 );
  shiftOut ( DS, SH_CP, MSBFIRST, 0b00110000 );
  digitalWrite ( ST_CP, HIGH );
  digitalWrite ( ST_CP, LOW );
  delayMicroseconds ( 50 );
  shiftOut ( DS, SH_CP, MSBFIRST, 0b00111000 );
  digitalWrite ( ST_CP, HIGH );
  digitalWrite ( ST_CP, LOW );
  delayMicroseconds ( 1 );
  shiftOut ( DS, SH_CP, MSBFIRST, 0b00110000 );
  digitalWrite ( ST_CP, HIGH );
  digitalWrite ( ST_CP, LOW );
  delayMicroseconds ( 50 );
  shiftOut ( DS, SH_CP, MSBFIRST, 0b00101000 );
  digitalWrite ( ST_CP, HIGH );
  digitalWrite ( ST_CP, LOW );
  delayMicroseconds ( 1 );
  shiftOut ( DS, SH_CP, MSBFIRST, 0b00100000 );
  digitalWrite ( ST_CP, HIGH );
  digitalWrite ( ST_CP, LOW );
  delayMicroseconds ( 50 );

  shiftOut ( DS, SH_CP, MSBFIRST, 0b00101000 );
  digitalWrite ( ST_CP, HIGH );
  delayMicroseconds ( 1 );
  digitalWrite ( ST_CP, LOW );
  shiftOut ( DS, SH_CP, MSBFIRST, 0b00100000 );
  digitalWrite ( ST_CP, HIGH );
  delayMicroseconds ( 1 );
  digitalWrite ( ST_CP, LOW );
  shiftOut ( DS, SH_CP, MSBFIRST, 0b10001000 );
  digitalWrite ( ST_CP, HIGH );
  delayMicroseconds ( 1 );
  digitalWrite ( ST_CP, LOW );
  shiftOut ( DS, SH_CP, MSBFIRST, 0b10000000 );
  digitalWrite ( ST_CP, HIGH );
  digitalWrite ( ST_CP, LOW );
  delayMicroseconds ( 50 );

  shiftOut ( DS, SH_CP, MSBFIRST, 0b00001000 );
  digitalWrite ( ST_CP, HIGH );
  digitalWrite ( ST_CP, LOW );
  delayMicroseconds ( 1 );
  shiftOut ( DS, SH_CP, MSBFIRST, 0 );
  digitalWrite ( ST_CP, HIGH );
  digitalWrite ( ST_CP, LOW );
  delayMicroseconds ( 1 );
  shiftOut ( DS, SH_CP, MSBFIRST, 0b10001000 );
  digitalWrite ( ST_CP, HIGH );
  digitalWrite ( ST_CP, LOW );
  delayMicroseconds ( 1 );
  shiftOut ( DS, SH_CP, MSBFIRST, 0b10000000 );
  digitalWrite ( ST_CP, HIGH );
  digitalWrite ( ST_CP, LOW );
  delayMicroseconds ( 50 );

  shiftOut ( DS, SH_CP, MSBFIRST, 0b00001000 );
  digitalWrite ( ST_CP, HIGH );
  digitalWrite ( ST_CP, LOW );
  delayMicroseconds ( 1 );
  shiftOut ( DS, SH_CP, MSBFIRST, 0 );
  digitalWrite ( ST_CP, HIGH );
  digitalWrite ( ST_CP, LOW );
  delayMicroseconds ( 1 );
  shiftOut ( DS, SH_CP, MSBFIRST, 0b00011000 );
  digitalWrite ( ST_CP, HIGH );
  digitalWrite ( ST_CP, LOW );
  delayMicroseconds ( 1 );
  shiftOut ( DS, SH_CP, MSBFIRST, 0b00010000 );
  digitalWrite ( ST_CP, HIGH );
  digitalWrite ( ST_CP, LOW );
  delayMicroseconds ( 1500 );

  shiftOut ( DS, SH_CP, MSBFIRST, 0b00001000 );
  digitalWrite ( ST_CP, HIGH );
  digitalWrite ( ST_CP, LOW );
  delayMicroseconds ( 1 );
  shiftOut ( DS, SH_CP, MSBFIRST, 0 );
  digitalWrite ( ST_CP, HIGH );
  digitalWrite ( ST_CP, LOW );
  delayMicroseconds ( 1 );
  shiftOut ( DS, SH_CP, MSBFIRST, 0b01101000 );
  digitalWrite ( ST_CP, HIGH );
  digitalWrite ( ST_CP, LOW );
  delayMicroseconds ( 1 );
  shiftOut ( DS, SH_CP, MSBFIRST, 0b01100000 );
  digitalWrite ( ST_CP, HIGH );
  digitalWrite ( ST_CP, LOW );
  delayMicroseconds ( 50 );

  shiftOut ( DS, SH_CP, MSBFIRST, 0b00001000 );
  digitalWrite ( ST_CP, HIGH );
  digitalWrite ( ST_CP, LOW );
  delayMicroseconds ( 1 );
  shiftOut ( DS, SH_CP, MSBFIRST, 0 );
  digitalWrite ( ST_CP, HIGH );
  digitalWrite ( ST_CP, LOW );
  delayMicroseconds ( 1 );
  shiftOut ( DS, SH_CP, MSBFIRST, 0b11001001 );
  digitalWrite ( ST_CP, HIGH );
  digitalWrite ( ST_CP, LOW );
  delayMicroseconds ( 1 );
  shiftOut ( DS, SH_CP, MSBFIRST, 0b11000001 );
  digitalWrite ( ST_CP, HIGH );
  digitalWrite ( ST_CP, LOW );
  delayMicroseconds ( 50 );

/*relay init*/
  relayLoTemp = EEPROM.read ( 0 );
  relayHiTemp = EEPROM.read ( 1 );
  fixTemp = EEPROM.read ( 2 );

  if ( relayLoTemp > relayHiTemp )
  {
    relayLoTemp = 20;
    relayHiTemp = 21;
  }

}

void loop() {
  
  mills = millis ();
  if ( radio.available() )
  {
    radioProceed ();
  }

  if ( mills - lastSend > PING_TIME || mills < lastSend )
  {
    sendPayload ( NULL, 0 );
    radio.startListening ();
  }

  if ( mills - tempTimer > THERM_TIME || mills < tempTimer )
  {
    sensors.requestTemperatures();
    tempC = ( sensors.getTempCByIndex(0) );
    tempTimer = mills;
    Serial.println ( tempC );
    checkRelay ();
  }
  if ( mills - lastScroll > SCROLL_TIME || mills < lastScroll )
  {
    updateLcd ();
    lastScroll = mills;
  }
  if ( mills - packetTimer > PACKET_TIMEOUT || mills < packetTimer )
  {
    receiveLen = 0;
  }

  /*Serial.println ( digitalRead ( LEFT_BUTTON ) );
  Serial.println ( digitalRead ( RIGHT_BUTTON ) );*/

  //alarm works
  if ( cookTimer > 0 )
  {
    if ( mills >= ( cookTimerMillis + cookTimer * 60000 ) && cookTimer != 0 )
    {
//      Serial.println ( "alarm" );
      alarmActive = true;
      tone ( SPEAKER, 400 );
    }
  }
  if ( alarmDisplayTimer > 0 )
  {
    if ( mills > alarmDisplayTimer && alarmDisplayTimer > 0 )
    {
      alarmDisplayTimer = 0;
      createRelayString ();
    }
  }
  
  leftButtonRead = digitalRead ( LEFT_BUTTON );
  rightButtonRead = digitalRead ( RIGHT_BUTTON );

  //buttons works
  if ( leftButtonRead == LOW && rightButtonRead == HIGH && ! leftButtonPressed )
  {
//    Serial.println ( "LEFT" );
    if ( buttonTimer == 0 )
      buttonTimer = mills;
    if ( mills - buttonTimer > BUTTON_THR )
    {
      leftPressed ();
      buttonTimer = 0;
      leftButtonPressed = true;
//      Serial.println ( cookTimer );
    }
  }
  if ( rightButtonRead == LOW && leftButtonRead == HIGH && ! rightButtonPressed )
  {
    if ( buttonTimer == 0 )
      buttonTimer = mills;
    if ( mills - buttonTimer > BUTTON_THR )
    {
      rightPressed ();
      buttonTimer = 0;
      rightButtonPressed = true;
      Serial.println ( cookTimer );
    }
  }
  if ( rightButtonRead == LOW && leftButtonRead == LOW && ! doubleButtonPressed )
  {
    buttonTimer = 0;
    doublePressed ();
    leftButtonPressed = true;
    rightButtonPressed = true;
    doubleButtonPressed = true;
  }
  if ( leftButtonPressed )
    if ( leftButtonRead == HIGH )
    {
      leftButtonPressed = false;
    }
  if ( rightButtonPressed )
    if ( rightButtonRead == HIGH )
    {
      rightButtonPressed = false;
    }
  if ( doubleButtonPressed )
    if ( rightButtonRead == HIGH && leftButtonRead == HIGH )
      doubleButtonPressed = false;
}

void sendPayload ( void* payload, int plen )
{
  lastSend = millis ();
  int segment = 0;
  byte buf[32];
  boolean success = false;
  int ctries = 0;
  radio.stopListening ();
  buf [ 2 ] = 0;
  buf [ 0 ] = ADDR & 255;
  buf [ 1 ] = ADDR >> 8;
  while ( ( ( segment + 1 ) * MAXPAYLOAD ) < plen )
  {
    memcpy ( buf + 3, payload + segment * MAXPAYLOAD, MAXPAYLOAD );
    while ( ! success && ctries < MAXTRIES )
    {
      success = radio.write ( buf, 32 );
      ctries++;
    }
    isRFConnected = true;
    if ( ! success )
    {
      Serial.println ( "Disconnected" );
      isRFConnected = false;
      return;
    }
  }
  buf [ 2 ] = 1;
  if ( payload != NULL )
    memcpy ( buf + 3, payload + segment * MAXPAYLOAD, plen - segment * MAXPAYLOAD );
  while ( ! success && ctries < MAXTRIES )
  {
    success = radio.write ( buf, 3 + plen - segment * MAXPAYLOAD );
    ctries++;
  }
  if ( ! success )
  {
    isRFConnected = false;
    return;
  }
  Serial.println ( "SENT" );
}

void radioProceed ()
{
  uint8_t flags;
  byte receivePayload[32];
  byte* payload;
  uint8_t len;
  int raddr;
   
  if ( radio.available()){                      // Did we receive a message?
      isRFConnected = true;
      while ( radio.available() )
      {
        Serial.println ( "RCV" );
        // Dump the payloads until we've gotten everything
        // Fetch the payload, and see if this was the last one.
    len = radio.getDynamicPayloadSize();
      radio.read( receivePayload, len );
          raddr = receivePayload [ 0 ] + ( receivePayload [ 1 ] << 8 );
          flags = receivePayload [ 2 ];
          if ( receiveLen == 0 )
          {
            cmd = receivePayload [ 3 ] + ( receivePayload [ 4 ] << 8 );
            payload = receivePayload + 5;
            len -= 5;
            Serial.println ( cmd );
            Serial.println ( len );
            packetTimer = mills;
          } else {
            payload = receivePayload + 3;
            len -= 3;
          }
          if ( len + receiveLen < MAXBUF && len > 0 )
          {
            memcpy ( receiveBuffer + receiveLen, payload, len );
            receiveLen += len;
          }
          if ( flags & 1 == 1 )
          {
            packetTimer = 0;
            receiveBuffer [ receiveLen ] = 0;
            proceedReceive ();
          }
      }
  }  
}

void proceedReceive ()
{
  if ( cmd == 0 )
  {
//Capabilities requested
  Serial.println ( "CAPAS requested" );
    byte buf [ 6 ];
    buf [ 0 ] = 0;
    buf [ 1 ] = 0;
    buf [ 2 ] = CAPS & 255;
    buf [ 3 ] = CAPS << 8;
    buf [ 4 ] = 4;
    buf [ 5 ] = 0;
    sendPayload ( buf, 6 );
    Serial.println ( "Capas sent" );
  }
  if ( cmd == 2 )
  {
//Got LCD update
//  int line = receiveBuffer [ 0 ];
  Serial.println ( "line received" );
    receiveBuffer [ receiveLen ] = 0;
    printLineLCD ( receiveBuffer + 1, receiveLen - 1, receiveBuffer [ 0 ] );
  }
  if ( cmd == 1 )
  {
//Thermal sensor requested
  Serial.println ( "Temp requested" );
    if ( receiveLen == 1 )
    {
      if ( fixTemp != receiveBuffer [ 0 ] )
      {
        fixTemp = receiveBuffer [ 0 ];
      
        EEPROM.write ( 2, fixTemp );
      }
    }
    radio.stopListening ();
    byte buf [ 4 ];
    memcpy ( buf, &cmd, sizeof ( int ) );
    memcpy ( buf + 2, &tempC, sizeof ( int ) );
    sendPayload ( buf, 4 );
  Serial.println ( "Temp sent" );
  }
  if ( cmd == 3 )
  {
  Serial.println ( "Relay requested" );
    relayReceived = receiveBuffer [ 0 ];

    byte buf [ 5 ];
    memcpy ( buf, &cmd, sizeof ( int ) );
    buf [ 2 ] = relayLoTemp;
    buf [ 3 ] = relayHiTemp;
    if ( relayStatus == LOW )
      buf [ 4 ] = 0;
    else
      buf [ 4 ] = 1;
    radio.stopListening ();
    sendPayload ( buf, 5 );
  Serial.println ( "Relay requested" );
  }
  receiveLen = 0;
  radio.startListening ();
}

void sendLcd ( int data, int a0 )
{
  /*Send data to LCD. Low level*/
  while ( LCDBusy )
    delayMicroseconds ( 1 );
  LCDBusy = true;
  int localData = data >> 4;
  int toSend = 0;
  toSend = data & 0b11110000;
  /*toSend += ( localData & 1 ) * 32;
  toSend += ( localData & 2 ) * 8;
  toSend += ( localData & 4 ) * 2;
  toSend += ( localData & 8 ) / 2;*/
  toSend |= EBit;
  toSend |= a0;
  toSend |= LCD_LED;
  shiftOut ( DS, SH_CP, MSBFIRST, toSend );
  digitalWrite ( ST_CP, HIGH );
  digitalWrite ( ST_CP, LOW );
  delayMicroseconds ( 1 );
  toSend ^= EBit;
  shiftOut ( DS, SH_CP, MSBFIRST, toSend );
  digitalWrite ( ST_CP, HIGH );
  digitalWrite ( ST_CP, LOW );
  delayMicroseconds ( 1 );

//  toSend = data & 0b00001111;
  localData = data & 0b00001111;
  /*toSend = 0;
  toSend += ( localData & 1 ) * 32;
  toSend += ( localData & 2 ) * 8;
  toSend += ( localData & 4 ) * 2;
  toSend += ( localData & 8 ) / 2;*/
  toSend = localData << 4;
  toSend |= EBit;
  toSend |= a0;
  toSend |= LCD_LED;
  shiftOut ( DS, SH_CP, MSBFIRST, toSend );
  digitalWrite ( ST_CP, HIGH );
  digitalWrite ( ST_CP, LOW );
  delayMicroseconds ( 1 );

  toSend ^= EBit;
  shiftOut ( DS, SH_CP, MSBFIRST, toSend );
  digitalWrite ( ST_CP, HIGH );
  digitalWrite ( ST_CP, LOW );
  LCDBusy = false;
  delayMicroseconds ( 50 );
}

void updateLcd ()
{

  /*Update strings on LCD*/
  int pos;
    if ( LCDStrPos [ 0 ] == -1 )
    {
        scrollString ( LCDStr_0, LCDStrSize [ 0 ], 0, 0 );
        LCDStrPos [ 0 ] = 0;
    }

    if ( LCDStrPos [ 1 ] == -1 || LCDStrSize [ 1 ] > LCD_WIDTH )
    {
        LCDStrPos [ 1 ]++;
        if ( LCDStrPos [ 1 ] == LCDStrSize [ 1 ] )
            LCDStrPos [ 1 ] = 0;
        scrollString ( LCDStr_1, LCDStrSize [ 1 ], LCDStrPos [ 1 ], 1 );
    }

    if ( LCDStrPos [ 2 ] == -1 || LCDStrSize [ 2 ] > LCD_WIDTH )
    {
        LCDStrPos [ 2 ]++;
        if ( LCDStrPos [ 2 ] == LCDStrSize [ 2 ] )
            LCDStrPos [ 2 ] = 0;
        scrollString ( LCDStr_2, LCDStrSize [ 2 ], LCDStrPos [ 2 ], 2 );
    }

    if ( LCDStrPos [ 3 ] == -1 )
    {
        Serial.println ( "Updating line 3" );
        scrollString ( LCDStr_3, LCDStrSize [ 3 ], 0, 3 );
        LCDStrPos [ 3 ] = 0;
    }

}
void scrollString ( char* str, byte strSize, byte strPos, int line )
{
    byte pos;
    moveCursor ( 0, line );
    /*Serial.println ( "Scrolling" );
    Serial.println ( strSize );
    Serial.println ( line );
    Serial.println ( strPos );
    Serial.println ( str );*/
    pos = 0;
    for ( byte index = strPos; pos < LCD_WIDTH && index < strSize; index++ )
    {
//        Serial.println ( "sending char" );
        sendLcd ( str [ index ], A0Bit );
        pos++;
    }
    for ( byte index = 0; pos < LCD_WIDTH; index++ )
    {
        if ( strSize <= LCD_WIDTH )
            sendLcd ( ' ', A0Bit );
        else
            sendLcd ( str [ index ], A0Bit );
        pos++;
    }
}

void moveCursor ( int x, int y )
{
  /*move cursor to position*/
  if ( x > 3 || x < 0 || y > 19 || y < 0 )
    return;
  int addr = 0;
  switch ( y )
  {
      case 0:
        addr = 0b10000000;
        break;
      case 1:
        addr = 0b11000000;
        break;
      case 2:
        addr = 0b10010100;
        break;
      case 3:
        addr = 0b11010100;
  }
  addr += x;
  sendLcd ( addr, 0 );
}

void printLineLCD ( char* str, int strSize, int l )
{
  /*update line on lcd*/
  byte s;
  char* lcdStrDest;

  if ( l > 3 || l < 0 )
    return;

  s = strSize;
  if ( strSize > LCDBUF_0 && l == 0 )
    s = LCDBUF_0;
  if ( strSize > LCDBUF_1 && l == 1 )
    s = LCDBUF_1;
  if ( strSize > LCDBUF_2 && l == 2 )
    s = LCDBUF_2;
  if ( strSize > LCDBUF_3 && l == 3 )
    s = LCDBUF_3;
  switch ( l )
  {
    case 0:
        lcdStrDest = LCDStr_0;
        break;
    case 1:
        lcdStrDest = LCDStr_1;
        break;
    case 2:
        lcdStrDest = LCDStr_2;
        break;
    default:
        lcdStrDest = LCDStr_3;
  }
  memcpy ( lcdStrDest, str, s );
  LCDStrSize [ l ] = s;
  LCDStrPos [ l ] = -1;


}

void createRelayString ()
{
   char str1 [ 3 ];
   String number;

  if ( mills < alarmDisplayTimer )
  {
    LCDStr_3[ 0 ] = 'T';
    LCDStr_3[ 1 ] = 'i';
    LCDStr_3[ 2 ] = 'm';
    LCDStr_3[ 3 ] = 'e';
    LCDStr_3[ 4 ] = 'r';
    LCDStr_3[ 5 ] = '=';
    number = String ( cookTimer, DEC );
    number.toCharArray ( str1, 3 );
//    dtostrf ( cookTimer, 2, 0, str1 );
    if ( cookTimer < 9 )
    {
      LCDStr_3[ 7 ] = str1 [ 0 ];
      LCDStr_3[ 6 ] = 48;
    } else {
      LCDStr_3[ 6 ] = str1 [ 0 ];
      LCDStr_3[ 7 ] = str1 [ 1 ];
    }
    LCDStr_3[ 8 ] = ' ';
    LCDStr_3[ 9 ] = 'm';
    LCDStr_3[ 10 ] = 'i';
    LCDStr_3[ 11 ] = 'n';
    LCDStr_3[ 12 ] = ' ';
    LCDStr_3[ 13 ] = ' ';
    LCDStr_3[ 14 ] = ' ';
    LCDStr_3[ 15 ] = ' ';
    LCDStr_3[ 16 ] = ' ';
    LCDStr_3[ 17 ] = ' ';
    LCDStr_3[ 18 ] = ' ';
    LCDStr_3[ 19 ] = ' ';
    LCDStrPos [ 3 ] = -1;
    return;
  }
  LCDStr_3[ 0 ] = 'T';
  LCDStr_3[ 1 ] = 'l';
  LCDStr_3[ 2 ] = 'o';
  if ( menuActive == 1 )
    LCDStr_3[ 3 ] = 0xc9;
  else
    LCDStr_3[ 3 ] = '=';
//  dtostrf ( relayLoTemp, 2, 0, str1 );
    number = String ( relayLoTemp, DEC );
    number.toCharArray ( str1, 3 );

  LCDStr_3[ 4 ] = str1[ 0 ];
  LCDStr_3[ 5 ] = str1[ 1 ];
  LCDStr_3[ 6 ] = 0x99;
  LCDStr_3[ 7 ] = 'C';
  if ( menuActive == 1 )
    LCDStr_3[ 8 ] = 0xc8;
  else
  LCDStr_3[ 8 ] = ' ';
  LCDStr_3[ 9 ] = 'T';
  LCDStr_3[ 10 ] = 'h';
  LCDStr_3[ 11 ] = 'i';
  if ( menuActive == 2 )
    LCDStr_3[ 12 ] = 0xc9;
  else
  LCDStr_3[ 12 ] = '=';
//  dtostrf ( relayHiTemp, 2, 0, str1 );
    number = String ( relayHiTemp, DEC );
    number.toCharArray ( str1, 3 );

  LCDStr_3[ 13 ] = str1[ 0 ];
  LCDStr_3[ 14 ] = str1[ 1 ];
  LCDStr_3[ 15 ] = 0x99;
  LCDStr_3[ 16 ] = 'C';
  if ( menuActive == 2 )
    LCDStr_3[ 17 ] = 0xc8;
  else
  LCDStr_3[ 17 ] = ' ';
  if ( relayStatus == LOW )
    LCDStr_3[ 18 ] = 0xd9;
  else
    LCDStr_3[ 18 ] = ' ';
  if ( isRFConnected )
    LCDStr_3[ 19 ] = '*';
  else
    LCDStr_3[ 19 ] = ' ';
  LCDStrSize [ 3 ] = 20;
  LCDStrPos [ 3 ] = -1;

}


void checkRelay ()
{
  if ( tempC == 255 )
    return;
  if ( isRFConnected )
  {
    if ( relayReceived == 0 )
      relayStatus = LOW;
    if ( relayReceived == 1 )
      relayStatus = HIGH;
  }
  if ( tempC - fixTemp < relayLoTemp && relayStatus == HIGH && ! isRFConnected )
  {
    relayStatus = LOW;
  };
  if ( tempC - fixTemp > relayHiTemp && relayStatus == LOW && ! isRFConnected )
  {
    relayStatus = HIGH;
  }
  digitalWrite ( RELAY, relayStatus );
  createRelayString ();
  return;
}

void rightPressed ()
{
  if ( menuActive == 0 )
  {
    if ( alarmActive )
    {
      alarmActive = false;
      noTone ( SPEAKER );
      cookTimer = 0;
      alarmDisplayTimer = 0;
    } else {
      cookTimer = cookTimer - ( mills - cookTimerMillis ) / 60000;
      cookTimerMillis = mills;
      cookTimer--;
      if ( cookTimer < 0 )
        cookTimer = 0;
      alarmDisplayTimer = mills + 2000;
    }
  }
  if ( menuActive == 1 && relayLoTemp > 0 )
  {
    relayLoTemp--;
  }
  if ( menuActive == 2 && relayHiTemp > relayLoTemp )
  {
    relayHiTemp--;
  }
  createRelayString ();
  return;
}

void leftPressed ()
{
  if ( menuActive == 0 )
  {
    if ( alarmActive )
    {
      alarmActive = false;
      noTone ( SPEAKER );
      cookTimer = 0;
      alarmDisplayTimer = 0;
    } else {
      cookTimer = cookTimer - ( mills - cookTimerMillis ) / 60000;
      cookTimerMillis = mills;
      if ( cookTimer < 0 )
        cookTimer = 0;
      cookTimer++;
      if ( cookTimer > 99 )
        cookTimer = 99;
 /*     if ( cookTimer == 1 )
        cookTimerMillis = millis ();*/
      alarmDisplayTimer = mills + 2000;
    }
  }
  if ( menuActive == 1 && relayLoTemp < relayHiTemp )
  {
    relayLoTemp++;
  }
  if ( menuActive == 2 )
  {
    relayHiTemp++;
  }
  createRelayString ();
  return;
}

void doublePressed ()
{
  if ( menuActive == 0 )
  {
    menuActive = 1;
    createRelayString ();
    return;
  };
  if ( menuActive == 1 )
  {
    menuActive = 2;
    createRelayString ();
    return;
  }
  if ( menuActive == 2 )
  {
    menuActive = 0;
    EEPROM.write ( 0, relayLoTemp );
    EEPROM.write ( 1, relayHiTemp );
  }
  createRelayString ();
  return;
}

