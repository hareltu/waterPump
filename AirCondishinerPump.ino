
/*
   This application connect to wifi,
   Take the water from tool to ground.

*/




///////////////////////////////////////include//////////////////////////////////////////
#include <ESP8266WiFi.h> //https://github.com/esp8266/Arduino
//needed for library
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h> //https://github.com/tzapu/WiFiManager

///////////////////////////---------------include-END---------------////////////////////




///////////////////////////////////////defines//////////////////////////////////////////
typedef enum
{
  INIT_MAIN_STATES = 0,
  PUMP_MAIN_STATES,
  TCP_MAIN_STATES,
  SERVER_MESSAGE_MAIN_STATES,
} MAIN_STATES;
#define SECOND_2 2000
#define SECOND_5 5000
#define PIN_D6 D6
#define PIN_D7 D7
///////////////////////////---------------defines-END---------------////////////////////





///////////////////////////////////////declare function/////////////////////////////////
bool sendServerMessageConst(const char mes[], uint8_t length);
bool sendServerMessage( char mes[], uint8_t length);
bool getServerMessage(uint8_t message[], uint8_t maxLength);
bool createTcpConnection(void);
void handleInterrupt(void);
void pumpOn(void);
void pumpOff(void);
bool pumpModuleMain(void);

/////////////////////----------------declare function-END--------///////////////////////




/////////////////////////////////declare global variable////////////////////////////////
uint8_t serverMes[100] = {0};
const byte pumpPin = PIN_D6;
const byte waterLineSensorPin = PIN_D7;//D7
const byte wemosD1MiniLedPin = LED_BUILTIN;
const char *host = "192.168.1.17";
const uint16_t port = 6666;
bool waterLineSensorFlg = false;
bool interrruptPinFlg = false;
bool highLow = false;
volatile byte interruptCounter = 0;
int numberOfInterrupts = 0;
unsigned long timeRef, ledRef = 0;
WiFiManager wifiManager;
WiFiClient client;


/////////////////////------------declare global variable-END------------////////////////




/////////////////////////////////function impemention////////////////////////////////

bool sendServerMessageConst(const char mes[], uint8_t length)
{
  bool reVal = false;
  char fullMes[100] = {0};
  if (client.available())
  {
    memcpy(fullMes, mes, length);
    client.println(fullMes);
    reVal = true;
  }
  return reVal;
}
bool sendServerMessage( char mes[], uint8_t length)
{
  bool reVal = false;
  char fullMes[100] = {0};
  if (client.available())
  {
    memcpy(fullMes, mes, sizeof(mes));
    client.println(fullMes);
    reVal = true;
  }
  return reVal;
}

bool getServerMessage(uint8_t message[], uint8_t maxLength)
{
  bool reVal = false;
  uint8_t buf[100] = {0};
  uint8_t sizeBuf = 100;
  if (client.available())
  {
    client.read(buf, sizeBuf);
    //Serial.print(buf);
    Serial.print("recieved data: ");
    Serial.print(buf[0]);
    if (buf[0] == 'E')
      reVal = true;
  }
  return reVal;
}
bool createTcpConnection(void)
{
  if (!client.connect(host, port))
  {
    Serial.println("connection failed");
    Serial.println("wait 5 sec...");
    delay(5000);
    return false;
  }
  Serial.println("connection succ");
  return true;
}
/*

   init:
   serial 115200
   interruptpin- pin D5- water line sensor
   ledpin pin D0 led pin
*/


/*
  handleInterrupt - sense the falling signal(high to low)


*/

void handleInterrupt(void)
{
  //after
  if (digitalRead(waterLineSensorPin) == LOW)
  {
    waterLineSensorFlg = true;
    /*if (interruptCounter > 10)
      {
      waterLineSensorFlg = true;
      }
      else
      {
      interruptCounter++;
      }*/
    interruptCounter++;//only for test
    timeRef = millis();
  }
  else if (digitalRead(waterLineSensorPin) == HIGH)
  {
    waterLineSensorFlg = false;
    interruptCounter = 0 ;
    timeRef = millis();
  }

}


void pumpOn(void)
{
  digitalWrite(pumpPin, HIGH);
}
void pumpOff(void)
{
  digitalWrite(pumpPin, LOW);
}

bool pumpModuleMain(void)
{
  static bool lastMode = false;
  bool reVal = true;
  if (waterLineSensorFlg)
  {
    if ( (millis() - timeRef > 250) && (digitalRead(waterLineSensorPin) == LOW))//chekc that the water line sensor in low value at least 250 ms
    {
      pumpOn();
      Serial.println("pump ON: ");
      digitalWrite(wemosD1MiniLedPin, LOW);   // turn the LED on (HIGH is the voltage level)
      timeRef = millis();
      if (lastMode != waterLineSensorFlg)
      {
        lastMode = waterLineSensorFlg;
        reVal = sendServerMessageConst("pump ON", sizeof("pump ON"));

      }
    }

  }
  else
  {
    if ( (millis() - timeRef > 250) && (digitalRead(waterLineSensorPin) == HIGH))
    {
      pumpOff();
      Serial.println("pump OFF: ");
      if (lastMode != waterLineSensorFlg)
      {
        lastMode = waterLineSensorFlg;
        reVal = sendServerMessageConst("pump OFF", sizeof("pump OFF"));
      }
      digitalWrite(wemosD1MiniLedPin, HIGH);   // turn the LED on (HIGH is the voltage level)
      timeRef = millis();
    }
  }
  return reVal;
}

#define DC_PUMP_PIN 13
#define AC_PUMP_PIN 14
#define LED_INDICAOTR_DC_PUMP_PIN 16
#define WATER_LINE_SENSOR_DC_PIN 17
#define WATER_LINE_SENSOR_AC_PIN 18
#define FILTER_TIME 2000 // 2 seconds
#define FILTER_TIME_DC_STOP (FILTER_TIME * 15)//30 seconds
const bool ON_MODE = true;
const bool OFF_MODE = false;
typedef struct {
  uint32_t dcPumpTimeOpertion;
  uint32_t acPumpTimeOpertion ;
  bool waterLineSensorDC ;
  bool waterLineSensorAC ;
  uint32_t dcPumpTimeRef ;
  uint32_t acPumpTimeRef ;
} INFORMATION_SYSTEM;

INFORMATION_SYSTEM inforamtionSystem ;

typedef enum
{
  DC_PUMP = 0,
  AC_PUMP,
} PUMP_TYPES;




void waterLineSensorDC_Interrupt(void);
void waterLineSensorAC_Interrupt(void);
void initWaterPumpControl(void);
void filterWaterLineSensor(void);
void dcPumpOn(void);
void dcPumpOff(void);
void acPumpOn(void);
void acPumpOff(void);
void dcPump(bool On_Off);
void acPump(bool On_Off);
void waterPumpControl(void);
void pumpMode(uint8_t pumpType, bool operationMode);

void initWaterPumpControl(void)
{
  inforamtionSystem.dcPumpTimeOpertion = 0;
  inforamtionSystem.acPumpTimeOpertion = 0;
  inforamtionSystem.waterLineSensorDC = false;
  inforamtionSystem.waterLineSensorAC = false;
  inforamtionSystem.dcPumpTimeRef = 0;
  inforamtionSystem.acPumpTimeRef = 0;

  pinMode(WATER_LINE_SENSOR_DC_PIN, INPUT_PULLUP);
  pinMode(WATER_LINE_SENSOR_AC_PIN, INPUT_PULLUP);
  pinMode(DC_PUMP_PIN, OUTPUT);
  pinMode(AC_PUMP_PIN, OUTPUT);
  pinMode(LED_INDICAOTR_DC_PUMP_PIN, OUTPUT);

  digitalWrite(DC_PUMP_PIN, LOW);
  digitalWrite(AC_PUMP_PIN, LOW);
  digitalWrite(LED_INDICAOTR_DC_PUMP_PIN, HIGH);//turn off(opposite mode
  attachInterrupt(digitalPinToInterrupt(WATER_LINE_SENSOR_DC_PIN), waterLineSensorDC_Interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(WATER_LINE_SENSOR_AC_PIN), waterLineSensorAC_Interrupt, CHANGE);


  digitalWrite(pumpPin, HIGH);

#ifdef DEBUG_MODE
  Serial.begin(115200);//debug
  Serial.print("contol pump main init ");
#endif




}
void waterLineSensorDC_Interrupt(void)
{
  //after
  if (digitalRead(WATER_LINE_SENSOR_DC_PIN) == LOW)
  {
    //inforamtionSystem.waterLineSensorDC = true;
    inforamtionSystem.dcPumpTimeRef = millis();
  }
  else if (digitalRead(WATER_LINE_SENSOR_DC_PIN) == HIGH)
  {
    inforamtionSystem.dcPumpTimeRef = millis();
  }
}

void waterLineSensorAC_Interrupt(void)
{
  //after
  if (digitalRead(WATER_LINE_SENSOR_AC_PIN) == LOW)
  {
    //inforamtionSystem.waterLineSensorDC = true;
    inforamtionSystem.acPumpTimeRef = millis();
  }
  else if (digitalRead(WATER_LINE_SENSOR_AC_PIN) == HIGH)
  {
    inforamtionSystem.acPumpTimeRef = millis();
  }
}


void filterWaterLineSensor(void)
{
  uint32_t tmpTime = millis();
  //dc sensor test
  if (digitalRead(WATER_LINE_SENSOR_DC_PIN) == LOW)
  {
    if ((tmpTime - inforamtionSystem.dcPumpTimeRef) >= FILTER_TIME)
    {
      inforamtionSystem.waterLineSensorDC = true;
    }
  }
  else if (digitalRead(WATER_LINE_SENSOR_DC_PIN) == HIGH)
  {
    if ((tmpTime - inforamtionSystem.dcPumpTimeRef) >= FILTER_TIME_DC_STOP)
    {
      inforamtionSystem.waterLineSensorDC = false;
    }
  }

  //AC sensor test
  if (digitalRead(WATER_LINE_SENSOR_AC_PIN) == LOW)
  {
    if ((tmpTime - inforamtionSystem.acPumpTimeRef)  >= FILTER_TIME)
    {
      inforamtionSystem.waterLineSensorAC = true;
    }
  }
  else if (digitalRead(WATER_LINE_SENSOR_AC_PIN) == HIGH)
  {
    if ((tmpTime - inforamtionSystem.acPumpTimeRef) >= FILTER_TIME)
    {
      inforamtionSystem.waterLineSensorAC = false;
    }
  }

}
void dcPumpOn(void)
{
  digitalWrite(DC_PUMP_PIN, HIGH);
  digitalWrite(LED_INDICAOTR_DC_PUMP_PIN, HIGH);
}
void dcPumpOff(void)
{
  digitalWrite(DC_PUMP_PIN, LOW);
  digitalWrite(LED_INDICAOTR_DC_PUMP_PIN, LOW);
}
void dcPump(bool On_Off)
{
  static uint32_t temporaryTime = 0;

  if (On_Off == ON_MODE)
  {
    dcPumpOn();
    if (!temporaryTime)
      temporaryTime = millis();
  }
  else
  {
    dcPumpOff();
    if (temporaryTime)
    {
      temporaryTime = millis() - temporaryTime;
      inforamtionSystem.dcPumpTimeOpertion += temporaryTime;
    }
    temporaryTime = 0;//initial vakue
  }
}
void acPumpOn(void)
{
  digitalWrite(AC_PUMP_PIN, HIGH);
}
void acPumpOff(void)
{
  digitalWrite(AC_PUMP_PIN, LOW);
}
void acPump(bool On_Off)
{
  static uint32_t temporaryTime = 0;

  if (On_Off == ON_MODE)
  {
    acPumpOn();
    if (!temporaryTime)
      temporaryTime = millis();
  }
  else
  {
    acPumpOff();
    if (temporaryTime)
    {
      temporaryTime = millis() - temporaryTime;
      inforamtionSystem.acPumpTimeOpertion += temporaryTime;
    }
    temporaryTime = 0;//initial vakue
  }
}


void pumpMode(uint8_t pumpType, bool operationMode)
{
  if (pumpType == DC_PUMP)
  {
    dcPump(operationMode);
  }
  else if (pumpType == AC_PUMP)
  {
    acPump(operationMode);
  }
}
void waterPumpControl(void)
{
  filterWaterLineSensor();//update the status of water line sensors
  if (inforamtionSystem.waterLineSensorAC)
  {
    pumpMode(DC_PUMP, ON_MODE);
    pumpMode(AC_PUMP, ON_MODE);
  }
  else
  {
    pumpMode(AC_PUMP, OFF_MODE);
    if (inforamtionSystem.waterLineSensorDC)
    {
      pumpMode(DC_PUMP, ON_MODE);
    }
    else
    {
      pumpMode(DC_PUMP, OFF_MODE);
    }
  }

  //read waterline sensor
  //ifwaterLineSensor is ON
  //  start DC_pump
  //  else
  // stop DC pump
  // stop AC pump
  //if waterLinSensorAC is ON
  //start AC ppump untill waterLineSensor DC is off
}

void setup() {

  Serial.begin(115200);
  pinMode(waterLineSensorPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(waterLineSensorPin), handleInterrupt, CHANGE);
  Serial.print("start");

  pinMode(pumpPin, OUTPUT);
  pinMode(wemosD1MiniLedPin, OUTPUT);
  digitalWrite(pumpPin, HIGH);
  Serial.print("test ");
  //wifiManager.autoConnect("AutoConnectAP");
}
void loop() {
  static MAIN_STATES mainSM = PUMP_MAIN_STATES;
  static uint32_t mainMS = 0;
  uint32_t nowMS;
  switch (mainSM)
  {
    case INIT_MAIN_STATES:
      //wifiManager.resetSettings();
      Serial.println("AutoConnectAP ");


      if (createTcpConnection())
      {
        mainSM = PUMP_MAIN_STATES;
      }

      break;
    case TCP_MAIN_STATES://network
      if (client.available())
      {
        sendServerMessageConst("harel turjeman", sizeof("harel turjeman"));
      }      else

      {
        createTcpConnection();
      }
      delay(500);
      break;
    case PUMP_MAIN_STATES: //app
      if ( pumpModuleMain() == false )
      {
        mainSM  = SERVER_MESSAGE_MAIN_STATES;//every 5 seconds go to app
      }

      break;
    case SERVER_MESSAGE_MAIN_STATES:
      mainSM  = PUMP_MAIN_STATES;
      break;
  }

  nowMS  = millis();
  if ((nowMS - mainMS > SECOND_5) && (mainSM != PUMP_MAIN_STATES))
  {
    mainSM  = PUMP_MAIN_STATES;//every 5 seconds go to app
    mainMS = nowMS;//store currnet time
  }


  /*   if (getServerMessage(serverMes,sizeof(serverMes)))
     {
       mainSM  = SERVER_MESSAGE_MAIN_STATES;
     }
  */
}
///////////////////---------------function impemention-END---------------/////////////////////
