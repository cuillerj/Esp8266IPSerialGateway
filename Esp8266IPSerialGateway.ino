/*
  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
  LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
  WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

  Written by Jean Cuiller
*/
/* This code run on ESP8266 chip
    It makes ESP8266 acting as Serial / UDP gateway
    It allows to route up to 30 bytes
    Is transfers any 2 consecutive bytes binary data
    Only those 3 consecutive bytes "0x7f7f7f & 0x7f7e7f" are forbiden
    There no need to modify the code to fit your need
    Configuration has to be done with a FTI (serial/USB) connection (3.3volts) (19200 b/s both NL&CR)
    Paramters are stored inside eeprom
    2 default services are defined
    It listens on specific udp port (udpListenPort) and can receive Udp data from any IP server
    By default it sends date to one specific IP address
    service "route" that route from a specific udp port (udpListenPort) to the serial link and from the serial link to a specific udp port (routePort)
    service "trace" that route  from the serial link to a specific udp port (tracePort) mainly used for debuging
    4 GPIO are used in output mode (3 LED 1 signal)
    2 GPIO are used in input mode (1 for set in debug mode, 1 for set in configuration mode) - take care to the 3.3v limitation !!
*/
// #define debugModeOn               // uncomment this define to debug the code

#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <EEPROM.h>

#define defaultSerialSpeed 38400   // the default speed of the gateway

/* *** Define look for string environment

*/
#include <LookForString.h>        // used to look for string inside commands and parameters
#define parametersNumber 19       // must fit with the paramList
#define commandnumber 30          // must fit with the commandList

/* *** commands list definition
    Define all the command's names
    Ended by "=" means this command will set the parameter of the seame name
*/
String commandList[commandnumber] = {"SSID1=", "PSW1=", "SSID2=", "PSW2=", "ShowWifi", "Restart", "DebugOn", "DebugOff", "ScanWifi", "SSID=0", "SSID=1", "SSID=2", "ShowEeprom", "EraseEeprom",
                                     "routePort=", "tracePort=", "IP1=", "IP2=", "IP3=", "IP4=", "stAddr=", "SSpeed=", "cnxLED=", "serialLED=", "pwrLED=", "confPin=", "debugPin=", "readyPin=", "listenPort=", ""
                                    };

/* *** paramters list definition
    Define all the parameter's names
    Parameters that can be set to value must have a name that fit with the commandList (without "=")
*/
String paramList[parametersNumber] = {"stAddr", "SSpeed", "cnxLED", "serialLED", "pwrLED", "confPin", "debugPin", "readyPin" , "SSID1", "PSW1", "SSID2", "PSW2",
                                      "routePort", "tracePort", "listenPort", "IP1", "IP2", "IP3", "IP4"
                                     };
/*
    define the exact number of characters that define the parameter's values
*/
unsigned int paramLength[parametersNumber] =  {4, 5, 2, 2, 2, 2, 2, 2, 50, 50, 50, 50, 4, 4, 4, 3, 3, 3, 3};
/*
   define the type of parameters 0x00 means String, 0x01 means number to be stored with one byte (<256), 0x02 means means number to be stored with two byte (< 65 535),  0x03 means means number to be stored with two byte (< 16 777 216)
*/
uint8_t paramType[parametersNumber] =         {0x02, 0x03, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x02, 0x02, 0x03, 0x01, 0x01, 0x01, 0x01};
/*
   define the default paramter's values that will be used before configuration
   The number of caracters must fit with the parameter length
*/
byte paramValue[parametersNumber][50] =       {"0001", "38400", "14", "12", "13", "04", "02", "05", "yourfirstssid", "yourfirstpsw", "yoursecondssid", "yoursecondpass", "1830", "1831", "8888", "192", "168", "001", "005"};
String *PparamList[parametersNumber];  // pointera array (to each paramter)
String *PcommandList[commandnumber];   // pointera array (to each command)

// *** define the look for string tools
LookForStr ParamNumber(PparamList, parametersNumber);  // define object that look for a string inside parameters list
LookForStr SerialInput(PcommandList, commandnumber);   // define object that look for a string inside commands list

/*
 * *** *** define the Eeprom management tools
*/
#include <ManageParamEeprom.h>              // include the code 
#define ramOffset 0                         // define the eeprom starting position (default 0 - can be change for other software compatibilty reason)
#define forceErase false                    // set to true in case of major issue with the eeprom tool
char keyword[] = "AzErTy";                  // must contain 6 charasters  - this keyword is used to check eeprom compatibity and can be modified
ManageParamEeprom storedParam (parametersNumber, ramOffset, keyword);

/*

*/
int configPin = 4;  // set to config mode when set to 3.3v - running mode when set to ground
int debugPin = 2; // Switch of udp trace when grounded  On when set to 3.3v
int readyPin = 5;  // relay off if wifi connection not established
int serialLED = 12;
int powerLED = 13;
int connectionLED = 14;
int stAddr = 0;
int SSpeed = defaultSerialSpeed;
int routePort = 1830;
int tracePort = 1831;
byte serverIP[4] = {0xc0, 0xa8, 0x01, 0x05};  //
int udpListenPort = 8888;
uint8_t IP1 = 0x00;
uint8_t IP2 = 0x00;
uint8_t IP3 = 0x00;
uint8_t IP4 = 0x00;
#define diagWifiConnection 0
#define diagIPAddress 1
#define diagMemory 2
#define nbUdpPort 2
#define bitDiagWifi 0
#define bitDiagIP 1
#define maxSSIDLength 50
#define maxPSWLength 50

//#define debug_PIN 14 // switch of udp trace when grounded
//#define led_PIN 15  // off if wifi connection not established
char ssid1[maxSSIDLength] = "";       // first SSID  will be used at first
char pass1[maxPSWLength] = "";      // first pass
char ssid2[maxSSIDLength] = "";        // second SSID will be used in case on connection failure with the first ssid
char pass2[maxPSWLength] = "";  // second pass

boolean connectionLEDOn = false;
boolean powerLEDOn = false;
boolean debugMode = false;
boolean serialLEDOn = false;
boolean restartCompleted = false;
boolean connectionStatus = false;
boolean serialFirstCall = true;
String ver = "IPSerialGateway";
uint8_t vers = 0x01;
uint8_t addrStation[4] = {0x00, 0x00, 0x04, 0x03}; // 2 1er octets reserves commence ensuite 1024 0x04 0x00
uint8_t typeStation[4] = {0x00, 0x00, 0x04, 0x01};
String Srequest = "                                                                                                                                                                                                                                                                                                                     ";
uint8_t addrStationLen = sizeof(addrStation);
uint8_t typeStationLen = sizeof(typeStation);
int udpPort[nbUdpPort];
String services[] = {"Route", "Trace"};
char packetBuffer[UDP_TX_PACKET_MAX_SIZE]; //buffer to hold incoming packet,
byte bufUdp[255];
uint8_t currentSSID = 0x00;         // current used SSID
char stationId[5];


// serial fields
#define sizeDataBin 255
byte dataBin[sizeDataBin];
#define maxUDPResp  100
char resp[maxUDPResp] = "";
int frameLen = 0;
int frameCount = 0;
uint8_t frameFlag = 0x00;
int udpFlag = 0;
byte bufParam[5];
byte bufSerialOut[maxUDPResp + 10];
IPAddress currentIP;
IPAddress nullIP = {0, 0, 0, 0};
//
// timers
unsigned long timeSerial = 0;
unsigned long timeMemory = 0;
unsigned long timeAffLED = 0;
unsigned long timeUdp = 0;
unsigned long timeLastSentUdp = 0;
unsigned long timeCheckWifi = 0;
unsigned long timeRestart = 0;
// internal data
uint8_t  diagByte = 0x03; // (,,,,,IP,Wifi)
//uint8_t SavDiag = Diag;
unsigned int freeMemory = 0;
int id;
WiFiUDP Udp;

void setup() {
  Serial.begin(defaultSerialSpeed);
  delay(1000);
#if defined(debugModeOn)
  Serial.println("start");
#endif
  InitLookForList();
  uint8_t paramVersion = storedParam.GetVersion();
#if defined(debugModeOn)
  Serial.print("eeprom version: ");
  Serial.println(paramVersion, HEX);
#endif
  if (paramVersion == 0xff || forceErase == true )
  {
    Serial.println("init eeprom: ");
    storedParam.Init();
    for (int i = 0; i < parametersNumber ; i++)
    {
      setInitialParam(i);
    }
  }

  LoadEerpromParamters();

  if (defaultSerialSpeed != SSpeed)
  {
#if defined(debugModeOn)
    Serial.print("restart serial at: ");
    Serial.println(SSpeed);
#endif
    delay(200);
    Serial.end();
    delay(500);
    Serial.begin(SSpeed);
    delay(1000);
  }

  udpPort[0] = routePort; //
  udpPort[1] = tracePort; //
  timeRestart = millis();

  ConnectWifi("default", "default");
  WiFi.mode(WIFI_STA);

  Udp.begin(udpListenPort);
  delay (5000);
  TraceToUdp("Ready! Use ", 0x02);
  pinMode(powerLED, OUTPUT);
  digitalWrite(powerLED, 0);
  pinMode(connectionLED, OUTPUT);
  digitalWrite(connectionLED, 0);
  pinMode(serialLED, OUTPUT);
  digitalWrite(serialLED, false);
  pinMode(readyPin, OUTPUT);
  // pinMode(led_PIN, OUTPUT);
  pinMode(configPin, INPUT);
  //  digitalWrite(led_PIN, true);
  pinMode(debugPin, INPUT_PULLUP);
}

void loop() {

  if (millis() - timeRestart >= 10000 && restartCompleted == false)
  {
    restartCompleted = true;
    if (CheckCurrentIP())                    // check ip address
    {
      if (configPin == true)
      {
        Serial.print("Ready to use ! ");
        PrintUdpConfig();
      }
      AffLed();
      digitalWrite(readyPin, true);
    }
    else
    {
      if (configPin == true)
      {
        Serial.print("Not ready to use ! ");
      }
      PrintUdpConfig();
    }
    Udp.begin(udpListenPort);
    delay (5000);
    if (configPin == true)
    {
      WiFi.printDiag(Serial);
    }
  }


  if (millis() - timeSerial >= 20)
  {
    int lenInput = Serial_have_message();
    {
      if (lenInput != 0)
      {
        String   lenS = "serial len:" + String(lenInput);
        TraceToUdp(lenS, 0x01);
        RouteToUdp(lenInput);
      }
    }
    timeSerial = millis();
  }


  if (millis() - timeMemory >= 30000)
  {
    boolean wifiStatus = false;
    if (WiFi.status() == WL_CONNECTED)
    {
      wifiStatus = 1;
    }
    if (connectionStatus == 1 && (CheckCurrentIP() && wifiStatus) == 0)
    {
      Serial.println("connection lost");
      digitalWrite(readyPin, false);
    }
    connectionStatus = CheckCurrentIP() && wifiStatus;

    freeMemory = ESP.getFreeHeap();
    String  strS = "free memory:" + String(freeMemory);
    TraceToUdp(strS, 0x01);
    if (debugMode == true)
    {
      Serial.println(strS);
    }
    timeMemory = millis();
  }
  if (millis() - timeUdp >= 100)
  {
    InputUDP();
    timeUdp;
  }
  // *** refresh LED
  if (millis() - timeAffLED > 1000)
  {
    AffLed();
    timeAffLED = millis();
  }
  // *** end of loop refresh LED
}

void ConnectWifi(char *ssid, char *pass)
{
  timeCheckWifi = millis();
  if (ssid == "default")
  {
    WiFi.begin();
  }
  else
  {
    WiFi.begin(ssid, pass);
  }
  if (digitalRead(configPin) == true)
  {
    Serial.print("\n Please wait... Connecting to "); Serial.println(ssid);
  }
  uint8_t i = 0;
  while (WiFi.status() != WL_CONNECTED && i++ < 5)
  {
    delay(2000);
    if (i == 5) {
      if (digitalRead(configPin) == true) // gateway config mode
      {
        Serial.print("Could not connect to "); Serial.println(ssid);
      }
      delay(500);
      bitWrite(diagByte, bitDiagWifi, 1);       // position bit diag
      Udp.stop();

    }
    else
    {
      bitWrite(diagByte, bitDiagWifi, 0);       // position bit diag
      WiFi.mode(WIFI_STA);
      Udp.begin(udpListenPort);
      delay (5000);
      if (digitalRead(configPin) == true) // gateway run mode
      {
        WiFi.printDiag(Serial);
      }
    }
  }
}
boolean CheckCurrentIP()
{
  currentIP = WiFi.localIP();
  if (currentIP == nullIP)
  {
    if (debugMode == true)
    {
      Serial.println("IP address issue");
    }
    bitWrite(diagByte, bitDiagIP, 1);       // position bit diag
    return false;

  }
  else
  {
    bitWrite(diagByte, bitDiagIP, 0);       // position bit diag
    return true;
  }
}

void PrintUdpConfig() {
  for (int i = 0; i < (nbUdpPort); i++)
  {
    Serial.print(services[i]);
    Serial.print(" Udp");
    Serial.print(i);
    Serial.print(" ");
    Serial.print(" port:");
    Serial.println(udpPort[i]);

  }
}

void SendToUdp( int mlen, int serviceId) {
  if ( millis() - timeLastSentUdp  > 200)
  {
    Udp.beginPacket(serverIP, udpPort[serviceId]);
    Udp.write(dataBin, mlen);
    Udp.endPacket();
  }
  else
  {
    delay (200);
    Udp.beginPacket(serverIP, udpPort[serviceId]);
    Udp.write(dataBin, mlen);
    Udp.endPacket();
  }
  timeLastSentUdp = millis();
}
void TraceToUdp(String req, uint8_t code)
{
  if ( digitalRead(debugPin) == 0)
  {
    int len = req.length() + 5;
    dataBin[0] = uint8_t(id / 256); // addrSSerial
    dataBin[1] = uint8_t(id);  // addrMSerial
    dataBin[2] = code;  // Print
    dataBin[3] = 0x00;

    if (len > sizeDataBin)
    {
      len = sizeDataBin;
    }
    dataBin[4] = uint8(len - 5); //len
    for (int i = 0; i < len - 5; i++)
    {

      dataBin[i + 5] = req[i];

    }
    SendToUdp(len, 1);
  }
}
void RouteToUdp(int len)
{
  dataBin[0] = bufParam[0]; // station address
  dataBin[1] = 0x3B;
  /// dataBin[2] = 0x64;
  dataBin[2] = bufParam[2];
  dataBin[3] = 0x3B;
  dataBin[4] = uint8(len);  //len
  dataBin[5] = 0x3B;
  if (len > sizeDataBin - 6)
  {
    len = sizeDataBin - 6;
  }
  for (int i = 0; i < len; i++)
  {
    dataBin[i + 6] =   bufUdp[i];
  }
  SendToUdp( len + 6 , 0);
}
int Serial_have_message() {
  if (Serial.available() )
  {
    digitalWrite(serialLED, 1);
    serialLEDOn = true;
    if (digitalRead(configPin) == false) // gateway run mode
    {
      while (Serial.available())
      {
        byte In1 = (Serial.read());
        switch (In1)
        {
          case 0x7f:  // looking for start of frame "0x7f-0x7e-0x7f-0x7e"
            if (frameFlag == 0)  // first 0x7f in the frame
            {
              frameFlag = 1;
              break;
            }
            if (frameFlag == 2)
            {
              frameFlag = 3;  // 0x7f after 0x7f-0x7e
              break;
            }
            if (frameFlag == 4)
            {
              frameFlag = 5;
              break;
            }
            if (frameFlag == 6)
            {
              frameFlag = 7;
              break;
            }

          //  }
          case 0x7e:
            if (frameFlag == 3)   // last 0x7e start frame completed
            {
              frameFlag = 8;
              //            TraceToUdp(" +", 0x01);
              frameLen = 0;
              frameCount = 0;
              udpFlag = 0;
              break;
            }
            if (frameFlag == 1)  // 0x7e after 0x7f
            {
              frameFlag = 2;
              break;
            }
            if (frameFlag == 5)
            {

              frameFlag = 6;
              break;
            }
            if (frameFlag == 7)
            {
              frameFlag = 8;
              //            TraceToUdp(" +", 0x01);
              frameLen = 0;
              frameCount = 0;
              udpFlag = 0;
              break;
            }

          default:
            if (frameFlag == 5 || frameFlag == 6 || frameFlag == 7 || frameFlag == 8)
            {
              frameFlag = 4;
              break;
            }
        }

        switch (frameFlag)
        {
          case 4:  // useful data
          case 5:  // useful data
          case 6:  // useful data
            frameCount = frameCount + 1;
            switch (frameCount)
            {
              case 1:
                bufParam[0] = In1;   // get the serial source address
                break;
              case 2:
                bufParam[1] = In1; // get the serial master  address - not currently used
                break;
              case 3:
                bufParam[2] = In1;  // get the request type
                break;
              case 4:              // get the ack flag
                bufParam[3] = In1;
                break;
              case 5:              // get the frame len
                frameLen = In1;
                bufParam[4] = In1;
                break;
              default:
                bufUdp[frameCount - 6] = In1; //
                if (frameCount == (frameLen + 5) && udpFlag == 0 && bufParam[2] != 0xff)
                {
                  udpFlag = 0xff;
                }
                if (bufParam[2] == 0xff)
                {
                  udpFlag = 1;
                }

                break;


            }

          default:

            break;
        }

        //end of for all chars in string
        if (udpFlag == 0xff)  // end of frame reached

        {
          udpFlag = 0x00;
          int retLen = frameLen;
          frameLen = 0;
          frameCount = 0;
          frameFlag = 0x00;
          return (retLen);

        }
        else
        {
          return (0);
        }
      }
    }
    else  // gateway configuration mode
    {
      Srequest = (Serial.readString());
      Serial.println(Srequest);
      commandReturn rc = SerialInput.GetCommand(Srequest); // look for commant inside the input
      int cmdIdx = rc.idxCommand;
      int cmdPos = rc.idxPos;
#if defined(debugModeOn)
Serial.print("idxCommand:");
Serial.println(cmdIdx);
#endif
      if (cmdIdx >= 0)
      {
        switch (cmdIdx) {
          case 0:    // SSID1=
            {
#if defined(debugModeOn)
              Serial.println("previous SSID:");
              for (int i = 0; i < maxSSIDLength; i++)
              {
                Serial.print(ssid1[i], HEX);
                Serial.print("-");

                Serial.println("");
              }
#endif
              for (int i = 0; i < maxSSIDLength; i++)
              {
                ssid1[i] = 0x00;
              }
              UpdateEepromParameter(cmdIdx, Srequest, cmdPos);
              String id = Srequest.substring(cmdPos);
              id.toCharArray(ssid1, id.length() - 1);
#if defined(debugModeOn)
              Serial.println("new SSID:");
              for (int i = 0; i < maxSSIDLength; i++)
              {
                Serial.print(ssid1[i], HEX);
                Serial.print("-");
              }
              Serial.println("");
#endif
              break;
            }
          case 1:    // PSW1=
            {
#if defined(debugModeOn)
              Serial.println("previous psw:");
              for (int i = 0; i < maxPSWLength; i++)
              {
                Serial.print(pass1[i], HEX);
                Serial.print("-");
              }
              Serial.println("");
#endif
              for (int i = 0; i < maxPSWLength; i++)
              {
                pass1[i] = 0x00;
              }
              UpdateEepromParameter(cmdIdx, Srequest, cmdPos);
              String id = Srequest.substring(cmdPos);
              id.toCharArray(pass1, id.length() - 1);
#if defined(debugModeOn)
              Serial.println("new psw:");
              for (int i = 0; i < maxPSWLength; i++)
              {
                Serial.print(pass1[i], HEX);
                Serial.print("-");
              }
              Serial.println("");
#endif
              break;
            }
          case 2:    // SSID2=
            {
              {
#if defined(debugModeOn)
                Serial.println("previous SSID:");
                for (int i = 0; i < maxSSIDLength; i++)
                {
                  Serial.print(ssid2[i], HEX);
                  Serial.print("-");
                }
                Serial.println("");
#endif
                for (int i = 0; i < maxSSIDLength; i++)
                {
                  ssid2[i] = 0x00;
                }
                UpdateEepromParameter(cmdIdx, Srequest, cmdPos);
                String id = Srequest.substring(cmdPos);
                id.toCharArray(ssid2, id.length() - 1);
#if defined(debugModeOn)
                Serial.println("new SSID:");
                for (int i = 0; i < maxSSIDLength; i++)
                {
                  Serial.print(ssid2[i], HEX);
                  Serial.print("-");
                }
                Serial.println("");
#endif
                break;
              }
            }
          case 3:     // PSW2=
            {
#if defined(debugModeOn)
              Serial.println("previous psw:");
              for (int i = 0; i < maxPSWLength; i++)
              {
                Serial.print(pass2[i], HEX);
                Serial.print("-");
              }
              Serial.println("");
#endif
              for (int i = 0; i < maxPSWLength; i++)
              {
                pass2[i] = 0x00;
              }
              UpdateEepromParameter(cmdIdx, Srequest, cmdPos);
              String id = Srequest.substring(cmdPos);
              id.toCharArray(pass2, id.length() - 1);
#if defined(debugModeOn)
              Serial.println("new psw:");
              for (int i = 0; i < maxPSWLength; i++)
              {
                Serial.print(pass2[i], HEX);
                Serial.print("-");
              }
              Serial.println("");
#endif
              break;
            }
          case 4:     // ShowWifi
            {
              WiFi.printDiag(Serial);
              Serial.println(WiFi.localIP());
              break;
            }
          case 5:     // Restart
            {
              Udp.stop();
              bitWrite(diagByte, bitDiagWifi, 0);       // position bit diagc              digitalWrite(connextionLED,false);
              WiFi.disconnect(true);
              delay(1000);
              restartCompleted = false;
              timeRestart = millis();
              InitConfig();
              break;
            }
          case 6:     // DebugOn
            {
              Serial.setDebugOutput(true);
              debugMode = true;
              break;
            }
          case 7:     // DebugOff
            {
              Serial.setDebugOutput(false);
              debugMode = false;
              break;
            }
          case 8:     // ScanWifi
            {
              ScanWifi();
              break;
            }
          case 9:    // SSID=0
            {
              currentSSID = 0x00;
              break;
            }
          case 10:    // SSID=1
            {
              currentSSID = 0x01;
              break;
            }
          case 11:    // SSID=2
            {
              currentSSID = 0x02;
              break;
            }
          case 12:    // ShowEeprom
            {
              ShowEeeprom();
              break;
            }
          case 13:    // EraseEeprom
            {
              EraseEeprom();
              break;
            }
          default:
            {
              UpdateEepromParameter(cmdIdx, Srequest, cmdPos);
              break;
            }
        }
      }
      else
      {
        Serial.println("Serial must be set NL & CR ");
        Serial.print("Supported commands are: ");
        for (int i = 0; i < commandnumber; i++)
        {
          Serial.print(commandList[i]);
          Serial.print(" ");
        }
        Serial.println("");
        delay(500);
      }
      return (0);
    }
  }
}
void InputUDP() {
  int packetSize = Udp.parsePacket();  // message from UDP

  if (packetSize)
  {
    //  TraceToUdp("udp input", 0x02);
    String lenS = "udp input" + String(packetSize);
    TraceToUdp(lenS, 0x01);
    bufSerialOut[1] = 0x7f;
    bufSerialOut[2] = 0x7e;
    bufSerialOut[3] = 0x7f;
    bufSerialOut[4] = 0x7e;

    Udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);
    //   int packetBufferLen=sizeof(packetBuffer);
    int packetBufferLen = packetSize;
    bufSerialOut[5] = packetSize;
    for (int i = 0; i < maxUDPResp; i++)
    {
      bufSerialOut[i + 6] = 0x00;
    }

    for (int i = 0; i < packetSize; i++)
    {

      bufSerialOut[i + 6] = packetBuffer[i];
      //     Serial.print(packetBuffer[i],HEX);
      //     Serial.print(":");
    }
    if (packetSize < 6)
    {
      packetSize = 6;
    }
    digitalWrite(serialLED, 1);
    serialLEDOn = true;
    Serial.write(bufSerialOut, packetSize + 6);

  }
}

void InitConfig()
{
  if (currentSSID == 0x00)
  {
    ConnectWifi("default", "default");
  }
  if (currentSSID == 0x01 )
  {
    ConnectWifi(ssid1, pass1);
  }
  if (currentSSID == 0x02 )
  {
    ConnectWifi(ssid2, pass2);
  }

  WiFi.mode(WIFI_STA);
  Udp.begin(udpListenPort);
  delay (5000);
  TraceToUdp("Ready! Use ", 0x02);
}
void ScanWifi() {
  Serial.println("scan start");

  // WiFi.scanNetworks will return the number of networks found
  int n = WiFi.scanNetworks();
  Serial.println("scan done");
  if (n == 0)
    Serial.println("no networks found");
  else
  {
    Serial.print(n);
    Serial.println(" networks found");
    for (int i = 0; i < n; ++i)
    {
      // Print SSID and RSSI for each network found
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.print(WiFi.SSID(i));
      Serial.print(" (");
      Serial.print(WiFi.RSSI(i));
      Serial.print(")");
      Serial.println((WiFi.encryptionType(i) == ENC_TYPE_NONE) ? " " : "*");
      delay(10);
    }
  }
  Serial.println("");
}
void AffLed()
{
  serialLEDOn = false;
  digitalWrite(serialLED, 0);
  if (restartCompleted == false)
  {
    //    digitalWrite(connectionLED, 0);
    //   connectionLEDOn = false;
    digitalWrite(powerLED, 1);
    digitalWrite(connectionLED, 0);
    powerLEDOn = true;
    connectionLEDOn = false;
    return;
  }
  if ((diagByte && 0b00000011) == 0b00000000)
  {
    digitalWrite(connectionLED, 1);
    connectionLEDOn = true;
  }
  if ((diagByte && 0b00000011) == 0b00000011)
  {
    digitalWrite(connectionLED, 0);
    connectionLEDOn = false;
  }
  else {
    if (( diagByte && 0b00000011) == 0b00000001 || ( diagByte && 0b00000011) == 0b00000010)
    {
      connectionLEDOn = !connectionLEDOn;
      digitalWrite(connectionLED, connectionLEDOn);
    }
  }
  if ( diagByte == 0b00000000 && digitalRead(configPin) == false)
  {
    digitalWrite(powerLED, 1);
    powerLEDOn = true;
  }
  else
  {
    powerLEDOn = !powerLEDOn;
    digitalWrite(powerLED, powerLEDOn);
  }

}
void setInitialParam(int number)
{
  int idxArray = number ;
  //Serial.print(paramList[number]);
  Serial.print(" set parameter : ");
  Serial.print(number);
  Serial.print(" size: ");
  //  uint8_t nb = paramLength[number];
  Serial.print(paramLength[idxArray]);
  Serial.println();

  if (paramType[idxArray] == 0x00)
  {
    Serial.print("0x00 rc: ");
    Serial.println(storedParam.SetParameter(number, paramLength[idxArray], paramValue[idxArray]));
  }
  if (paramType[idxArray] == 0x01)
  {
    unsigned int value = ExtractNumericValue(idxArray) ;
    uint8_t byteToStore[1];
    byteToStore[0] = uint8_t(value);
    Serial.print("0x01 rc: ");
    Serial.println(storedParam.SetParameter(number, 1, byteToStore));
  }
  if (paramType[idxArray] == 0x02 )
  {
    unsigned int value = ExtractNumericValue(idxArray) ;
    uint8_t byteToStore[2];
    byteToStore[1] = uint8_t(value) ;
    byteToStore[0] = uint8_t(value / 256) ;
    Serial.print(" 0x02 rc: ");
    Serial.println(storedParam.SetParameter(number, 2, byteToStore));
  }
  if (paramType[idxArray] == 0x03 )
  {
    unsigned int value = ExtractNumericValue(idxArray) ;
    uint8_t byteToStore[3];
    byteToStore[2] = uint8_t(value) ;
    byteToStore[1] = uint8_t(value / 256) ;
    byteToStore[0] = uint8_t(value / (256 * 256)) ;
    Serial.print(" 0x03 rc: ");
    Serial.println(storedParam.SetParameter(number, 3, byteToStore));
  }
}

unsigned int  ExtractNumericValue(uint8_t idxArray)
{
  uint8_t  byteToStore[3];

  unsigned long val = 0;
  for (int i = 0; i < paramLength[idxArray]; i++)
  {
    if (paramType[idxArray] == 0x01 || paramType[idxArray] == 0x02 || paramType[idxArray] == 0x03)
    {
      unsigned int idx = uint8_t(paramValue[idxArray][i]) - 0x30;
      unsigned int degre = paramLength[idxArray] - i - 1;
      int pwr = 1;
      for (int j = i; j < paramLength[idxArray] - 1; j++)
      {
        pwr = pwr * 10;
      }
      val = val + idx * pwr;
    }
    if (paramType[idxArray] == 0x01)
    {
      byteToStore[0] = uint8_t(val ) ;

    }
    if (paramType[idxArray] == 0x02)
    {
      byteToStore[1] = uint8_t(val) ;
      byteToStore[0] = uint8_t(val / 256) ;
    }
    if (paramType[idxArray] == 0x03)
    {
      byteToStore[2] = uint8_t(val) ;
      byteToStore[1] = uint8_t(val / 256) ;
      byteToStore[0] = uint8_t(val / (256 * 256)) ;
    }
  }
  return (val);
}

void InitLookForList()
{
  for (int i = 0; i < commandnumber ; i++)
  {
    PcommandList[i] = &commandList[i];
  }
  for (int i = 0; i < parametersNumber ; i++)
  {
    PparamList[i] = &paramList[i];
  }
  ParamNumber.InitCommandsList(PparamList, parametersNumber);
  SerialInput.InitCommandsList(PcommandList, commandnumber);
}

void ShowEeeprom()
{
  for (int i = 0; i < parametersNumber; i++)
  {
    String  Srequest = paramList[i];
    Serial.print(Srequest);
    Serial.print(": ");
    commandReturn rc = ParamNumber.GetCommand(Srequest); // look for commant inside the input
    int cmdIdx = rc.idxCommand;
    if (cmdIdx != -1)
    {
      if (paramType[i] == 0x01 || paramType[i] == 0x02 || paramType[i] == 0x03 )
      {
        numericParameter numericRC = storedParam.GetNumericParameter(cmdIdx);
        Serial.println(numericRC.parameterNumericValue);
      }
      if (paramType[i] == 0x00 )
      {
        parameter RC = storedParam.GetParameter(cmdIdx);
        Serial.print(RC.parameterValue);
      }
    }
  }
}
void EraseEeprom()
{
  Serial.println("erase eeprom please wait");
  storedParam.Erase(0);
  Serial.println("need power off / on");
}

void UpdateEepromParameter(uint8_t cmdIdx, String Srequest, int cmdPos)
{
  String  Sparam = commandList[cmdIdx];               // get command string
  Sparam = Sparam.substring(0, sizeof(Sparam) - 1); // remove last char " = "
  String  Svalue = Srequest;                       // get command value
  Svalue = Svalue.substring(cmdPos);              //
  //  int valueLen = sizeof(Svalue);
  char Cvalue[maxParameterLen + 50];
  Svalue.toCharArray(Cvalue, sizeof(Svalue));

  commandReturn rc = ParamNumber.GetCommand(Sparam); // look for parameter inside the input
  int paramIdx = rc.idxCommand;
  // int paramPos = rc.idxPos;

  if (paramIdx != -1)
  {
    int idxArray = paramIdx ;
    Serial.print(" set parameter : ");
    Serial.print(idxArray);
    Serial.print(" size: ");
    //  uint8_t nb = paramLength[number];
    Serial.print(paramLength[idxArray]);
    Serial.println();

    if (paramType[idxArray] == 0x00)
    {
      byte Bvalue[maxParameterLen + 50];
      Svalue.getBytes(Bvalue, paramLength[idxArray]);
      Serial.print("0x00 rc: ");
      Serial.println(storedParam.SetParameter(idxArray, paramLength[idxArray], Bvalue));
    }

    if (paramType[idxArray] == 0x01)
    {
      int value = atoi(Cvalue) ;
      uint8_t byteToStore[1];
      byteToStore[0] = uint8_t(value);
      Serial.print("0x01 rc: ");
      Serial.println(storedParam.SetParameter(idxArray, 1, byteToStore));
    }
    if (paramType[idxArray] == 0x02 )
    {
      int value = atoi(Cvalue) ;
#if defined(debugModeOn)
      Serial.print("atoi: ");
      Serial.println(value);
#endif
      uint8_t byteToStore[2];
      byteToStore[1] = uint8_t(value) ;
      byteToStore[0] = uint8_t(value / 256) ;
      Serial.print(" 0x02 rc: ");
      Serial.println(storedParam.SetParameter(idxArray, 2, byteToStore));
    }
    if (paramType[idxArray] == 0x03 )
    {
      int value = atoi(Cvalue) ;
#if defined(debugModeOn)
      Serial.print("atoi: ");
      Serial.println(value);
#endif
      uint8_t byteToStore[3];
      byteToStore[2] = uint8_t(value) ;
      byteToStore[1] = uint8_t(value / 256) ;
      byteToStore[0] = uint8_t(value / (256 * 256)) ;
      Serial.print(" 0x03 rc: ");
      Serial.println(storedParam.SetParameter(idxArray, 3, byteToStore));
    }
  }
}
void LoadEerpromParamters()
{
  String  Srequest = "stAddr";
#if defined(debugModeOn)
  Serial.print(Srequest);
  Serial.print(": ");
#endif
  commandReturn rc = ParamNumber.GetCommand(Srequest); // look for commant inside the input
  int cmdIdx = rc.idxCommand;
  if (cmdIdx != -1)
  {
    numericParameter numericRC = storedParam.GetNumericParameter(cmdIdx);
#if defined(debugModeOn)
    Serial.println(numericRC.parameterNumericValue);
#endif
    stAddr = numericRC.parameterNumericValue;
  }

  Srequest = "SSpeed";
#if defined(debugModeOn)
  Serial.print(Srequest);
  Serial.print(": ");
#endif
  rc = ParamNumber.GetCommand(Srequest); // look for commant inside the input
  cmdIdx = rc.idxCommand;
  if (cmdIdx != -1)
  {
    numericParameter numericRC = storedParam.GetNumericParameter(cmdIdx);
#if defined(debugModeOn)
    Serial.println(numericRC.parameterNumericValue);
#endif
    SSpeed = numericRC.parameterNumericValue;
  }

  Srequest = "cnxLED";
#if defined(debugModeOn)
  Serial.print(Srequest);
  Serial.print(": ");
#endif
  rc = ParamNumber.GetCommand(Srequest); // look for commant inside the input
  cmdIdx = rc.idxCommand;
  if (cmdIdx != -1)
  {
    numericParameter numericRC = storedParam.GetNumericParameter(cmdIdx);
#if defined(debugModeOn)
    Serial.println(numericRC.parameterNumericValue);
#endif
    connectionLED = numericRC.parameterNumericValue;
  }

  Srequest = "serialLED";
#if defined(debugModeOn)
  Serial.print(Srequest);
  Serial.print(": ");
#endif
  rc = ParamNumber.GetCommand(Srequest); // look for commant inside the input
  cmdIdx = rc.idxCommand;
  if (cmdIdx != -1)
  {
    numericParameter numericRC = storedParam.GetNumericParameter(cmdIdx);
#if defined(debugModeOn)
    Serial.println(numericRC.parameterNumericValue);
#endif
    serialLED = numericRC.parameterNumericValue;
  }


  Srequest = "pwrLED";
#if defined(debugModeOn)
  Serial.print(Srequest);
  Serial.print(": ");
#endif
  rc = ParamNumber.GetCommand(Srequest); // look for commant inside the input
  cmdIdx = rc.idxCommand;
  if (cmdIdx != -1)
  {
    numericParameter numericRC = storedParam.GetNumericParameter(cmdIdx);
#if defined(debugModeOn)
    Serial.println(numericRC.parameterNumericValue);
#endif
    powerLED = numericRC.parameterNumericValue;
  }

  Srequest = "confPin";
#if defined(debugModeOn)
  Serial.print(Srequest);
  Serial.print(": ");
#endif
  rc = ParamNumber.GetCommand(Srequest); // look for commant inside the input
  cmdIdx = rc.idxCommand;
  if (cmdIdx != -1)
  {
    numericParameter numericRC = storedParam.GetNumericParameter(cmdIdx);
#if defined(debugModeOn)
    Serial.println(numericRC.parameterNumericValue);
#endif
    configPin = numericRC.parameterNumericValue;
  }

  Srequest = "debugPin";
#if defined(debugModeOn)
  Serial.print(Srequest);
  Serial.print(": ");
#endif
  rc = ParamNumber.GetCommand(Srequest); // look for commant inside the input
  cmdIdx = rc.idxCommand;
  if (cmdIdx != -1)
  {
    numericParameter numericRC = storedParam.GetNumericParameter(cmdIdx);
#if defined(debugModeOn)
    Serial.println(numericRC.parameterNumericValue);
#endif
    debugPin = numericRC.parameterNumericValue;

  }

  Srequest = "readyPin";
#if defined(debugModeOn)
  Serial.print(Srequest);
  Serial.print(": ");
#endif
  rc = ParamNumber.GetCommand(Srequest); // look for commant inside the input
  cmdIdx = rc.idxCommand;
  if (cmdIdx != -1)
  {
    numericParameter numericRC = storedParam.GetNumericParameter(cmdIdx);
#if defined(debugModeOn)
    Serial.println(numericRC.parameterNumericValue);
#endif
    readyPin = numericRC.parameterNumericValue;
  }

  Srequest = "SSID1";
#if defined(debugModeOn)
  Serial.print(Srequest);
  Serial.print(": ");
#endif
  rc = ParamNumber.GetCommand(Srequest); // look for commant inside the input
  cmdIdx = rc.idxCommand;
  if (cmdIdx != -1)
  {
    parameter RC = storedParam.GetParameter(cmdIdx);
#if defined(debugModeOn)
    Serial.println(RC.parameterValue);
#endif
    for (int i = 0; i < sizeof(ssid1); i++)
    {
      ssid1[i] = RC.parameterValue[i];
    }
  }

  Srequest = "PSW1";
#if defined(debugModeOn)
  Serial.print(Srequest);
  Serial.print(": ");
#endif
  rc = ParamNumber.GetCommand(Srequest); // look for commant inside the input
  cmdIdx = rc.idxCommand;
  if (cmdIdx != -1)
  {
    parameter RC = storedParam.GetParameter(cmdIdx);
#if defined(debugModeOn)
    Serial.println(RC.parameterValue);
#endif
    for (int i = 0; i < sizeof(pass1); i++)
    {
      pass1[i] = RC.parameterValue[i];
    }
  }

  Srequest = "SSID2";
#if defined(debugModeOn)
  Serial.print(Srequest);
  Serial.print(": ");
#endif
  rc = ParamNumber.GetCommand(Srequest); // look for commant inside the input
  cmdIdx = rc.idxCommand;
  if (cmdIdx != -1)
  {
    parameter RC = storedParam.GetParameter(cmdIdx);
#if defined(debugModeOn)
    Serial.println(RC.parameterValue);
#endif
    for (int i = 0; i < sizeof(ssid2); i++)
    {
      ssid2[i] = RC.parameterValue[i];
    }
  }

  Srequest = "PSW2";
#if defined(debugModeOn)
  Serial.print(Srequest);
  Serial.print(": ");
#endif
  rc = ParamNumber.GetCommand(Srequest); // look for commant inside the input
  cmdIdx = rc.idxCommand;
  if (cmdIdx != -1)
  {
    parameter RC = storedParam.GetParameter(cmdIdx);
#if defined(debugModeOn)
    Serial.println(RC.parameterValue);
#endif
    for (int i = 0; i < sizeof(pass2); i++)
    {
      pass2[i] = RC.parameterValue[i];
    }
  }

  Srequest = "routePort";
#if defined(debugModeOn)
  Serial.print(Srequest);
  Serial.print(": ");
#endif
  rc = ParamNumber.GetCommand(Srequest); // look for commant inside the input
  cmdIdx = rc.idxCommand;
  if (cmdIdx != -1)
  {
    numericParameter numericRC = storedParam.GetNumericParameter(cmdIdx);
#if defined(debugModeOn)
    Serial.println(numericRC.parameterNumericValue);
#endif
    routePort = numericRC.parameterNumericValue;
  }

  Srequest = "tracePort";
#if defined(debugModeOn)
  Serial.print(Srequest);
  Serial.print(": ");
#endif
  rc = ParamNumber.GetCommand(Srequest); // look for commant inside the input
  cmdIdx = rc.idxCommand;
  if (cmdIdx != -1)
  {
    numericParameter numericRC = storedParam.GetNumericParameter(cmdIdx);
#if defined(debugModeOn)
    Serial.println(numericRC.parameterNumericValue);
#endif
    tracePort = numericRC.parameterNumericValue;
  }

  Srequest = "listenPort";
#if defined(debugModeOn)
  Serial.print(Srequest);
  Serial.print(": ");
#endif
  rc = ParamNumber.GetCommand(Srequest); // look for commant inside the input
  cmdIdx = rc.idxCommand;
  if (cmdIdx != -1)
  {
    numericParameter numericRC = storedParam.GetNumericParameter(cmdIdx);
#if defined(debugModeOn)
    Serial.println(numericRC.parameterNumericValue);
#endif
    udpListenPort = numericRC.parameterNumericValue;
  }

  Srequest = "IP1";
#if defined(debugModeOn)
  Serial.print(Srequest);
  Serial.print(": ");
#endif
  rc = ParamNumber.GetCommand(Srequest); // look for commant inside the input
  cmdIdx = rc.idxCommand;
  if (cmdIdx != -1)
  {
    numericParameter numericRC = storedParam.GetNumericParameter(cmdIdx);
#if defined(debugModeOn)
    Serial.println(numericRC.parameterNumericValue);
#endif
    IP1 = numericRC.parameterNumericValue;
    serverIP[0] = IP1;
  }

  Srequest = "IP2";
#if defined(debugModeOn)
  Serial.print(Srequest);
  Serial.print(": ");
#endif
  rc = ParamNumber.GetCommand(Srequest); // look for commant inside the input
  cmdIdx = rc.idxCommand;
  if (cmdIdx != -1)
  {
    numericParameter numericRC = storedParam.GetNumericParameter(cmdIdx);
#if defined(debugModeOn)
    Serial.println(numericRC.parameterNumericValue);
#endif
    IP2 = numericRC.parameterNumericValue;
    serverIP[1] = IP2;
  }

  Srequest = "IP3";
#if defined(debugModeOn)
  Serial.print(Srequest);
  Serial.print(": ");
#endif
  rc = ParamNumber.GetCommand(Srequest); // look for commant inside the input
  cmdIdx = rc.idxCommand;
  if (cmdIdx != -1)
  {
    numericParameter numericRC = storedParam.GetNumericParameter(cmdIdx);
#if defined(debugModeOn)
    Serial.println(numericRC.parameterNumericValue);
#endif
    IP3 = numericRC.parameterNumericValue;
    serverIP[2] = IP3;
  }

  Srequest = "IP4";
#if defined(debugModeOn)
  Serial.print(Srequest);
  Serial.print(": ");
#endif
  rc = ParamNumber.GetCommand(Srequest); // look for commant inside the input
  cmdIdx = rc.idxCommand;
  if (cmdIdx != -1)
  {
    numericParameter numericRC = storedParam.GetNumericParameter(cmdIdx);
#if defined(debugModeOn)
    Serial.println(numericRC.parameterNumericValue);
#endif
    IP4 = numericRC.parameterNumericValue;
    serverIP[3] = IP4;
  }
}


