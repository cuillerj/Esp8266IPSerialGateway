/*
    release notes version 2.1
    GPIO rnot longer used for activate udp trace due to some stability issue
    Trace to be activated by compile switch

    release notes version 2.2
    ShowWifi print IP ports

    release notes version 2.3
    regurarly send status frame

    release notes version 2.5
    listen 2 UDP ports - one UDPG dedicated to receive gateway commands (used to dynamicaly modify server IP addrress and port
      and to switch between SSID according to indicator value sotred n dtabase)
    automatic switch between 2 wifi in case of issue
    release notes version 2.6
     send duration since reboot in status frame
    release notes version 2.7
      configPin debug and modification: set pullUp, High:normal mode Low:configuration mode
    release notes version 2.8
      tracePort renamed commandPort
    release notes version 2.9
      use SSID1 at the boot instead of 0 default
      correction bug commandPort from eeprom
    release notes version 3.0
      correction bug switch wifi
    release notes version 3.1
      correction bug read serial before end of reboot
*/
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
    It transfers any 2 consecutive bytes binary data
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
//#define debugModeOn               // uncomment this define to debug the code
#define forceErase false            // set to true in case of major issue with the eeprom tool

#define versionID 31
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <EEPROM.h>

#define defaultSerialSpeed 38400   // the default speed of the gateway

/* *** Define look for string environment

*/
#include <LookForString.h>        // used to look for string inside commands and parameters
#include <HomeAutomationBytesCommands.h>
#define parametersNumber 20       // must fit with the paramList
#define commandnumber 31          // must fit with the commandList

/* *** commands list definition
    Define all the command's names
    Ended by "=" means this command will set the parameter of the seame name
*/
String commandList[commandnumber] = {"SSID1=", "PSW1=", "SSID2=", "PSW2=", "ShowWifi", "Restart", "DebugOn", "DebugOff", "ScanWifi", "SSID=0", "SSID=1", "SSID=2", "ShowEeprom", "EraseEeprom",
                                     "routePort=", "commandPort=", "IP1=", "IP2=", "IP3=", "IP4=", "stAddr=", "SSpeed=", "cnxLED=", "serialLED=", "pwrLED=", "confPin=", "debugPin=", "readyPin=", "listenPort=", "gatewayPort=" ""
                                    };

/* *** paramters list definition
    Define all the parameter's names
    Parameters that can be set to value must have a name that fit with the commandList (without "=")
*/
String paramList[parametersNumber] = {"stAddr", "SSpeed", "cnxLED", "serialLED", "pwrLED", "confPin", "debugPin", "readyPin" , "SSID1", "PSW1", "SSID2", "PSW2",
                                      "routePort", "commandPort", "listenPort", "IP1", "IP2", "IP3", "IP4", "gatewayPort"
                                     };
/*
    define the exact number of characters that define the parameter's values
*/
unsigned int paramLength[parametersNumber] =  {4, 5, 2, 2, 2, 2, 2, 2, 50, 50, 50, 50, 4, 4, 4, 3, 3, 3, 3, 4};
/*
   define the type of parameters 0x00 means String, 0x01 means number to be stored with one byte (<256), 0x02 means means number to be stored with two byte (< 65 535),  0x03 means means number to be stored with two byte (< 16 777 216)
*/
uint8_t paramType[parametersNumber] =         {0x02, 0x03, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x02, 0x02, 0x03, 0x01, 0x01, 0x01, 0x01, 0x03};
/*
   define the default paramter's values that will be used before configuration
   The number of caracters must fit with the parameter length
*/
byte paramValue[parametersNumber][50] =       {"1000", "38400", "14", "12", "13", "04", "02", "05", "cuiller", "116A2E2CC25E3DDC3593E13D29", "cuiller3", "5E98D9BEC6", "1901", "1902", "8888", "192", "168", "001", "005", "8889"};
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

char keyword[] = "AzErTy";                  // must contain 6 charasters  - this keyword is used to check eeprom compatibity and can be modified
ManageParamEeprom storedParam (parametersNumber, ramOffset, keyword);

/*

*/
int configPin = 4;  // set to config mode when set to 3.3v - running mode when set to ground
int debugPin = 2; // Switch of udp trace when grounded  On when set to 3.3v - no longer used
int readyPin = 5;  // relay off if wifi connection not established
int serialLED = 12;
int powerLED = 13;
int connectionLED = 14;
int stAddr = 0;
int SSpeed = defaultSerialSpeed;
int routePort = 1901;
int commandPort = 1902;
byte serverIP[4] = {0xc0, 0xa8, 0x01, 0x05};  //
IPAddress remoteAddr ;
int udpListenPort = 8888;
int gatewayPort = 8889;
uint8_t IP1 = 0x00;
uint8_t IP2 = 0x00;
uint8_t IP3 = 0x00;
uint8_t IP4 = 0x00;
uint8_t frameNumber = 0x00;
#define bitDiagWifi 0
#define bitDiagIP 1
#define diagMemory 2

#define nbUdpPort 2
#define maxSSIDLength 50
#define maxPSWLength 50

char ssid1[maxSSIDLength] = "";       // first SSID  will be used at first
char pass1[maxPSWLength] = "";      // first pass
char ssid2[maxSSIDLength] = "";        // second SSID will be used in case on connection failure with the first ssid
char pass2[maxPSWLength] = "";  // second pass

boolean debugMode = false;
boolean restartCompleted = false;
boolean connectedStatus = false;
boolean serialFirstCall = true;
String ver = "IPSerialGateway";
uint8_t vers = 0x01;
uint8_t addrStation[4] = {0x00, 0x00, 0x04, 0x03}; // 2 1er octets reserves commence ensuite 1024 0x04 0x00
uint8_t typeStation[4] = {0x00, 0x00, 0x04, 0x01};
//String Srequest = "                                                                                                                                                                                                                                                                                                                     ";
String Srequest;
uint8_t addrStationLen = sizeof(addrStation);
uint8_t typeStationLen = sizeof(typeStation);
int udpPort[nbUdpPort];
String services[] = {"Route", "Cmde"};
char packetBuffer[UDP_TX_PACKET_MAX_SIZE]; //buffer to hold incoming packet,
byte bufUdp[255];
uint8_t currentSSID = 0x01;         // current used SSID
uint8_t retryWifi = 0x00;         //
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
unsigned long timeCheck = 0;
unsigned long timeAffLED = 0;
unsigned long timeUdp = 0;
unsigned long timeLastSentUdp = 0;
unsigned long timeCheckWifi = 0;
unsigned long timeRestart = 0;
unsigned long timeUpdateStatus = 0;
unsigned long diagTime = 0;
#define updateStatusCycle 300000
#define diagTimeCycle 120000
// internal data
#define WifidiagBit 0
#define IPdiagBit 1
uint8_t  diagByte = WifidiagBit | IPdiagBit;
//uint8_t SavDiag = Diag;
unsigned int freeMemory = 0;
int id;
WiFiUDP Udp;
WiFiUDP UdpG;

void setup() {
  Serial.begin(defaultSerialSpeed);
  delay(1000);
#if defined(debugModeOn)
  Serial.println("start");
#endif
  delay(1000);
  pinMode(powerLED, OUTPUT);
  digitalWrite(powerLED, 1);
  pinMode(connectionLED, OUTPUT);
  digitalWrite(connectionLED, 1);
  pinMode(serialLED, OUTPUT);
  digitalWrite(serialLED, 1);
  pinMode(readyPin, OUTPUT);
  Srequest.reserve(60);
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
  udpPort[1] = commandPort; //
  timeRestart = millis();

  // ConnectWifi("default", "default");
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
#if defined(debugModeOn)
  Serial.println("set wifi mode ");
#endif
  WiFi.mode(WIFI_STA);
#if defined(debugModeOn)
  Serial.println("udp begin ");
#endif
  Udp.begin(udpListenPort);
#if defined(debugModeOn)
  Serial.println("gw udp begin ");
#endif
  UdpG.begin(gatewayPort);
  delay (5000);
#if defined(debugModeOn)
  Serial.println("trace");
#endif
  TraceToUdp("Ready! Use ", 0x02);
  pinMode(configPin, INPUT_PULLUP);
  digitalWrite(powerLED, 0);
  digitalWrite(connectionLED, 0);
  digitalWrite(serialLED, 0);
  delay(1000);
  digitalWrite(readyPin, 0);
}

void loop() {
  if (millis() - timeRestart >= 20000 && restartCompleted == false)
  {
#if defined(debugModeOn)
    Serial.println("reboot complete");
#endif
    restartCompleted = true;
    if (CheckCurrentIP())                    // check ip address
    {
      if (!digitalRead(configPin))
      {
        Serial.print("Ready to use ! ");
        PrintUdpConfig();
      }
      digitalWrite(readyPin, true);
    }
    else
    {
      if (!digitalRead(configPin))
      {
        Serial.print("Not ready to use ! ");
        PrintUdpConfig();
      }
    }
    Udp.begin(udpListenPort);
    UdpG.begin(gatewayPort);
    delay (5000);
    if (!digitalRead(configPin))
    {
      WiFi.printDiag(Serial);
    }
  }
  if (((bitRead(diagByte, WifidiagBit) || bitRead(diagByte, IPdiagBit))) &&  millis()  > diagTime + diagTimeCycle &&  retryWifi <= 5)
  {
#if defined debugModeOn
    Serial.println("request default wifi connection");
#endif
    diagTime = millis();
    //ConnectWifi("default", "default");

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
  }
  if (restartCompleted && millis() - timeSerial >= 20)
  {
    int lenInput = Serial_have_message();
    {
      if (lenInput != 0)
      {
        String   lenS = "serial len:" + String(lenInput);
        //  TraceToUdp(lenS, 0x01);
#if defined(debugModeOn)
        Serial.println(lenS);
        delay(100);
#endif
        RouteToUdp(lenInput);
      }
    }
    timeSerial = millis();
  }

  if (millis() - timeCheck >= 30000)
  {

    boolean wifiStatus = false;
    if (WiFi.status() == WL_CONNECTED)
    {
      wifiStatus = true;
      bitWrite(diagByte, WifidiagBit, 0);
#if defined debugModeOn
      Serial.println("wifi connected ");
#endif
    }
    else {
      bitWrite(diagByte, WifidiagBit, 1);
      digitalWrite(readyPin, false);
#if defined debugModeOn
      Serial.println("wifi not connected ");
#endif
    }
    freeMemory = ESP.getFreeHeap();
    String  strS = "free memory:" + String(freeMemory);
    //  TraceToUdp(strS, 0x01);
    if (debugMode == true)
    {
      Serial.println(strS);
    }
    connectedStatus = CheckCurrentIP() && wifiStatus;
    timeCheck = millis();
  }
  if (millis() - timeUdp >= 100)
  {
    InputUDP();
    InputUDPG();
    timeUdp = millis();
  }
  // *** refresh LED
  if (millis() - timeAffLED > 1000 && restartCompleted == true)
  {
    AffLed();
    timeAffLED = millis();
  }
  // *** end of loop refresh LED
  if (millis() - timeUpdateStatus >= updateStatusCycle)
  {
    SendStatus();
    timeUpdateStatus = millis();
  }
  if (!connectedStatus && (millis() > timeCheckWifi + 60000))
  {
#if defined(debugModeOn)
    Serial.println(":");
#endif
    retryWifi = 0;
    currentSSID = (currentSSID + 1) % 3;
    Restart();
    timeCheckWifi = millis();
  }
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
    int posCr = 0;
    for ( posCr = 0; posCr < maxSSIDLength; posCr++)
    {
      if (ssid[posCr] == 0x0d || ssid[posCr] == 0x0a || ssid[posCr] == 0x00)  { // look for Cr end of ssid
        break;
      }
    }


    for (int i = posCr; i < maxSSIDLength ; i++) {
      ssid[i] = 0x00;
#if defined debugModeOn
      Serial.print(ssid[i], HEX);
#endif
    }
#if defined debugModeOn
    Serial.println();
#endif
    for ( posCr = 0; posCr < maxPSWLength; posCr++)
    {
      if (pass[posCr] == 0x0d || pass[posCr] == 0x0a || pass[posCr] == 0x00)  { // look for Cr end of ssid
        break;
      }
    }
    for (int i = posCr; i < maxPSWLength ; i++) {
      pass[i] = 0x00;
    }

    WiFi.begin(ssid, pass);
  }
  if (!digitalRead(configPin) && retryWifi < 5)
  {
    Serial.println();
    Serial.print("gateway version:");
    Serial.println(versionID);
    Serial.print("\n Please wait... Connecting to ");
    Serial.print(ssid);
    Serial.print(" retry:");
    Serial.println(retryWifi);
  }

  while (WiFi.status() != WL_CONNECTED && retryWifi <= 5)
  {
    delay(2000);
    if (retryWifi >= 5) {
      if (!digitalRead(configPin)) // gateway config mode
      {
        Serial.print("Could not connect to "); Serial.print(ssid);
        Serial.print(" retry:"); Serial.println(retryWifi);
      }
      delay(500);
      bitWrite(diagByte, bitDiagWifi, 1);       // position bit diag
      if (digitalRead(configPin)) // gateway config mode
      {
        break;
      }
      else {
        ScanWifi();
        retryWifi++;
        break;
        //Udp.stop();
      }
    }
    else
    {
      retryWifi++;
      WiFi.mode(WIFI_STA);
      Udp.begin(udpListenPort);
      UdpG.begin(gatewayPort);
      delay (5000);
      if (!digitalRead(configPin)) // gateway run mode
      {
        WiFi.printDiag(Serial);
      }
    }
  }
  if (WiFi.status() == WL_CONNECTED) {
    bitWrite(diagByte, bitDiagWifi, 0);       // position bit diag
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
    digitalWrite(readyPin, false);
    return false;
  }
  else
  {
    if (debugMode == true)
    {
      Serial.println(currentIP);
    }
    bitWrite(diagByte, bitDiagIP, 0);       // position bit diag
    digitalWrite(readyPin, true);
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

void SendToUdpG( int mlen, int serviceId) {
  if ( millis() - timeLastSentUdp  > 200)
  {
    UdpG.beginPacket(serverIP, udpPort[serviceId]);
    UdpG.write(dataBin, mlen);
    UdpG.endPacket();
  }
  else
  {
    delay (200);
    UdpG.beginPacket(serverIP, udpPort[serviceId]);
    UdpG.write(dataBin, mlen);
    UdpG.endPacket();
  }
  timeLastSentUdp = millis();
}
void TraceToUdp(String req, uint8_t code)
{
#if defined(traceOn)
  //  if ( digitalRead(debugPin) == 0)
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
#endif
}
void RouteToUdp(int len)
{
  dataBin[0] = bufParam[0]; // station address
  dataBin[1] = 0x3B;
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
    if (digitalRead(configPin)) // gateway run mode
    {
#if defined(debugModeOn)
      Serial.println("run");
      delay(100);
#endif
      while (Serial.available())
      {
#if defined(debugModeOn)
        Serial.println("S");
        delay(100);
#endif
        byte In1 = (Serial.read());

#if defined(debugModeOn)
        Serial.println(In1, HEX);
        delay(100);
#endif
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
      delay(50);
    }
    else  // gateway configuration mode
    {
      Srequest = (Serial.readString());
      if (Srequest.length() == 0) {
        return 0;
      }
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
              }
              Serial.println("");
#endif
              for (int i = 0; i < maxSSIDLength; i++)
              {
                ssid1[i] = 0x00;
              }
              Srequest.replace("\n", "");
              UpdateEepromParameter(cmdIdx, Srequest, cmdPos);
              String id = Srequest.substring(cmdPos);
              LoadEerpromParamters();
#if defined(debugModeOn)
              Serial.println("new SSID:");
              for (int i = 0; i < id.length() - 1; i++)
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
              Srequest.replace("\n", "");
              UpdateEepromParameter(cmdIdx, Srequest, cmdPos);
              String id = Srequest.substring(cmdPos);
              LoadEerpromParamters();
#if defined(debugModeOn)
              Serial.println("new psw:");
              for (int i = 0; i <  id.length() + 1; i++)
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
#if defined(debugModeOn)
                Serial.println("Srequest:");
                for (int i = 0; i < maxSSIDLength; i++)
                {
                  Serial.print(Srequest[i], HEX);
                  Serial.print("-");
                }
                Serial.println("");
#endif
                Srequest.replace("\n", "");
                UpdateEepromParameter(cmdIdx, Srequest, cmdPos);
                String id = Srequest.substring(cmdPos);
                LoadEerpromParamters();
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
              Srequest.replace("\n", "");
              UpdateEepromParameter(cmdIdx, Srequest, cmdPos);
              String id = Srequest.substring(cmdPos);
              LoadEerpromParamters();
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
              Serial.print("ports:");
              Serial.print(routePort);
              Serial.print(" G:");
              Serial.print(commandPort);
              Serial.print(" ");
              Serial.print(udpListenPort);
              Serial.print(" G:");
              Serial.println(gatewayPort);
              break;
            }
          case 5:     // Restart
            {
              Restart();
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
              LoadEerpromParamters();
              break;
            }
          case 11:    // SSID=2
            {
              currentSSID = 0x02;
              LoadEerpromParamters();
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
    Srequest = "";

  }
}
void InputUDP() {
  int packetSize = Udp.parsePacket();  // message from UDP

  if (packetSize)
  {
    String lenS = "udp input" + String(packetSize);
    TraceToUdp(lenS, 0x01);
    bufSerialOut[1] = 0x7f;
    bufSerialOut[2] = 0x7e;
    bufSerialOut[3] = 0x7f;
    bufSerialOut[4] = 0x7e;

    Udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);
    int packetBufferLen = packetSize;
    bufSerialOut[5] = packetSize;
    for (int i = 0; i < maxUDPResp; i++)
    {
      bufSerialOut[i + 6] = 0x00;
    }

    for (int i = 0; i < packetSize; i++)
    {

      bufSerialOut[i + 6] = packetBuffer[i];
    }
    if (packetSize < 6)
    {
      packetSize = 6;
    }
    digitalWrite(serialLED, 1);
    Serial.write(bufSerialOut, packetSize + 6);
  }
}
void InputUDPG() {
#define commandBytePosition 7
#define crcLen 2
#define selectSSID indicatorsRequest
  int packetSize = 0;
  packetSize = UdpG.parsePacket();  // message from UDP
  if (packetSize)
  {
    String lenS = "udp input" + String(packetSize);
    UdpG.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);
    remoteAddr = UdpG.remoteIP();
    int packetBufferLen = packetSize;
    uint8_t computeCRC = CRC8(&packetBuffer[commandBytePosition], packetSize - commandBytePosition - crcLen);
#if defined debugModeOn
    Serial.print("from:");
    Serial.println(remoteAddr);
#endif
    if (computeCRC != packetBuffer[packetSize - 1])
    {
#if defined debugModeOn
      Serial.print("erreur expected crc:");
      Serial.println(computeCRC);
#endif
    }
#if defined debugModeOn
    Serial.print("receive on G len:");
    Serial.print(packetSize);
    Serial.print(" crc:");
    Serial.println(packetBuffer[packetSize - 1], HEX);
    for (int i = 0; i < packetSize; i++)
    {
      Serial.print(packetBuffer[i], HEX);
      Serial.print(":");
    }
    Serial.println("");
#endif
    if (packetBuffer[commandBytePosition] == serviceInfoRequest) {
      serverIP[0] = remoteAddr[0];
      serverIP[1] = remoteAddr[1];
      serverIP[2] = remoteAddr[2];
      serverIP[3] = remoteAddr[3];
      uint8_t idx = packetBuffer[commandBytePosition + 1] - 1;
      unsigned int value = packetBuffer[commandBytePosition + 2] * 256 + packetBuffer[commandBytePosition + 3];
#if defined debugModeOn
      Serial.print("serverIP:");
      Serial.print(idx);
      Serial.print(":");
      Serial.println(value);
#endif
      udpPort[idx] = value;
    }
    if (packetBuffer[commandBytePosition] == selectSSID) {
      uint8_t idx = packetBuffer[commandBytePosition + 8];
      if (!digitalRead(configPin) )
      {
        Serial.print("select SSID:");
        Serial.println(idx);
      }
      if ( idx == 0 || idx == 1 || idx == 2)
      {
        currentSSID = idx;
        if (idx == 0)
        {
          return;
        }
      }
      else {
        return;
      }

      Udp.stop();
      bitWrite(diagByte, bitDiagWifi, 0);       // position bit diagc              digitalWrite(connextionLED,false);
      WiFi.disconnect(true);
      delay(1000);
      restartCompleted = false;
      timeRestart = millis();
      Restart();
      SendStatus();
    }
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
  UdpG.begin(gatewayPort);
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
  digitalWrite(serialLED, 0);
  if (!bitRead(diagByte, WifidiagBit) && !bitRead(diagByte, IPdiagBit))
  {
#if defined debugOn
    Serial.print("cnx ko:");
    Serial.println(diagByte, HEX);
#endif
    digitalWrite(connectionLED, 0);

  }
  else if (bitRead(diagByte, WifidiagBit) && bitRead(diagByte, IPdiagBit))
  {
#if defined debugOn
    Serial.print("cnx ok:");
    Serial.println(diagByte, HEX);
#endif
    digitalWrite(connectionLED, 1);
  }
  else {
#if defined debugOn
    Serial.print("cnx half:");
    Serial.println(diagByte, HEX);
#endif
    digitalWrite(connectionLED, !digitalRead(connectionLED));
  }
  if ( diagByte == 0b00000000 && digitalRead(configPin))
  {
    digitalWrite(powerLED, 1);
  }
  else
  {
    digitalWrite(powerLED, !digitalRead(powerLED));
  }

}
void setInitialParam(int number)
{
  int idxArray = number ;
  //Serial.print(paramList[number]);
  Serial.print(" set parameter : ");
  Serial.print(number);
  Serial.print(" size: ");
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
    commandReturn rc = ParamNumber.GetCommand(Srequest); // look for command inside the input
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
        Serial.println(RC.parameterValue);
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
  char Cvalue[maxParameterLen + 50];
  Svalue.toCharArray(Cvalue, sizeof(Svalue));

  commandReturn rc = ParamNumber.GetCommand(Sparam); // look for parameter inside the input
  int paramIdx = rc.idxCommand;
  if (paramIdx != -1)
  {
    int idxArray = paramIdx ;
    Serial.print(" set parameter : ");
    Serial.print(idxArray);
    Serial.print(" size: ");
    Serial.print(paramLength[idxArray]);
    Serial.println();
    if (paramType[idxArray] == 0x00)
    {
      byte Bvalue[maxParameterLen + 50];
      for (int i = 0; i < sizeof(Bvalue); i++) {
        Bvalue[i] = 0x00;
      }
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
    for (int i = 0; i < sizeof(ssid2) - 1; i++)
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
    for (int i = 0; i < sizeof(pass2) - 1; i++)
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

  Srequest = "commandPort";
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
    commandPort = numericRC.parameterNumericValue;
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

  Srequest = "gatewayPort";
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
    gatewayPort = numericRC.parameterNumericValue;
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

void SendStatus()
{
#define frameLen 26
  dataBin[0] = frameNumber;
  dataBin[1] = 0x00;
  dataBin[2] = uint8_t(frameLen);
  dataBin[3] = uint8_t(stAddr / 256); // addrSSerial
  dataBin[4] = uint8_t(stAddr);  // addrMSerial
  dataBin[5] = 0x00;  // Print
  dataBin[6] = statusResponse;  //
  dataBin[7] = diagByte;
  dataBin[8] = 0x00;
  dataBin[9] = versionID;
  dataBin[10] = 0x00;
  dataBin[11] = currentSSID;
  dataBin[12] = 0x00;
  dataBin[13] = serverIP[0] ;
  dataBin[14] = serverIP[1] ;
  dataBin[15] = serverIP[2] ;
  dataBin[16] = serverIP[3] ;
  dataBin[17] = 0x00;
  dataBin[18] = currentIP[0] ;
  dataBin[19] = currentIP[1] ;
  dataBin[20] = currentIP[2] ;
  dataBin[21] = currentIP[3] ;
  dataBin[22] = 0x00;
  unsigned int minuteSinceReboot = (millis() / 60000);
  dataBin[23] = uint8_t(minuteSinceReboot / 256);
  dataBin[24] = uint8_t(minuteSinceReboot);
  SendToUdpG(frameLen, 1);
  frameNumber++;
#if defined(debugModeOn)
  Serial.println("send status");
#endif
}
void Restart() {
  Udp.stop();
  bitWrite(diagByte, bitDiagWifi, 0);       // position bit diagc              digitalWrite(connextionLED,false);
  WiFi.disconnect(true);
  delay(1000);
  restartCompleted = false;
  timeRestart = millis();
  retryWifi = 0x00;
  InitConfig();
}
