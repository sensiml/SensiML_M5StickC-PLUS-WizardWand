/* ------------------------------------------------------------------------------------------------------------------------------------------------------
*   SensiML M5StickC PLUS Unified Demo Game/DCL/Test Application - Arduino M5StickC PLUS (ESP32-PICO)
*   Version: 0.91
*   Date: February 22, 2023
*   Author: Chris Rogers
*   Purpose: Example demonstrating integration of SensiML Knowledge Pack into ESP32 WiFi enabled connected IoT device application
*            Provide multi-mode operation for raw IMU data capture mode and example gesture recognition 'Wizard Wand' game with integrated ML model
*            Data Capture Mode:  Streams raw MPU6886 6DoF IMU sensor data via WiFi from M5StickC PLUS to SensiML DCL at configurable rates from 10Hz to 1kHz
*            Game Mode: Supports WiFi connected Windows C# companion game UI that conveys timed gesture recognition game status via REST API
*
*   Copyright (c) 2023 SensiML Corporation
*
*   Redistribution and use in source and binary forms, with or without
*   modification, are permitted provided that the following conditions are met:
*
*   1. Redistributions of source code must retain the above copyright notice,
*      this list of conditions and the following disclaimer.
*
*   2. Redistributions in binary form must reproduce the above copyright notice,
*      this list of conditions and the following disclaimer in the documentation
*      and/or other materials provided with the distribution.
*
*   3. Neither the name of the copyright holder nor the names of its contributors
*      may be used to endorse or promote products derived from this software
*      without specific prior written permission.
*
*   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
*   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
*   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
*   ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
*   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
*   DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
*   SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*   OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
------------------------------------------------------------------------------------------------------------------------------------------------------ */

#include <M5StickCPlus.h> //Tested on version 2.0.3
#include <WiFi.h> // Tested on version 1.2.7
#include <Adafruit_NeoPixel.h>  //Tested on version 1.10.7
#include <EEPROM.h>
#include "src/includes/ww_lcd_bitmaps.h"
#include "src/includes/ww_melodies.h"
#include "src/kb/kb.h"
#define NUM_PIXELS 7
#define NEOPIXEL_PIN G33
#define BUTTON_A G37
#define BUTTON_B G39
#define BUZZER_PIN G26
#define GAME_MODE 0   //Play Wizard game > Supports /gamestatus and /reset endpoints
#define DCL_MODE 1    //Stream raw sensor data to DCL and/or Open Gateway > Supports /config and /stream endpoints
#define TEST_MODE 2   //To do: Stream raw sensor data to DCL and KP results > Supports /config, /stream, /disconnect and /results endpoints
#define APP_TITLE "SensiML M5StickC+ FW"
#define APP_VERSION "Version 0.91"

const int SAMPLE_SELECT[] = {99,19,9,7,4,3,1,0};
int sampleRate = M5.IMU.ODR_250Hz;  // valid MStickC+ w/ FIFO modified lib are ODR_10Hz (99), ODR_50Hz (19), ODR_100Hz (9), ODR_125Hz (7), ODR_200Hz (4), ODR_250Hz (3), ODR_500Hz (1), ODR_1kHz (0)

// Sensor Data Array and SSFv1 HTTP streaming format declarations
const int CHANNELS_PER_SAMPLE = 7;  // for applications using the onboard IMU motion sensor MPU6886 -> accel_X, accel_Y, accel_Z, gyro_X, gyro_y, gyro_Z, target_label_class
int SAMPLES_PER_PACKET = 100 / (sampleRate + 1);  //Adjust samples/packet to yield 10 packets/sec stream no matter the sample rate
int PACKET_SIZE = SAMPLES_PER_PACKET * CHANNELS_PER_SAMPLE; // accelx,y,z + gyrox,y,z + target gesture class number from BtnA trigger
char dataPacket[700 * 2];  // Define as maximum array size for 1kHz sample rate
uint32_t packetIndex = 0;
String ssfConfig =""; // DCL simple streaming format descriptor string (see https://sensiml.com/documentation/simple-streaming-specification/simple-wifi-streaming.html#simple-stream-wi-fi-endpoints)

// EEPROM value for Wi-Fi network credentials and future use
/**********************************************************************************************************************************/
/* IMPORTANT NOTE: WiFi credentials stored in EEPROM are not encrypted and flashed to device NVM in cleartext for simplicity      */
/*                 Use caution in storing sensitive network credentials and/or use a separate WiFi test AP for use and evaluation */
/**********************************************************************************************************************************/
char eepromChk[4] = "";
char wifiSSID[33] = "";     // array defined for maximum SSID length
char wifiPassword[64] = ""; // array defined for WPA2 maximum password length
char eepromReserved[26] = {0};

// IMU sensor channel variables
int16_t accX = 0;
int16_t accY = 0;
int16_t accZ = 0;
int16_t gyroX = 0;
int16_t gyroY = 0;
int16_t gyroZ = 0;

// HTTP timer declarations
const long TIMEOUT_TIME = 2000; // 2sec TCP/HTTP session timeout
unsigned long httpTime;
// HTTP state variabbles
String header = "";
int crcount = 0;
bool streaming = false;

// Audio buzzer declarations
int freq = 50;
uint8_t ledChannel = 0;
int resolution = 10;

// Pixel Strobe variable initialization
uint16_t psHue = 32768;
uint8_t psLevel = 0;
uint8_t psStep = 1;
uint16_t PSDelay = 0;

// Button variables
uint8_t btnState[2] = {HIGH, HIGH};
uint32_t btnTime[2] = {0, 0};
                                  
WiFiServer server(80);  // Set web server port number to 80
WiFiClient client;   // Instantiate WiFi client

// IMU sample array
int16_t pSample[7];

// Game Model declarations
const char gestureset[11] = {0, 1, 2, 3, 1, 4, 5, 6, 10, 9, 11};  // Corresponding bitmap index to display for each gamestate value
const char classmap[8] = {0, 6, 1, 5, 6, 2, 4, 3}; // From model.json "S",E","N","S","I","M","L" starting from index 1
const char classlabel[9] = "UEILMNSU";  // Value to display in serial debug console output for target and recognized gesture class
const uint32_t LAST_GESTURE_HOLD_MSEC = 2000; //Retain display of last gesture on remote UI

// Application state variables
const char NUM_GESTURES = 7;  // Number of distinct gestures to train, including UNKNOWN (used in DCL_Mode only)
int gestureresult = 0;
int lastgestureresult = -1;
uint32_t lgrstickytimer;
uint32_t StartTime;
uint32_t GameOverTime = 20 * 1000; // Default time limit for game is set at 20 seconds
uint32_t secsRemaining = 0;
uint8_t pgmMode = GAME_MODE;
int GameState = 0;  // Primary state variable for game progression 0=Ready to Play, 1 thu 7 = gesture sequence, 8=Winner, 9=Loser
int GameOverState = 0;  // Variable to retain furthest level acheived in game if player lost
bool resetReady = false;
char gestureIndex = 0;  // State variable for target gesture class during DCL Mode operation
int16_t sampleIndex = 0;
bool isStreaming = false; // Boolean state variable for DCL_MODE streaming state
char serialCommand[160] = ""; // Variable to hold serial commands for device settings

bool GetWiFiCreds(char *ssid, char *pwd) {

  int i;
  uint8_t byteval;

  EEPROM.get(0,eepromChk); //Check for uninitialized credentials
  if (strstr(eepromChk, "SML")==NULL) {
    Serial.println("Warning: No Wi-Fi credentials saved in NVM memory.  Use \"setwifi=[ssid],[pwd]\" to set network credentials in device.");
    return false;
  }
  for (i=0; i<33; i++) {  // restore WiFi SSID from EEPROM
    ssid[i] = EEPROM.read(4+i);
  }
  for (i=0; i<64; i++) {  // restore WiFi password from EEPROM
    pwd[i] = EEPROM.read(37+i);
  }
  byteval = EEPROM.read(101); // restore chosen program mode from EEPROM
  if (byteval<=TEST_MODE) {
    pgmMode = byteval;
  }
  return true;
}

bool SetWiFiCreds(char *ssid, char *pwd) {

  int i;
  
  if (strlen(ssid)<2) {
    Serial.println("SSID less than minimum 2 character length.");
    return false;
  }
  if (strlen(ssid)>32) {
    Serial.println("SSID exceeds maximum 32 character length.");
    return false;
  }
  if (strlen(pwd)<8) {
    Serial.println("Password less than minimum 8 character length.");
    return false;
  }
  if (strlen(pwd)>63) {
    Serial.println("Password exceeds maximum 63 character length.");
    return false;
  }
  if ((strchr(ssid,'?')!=NULL) || (strchr(ssid,'\"')!=NULL) || (strchr(ssid,'$')!=NULL) || (strchr(ssid,'[')!=NULL) || (strchr(ssid,']')!=NULL) ||
  (strchr(ssid,'\\')!=NULL) || (strchr(ssid,'+')!=NULL)) {
    Serial.println("Password contains one or more invalid characters.");
    return false;
  }
  EEPROM.put(0, "SML");
  for (i=0; i<33; i++) {
    EEPROM.write(4+i, (char)ssid[i]);
  }
  for (i=0; i<64; i++) {
    EEPROM.write(37+i, (char)pwd[i]);
  }
  EEPROM.write(101,pgmMode);
  EEPROM.commit();
  return true;
}

void trimstr(char *s)
{
  int  i,j;
 
  for (i=0; (s[i]==' ') || (s[i]=='\t'); i++);
    
  for (j=0; s[i]; i++) {
    s[j++] = s[i];
  }
  s[j] = '\0';
  for (i = 0; s[i]!='\0'; i++)
  {
    if ((s[i]!=' ') && (s[i] != '\t') && (s[i] != '\r') && (s[i] != '\n')) {
        j = i;
    }
  }
  s[j+1] = '\0';
}

void ProcessSerialCommand() {

  char *a, *b;
  int i;
  char newch[] = "X";
  char cmd[80] = "";
  char p1[80] = "";
  char p2[80] = "";

  while (Serial.available()) {
    newch[0] = Serial.read();
    strncat(serialCommand, newch, 1);
    if ((newch[0] == '\n') || (newch[0] == '\r')) {
      a = strchr(serialCommand,'=');
      if (a != NULL) {
        memset(cmd, '\0', sizeof(cmd));
        strncpy(cmd, serialCommand, a - serialCommand);
        strncpy(p1, a + 1, strlen(serialCommand) - (a - serialCommand) + 1);
        a = strchr(p1,',');
        if (a != NULL) {
          memset(p2, '\0', sizeof(p2));
          strncpy(p2, a + 1, strlen(p1) - (a - p1) + 1);
          memset(a, '\0', 1);
        }
      }
      else {
        memset(cmd, '\0', sizeof(cmd));
        strncpy(cmd, serialCommand, strlen(serialCommand));
      }
      trimstr(cmd);
      trimstr(p1);
      trimstr(p2);
      if (strcmp(cmd,"setwifi")==0) {
        if (SetWiFiCreds(p1, p2)) {
          Serial.println("Successfully updated SSID and password in device NVM.");
        }
      }
      if (strcmp(cmd,"setssid")==0) {
          if (SetWiFiCreds(p1, wifiPassword)) {
            Serial.println("Successfully updated SSID in device NVM.");
          }
      }
      if (strcmp(cmd,"setpassword")==0) {
          if (SetWiFiCreds(wifiSSID, p1)) {
            Serial.println("Successfully updated password in device NVM.");
          }
      }
      if (strcmp(cmd,"reconnect")==0) {
        WiFi.disconnect();
        GetWiFiCreds(wifiSSID, wifiPassword);
        Serial.print("Attempting to reconnect to ");
        Serial.print(wifiSSID);
        WiFi.begin(wifiSSID, wifiPassword);
        i = 0;
        while ((WiFi.status() != WL_CONNECTED) && (i<60)) {
          delay(500);
          i++;
          Serial.print(".");
        }
        if (WiFi.status() == WL_CONNECTED) {
          Serial.print("WiFi Connected. IP:");
          Serial.println(WiFi.localIP());
          server.begin();
        }
        else {
          Serial.println();
          Serial.println("Connection attempt failed.");
        }
      }
      if ((strcmp(cmd,"help")==0) || (strcmp(cmd,"?")==0)) {
        Serial.println("Recognized serial command list:\r\nreconnect\r\nsetpassword=[pwd]\r\nsetssid=[ssid]\r\nsetwifi=[ssid],[pwd]\r\n");
      }
      strcpy(serialCommand, "");
    }
  }
}

Adafruit_NeoPixel pixels(NUM_PIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

void DisplayTargetGesture (int gesturenum) {  // Render the appropriate bitmap RGB565 array to the LCD screen for current game state

    M5.Lcd.setRotation(3);
    if (pgmMode==GAME_MODE) {
      M5.Lcd.drawBitmap(0, 0, 240, 135, (uint16_t*)GESTURE_BMP[gestureset[gesturenum]]);
      Serial.print("Game Mode > State: ");
      Serial.println(GameState);
    }
    else {
      M5.Lcd.drawBitmap(0, 0, 240, 135, (uint16_t*)GESTURE_BMP[gesturenum]);
      Serial.print("DCL/TEST Mode > State: ");
      Serial.println((uint8_t)gestureIndex);
    }
}

void DisplayLastGesture (int gesturenum) {  // Render the appropriate bitmap RGB565 array to the LCD screen for last detected gesture

    M5.Lcd.setRotation(3);
    if (pgmMode==GAME_MODE) {
      M5.Lcd.drawBitmap(0, 0, 240, 135, (uint16_t*)GESTURE_BMP[gesturenum]);
    }
}

void DisplayPixelArray (int state) {  // Convey game state progression in 7 element NeoPixel array

  pixels.clear();
  pixels.setPixelColor(0,uint32_t(state>1)*pixels.Color(0,150,0));
  pixels.setPixelColor(1,uint32_t(state>2)*pixels.Color(150,0,0));
  pixels.setPixelColor(2,uint32_t(state>3)*pixels.Color(0,0,150));
  pixels.setPixelColor(3,uint32_t(state>4)*pixels.Color(0,150,150));
  pixels.setPixelColor(4,uint32_t(state>5)*pixels.Color(150,0,150));
  pixels.setPixelColor(5,uint32_t(state>6)*pixels.Color(150,150,0));
  pixels.setPixelColor(6,uint32_t(state>7)*pixels.Color(150,150,150));
  pixels.show();
}

void RotatePixelArray() {   // Indicate "spell" gesture in progress with NeoPixel animation

  pixels.setPixelColor(0,0);
  pixels.setPixelColor(psStep-1+(psStep<2)*6,0x6F0F00);
  pixels.setPixelColor(psStep,0xFF4F00);
  pixels.setPixelColor(psStep+1-(psStep>5)*6,0x6F0F00);
  pixels.setPixelColor(psStep+2-(psStep>4)*6,0);
  pixels.setPixelColor(psStep+3-(psStep>3)*6,0);
  pixels.setPixelColor(psStep+4-(psStep>2)*6,0);
  pixels.show();
  psStep = (psStep % 6) + 1;
}

void StrobePixelArray() {  // Indicate winner with strobing cyclic hue NeoPixel animation

  psHue += 8;
  psLevel = psLevel + psStep;
  if (psLevel >254) {
    psStep = -1;
  }
  if (psLevel < 1) {
    psStep = 1;
  }
  pixels.fill(pixels.gamma32(pixels.ColorHSV(psHue, 255, psLevel)),0,7);
  pixels.show();
}

void DisplayHourglass (uint32_t timer, uint32_t timeout) {  // Convey time remaining in game using red bar graph at top of LCD

  int xval;

  xval = trunc(135*(float(timer)/float(timeout)));
  M5.Lcd.fillRect(230,0,10,xval,RED);
}

void PlayMelody(const uint8_t musicdata[][2]) {   // Play context specific audio melody for various game states

  volatile uint32_t n;
  
  n = 0;
  do {
    ledcWriteTone(ledChannel, (uint32_t)musicdata[n][0] * 50);
    delay((uint16_t)musicdata[n][1] * 10);
    n++;
  } while (musicdata[n][0]<255);
  ledcWriteTone(ledChannel, 0);
}

int ButtonPressTime(uint8_t button) {  // returns 0 if button is currently depressed, -1 if not depressed, and number of milliseconds held down if just released

  if (digitalRead(button)==LOW) {  //Button is being pressed, set state variable and counter if this just now happened
    if (btnState[button==BUTTON_B]==HIGH) {
      btnState[button==BUTTON_B] = LOW;
      btnTime[button==BUTTON_B] = millis();
    }
    return 0;
  }
  
  if ((digitalRead(button)==HIGH) && (btnState[button==BUTTON_B]==LOW)) { // Button is just now released, return total time pressed
    btnState[button==BUTTON_B] = HIGH;
    int32_t timeval = millis() - btnTime[button==BUTTON_B];
    return timeval;
  }
  
  return -1;  //  Button not depressed
}

int GetSensorDataSample(int samplenum) {  //get one sample frame from IMU FIFO and return number of bytes read

  int16_t temp;
  
  if (M5.IMU.getFIFOData(&accX,&accY,&accZ,&gyroX,&gyroY,&gyroZ,&temp) == 0){ //MPU6886 FIFO read returns accelXYZ, gyroXYZ and chip temp, we discard temp
    uint16_t offset = samplenum * CHANNELS_PER_SAMPLE * 2;
    dataPacket[offset + 0] = (char)(accX & 0xff);
    dataPacket[offset + 1] = (char)(accX >> 8 & 0xff);
    dataPacket[offset + 2] = (char)(accY & 0xff);
    dataPacket[offset + 3] = (char)(accY >> 8 & 0xff);
    dataPacket[offset + 4] = (char)(accZ & 0xff);
    dataPacket[offset + 5] = (char)(accZ >> 8 & 0xff);
    dataPacket[offset + 6] = (char)(gyroX & 0xff);
    dataPacket[offset + 7] = (char)(gyroX >> 8 & 0xff);
    dataPacket[offset + 8] = (char)(gyroY & 0xff);
    dataPacket[offset + 9] = (char)(gyroY >> 8 & 0xff);
    dataPacket[offset + 10] = (char)(gyroZ & 0xff);
    dataPacket[offset + 11] = (char)(gyroZ >> 8 & 0xff);
    char btnAState = digitalRead(BUTTON_A);
    dataPacket[offset + 12] = (char)0;
    dataPacket[offset + 13] = (char)(((gestureIndex << 2) * (btnAState == LOW))  & 0xff);
    return 14;
  }
  return 0;
}

int CheckGestureMatch() { // Collect IMU sensor raw data to SensiML Knowledge Pack ring buffer and invoke classification when button A is released

  int classresult = -1; // Variable to store knowledge pack result from SensiML Embedded SDK, default is -1 > no classification
  static int oneshot = 0; // State variable to supress redundant model processing calls per button trigger release period

  while (M5.IMU.getFIFOData(pSample) == 0) {
    pSample[6] = ((digitalRead(BUTTON_A)==LOW) << 12);
    if (pSample[6]) {   //Gesture segment trigger BUTTON_A still pressed, so just stuff sensor values in SensiML KP ring buffer
      kb_data_streaming((SENSOR_DATA_T *)pSample, CHANNELS_PER_SAMPLE, 0);
      oneshot = 0;
    }
    else if (!oneshot) {  // Upon release of Gesture segment trigger BUTTON_A, perform a one-shot SensiML KP classification
      classresult = kb_run_model((SENSOR_DATA_T *)pSample, CHANNELS_PER_SAMPLE, 0);
      oneshot = 1;
      kb_flush_model_buffer(0); // Flush model buffer for new triggered gesture
      kb_reset_model(0);
      Serial.print("SensiML Classification Result: ");
      Serial.print(classlabel[classresult]);
      Serial.print("(");
      Serial.print(classresult);
      Serial.print(")  Target: ");
      Serial.println(classlabel[classmap[GameState]]);
      return classresult;
    } //else
  } //while
  return -1;
}

void ResetGame() {

  GameState = 0;
  GameOverState = 0;
  psHue = 32768;
  psLevel = 0;
  psStep = 1;
  resetReady = false;
  DisplayTargetGesture(GameState);
  DisplayPixelArray(GameState);
  PlayMelody(readymelody);
}

void ResetDCL() {

  char ratestr[7] = "";
  IPAddress ipAddr;
  
  gestureIndex = 0;
  ssfConfig = "{\"sample_rate\":";
  itoa(int(1000 / (sampleRate + 1)), ratestr, 10);
  ssfConfig += ratestr;
  ssfConfig += ",\"version\":1,\"samples_per_packet\":";
  ssfConfig += SAMPLES_PER_PACKET;
  ssfConfig += ",\"column_location\":{\"AccelerometerX\":0,\"AccelerometerY\":1,\"AccelerometerZ\":2,\"GyroscopeX\":3,\"GyroscopeY\":4,\"GyroscopeZ\":5,\"Trigger\":6}}\r\n";
  Serial.print("DCL Mode - SSF Header:");
  Serial.println(ssfConfig);
  M5.Lcd.setRotation(3);
  DisplayTargetGesture(11); // DCL Mode 'Ready to Connect' Bitmap
  M5.Lcd.setRotation(0);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setTextColor(BLACK, WHITE);
  ipAddr = WiFi.localIP();
  M5.Lcd.setCursor(32, 115);
  M5.Lcd.print(ipAddr[0]);
  M5.Lcd.setCursor(88, 115);
  M5.Lcd.print(ipAddr[1]);
  M5.Lcd.setCursor(32, 145);
  M5.Lcd.print(ipAddr[2]);
  M5.Lcd.setCursor(88, 145);
  M5.Lcd.print(ipAddr[3]);
  PlayMelody(readymelody);
}

void ProgramSettings() { // Display game info, IP addr, and allow editing of master game timer using y-axis tilt of wand; requires press of BUTTON_A at startup

  uint8_t editParam=0;
  bool saveSettings = false;
  char ratestr[7] = "";
  uint8_t rateselect = 5;
  String SerialResponse = "";
  IPAddress ipAddr;
  uint8_t scrolltimer = 0;
  uint8_t scrollchar = 0;
  String ssidstrg = "";
  String tempstrg = "";
  
  do {} while (digitalRead(BUTTON_A) == LOW); // Await Button A release from power-on Settings mode press
  // Print local IP address and start web server
  M5.Lcd.println(APP_TITLE);
  M5.Lcd.println(APP_VERSION);
  do {
    M5.Lcd.setCursor(0, 34);
    M5.Lcd.setTextColor(WHITE,BLACK);
    M5.Lcd.print("WiFi:");
      if (WiFi.status() == WL_CONNECTED) {
        ssidstrg = WiFi.SSID();
        if (ssidstrg.length() > 15) {
          scrolltimer++;
          if (scrolltimer>8) {
            scrollchar++;
            scrolltimer = 0;
            if ((scrollchar + 15) > ssidstrg.length()) {
              scrollchar = 0;
            }
          }
          tempstrg = ssidstrg.substring(scrollchar, scrollchar + 15);
          M5.Lcd.println(tempstrg);
        }
        else {
        M5.Lcd.printf("%-15s",ssidstrg);
        M5.Lcd.println();
        }
      }
      else {
        M5.Lcd.setTextColor(RED,BLACK);
        M5.Lcd.println("Not Connected  ");
      }
    M5.Lcd.setTextColor(WHITE,BLACK);
    M5.Lcd.print("IP:");
      if (WiFi.status() == WL_CONNECTED) {
        ipAddr = WiFi.localIP();
        M5.Lcd.printf("%3d.%3d.%3d.%3d  ", ipAddr[0], ipAddr[1], ipAddr[2], ipAddr[3]);
        M5.Lcd.println();
      }
      else {
        M5.Lcd.setTextColor(RED,BLACK);
        M5.Lcd.println("Not Connected    ");
      }
    M5.Lcd.setTextColor(WHITE,BLACK*(editParam!=0)+BLUE*(editParam==0));
    M5.Lcd.print("Program Mode: ");
    switch (pgmMode) {
      case GAME_MODE:
        M5.Lcd.println("GAME  ");
        break;
      case DCL_MODE:
        M5.Lcd.println("DCL   ");
        break;
      case TEST_MODE:
        M5.Lcd.println("TEST  ");
        break;
    }
    if (pgmMode==GAME_MODE) {
      M5.Lcd.setTextColor(WHITE,BLACK*(editParam!=1)+BLUE*(editParam==1));
      M5.Lcd.printf("Game Time(sec): %3d ", int(GameOverTime / 1000.0));
      M5.Lcd.println();
    }
    else {
      M5.Lcd.setTextColor(WHITE,BLACK*(editParam!=1)+BLUE*(editParam==1));
      M5.Lcd.printf("Sample Rate(Hz):%4d", int(1000 / (sampleRate + 1)));
      M5.Lcd.println();
    }
    M5.Lcd.setTextColor(WHITE,BLACK*(editParam!=2)+BLUE*(editParam==2));
    M5.Lcd.println("Save Values & Exit  ");
    if (ButtonPressTime(BUTTON_A)>0) {
      editParam = (editParam + 1) % 3;
    }
    if (ButtonPressTime(BUTTON_B)>0) {
      switch (editParam) {
        case 0:
          pgmMode = (pgmMode + 1) % 2;  // TEST mode reserved for future revision
          break;
        case 1:
          if (pgmMode==GAME_MODE) {
            GameOverTime = (GameOverTime + 1000) % 41000;
          }
          else {
            rateselect = (rateselect+1) % 8;
            sampleRate = SAMPLE_SELECT[rateselect];
          }
          break;
        case 2:
          saveSettings = true;
          break;
      }
    }
    ProcessSerialCommand();
  } while (saveSettings==false);
}

void ServiceHttpRequests(WiFiClient c) {

  String httpResponse = "";
  
  httpTime = millis();  
  while (c && !isStreaming) {
    if (c.connected() && ((millis() - httpTime) <= TIMEOUT_TIME)) {
      httpTime = millis();
      if (c.available()) {
        char ch = c.read();             // read a byte, then
        Serial.write(ch);                    // print it out the serial monitor
        header += ch;
        if (ch == '\n') {
          crcount++;
        }
        else {
          if (ch != '\r') {
            crcount = 0;
          }
        }
        if (crcount>1) {
          crcount = 0;

          if (header.indexOf("GET /gamestatus") >= 0) {
            if (pgmMode==GAME_MODE) {
              if ((GameState>0) && (GameState<8)) {
                secsRemaining = uint32_t((GameOverTime-(millis()-StartTime))/1000);
              }
              else if (GameState==0) {
                secsRemaining = int(GameOverTime/1000);
              }
              httpResponse = "HTTP/1.1 200 OK\r\nContent-type:text/json\r\n\r\n{\"game_state\":";
              httpResponse += GameState + GameOverState;  // for GameState values 9-16 correspond to Lost and State-9 is highest level achieved
              httpResponse += ",\"secs_remaining\":";
              httpResponse += secsRemaining;
              httpResponse += ",\"last_classification\":";
              httpResponse += lastgestureresult;
              httpResponse += "}";
            }
            else {
              httpResponse = "HTTP/1.1 409 Conflict: Device not in GAME Mode\r\nContent-type:none\r\n";
            }
          }   // if "GET /gamestatus"
        
          if (header.indexOf("GET /config") >= 0) {
            if ((pgmMode==DCL_MODE) || (pgmMode==TEST_MODE)) {
            httpResponse = "HTTP/1.1 200 OK\r\nContent-type:text/json\r\n\r\n";
            httpResponse += ssfConfig;
            }
            else {
            httpResponse = "HTTP/1.1 409 Conflict: Device not in DCL or TEST Mode\r\nContent-type:none\r\n";
            }
          }  // if "GET /config"
          
          if (header.indexOf("GET /stream") >= 0) {
              if ((pgmMode==DCL_MODE) || (pgmMode==TEST_MODE)) {
              isStreaming = true;
              M5.IMU.enableFIFO((MPU6886::Fodr)sampleRate);   //Start IMU FIFO mode
              DisplayTargetGesture(++gestureIndex);
              httpResponse = "HTTP/1.1 200 OK\r\nContent-type:application/octet-stream\r\n";
              }
              else {
              httpResponse = "HTTP/1.1 409 Conflict: Device not in DCL or TEST Mode\r\nContent-type:none\r\n";
              }
          }  // if "GET /stream"

          if (header.indexOf("GET /reset") >= 0) {
              if (pgmMode==GAME_MODE) {
              ResetGame();
              httpResponse = "HTTP/1.1 200 OK\r\nContent-type:none\r\n";
              }
              else {
              httpResponse = "HTTP/1.1 409 Conflict: Device not in GAME Mode\r\nContent-type:none\r\n";
              }
          }  // if "GET /reset"
        
          if (header.indexOf("GET /disconnect") >= 0) {
              if ((pgmMode==DCL_MODE) || (pgmMode==TEST_MODE)) {
              isStreaming = true;
              httpResponse = "HTTP/1.1 200 OK\r\nContent-type:application/octet-stream\r\n\r\n";
              }
              else {
              httpResponse = "HTTP/1.1 409 Conflict: Device not in DCL or TEST Mode\r\nContent-type:none\r\n";
              }
          }  // if "GET /disconnect"

          if (header.indexOf("GET /set-game-mode") >= 0) {
              if (pgmMode!=GAME_MODE) {
                pgmMode = GAME_MODE;
                EEPROM.write(101,pgmMode);
                EEPROM.commit();
                ResetGame();
              }
              httpResponse = "HTTP/1.1 200 OK\r\nContent-type:none\r\n";
          }  // if "GET /set-game-mode"

          if (header.indexOf("GET /set-dcl-mode") >= 0) {
              if (pgmMode!=DCL_MODE) {
                pgmMode = DCL_MODE;
                EEPROM.write(101,pgmMode);
                EEPROM.commit();
                ResetDCL();
              }
              httpResponse = "HTTP/1.1 200 OK\r\nContent-type:none\r\n";
          }  // if "GET /set-dcl-mode"
   
        
          if (httpResponse.length()>0) {
              c.println(httpResponse);
              Serial.println(httpResponse);
          }
          header = "";
          if (!isStreaming) {
            c.stop();
          }
        }
      }
    } 
  }
}

void setup() {

  int i = 0;  // timeout variable for connectivity

  M5.begin();
  M5.IMU.Init();
  EEPROM.begin(128);
  Serial.begin(500000);
  Serial.println();
  Serial.print(APP_TITLE);
  Serial.print(" ");
  Serial.println(APP_VERSION);
  pinMode(BUTTON_A, INPUT_PULLUP);
  pinMode(BUTTON_B, INPUT_PULLUP);

  //Initialize LCD Screen
  M5.Lcd.setRotation(3);
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(0, 0);

  // Configure audio speaker
  ledcSetup(ledChannel, freq, resolution);
  ledcAttachPin(BUZZER_PIN, ledChannel);
  ledcWrite(ledChannel, 0);

  // Initialize NeoPixel array on pin G33
  pixels.begin();
  pixels.clear();

  // Connect to Wi-Fi network with SSID and password
  if (GetWiFiCreds(wifiSSID, wifiPassword)) {
    Serial.print("Connecting to ");
    Serial.println(wifiSSID);
    WiFi.begin(wifiSSID, wifiPassword);
    M5.Lcd.println("Attempting WiFi Connection");
    while ((WiFi.status() != WL_CONNECTED) && (i<60)) {
      delay(500);
      i++;
      M5.Lcd.print(".");
    }
    if (WiFi.status() == WL_CONNECTED) {
      Serial.print("IP:");
      Serial.println(WiFi.localIP());
      server.begin();
    }
  }

  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setCursor(0, 0);
  if ((digitalRead(BUTTON_A) == LOW) || (WiFi.status() != WL_CONNECTED)) {  // ButtonA has been pressed on power up, or WiFi failed => Go to Game Settings UI
    ProgramSettings();
  }
  if (pgmMode==GAME_MODE) {
    ResetGame();
  }
  else {
    ResetDCL();
  }
  kb_model_init();
}


void loop(){
  while (pgmMode==GAME_MODE) {
    client = server.available();   // Listen for incoming clients
    if (client) {
      ServiceHttpRequests(client);
    }
    
    if ((GameState > 0) && (GameState < 8)) { // Active timed game play states
      gestureresult =  CheckGestureMatch();
      if (gestureresult!=-1) {
        lastgestureresult = gestureresult;
        lgrstickytimer = millis();
        if (gestureresult!=(classmap[GameState])) {
          switch (lastgestureresult) {   
            case 0:
            case 7:
              DisplayLastGesture(7); // Display Unknown gesture bitmap
              break;
            case 1: 
            case 2: 
            case 3: 
            case 4: 
            case 5: 
            case 6:
              DisplayLastGesture(8);  // Display wrong gesture bitmap
              M5.Lcd.setRotation(0);
              M5.Lcd.setTextSize(4);
              M5.Lcd.setTextColor(BLACK, WHITE);            
              M5.Lcd.setCursor(90, 100);
              M5.Lcd.print(classlabel[lastgestureresult]);
          }
        }
      }
      else if ((lastgestureresult!=-1) && (((millis()-lgrstickytimer) > LAST_GESTURE_HOLD_MSEC) || (digitalRead(BUTTON_A)==LOW))) {
        lastgestureresult=-1;
        DisplayTargetGesture(GameState);
      }
      if (gestureresult==(classmap[GameState])) {
        GameState++;
        DisplayTargetGesture(GameState);
        DisplayPixelArray(GameState);
        if (GameState == 8) { // player has made it through all gestures, declare winner
          M5.IMU.disableFIFO();
          PlayMelody(winnermelody);
        }  
        else {
          if (GameState > 0) {
            PlayMelody(matchmelody);
          }
          M5.IMU.resetFIFO(); //Flush IMU FIFO to avoid sync issues with BUTTON_A trigger channel
        }
      }

      PSDelay++;
      if (PSDelay>60) {
        PSDelay = 0;
        DisplayHourglass ((millis()-StartTime),GameOverTime);
        if (digitalRead(BUTTON_A) == LOW) {
          RotatePixelArray();
        }
        else {
          DisplayPixelArray(GameState);
        }
      }
      if (((millis()-StartTime) > GameOverTime) && (GameState < 8)) {  // If in the middle of game play and timer expires, declare loser
        DisplayPixelArray(GameState);
        GameOverState = GameState;
        GameState = 9;
        M5.IMU.disableFIFO();
        DisplayTargetGesture(GameState);
        PlayMelody(losermelody);
      }
    }   
  
    if (GameState > 7) {  // If game is over, check for long press on Button A signaling game reset
      int32_t APressed = ButtonPressTime(BUTTON_A);
      if ((APressed == -1) && !resetReady){  // First user must release Button A from game play before checking for long press reset
        resetReady = true;
      }
      if ((APressed > 1500) && resetReady) {  // Long press on game reset button => Reset game
        ResetGame();
      }
    }
          
    if ((digitalRead(BUTTON_A) == LOW) && (GameState == 0)) { // Press on game button trigger button in GameState 0 => Start timer and game play
        GameState++;
        DisplayTargetGesture(GameState);
        DisplayPixelArray(GameState);
        PlayMelody(gomelody);
        StartTime = millis();
        M5.IMU.enableFIFO((MPU6886::Fodr)sampleRate);   //Start IMU FIFO mode
  
    }
    
    if (GameState == 8) {
      StrobePixelArray(); // Continue to strobe pixel array for winner until game is reset
    }
  } // while GAME_MODE

  while (pgmMode==DCL_MODE) {
    if (isStreaming && client.connected()) {
      if (ButtonPressTime(BUTTON_B)>0) {
        gestureIndex++;
        if (gestureIndex>NUM_GESTURES) {
          gestureIndex = 1;
        }
        DisplayTargetGesture (gestureIndex);
      }
      if (GetSensorDataSample(sampleIndex) > 0) {
        sampleIndex++;
        if (sampleIndex >= SAMPLES_PER_PACKET) {
          client.write(dataPacket, PACKET_SIZE * 2);
          sampleIndex = 0;
          packetIndex++;
        }
      }  
    } //if isStreaming
    else {
      if (isStreaming) {
        M5.IMU.disableFIFO();
        isStreaming = false;
        ResetDCL();
      }
      client = server.available();   // Listen for incoming clients
      if (client) {
        ServiceHttpRequests(client);
      }
    }
  } // while DCL_MODE
}
