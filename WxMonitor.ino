/**
   WxMon - Weather Monitor

  Copyright 2017 Costin STROIE <costinstroie@eridu.eu.org>

  This file is part of Weather Station.

  WxMon is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by the Free
  Software Foundation, either version 3 of the License, or (at your option) any
  later version.

  WxMon is distributed in the hope that it will be useful, but
  WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
  more details.

  You should have received a copy of the GNU General Public License along with
  WxMon.  If not, see <http://www.gnu.org/licenses/>.
*/

// The DEBUG flag
//#define DEBUG

// LCD: use the HD447890 library and Wire i2c library
#include <Wire.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h> // include i/o class header

// WiFi
#include <ESP8266WiFi.h>
#include <WiFiManager.h>
#include <WiFiUdp.h>

// NTP
#include <NTPClient.h>

// Timer
#include <AsyncDelay.h>

// MQTT
#include <PubSubClient.h>

// DHT
#include "DHT.h"

// RC Switch
#include <RCSwitch.h>


// Device name
#if defined(DEBUG)
String NODENAME = "DevNode";
String LC_NODENAME = "devnode";
#else
String NODENAME = "WxMon";
String LC_NODENAME = "wxmon";  // FIXME DNRY
#endif

// LCD
#define WIRECLOCK     400000L   // tell hd44780 to use 400kHz i2c clock rate
hd44780_I2Cexp        lcd;      // auto locate and autoconfig interface pins
#define HD44780_LCDOBJECT       // tell the hd44780 sketch the lcd object has been declared
const int PIR_PIN = D5;
const int LCD_INTERVAL = 4 * 1000;
const int PIR_INTERVAL = 300 * 1000;
enum LCD_SCREENS {SCR_CLK, SCR_WIFI, SCR_TEMP, SCR_HMDT, SCR_OTP, SCR_OHM, SCR_ODP, SCR_OPS, SCR_NOW, SCR_TOD, SCR_TON, SCR_TOM, SCR_DAT, SCR_BAR, SCR_SUN, SCR_MON};
enum LCD_CHARS {LCD_LOGO, LCD_NUM, LCD_MOON};

AsyncDelay delayLCD;
AsyncDelay delayPIR;
int lcdIndex = 0;
int lcdChars;

// Array of 8 logo characters defined column-wise
const uint8_t LCD_BGLOGO[8 * 8] PROGMEM = {
  B11011, B00000, B10001, B00000, B00000, B01100, B01100, B00000,
  B11011, B00000, B11011, B00000, B00000, B01100, B01100, B00000,
  B11011, B11011, B11111, B01110, B11110, B00000, B11110, B01011,
  B11011, B11011, B11111, B11111, B11111, B01100, B01100, B01111,
  B11111, B01110, B11011, B11011, B11011, B01100, B01100, B01100,
  B11111, B01110, B11011, B11011, B11011, B01100, B01100, B01100,
  B01110, B11011, B11011, B11111, B11011, B01110, B01110, B01100,
  B01010, B11011, B11011, B01110, B11011, B00110, B00110, B01100,
};

// Array of 8 characters defined column-wise
const uint8_t LCD_BGNUM_SHAPES[8 * 8] PROGMEM = {
  B11111, B00000, B11111, B11111, B00000, B00000, B00000, B00000,
  B11111, B00000, B11111, B11111, B00000, B00000, B00000, B00000,
  B11111, B00000, B00000, B00000, B00000, B00001, B10000, B00000,
  B00000, B00000, B00000, B00000, B00000, B00011, B11000, B00000,
  B00000, B00000, B00000, B00000, B00000, B00011, B11000, B00000,
  B00000, B11111, B00000, B00000, B00000, B00001, B10000, B00000,
  B00000, B11111, B11111, B00000, B11111, B00000, B00000, B00000,
  B00000, B11111, B11111, B00000, B11111, B00000, B00000, B00000,
};

// NTP
const int  TZ = 2;
const char NTP_SERVER[] = "europe.pool.ntp.org";
const int  NTP_INTERVAL = 765 * 1000;
WiFiUDP ntpUDP;
NTPClient NTP_Client(ntpUDP, NTP_SERVER, 3600 * TZ, NTP_INTERVAL);

// Wx
String WX_STATION = "ROXX0003";
String strReports = "now tod ton tom dat sun mon bar ";
String wxReport[8][2];
String strLnSep = ", ";

// Sensors
String strSensors = "itp ihm otp ohm odp ops ";
int snsReport[6][2];
const int SNS_INTERVAL = 600 * 1000;

// MQTT parameters
#if defined(DEBUG)
const char MQTT_ID[] = "devnode-eridu-eu-org";
#else
const char MQTT_ID[] = "wxmon-eridu-eu-org";
#endif
const char MQTT_SERVER[] = "eridu.eu.org";
const int  MQTT_PORT = 1883;
const int  MQTT_INTERVAL = 5000;
String MQTT_WX   = String("wx/" + WX_STATION + "/");
String MQTT_CMD  = "command/" + LC_NODENAME + "/";
String MQTT_RCS  = "command/rcs/";
String MQTT_REPORT = "report/" + LC_NODENAME;
String MQTT_REPORT_WIFI = MQTT_REPORT + "/wifi";
String MQTT_SENSOR = "sensor/indoor"; // + LC_NODENAME;
WiFiClient WiFi_Client;
PubSubClient MQTT_Client(WiFi_Client);
AsyncDelay delayMQTT;

// DHT
const int DHT_TYPE      = DHT11;
const int DHT_PIN       = D4;
const int DHT_INTERVAL  = 60 * 1000;
DHT dht(DHT_PIN, DHT_TYPE);
float dhtTemp, dhtHmdt;
bool dhtValid = false;
AsyncDelay delayDHT;

// RC Switch
const int RCS_PIN = D3;
const char rcsHomeCode[] = "11111";
RCSwitch rcs = RCSwitch();

// Beep
const int BEEP_PIN = D10;

// Voltage
ADC_MODE(ADC_VCC);

// FIXME Character transformation
const char chrUtfDeg[] = {194, 176, 0};
const char chrWinDeg[] = {176, 0};
const char chrLcdDeg[] = {223, 0};
const String strUtfDeg = chrUtfDeg;
const String strWinDeg = chrWinDeg;
const String strLcdDeg = chrLcdDeg;

/**
  LCD initialization
*/
void lcdInit() {
  lcd.begin(16, 2);
  lcd.setBacklight(HIGH);
  lcdLogo();
}

/**
  LCD display the logo
*/
void lcdLogo() {
  byte text[10] = {0, 1, 32, 2, 3, 4, 5, 6, 3, 7}; // "Wx Monitor"
  lcdDefChars(LCD_LOGO);
  //Serial.println(text);
  lcd.clear();
  lcd.setCursor(3, 0);
  for (byte item = 0; item < sizeof(text); item++) {
    lcd.write(text[item]);
  }
}

/**
  LCD Create the custom characters: 8 characters defined column-wise
*/
void lcdDefBig(const uint8_t chars[]) {
  for (uint8_t i = 0; i < 8; i++) {
    uint8_t charMap[8];
    for (uint8_t j = 0; j < 8; j++) {
      charMap[j] = pgm_read_byte(chars + (j * 8) + i);
    }
    lcd.createChar(i, charMap);
  }
}

/**
  LCD Define the custom charaters based on the specified parameter
*/
void lcdDefChars(int lcdCharsType) {
  switch (lcdCharsType) {
    case LCD_LOGO:
      lcdDefBig(LCD_BGLOGO);
      lcdChars = lcdCharsType;
      break;
    case LCD_NUM:
      lcdDefBig(LCD_BGNUM_SHAPES);
      lcdChars = lcdCharsType;
      break;
  }
}

/**
  Copy the source array to destination array and return the length

  @param dst the destination array
  @param src the source array
  @param len the length of the source array
  @return the number of columns (half the length of the source array)
*/
byte copyArray(byte dst[], byte src[], byte len) {
  memcpy(dst, src, len);
  return len >> 1;
}

/**
  LCD Construct the big characters (on 2 lines)

  @param chr the character to define
  @param charShapes array of needed character shapes
  @return the number of display columns the character uses
*/
byte lcdBigConstruct(char chr, byte charShapes[]) {
  byte charCols = 0;
  switch (chr) {
    case '0': {
        byte tmpShapes[] = {255, 3, 255, 255, 4, 255};
        charCols = copyArray(charShapes, tmpShapes, sizeof(tmpShapes));
      }
      break;
    case '1': {
        byte tmpShapes[] = {0, 255, 32, 1, 255, 1};
        charCols = copyArray(charShapes, tmpShapes, sizeof(tmpShapes));
      }
      break;
    case '2': {
        byte tmpShapes[] = {0, 2, 255, 255, 4, 1};
        charCols = copyArray(charShapes, tmpShapes, sizeof(tmpShapes));
      }
      break;
    case '3': {
        byte tmpShapes[] = {0, 2, 255, 1, 4, 255};
        charCols = copyArray(charShapes, tmpShapes, sizeof(tmpShapes));
      }
      break;
    case '4': {
        byte tmpShapes[] = {255, 4, 255, 32, 32, 255};
        charCols = copyArray(charShapes, tmpShapes, sizeof(tmpShapes));
      }
      break;
    case '5': {
        byte tmpShapes[] = {255, 2, 0, 1, 4, 255};
        charCols = copyArray(charShapes, tmpShapes, sizeof(tmpShapes));
      }
      break;
    case '6': {
        byte tmpShapes[] = {255, 2, 0, 255, 4, 255};
        charCols = copyArray(charShapes, tmpShapes, sizeof(tmpShapes));
      }
      break;
    case '7': {
        byte tmpShapes[] = {0, 3, 255, 32, 32, 255};
        charCols = copyArray(charShapes, tmpShapes, sizeof(tmpShapes));
      }
      break;
    case '8': {
        byte tmpShapes[] = {255, 2, 255, 255, 4, 255};
        charCols = copyArray(charShapes, tmpShapes, sizeof(tmpShapes));
      }
      break;
    case '9': {
        byte tmpShapes[] = {255, 2, 255, 1, 4, 255};
        charCols = copyArray(charShapes, tmpShapes, sizeof(tmpShapes));
      }
      break;
    case 'C': {
        byte tmpShapes[] = {255, 3, 0, 255, 4, 1};
        charCols = copyArray(charShapes, tmpShapes, sizeof(tmpShapes));
      }
      break;
    case '-': {
        byte tmpShapes[] = {32, 1, 1, 32, 32, 32};
        charCols = copyArray(charShapes, tmpShapes, sizeof(tmpShapes));
      }
      break;
    case ' ': {
        byte tmpShapes[] = {32, 32, 32, 32, 32, 32};
        charCols = copyArray(charShapes, tmpShapes, sizeof(tmpShapes));
      }
      break;
    case ':': {
        byte tmpShapes[] = {5, 6, 5, 6};
        charCols = copyArray(charShapes, tmpShapes, sizeof(tmpShapes));
      }
      break;
    case '.': {
        byte tmpShapes[] = {32, 32, 5, 6};
        charCols = copyArray(charShapes, tmpShapes, sizeof(tmpShapes));
      }
      break;
    case '\'': {
        byte tmpShapes[] = {5, 6, 32, 32};
        charCols = copyArray(charShapes, tmpShapes, sizeof(tmpShapes));
      }
      break;
    case '%': {
        byte tmpShapes[] = {5, 6, 1, 0, 1, 0, 5, 6};
        charCols = copyArray(charShapes, tmpShapes, sizeof(tmpShapes));
      }
      break;
  }
  return charCols;
}

/**
  LCD Write one big custom charater (on 2 lines)

  @param chr the character to write
  @param col the column to write the character
*/
void lcdBigWrite(char chr, byte col) {
  byte charShapes[10];
  byte charCols = lcdBigConstruct(chr, charShapes);
  for (byte line = 0; line < 2; line++) {
    lcd.setCursor(col, line);
    for (byte item = line * charCols; item < charCols * (line + 1); item++) {
      lcd.write(byte(charShapes[item]));
    }
  }
}

/**
  LCD Write big custom charaters

  @param text the text to write
  @param cols the columns to write each character
  @param type the character type to use
*/
void lcdBigPrint(const char* text, const byte * cols, int type) {
  if (lcdChars != type) {
    lcdDefChars(type);
  }
  for (int i = 0; i <= sizeof(text); i++) {
    lcdBigWrite(text[i], cols[i]);
  }
}

/**
  LCD Display the current time with big characters
*/
bool lcdShowTime() {
  unsigned long rawTime = NTP_Client.getEpochTime();
  unsigned long hours = (rawTime % 86400L) / 3600;
  unsigned long minutes = (rawTime % 3600) / 60;
  char text[6] = "";
  sprintf(text, "%02d:%02d", hours, minutes);
  byte cols[] = {0, 4, 7, 9, 13};
#if defined(DEBUG)
  Serial.print("SCR_CLK ");
  Serial.println(text);
#endif
  lcd.clear();
  lcdBigPrint(text, cols, LCD_NUM);
  return true;
}

/**
  LCD Display the sensor temperature with big characters
*/
bool lcdShowTemp() {
  if (dhtValid) {
    char text[6] = "";
    sprintf(text, "% d'C", (int)dhtTemp);
    byte cols[] = {0, 4, 8, 11, 13};
#if defined(DEBUG)
    Serial.print("SCR_TEMP ");
    Serial.println(text);
#endif
    lcd.clear();
    lcdBigPrint(text, cols, LCD_NUM);
  }
  return dhtValid;
}

/**
  LCD Display the sensor humidity with big characters
*/
bool lcdShowHmdt() {
  if (dhtValid) {
    char text[6] = "";
    sprintf(text, "% d%%", (int)dhtHmdt);
    byte cols[] = {0, 4, 8, 12};
#if defined(DEBUG)
    Serial.print("SCR_HMDT ");
    Serial.println(text);
#endif
    lcd.clear();
    lcdBigPrint(text, cols, LCD_NUM);
  }
  return dhtValid;
}

/**
  LCD Display the WiFi parameters

  @param serial print on serial port too
*/
bool lcdShowWiFi(bool serial) {
  if (WiFi.isConnected()) {
    if (serial == true) {
      String text = "\nWiFi connected to ";
      text += WiFi.SSID();
      text += " on channel ";
      text += WiFi.channel();
      text += ", RSSI ";
      text += WiFi.RSSI();
      text += " dBm.";
      text += "\n IP : ";
      text += WiFi.localIP().toString();
      text += "\n GW : ";
      text += WiFi.gatewayIP().toString();
      text += "\n DNS: ";
      text += WiFi.dnsIP().toString();
      text += "\n\n";
      Serial.print(text);
    }
    lcd.setCursor(0, 0);
    lcd.printf("WiFi % 11s", WiFi.SSID().c_str());
    lcd.setCursor(0, 1);
    lcd.printf("% 16s", WiFi.localIP().toString().c_str());
    return true;
  }
  else {
    return false;
  }
}

/**
  LCD Display the weather report

  @param report the report to display
*/
bool lcdShowWeather(char* report) {
  String tplUpLn, tplLwLn;
  int idxReport = strReports.indexOf(String(report) + " ");
  if (idxReport != -1) {
    idxReport /= 4;
    if        (wxReport[idxReport][0] != "") {
      if (report == "now") {
        tplUpLn = "Now% 13s";
        tplLwLn = "";
      }
      else if (report == "tod") {
        tplUpLn = "Today  % 9s";
        tplLwLn = "";
      }
      else if (report == "ton") {
        tplUpLn = "Tonight% 9s";
        tplLwLn = "";
      }
      else if (report == "tom") {
        tplUpLn = "Tmrrow % 9s";
        tplLwLn = "";
      }
      else if (report == "dat") {
        tplUpLn = "AfTmrw % 9s";
        tplLwLn = "";
      }
      else if (report == "sun") {
        tplUpLn = "Sunrise % 8s";
        tplLwLn = "Sunset  % 8s";
      }
      else if (report == "mon") {
        tplUpLn = "Moon % 11s";
        tplLwLn = "";
      }
      else if (report == "bar") {
        tplUpLn = "Baro% 12s";
        tplLwLn = "";
      }
      // Clear the screen
      lcd.clear();
      // Print the upper line
      lcd.setCursor(0, 0);
      if (tplUpLn != "") {
        lcd.printf(tplUpLn.c_str(), wxReport[idxReport][0].c_str());
      }
      else {
        lcd.print(wxReport[idxReport][0].c_str());
      }
      //Serial.println("LN 1: " + wxReport[idxReport][0]);
      // Print the lower line
      lcd.setCursor(0, 1);
      if (tplLwLn != "") {
        lcd.printf(tplLwLn.c_str(), wxReport[idxReport][1].c_str());
      }
      else {
        lcd.print(wxReport[idxReport][1].c_str());
      }
      //Serial.println("LN 2: " + wxReport[idxReport][1]);
      return true;
    }
  }
  return false;
}

/**
  LCD Display the sensors

  @param sensor the sensor to display
*/
bool lcdShowSensor(char* sensor) {
  char text[6] = "";
  int idxReport = strSensors.indexOf(String(sensor) + " ");
  if (idxReport != -1) {
    idxReport /= 4;
    if (snsReport[idxReport][1] > 0) {
      if      (sensor == "otp") {
        sprintf(text, "% d'C", snsReport[idxReport][0]);
        byte cols[] = {0, 4, 8, 11, 13};
#if defined(DEBUG)
        Serial.print("SCR_OTP ");
        Serial.println(text);
#endif
        lcd.clear();
        lcdBigPrint(text, cols, LCD_NUM);
      }
      else if (sensor == "ohm") {
        sprintf(text, "% d%%", snsReport[idxReport][0]);
        byte cols[] = {0, 4, 8, 12};
#if defined(DEBUG)
        Serial.print("SCR_OHM ");
        Serial.println(text);
#endif
        lcd.clear();
        lcdBigPrint(text, cols, LCD_NUM);
      }
      else if (sensor == "odp") {
        sprintf(text, "% d'C", snsReport[idxReport][0]);
        byte cols[] = {0, 4, 8, 11, 13};
#if defined(DEBUG)
        Serial.print("SCR_ODP ");
        Serial.println(text);
#endif
        lcd.clear();
        lcdBigPrint(text, cols, LCD_NUM);
      }
      else if (sensor == "ops") {
        sprintf(text, "% d", snsReport[idxReport][0]);
        byte cols[] = {0, 4, 8, 12};
#if defined(DEBUG)
        Serial.print("SCR_OPS ");
        Serial.println(text);
#endif
        lcd.clear();
        lcdBigPrint(text, cols, LCD_NUM);
      }
      if (snsReport[idxReport][1] + SNS_INTERVAL < millis()) {
        snsClear(sensor);
      }
      return true;
    }
  }
  return false;
}

/**
  LCD Clear the display then print the two lines

  @param upLine text to print on upper line
  @param lwLine text to print on lower line
*/
void lcdScreen(char* upLine, char* lwLine) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(upLine);
  lcd.setCursor(0, 1);
  lcd.print(lwLine);
}

/**
  Try to display a screen identified by index; if we succeded,
  recommend the next screen, else recommend the default (0).
  If default has been displayed, return an invalid value (-1).

  @param index the screen index to display
  @return recommended next index if the required screen has succeeded,
          -1 if the default screen has been selected,
          0 if the screen failed
*/
int lcdShowScreen(int index) {
  bool result;
#if defined(DEBUG)
  Serial.print(F("SCR_IDX "));
  Serial.println(index);
#endif
  switch (index) {
    case SCR_WIFI:
      result = lcdShowWiFi(false);
      break;
    case SCR_TEMP:
      result = lcdShowTemp();
      break;
    case SCR_HMDT:
      result = lcdShowHmdt();
      break;
    case SCR_OTP:
      result = lcdShowSensor("otp");
      break;
    case SCR_OHM:
      result = lcdShowSensor("ohm");
      break;
    case SCR_ODP:
      result = lcdShowSensor("odp");
      break;
    case SCR_OPS:
      result = lcdShowSensor("ops");
      break;
    case SCR_NOW:
      result = lcdShowWeather("now");
      break;
    case SCR_TOD:
      result = lcdShowWeather("tod");
      break;
    case SCR_TON:
      result = lcdShowWeather("ton");
      break;
    case SCR_TOM:
      result = lcdShowWeather("tom");
      break;
    case SCR_DAT:
      result = lcdShowWeather("dat");
      break;
    case SCR_BAR:
      result = lcdShowWeather("bar");
      break;
    case SCR_SUN:
      result = lcdShowWeather("sun");
      break;
    case SCR_MON:
      result = lcdShowWeather("mon");
      break;
    default:
      result = lcdShowTime();
      return -1;
  }
  if (result == true) {
    return index + 1;
  }
  else {
    return 0;
  }
}

/**
  Select the screen to display, in case of error, try the next screen.
*/
void lcdRotateScreens() {
  int nextIndex;
  do {
    nextIndex = lcdShowScreen(lcdIndex);
    if (nextIndex == -1) {
      lcdIndex = 1;
      nextIndex = 1;
    }
    else if (nextIndex == 0) {
      lcdIndex += 1;
    }
    else {
      lcdIndex = nextIndex;
    }
#if defined(DEBUG)
    Serial.print(F("SCR_NXT "));
    Serial.println(nextIndex);
#endif
  } while (nextIndex <= 0);
}

/**
  Try to reconnect to MQTT server

  @return boolean reconnection success
*/
boolean mqttReconnect() {
  Serial.println(F("MQTT connecting..."));
  if (MQTT_Client.connect(MQTT_ID)) {
    // Publish the connection report
    MQTT_Client.publish(String(MQTT_REPORT_WIFI + "/hostname").c_str(), WiFi.hostname().c_str(), true);
    MQTT_Client.publish(String(MQTT_REPORT_WIFI + "/mac").c_str(), WiFi.macAddress().c_str(), true);
    MQTT_Client.publish(String(MQTT_REPORT_WIFI + "/ssid").c_str(), WiFi.SSID().c_str(), true);
    MQTT_Client.publish(String(MQTT_REPORT_WIFI + "/rssi").c_str(), String(WiFi.RSSI()).c_str(), true);
    MQTT_Client.publish(String(MQTT_REPORT_WIFI + "/ip").c_str(), WiFi.localIP().toString().c_str(), true);
    MQTT_Client.publish(String(MQTT_REPORT_WIFI + "/gw").c_str(), WiFi.gatewayIP().toString().c_str(), true);
    // Subscribe
    MQTT_Client.subscribe(String(MQTT_CMD + "#").c_str());
    MQTT_Client.subscribe(String(MQTT_RCS + "#").c_str());
    MQTT_Client.subscribe(String(MQTT_WX  + "#").c_str());
    // TODO
    MQTT_Client.subscribe("sensor/#");
    Serial.print(F("MQTT connected to "));
    Serial.println(MQTT_SERVER);
  }
  return MQTT_Client.connected();
}

/**
  Message arrived in MQTT subscribed topics

  @param topic the topic the message arrived on (const char[])
  @param payload the message payload (byte array)
  @param length the length of the message payload (unsigned int)
*/
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  // Make a copy
  char message[100];
  if (length > 100) length = 100;
  memcpy(message, payload, length);
  message[length] = '\0';

  // Create string objects
  String strTopic = String(topic);
  String strMessage = String(message);
  Serial.println("MQTT " + strTopic + ": " + strMessage);

  // Decompose the topic
  String strRoot, strTrunk, strBranch;
  int idxSepOne = strTopic.indexOf('/');
  if (idxSepOne != -1) {
    strRoot = strTopic.substring(0, idxSepOne);
#if defined(DEBUG)
    Serial.println("MQTT_ROOT " + strRoot);
#endif
    int idxSepTwo = strTopic.indexOf('/', idxSepOne + 1);
    if (idxSepTwo != -1) {
      strTrunk = strTopic.substring(idxSepOne + 1, idxSepTwo);
#if defined(DEBUG)
      Serial.println("MQTT_TRNK " + strTrunk);
#endif
      strBranch = strTopic.substring(idxSepTwo + 1);
#if defined(DEBUG)
      Serial.println("MQTT_BRNC " + strBranch);
#endif

      // Dispatcher
      if (strRoot == "wx" && strTrunk == WX_STATION) {
        wxProcess(strBranch, strMessage);
      }
      else if (strRoot == "sensor") {
        snsProcess(strTopic.substring(idxSepOne + 1), strMessage);
      }
      else if (strRoot == "command") {
        if (strTrunk == "rcs") {
          rcsProcess(strBranch, strMessage);
        }
        else if (strTrunk == LC_NODENAME) {
          if (strBranch == "restart") {
            ESP.restart();
          }
          else if (strBranch == "beep") {
            tone(BEEP_PIN, 2000, 500);
          }
          else if (strBranch == "light") {
            if (strMessage == "on") {
              lcd.backlight();
            }
            else if (strMessage == "off") {
              lcd.noBacklight();
            }
          }
        }
      }
    }
  }
}

/**
  Process the MQTT weather topics and messages and create weather reports

  @param strBranch the MQTT branch topic
  @param strMessage the MQTT message
*/
void wxProcess(String strBranch, String strMessage) {
  // Check the report index
  int idxReport = strReports.indexOf(strBranch + " ");
  if (idxReport != -1) {
    idxReport /= 4;
    // Convert from UTF-8 or CP1252 to LCD charset
    strMessage.replace(strUtfDeg, strLcdDeg);
    strMessage.replace(strWinDeg, strLcdDeg);
    // Check the line separator
    int idxLnSep = strMessage.indexOf(strLnSep);
    if (idxLnSep != -1) {
      // Store the two-line report
      wxReport[idxReport][0] = strMessage.substring(0, idxLnSep);
      wxReport[idxReport][1] = strMessage.substring(idxLnSep + 2);
      // The 'tod' and 'ton' reports are mutually exclusive
      if      (strBranch == "tod") wxClear("ton");
      else if (strBranch == "ton") wxClear("tod");
    }
  }
}

/**
  Clear a weather report

  @param report the report to be clean
*/
void wxClear(String report) {
  int idxReport = strReports.indexOf(report + " ");
  if (idxReport != -1) {
    idxReport /= 4;
    wxReport[idxReport][0] = "";
    wxReport[idxReport][1] = "";
  }
}

/**
  Process the sensor topics and messages and create sensor reports

  @param strSensor the sensor name
  @param strMessage the sensor value
*/
void snsProcess(String strSensor, String strMessage) {
  if      (strSensor == "outdoor/temperature") {
    snsUpdate("otp", strMessage);
  }
  else if (strSensor == "outdoor/humidity") {
    snsUpdate("ohm", strMessage);
  }
  else if (strSensor == "outdoor/dewpoint") {
    snsUpdate("odp", strMessage);
  }
  else if (strSensor == "outdoor/pressure") {
    snsUpdate("ops", strMessage);
  }
}
/**
  Update a sensor

  @param snsName the sensor name
  @param strMessage the sensor value
*/
void snsUpdate(String snsName, String strMessage) {
  if (snsName != "") {
    // Check the report index
    int idxReport = strSensors.indexOf(snsName + " ");
    if (idxReport != -1) {
      idxReport /= 4;
      snsReport[idxReport][0] = strMessage.toInt();
      snsReport[idxReport][1] = millis() / 1000;
#if defined(DEBUG)
      Serial.println("SNS_NAM " + snsName);
      Serial.println("SNS_VAL " + String(snsReport[idxReport][0]));
#endif
    }
  }
}

/**
  Clear a sensor

  @param sensor the sensor to be clean
*/
void snsClear(String snsName) {
  int idxReport = strSensors.indexOf(snsName + " ");
  if (idxReport != -1) {
    idxReport /= 4;
    snsReport[idxReport][0] = 0;
    snsReport[idxReport][1] = 0;
  }
}

// TODO use and array
void rcsProcess(String strBranch, String strMessage) {
  // Uppercase
  strBranch.toUpperCase();
  strMessage.toUpperCase();
  String rcsButtonCode = "00000";

  switch (strBranch.charAt(0)) {
    case 'A':
      rcsButtonCode = "10000"; break;
    case 'B':
      rcsButtonCode = "01000"; break;
    case 'C':
      rcsButtonCode = "00100"; break;
    case 'D':
      rcsButtonCode = "00010"; break;
    case 'E':
      rcsButtonCode = "00001"; break;
    case 'Z':
      rcsButtonCode = "11111"; break;
  }

  if (strMessage == "ON") {
    rcs.switchOn(rcsHomeCode, rcsButtonCode.c_str());
    //Serial.print(strBranch);
    //Serial.println("ON");
  }
  else if (strMessage == "OFF") {
    rcs.switchOff(rcsHomeCode, rcsButtonCode.c_str());
    //Serial.print(strBranch);
    //Serial.println("OFF");
  }
}

/**
  Read the DHT sensor and store the data in two globals: dhtTemp and dhtHmdt

  @return boolean reading success
*/
bool dhtRead() {
  dhtTemp = dht.readTemperature();
  dhtHmdt = dht.readHumidity();
  // Check if any reads failed
  if (isnan(dhtHmdt) || isnan(dhtTemp)) {
    dhtValid = false;
    Serial.println(F("Failed to read from DHT sensor!"));
  }
  else {
    dhtValid = true;
  }
  return dhtValid;
}

/**
  On motion detection, turn the backlight on and display the first screen
*/
void pirInterrupt() {
  // Do not reset the screens on subsequent movements
  if (delayPIR.isExpired()) {
    lcd.backlight();
    lcdIndex = 0;
    lcdRotateScreens();
  }
  // Re-start the timer to disable the backlight
  delayPIR.start(PIR_INTERVAL, AsyncDelay::MILLIS);
  //Serial.println("Motion detected");
}

/**
  Feedback notification when SoftAP is started
*/
void wifiCallback (WiFiManager *wifiMgr) {
  String strMsg = "Connect to ";
  strMsg += wifiMgr->getConfigPortalSSID();
  Serial.println(strMsg);
  lcd.setCursor(0, 1);
  lcd.print(strMsg.c_str());
}

void setup() {
  // Init the serial com
  Serial.println();
  Serial.begin(115200);
  Serial.print(NODENAME);
  Serial.print(" ");
  Serial.println(__DATE__);

  // RCS
  rcs.enableTransmit(RCS_PIN);

  // LCD init
  lcdInit();

  // Try to connect to WiFi
  WiFiManager wifiManager;
  wifiManager.setTimeout(300);
  wifiManager.setAPCallback(wifiCallback);
  wifiManager.autoConnect(NODENAME.c_str());
  while (!wifiManager.autoConnect(NODENAME.c_str())) {
    String strMsg = "No WiFi network ";
    Serial.println(strMsg);
    lcd.setCursor(0, 1);
    lcd.print(strMsg.c_str());
    delay(1000);
  }

  // Connected
  lcdShowWiFi(true);

  // Start the NTP client
  NTP_Client.begin();

  // Start the MQTT client
  MQTT_Client.setServer(MQTT_SERVER, MQTT_PORT);
  MQTT_Client.setCallback(mqttCallback);
  delayMQTT.start(MQTT_INTERVAL, AsyncDelay::MILLIS);

  // DHT timer
  delayDHT.start(DHT_INTERVAL, AsyncDelay::MILLIS);

  // Finally, start the LCD and PIR timers
  delayLCD.start(LCD_INTERVAL, AsyncDelay::MILLIS);
  delayPIR.start(PIR_INTERVAL, AsyncDelay::MILLIS);

  // PIR pin and interrupt
  pinMode(PIR_PIN, INPUT);
  attachInterrupt(PIR_PIN, pirInterrupt, FALLING);
}

void loop() {
  // Process incoming MQTT messages and maintain connection
  if (!MQTT_Client.loop()) {
    // Not connected, try to reconnect every MQTT_INTERVAL seconds
    if (delayMQTT.isExpired()) {
      mqttReconnect();
      delayMQTT.repeat();
    }
  }
  // Rotate the LCD screens
  if (delayLCD.isExpired()) {
    lcdRotateScreens();
    delayLCD.repeat();
  }
  // Check if motion detection has expired
  if (delayPIR.isExpired()) {
    lcd.noBacklight();
  }
  // Read the sensors and publish telemetry
  if (delayDHT.isExpired()) {
    dhtRead();
    if (dhtValid) {
      char text[4] = "";
      sprintf(text, "%d", (int)dhtTemp);
      MQTT_Client.publish(String(MQTT_SENSOR + "/temperature").c_str(), text);
      sprintf(text, "%d", (int)dhtHmdt);
      MQTT_Client.publish(String(MQTT_SENSOR + "/humidity").c_str(), text);
    };
    MQTT_Client.publish(String(MQTT_REPORT_WIFI + "/rssi").c_str(), String(WiFi.RSSI()).c_str());
    MQTT_Client.publish(String(MQTT_REPORT + "/uptime").c_str(), String(millis() / 1000).c_str());
    MQTT_Client.publish(String(MQTT_REPORT + "/heap").c_str(), String(ESP.getFreeHeap()).c_str());
    int vcc = ESP.getVcc();
    MQTT_Client.publish(String(MQTT_REPORT + "/vcc").c_str(), String(String(vcc / 1000) + "." + String(vcc % 1000)).c_str());
    // Repeat
    delayDHT.repeat();
  }

  // NTP
  NTP_Client.update();
}
