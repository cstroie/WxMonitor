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
#define DEBUG
#define DEVEL

// LCD: use the HD447890 library and Wire i2c library
//#define SDA 0
//#define SCL 2
#include <Wire.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h> // include i/o class header

// WiFi
#include <ESP8266WiFi.h>
#include <WiFiManager.h>
#include <WiFiUdp.h>

// NTP
//#include <TimeLib.h>

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
const char NODENAME[] = "DevNode";
const char nodename[] = "devnode";
#else
const char NODENAME[] = "WxMon";
const char nodename[] = "wxmon";
#endif
const char VERSION[]  = "3.1";

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

// Time synchronization and keeping
const char    ntpServer[] PROGMEM   = "europe.pool.ntp.org";  // NTP server to connect to (RFC5905)
const int     ntpPort               = 123;                    // NTP port
unsigned long ntpNextSync           = 0UL;                    // Next time to syncronize
unsigned long ntpDelta              = 0UL;                    // Difference between real time and internal clock
bool          ntpOk                 = false;                  // Flag to know the time is accurate
const int     ntpTZ                 = 3;                      // Time zone
WiFiUDP       ntpClient;                                      // NTP UDP client

// Wx
const char wxStation[] = "ROXX0003";
String strReports = "now tod ton tom dat sun mon bar ";
String wxReport[8][2];
String strLnSep = ", ";

// Sensors
String strSensors = "itp ihm otp ohm odp ops ";
int snsReport[6][2];
const int SNS_INTERVAL = 600 * 1000;

// MQTT parameters
// TODO FPSTR()
#ifdef DEBUG
const char mqttId[]       = "devnode-eridu-eu-org";
#else
const char mqttId[]       = "wxmon-eridu-eu-org";
#endif
const char mqttServer[]   = "eridu.eu.org";
const int  mqttPort       = 1883;
const int  mqttDelay      = 5000;
const char mqttTopicWx[]  = "wx";
const char mqttTopicCmd[] = "command";
const char mqttTopicSns[] = "sensor";
const char mqttTopicRpt[] = "report";

WiFiClient WiFi_Client;
PubSubClient mqttClient(WiFi_Client);
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
  Convert IPAddress to char array
*/
char charIP(const IPAddress ip, char *buf, size_t len, boolean pad = false) {
  if (pad) snprintf_P(buf, len, PSTR("%3d.%3d.%3d.%3d"), ip[0], ip[1], ip[2], ip[3]);
  else     snprintf_P(buf, len, PSTR("%d.%d.%d.%d"), ip[0], ip[1], ip[2], ip[3]);
}

/**
  Get current time as UNIX time (1970 epoch)

  @param sync flag to show whether network sync is to be performed
  @return current UNIX time
*/
unsigned long timeUNIX(bool sync = true) {
  // Check if we need to sync
  if (millis() >= ntpNextSync and sync) {
    // Try to get the time from Internet
    unsigned long utm = ntpSync();
    if (utm == 0) {
      // Time sync has failed, sync again over one minute
      ntpNextSync += 1UL * 60 * 1000;
      ntpOk = false;
      // Try to get old time from eeprom, if time delta is zero
    }
    else {
      // Compute the new time delta
      ntpDelta = utm - (millis() / 1000);
      // Time sync has succeeded, sync again in 8 hours
      ntpNextSync += 8UL * 60 * 60 * 1000;
      ntpOk = true;
      Serial.print(F("Network UNIX Time: 0x"));
      Serial.println(utm, 16);
    }
  }
  // Get current time based on uptime and time delta,
  // or just uptime for no time sync ever
  return (millis() / 1000) + ntpDelta + ntpTZ * 3600;
}

/**
  © Francesco Potortì 2013 - GPLv3 - Revision: 1.13

  Send an NTP packet and wait for the response, return the Unix time

  To lower the memory footprint, no buffers are allocated for sending
  and receiving the NTP packets.  Four bytes of memory are allocated
  for transmision, the rest is random garbage collected from the data
  memory segment, and the received packet is read one byte at a time.
  The Unix time is returned, that is, seconds from 1970-01-01T00:00.
*/
unsigned long ntpSync() {
  // Open socket on arbitrary port
  bool ntpOk = ntpClient.begin(12321);
  // NTP request header: Only the first four bytes of an outgoing
  // packet need to be set appropriately, the rest can be whatever.
  const long ntpFirstFourBytes = 0xEC0600E3;
  // Fail if UDP could not init a socket
  if (!ntpOk) return 0UL;
  // Clear received data from possible stray received packets
  ntpClient.flush();
  // Send an NTP request
  char ntpServerBuf[strlen_P((char*)ntpServer) + 1];
  strncpy_P(ntpServerBuf, (char*)ntpServer, sizeof(ntpServerBuf));
  if (!(ntpClient.beginPacket(ntpServerBuf, ntpPort) &&
        ntpClient.write((byte *)&ntpFirstFourBytes, 48) == 48 &&
        ntpClient.endPacket()))
    return 0UL;                             // sending request failed
  // Wait for response; check every pollIntv ms up to maxPoll times
  const int pollIntv = 150;                 // poll every this many ms
  const byte maxPoll = 15;                  // poll up to this many times
  int pktLen;                               // received packet length
  for (byte i = 0; i < maxPoll; i++) {
    if ((pktLen = ntpClient.parsePacket()) == 48) break;
    delay(pollIntv);
  }
  if (pktLen != 48) return 0UL;             // no correct packet received
  // Read and discard the first useless bytes (32 for speed, 40 for accuracy)
  for (byte i = 0; i < 40; ++i) ntpClient.read();
  // Read the integer part of sending time
  unsigned long ntpTime = ntpClient.read(); // NTP time
  for (byte i = 1; i < 4; i++)
    ntpTime = ntpTime << 8 | ntpClient.read();
  // Round to the nearest second if we want accuracy
  // The fractionary part is the next byte divided by 256: if it is
  // greater than 500ms we round to the next second; we also account
  // for an assumed network delay of 50ms, and (0.5-0.05)*256=115;
  // additionally, we account for how much we delayed reading the packet
  // since its arrival, which we assume on average to be pollIntv/2.
  ntpTime += (ntpClient.read() > 115 - pollIntv / 8);
  // Discard the rest of the packet
  ntpClient.flush();
  return ntpTime - 2208988800UL;            // convert to Unix time
}



/**
  LCD initialization
*/
void lcdInit() {
  lcd.begin(20, 4);
  lcd.setBacklight(HIGH);
  lcdLogo();
}

/**
  LCD display the logo
*/
void lcdLogo() {
  byte text[] = {0, 1, 32, 2, 3, 4, 5, 6, 3, 7}; // "Wx Monitor"
  lcdDefChars(LCD_LOGO);
  lcd.clear();
  lcd.setCursor(5, 1);
  for (byte item = 0; item < sizeof(text); item++)
    lcd.write(text[item]);
}

/**
  LCD Create the custom characters: 8 characters defined column-wise
*/
void lcdDefBig(const uint8_t chars[]) {
  for (uint8_t i = 0; i < 8; i++) {
    uint8_t charMap[8];
    for (uint8_t j = 0; j < 8; j++)
      charMap[j] = pgm_read_byte(chars + (j * 8) + i);
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
byte copyArray(byte *dst, byte *src, byte len) {
  memcpy(dst, src, len);
  return len >> 1;
}

/**
  LCD Construct the big characters (on 2 lines)

  @param chr the character to define
  @param charShapes array of needed character shapes
  @return the number of display columns the character uses
*/
byte lcdBigConstruct(char chr, byte *charShapes, size_t len) {
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
void lcdBigWrite(char chr, byte col, byte row = 0) {
  byte charShapes[10];
  byte charCols = lcdBigConstruct(chr, charShapes, sizeof(charShapes));
  for (byte line = 0; line < 2; line++) {
    lcd.setCursor(col, line + row);
    for (byte item = line * charCols; item < charCols * (line + 1); item++)
      lcd.write(byte(charShapes[item]));
  }
}

/**
  LCD Write big custom charaters

  @param text the text to write
  @param cols the columns to write each character
  @param type the character type to use
*/
void lcdBigPrint(const char *text, const byte *cols, byte row = 0, int type = LCD_NUM) {
  if (lcdChars != type) lcdDefChars(type);
  for (int i = 0; i <= sizeof(text); i++)
    lcdBigWrite(text[i], cols[i]);
}

/**
  LCD Display the current time with big characters
*/
bool lcdShowTime() {
  char buf[8] = "";
  // Get the time, but do not open a connection to server
  unsigned long utm = timeUNIX(false);
  // Compute hour, minute and second
  int hh = (utm % 86400L) / 3600;
  int mm = (utm % 3600) / 60;
  int ss =  utm % 60;
  // Create the formatted time
  snprintf_P(buf, sizeof(buf), PSTR("%02d:%02d"), hh, mm);
  // Define the columns
  byte cols[] = {2, 6, 9, 11, 15};
#ifdef DEBUG
  Serial.print(F("SCR_CLK "));
  Serial.println(buf);
#endif
  lcd.clear();
  lcdBigPrint(buf, cols, 1, LCD_NUM);
  return true;
}

/**
  LCD Display the sensor temperature with big characters
*/
bool lcdShowTemp() {
  if (dhtValid) {
    char buf[8] = "";
    // Create the formatted time
    snprintf_P(buf, sizeof(buf), PSTR("% d'C"), (int)dhtTemp);
    // Define the columns
    byte cols[] = {4, 8, 2, 15, 17};
#if defined(DEBUG)
    Serial.print("SCR_TEMP ");
    Serial.println(buf);
#endif
    lcd.clear();
    lcdBigPrint(buf, cols, 1, LCD_NUM);
  }
  return dhtValid;
}

/**
  LCD Display the sensor humidity with big characters
*/
bool lcdShowHmdt() {
  if (dhtValid) {
    char buf[8] = "";
    // Create the formatted time
    snprintf_P(buf, sizeof(buf), PSTR("% d%%"), (int)dhtHmdt);
    // Define the columns
    byte cols[] = {4, 8, 12, 16};
#if defined(DEBUG)
    Serial.print("SCR_HMDT ");
    Serial.println(buf);
#endif
    lcd.clear();
    lcdBigPrint(buf, cols, 1, LCD_NUM);
  }
  return dhtValid;
}

/**
  LCD Display the WiFi parameters

  @param serial print on serial port too
*/
bool lcdShowWiFi(bool serial) {
  if (WiFi.isConnected()) {
    char ipbuf[16], gwbuf[16], nsbuf[16];

    charIP(WiFi.localIP(), ipbuf, sizeof(ipbuf), true);
    charIP(WiFi.gatewayIP(), gwbuf, sizeof(ipbuf), true);
    charIP(WiFi.dnsIP(), nsbuf, sizeof(ipbuf), true);

    Serial.println();
    Serial.print(F("WiFi connected to "));
    Serial.print(WiFi.SSID());
    Serial.print(F(" on channel "));
    Serial.print(WiFi.channel());
    Serial.print(F(", RSSI "));
    Serial.print(WiFi.RSSI());
    Serial.println(F(" dBm."));
    Serial.print(F(" IP : "));
    Serial.println(ipbuf);
    Serial.print(F(" GW : "));
    Serial.println(gwbuf);
    Serial.print(F(" DNS: "));
    Serial.println(nsbuf);
    Serial.println();

    lcd.setCursor(0, 0);
    lcd.printf("WiFi % 15s", WiFi.SSID().c_str());
    lcd.setCursor(0, 1);
    lcd.printf("  IP% 16s", ipbuf);
    lcd.setCursor(0, 2);
    lcd.printf("  GW% 16s", gwbuf);
    lcd.setCursor(0, 3);
    lcd.printf(" DNS% 16s", nsbuf);
    return true;
  }
  else return false;
}

/**
  LCD Display the weather report

  @param report the report to display
*/
bool lcdShowWeather(char* report) {
  String tplUpLn, tplLwLn;
  int idxReport = strReports.indexOf(String(report) + " ");
  if (idxReport != -1) {
    idxReport >> 2;
    if (wxReport[idxReport][0] != "") {
      if (report == "now") {
        tplUpLn = "Now% 17s";
        tplLwLn = "";
      }
      else if (report == "tod") {
        tplUpLn = "Today  % 13s";
        tplLwLn = "";
      }
      else if (report == "ton") {
        tplUpLn = "Tonight% 13s";
        tplLwLn = "";
      }
      else if (report == "tom") {
        tplUpLn = "Tmrrow % 13s";
        tplLwLn = "";
      }
      else if (report == "dat") {
        tplUpLn = "AfTmrw % 13s";
        tplLwLn = "";
      }
      else if (report == "sun") {
        tplUpLn = "Sunrise % 12s";
        tplLwLn = "Sunset  % 12s";
      }
      else if (report == "mon") {
        tplUpLn = "Moon % 15s";
        tplLwLn = "";
      }
      else if (report == "bar") {
        tplUpLn = "Baro% 16s";
        tplLwLn = "";
      }
      // Clear the screen
      lcd.clear();
      // Print the upper line
      lcd.setCursor(0, 0);
      if (tplUpLn != "")
        lcd.printf(tplUpLn.c_str(), wxReport[idxReport][0].c_str());
      else
        lcd.print(wxReport[idxReport][0].c_str());
      Serial.println("LN 1: " + wxReport[idxReport][0]);
      // Print the lower line
      lcd.setCursor(0, 1);
      if (tplLwLn != "")
        lcd.printf(tplLwLn.c_str(), wxReport[idxReport][1].c_str());
      else
        lcd.print(wxReport[idxReport][1].c_str());
      Serial.println("LN 2: " + wxReport[idxReport][1]);
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
void lcdScreen(char *upLine, char *lwLine) {
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

/** MQTT publishing wrapper, using strings, with retain flag on

  @param topic the MQTT topic
  @param payload the MQTT message to send to topic
  @return the publishig status
*/
boolean mqttPubRetain(const String &topic, const String &payload) {
  yield();
  return mqttClient.publish(topic.c_str(), payload.c_str(), true);
}

/** MQTT publishing wrapper, using strings

  @param topic the MQTT topic
  @param payload the MQTT message to send to topic
  @return the publishig status
*/
boolean mqttPub(const String &topic, const String &payload) {
  yield();
  return mqttClient.publish(topic.c_str(), payload.c_str(), false);
}

/**
  Publish to topic
*/
boolean mqttPub(const char *payload, const char *lvl1, const char *lvl2 = NULL, const char *lvl3 = NULL, const boolean retain = false) {
  char buf[64];
  strcpy(buf, lvl1);
  if (lvl2 != NULL) {
    strcat(buf, "/");
    strcat(buf, lvl2);
  }
  if (lvl3 != NULL) {
    strcat(buf, "/");
    strcat(buf, lvl3);
  }
  yield();
  return mqttClient.publish(buf, payload, retain);
}

/**
  Publish to topic and retain
*/
boolean mqttPubRet(const char *payload, const char *lvl1, const char *lvl2 = NULL, const char *lvl3 = NULL, const boolean retain = true) {
  return mqttPub(payload, lvl1, lvl2, lvl3, retain);
}

/**
  Publish to topic
*/
boolean mqttPub(const int payload, const char *lvl1, const char *lvl2 = NULL, const char *lvl3 = NULL, const boolean retain = false) {
  char buf[8];
  sprintf(buf, "%d", payload);
  return mqttPub(buf, lvl1, lvl2, lvl3, retain);
}

/**
  Publish to topic and retain
*/
boolean mqttPubRet(const int payload, const char *lvl1, const char *lvl2 = NULL, const char *lvl3 = NULL, const boolean retain = true) {
  return mqttPub(payload, lvl1, lvl2, lvl3, retain);
}

/**
  Subscribe to all subtopics in topic or topic/subtopic
*/
void mqttSubscribeAll(const char *lvl1, const char *lvl2 = NULL) {
  char buf[64];
  strcpy(buf, lvl1);
  if (lvl2 != NULL) {
    strcat(buf, "/");
    strcat(buf, lvl2);
  }
  strcat(buf, "/#");
  mqttClient.subscribe(buf);
}

/**
  Try to reconnect to MQTT server

  @return boolean reconnection success
*/
boolean mqttReconnect() {
  Serial.println(F("MQTT connecting..."));
  if (mqttClient.connect(mqttId)) {
    // Publish the connection report
    char buf[32];
    strcpy(buf, mqttTopicRpt);
    strcat(buf, "/");
    strcat(buf, nodename);
    strcat(buf, "/");
    strcat(buf, "wifi");
    mqttPubRet(WiFi.hostname().c_str(), buf, "hostname");
    mqttPubRet(WiFi.macAddress().c_str(), buf, "mac");
    mqttPubRet(WiFi.SSID().c_str(), buf, "ssid");
    mqttPubRet(WiFi.RSSI(), buf, "rssi");
    char ipbuf[16];
    charIP(WiFi.localIP(), ipbuf, sizeof(ipbuf));
    mqttPubRet(ipbuf, buf, "ip");
    charIP(WiFi.gatewayIP(), ipbuf, sizeof(ipbuf));
    mqttPubRet(ipbuf, buf, "gw");
    // Subscribe
    mqttSubscribeAll(mqttTopicCmd, nodename);   // Subscribe to command topic
    mqttSubscribeAll(mqttTopicCmd, "rcs");      // Subscribe to RCS command topic
    mqttSubscribeAll(mqttTopicWx,  wxStation);  // Subscribe to weather topic
    mqttSubscribeAll(mqttTopicSns, "outdoor");  // Subscribe to outdoor sensors topic

    Serial.print(F("MQTT connected to "));
    Serial.println(mqttServer);
  }
  yield();
  return mqttClient.connected();
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
#ifdef DEBUG
  Serial.println("MQTT " + strTopic + ": " + strMessage);
#endif

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
      if (strRoot == "wx" && strTrunk == wxStation) {
        wxProcess(strBranch, strMessage);
      }
      else if (strRoot == "sensor") {
        snsProcess(strTopic.substring(idxSepOne + 1), strMessage);
      }
      else if (strRoot == "command") {
        if (strTrunk == "rcs") {
          rcsProcess(strBranch, strMessage);
        }
        else if (strTrunk == nodename) {
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
  Serial.print(F(" "));
  Serial.println(__DATE__);

  // RCS
  rcs.enableTransmit(RCS_PIN);

  // LCD init
  lcdInit();

  // Try to connect to WiFi
  WiFiManager wifiManager;
  wifiManager.setTimeout(300);
  wifiManager.setAPCallback(wifiCallback);
  while (!wifiManager.autoConnect(NODENAME)) {
    String strMsg = "No WiFi network ";
    Serial.println(strMsg);
    lcd.setCursor(0, 1);
    lcd.print(strMsg.c_str());
    delay(1000);
  }

  // Connected
  lcdShowWiFi(true);

  // Start time sync
  timeUNIX();
  yield();

  // Start the MQTT client
  mqttClient.setServer(mqttServer, mqttPort);
  mqttClient.setCallback(mqttCallback);
  delayMQTT.start(mqttDelay, AsyncDelay::MILLIS);
  yield();

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
  if (!mqttClient.loop()) {
    // Not connected, try to reconnect every mqttDelay seconds
    if (delayMQTT.isExpired()) {
      mqttReconnect();
      delayMQTT.repeat();
    }
  }
  yield();

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
    // Publish DHT data
    if (dhtValid) {
      char text[8] = "";
      sprintf(text, "%d", (int)dhtTemp);
      mqttPub(text, mqttTopicSns, "indoor", "temperature");
      sprintf(text, "%d", (int)dhtHmdt);
      mqttPub(text, mqttTopicSns, "indoor", "humidity");
    };
    
    // Publish the connection report
    char topic[32], buf[16];
    // Create the topic
    strcpy(topic, mqttTopicRpt);
    strcat(topic, "/");
    strcat(topic, nodename);
    // Create and publish the reports
    snprintf_P(buf, sizeof(buf), PSTR("%d"), millis() / 1000);
    mqttPubRet(buf, topic, "uptime");
    snprintf_P(buf, sizeof(buf), PSTR("%d"), ESP.getFreeHeap());
    mqttPubRet(buf, topic, "heap");
    snprintf_P(buf, sizeof(buf), PSTR("%d"), ESP.getVcc());
    mqttPubRet(buf, topic, "vcc");
    // Add the WiFi topic
    strcat(topic, "/");
    strcat(topic, "wifi");
    mqttPubRet(WiFi.RSSI(), topic, "rssi");

    // Repeat
    delayDHT.repeat();
  }
}
