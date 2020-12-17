/**
  WxMonitor - Weather Monitor

  Copyright (c) 2017-2018 Costin STROIE <costinstroie@eridu.eu.org>

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, orl
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

// The DEBUG flag
//#define DEBUG

// User settings
#include "config.h"

// NTP
#define NTP_SERVER    "europe.pool.ntp.org"
#define NTP_TZ        +2

// LCD: use the HD447890 library and Wire i2c library
#define SDA 0
#define SCL 2
#include <Wire.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h> // include i/o class header

// WiFi
#include <ESP8266WiFi.h>
#ifdef USE_SSL
#include <WiFiClientSecure.h>
#else
#include <WiFiClient.h>
#endif
#include <WiFiManager.h>
#include <WiFiUdp.h>
#include <ESP8266mDNS.h>
#include <ArduinoOTA.h>

// Network Time Protocol
#include "ntp.h"
NTP ntp;

// MQTT
#include <PubSubClient.h>

// DHT11 sensor
#include <SimpleDHT.h>


// Device name
#if defined(DEBUG)
const char NODENAME[] = "DevNode";
const char nodename[] = "devnode";
#else
const char NODENAME[] = "WxMon";
const char nodename[] = "wxmon";
#endif
const char VERSION[]  = "3.6.3";

// OTA
int otaProgress       = 0;
int otaPort           = 8266;

// LCD
//#define             WIRECLOCK     400000L   // tell hd44780 to use 400kHz i2c clock rate
hd44780_I2Cexp      lcd;                    // auto locate and autoconfig interface pins
#define             HD44780_LCDOBJECT       // tell the hd44780 sketch the lcd object has been declared
const int           LCD_COLS      = 20;     // LCD columns
const int           LCD_ROWS      = 4;      // LCD rows
enum                LCD_SCREENS   {SCR_CLK, SCR_WIFI, SCR_TEMP, SCR_HMDT, SCR_OTP, SCR_OHM, SCR_ODP, SCR_OPS, SCR_NOW, SCR_TOD, SCR_TON, SCR_TOM, SCR_DAT, SCR_BAR, SCR_SUN, SCR_MON, SCR_ALL};
enum                LCD_CHARS     {LCD_LOGO, LCD_BGNUM, LCD_LGNUM, LCD_MOON, LCD_ALL};
const unsigned long lcdDelay      = 4000UL; // Delay between changing the screen
unsigned long       lcdNextTime   = 0UL;    // Next time to change the screen
int                 lcdIndex      = 0;      // Global LCD screen index
bool                lcdLight      = true;   // Global flag to keep track of LCD backlight status
int                 lcdChars      = -1;     // Global LCD custom characters type

// Array of 8 logo characters defined column-wise
const uint8_t lcdLogoShapes[] PROGMEM = {
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
const uint8_t lcdLgNumShapes[] PROGMEM = {
  B11111, B00000, B00000, B00000, B11111, B11111, B00011, B00000,
  B11111, B00000, B00000, B00000, B01111, B11110, B01111, B00000,
  B11111, B00000, B00000, B00000, B01111, B11110, B01111, B01110,
  B11111, B00000, B00000, B00000, B00011, B11000, B11111, B11111,
  B00000, B11111, B00011, B11000, B00000, B00000, B11111, B11111,
  B00000, B11111, B01111, B11110, B00000, B00000, B11110, B01110,
  B00000, B11111, B01111, B11110, B00000, B00000, B11110, B00000,
  B00000, B11111, B11111, B11111, B00000, B00000, B11000, B00000,
};

// Large numbers and symbols
const static byte lgNum0[] PROGMEM = {0x02, 0x01, 0x03,  0xff, 0x20, 0xff,  0xff, 0x20, 0xff,  0x04, 0x00, 0x05}; // '0'
const static byte lgNum1[] PROGMEM = {0x01, 0x03, 0x20,  0x20, 0xff, 0x20,  0x20, 0xff, 0x20,  0x00, 0x00, 0x00}; // '1'
const static byte lgNum2[] PROGMEM = {0x02, 0x01, 0x03,  0x02, 0x01, 0xff,  0xff, 0x20, 0x20,  0x04, 0x00, 0x00}; // '2'
const static byte lgNum3[] PROGMEM = {0x02, 0x01, 0x03,  0x20, 0x01, 0xff,  0x20, 0x20, 0xff,  0x00, 0x00, 0x05}; // '3'
const static byte lgNum4[] PROGMEM = {0x01, 0x20, 0x20,  0xff, 0x01, 0xff,  0x20, 0x20, 0xff,  0x20, 0x20, 0x05}; // '4'
const static byte lgNum5[] PROGMEM = {0x01, 0x01, 0x01,  0xff, 0x01, 0x03,  0x20, 0x20, 0xff,  0x00, 0x00, 0x05}; // '5'
const static byte lgNum6[] PROGMEM = {0x02, 0x01, 0x03,  0xff, 0x01, 0x03,  0xff, 0x20, 0xff,  0x04, 0x00, 0x05}; // '6'
const static byte lgNum7[] PROGMEM = {0x01, 0x01, 0x01,  0x20, 0x02, 0xff,  0x20, 0xff, 0x20,  0x20, 0x00, 0x20}; // '7'
const static byte lgNum8[] PROGMEM = {0x02, 0x01, 0x03,  0xff, 0x01, 0xff,  0xff, 0x20, 0xff,  0x04, 0x00, 0x05}; // '8'
const static byte lgNum9[] PROGMEM = {0x02, 0x01, 0x03,  0xff, 0x20, 0xff,  0x04, 0x00, 0xff,  0x20, 0x20, 0x05}; // '9'
const static byte lgNumC[] PROGMEM = {0x02, 0x01, 0x03,  0xff, 0x20, 0x00,  0xff, 0x20, 0x01,  0x04, 0x00, 0x05}; // 'C'
const static byte lgNumc[] PROGMEM = {0x02, 0x01, 0x03,  0xff, 0x20, 0x20,  0x04, 0x00, 0x05,  0x20, 0x20, 0x20}; // 'c'
const static byte lgNumM[] PROGMEM = {0x20, 0x20, 0x20,  0x20, 0x01, 0x01,  0x20, 0x20, 0x20,  0x20, 0x20, 0x20}; // '-'
const static byte lgNumS[] PROGMEM = {0x20, 0x20, 0x20,  0x20, 0x20, 0x20,  0x20, 0x20, 0x20,  0x20, 0x20, 0x20}; // ' '
const static byte lgNumN[] PROGMEM = {0x20,  0x07,  0x07,  0x20};                                                 // ':'
const static byte lgNumD[] PROGMEM = {0x20,  0x20,  0x20,  0x07};                                                 // '.'
const static byte lgNumG[] PROGMEM = {0x07,  0x20,  0x20,  0x20};                                                 // '''
const static byte lgNumP[] PROGMEM = {0x07, 0x20, 0x02,  0x02, 0x06, 0x05,  0x05, 0x20, 0x07,  0x20, 0x20, 0x20}; // '%'
const byte* const lgNum[]  PROGMEM = {lgNum0, lgNum1, lgNum2, lgNum3, lgNum4, lgNum5, lgNum6, lgNum7, lgNum8, lgNum9,
                                      lgNumC, lgNumc, lgNumM, lgNumS, lgNumN, lgNumD, lgNumG, lgNumP
                                     };
const char lgNumChars[]             = "0123456789Cc- :.'%";

// Wx
const char    wxStation[]           = "ROXX0003";
enum          WX_KEYS               {WX_NOW, WX_TOD, WX_TON, WX_TOM, WX_DAT, WX_SUN, WX_MON, WX_BAR, WX_ALL};
char          wxRepKeys[][4]        = {"now", "tod", "ton", "tom", "dat", "sun", "mon", "bar"};
char          wxReport[WX_ALL][2][LCD_COLS];                  // Weather storage

// MQTT parameters
#ifdef USE_MQTT_SSL
WiFiClientSecure    wifiClient;                                 // Secure WiFi TCP client for MQTT
#else
WiFiClient          wifiClient;                                 // Plain WiFi TCP client for MQTT
#endif
PubSubClient        mqttClient(wifiClient);                     // MQTT client, based on WiFi client
#ifdef DEVEL
const char          mqttId[]       = "wxmon-dev-eridu-eu-org";  // Development MQTT client ID
#else
const char          mqttId[]       = "wxmon-eridu-eu-org";      // Production MQTT client ID
#endif
const char          mqttServer[]   = "eridu.eu.org";            // MQTT server address to connect to
#ifdef USE_MQTT_SSL
const int           mqttPort       = 8883;                      // Secure MQTT port
#else
const int           mqttPort       = 1883;                      // Plain MQTT port
#endif
const unsigned long mqttDelay      = 5000UL;                    // Delay between reconnection attempts
unsigned long       mqttNextTime   = 0UL;                       // Next time to reconnect
// Various MQTT topics
const char          mqttTopicWx[]  = "wx";
const char          mqttTopicCmd[] = "command";
const char          mqttTopicSns[] = "sensor";
const char          mqttTopicRpt[] = "report";

// DHT11
SimpleDHT11         dht;                          // The DHT11 temperature/humidity sensor
bool                dhtOK         = false;        // The temperature/humidity sensor presence flag
bool                dhtDrop       = true;         // Always drop the first reading
const unsigned long snsDelay      = 60000UL;      // Delay between sensor readings
unsigned long       snsNextTime   = 0UL;          // Next time to read the sensors
const int           pinDHT        = D2;           // Temperature/humidity sensor input pin
// Sensors
enum                SNS_KEYS      {SNS_ITP, SNS_IHM, SNS_OTP, SNS_OHM, SNS_ODP, SNS_OPS, SNS_ALL};
int                 snsReport[SNS_ALL][2];        // Sensors storage
const int           snsTTL        = 600;          // Sensors readings time to live (seconds)

// Set ADC to Voltage
ADC_MODE(ADC_VCC);

// Beep
const int           pinBeep       = D5 ;          // Beep output pin

// PIR
const unsigned long pirDelay      = 300000UL;     // Delay after that the light is closed
unsigned long       pirNextTime   = 0UL;          // Next time to close the light
const int           pinPIR        = D1;

// Character transformation
const char chrUtfDeg[] = {194, 176, 0};
const char chrWinDeg[] = {176, 0};
const char chrLcdDeg[] = {223, 0};

/**
  Replace a substring
*/
void strrpl(char string[], const char *search, const char *replace) {
  char buffer[100];
  char *p = string;
  while ((p = strstr(p, search))) {
    strncpy(buffer, string, p - string);
    buffer[p - string] = '\0';
    strcat(buffer, replace);
    strcat(buffer, p + strlen(search));
    strcpy(string, buffer);
    p++;
    yield();
  }
}

/**
  Convert IPAddress to char array
*/
char charIP(const IPAddress ip, char *buf, size_t len, boolean pad = false) {
  if (pad) snprintf_P(buf, len, PSTR("%3d.%3d.%3d.%3d"), ip[0], ip[1], ip[2], ip[3]);
  else     snprintf_P(buf, len, PSTR("%d.%d.%d.%d"),     ip[0], ip[1], ip[2], ip[3]);
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
  LCD display the logo "Wx Monitor" and version
*/
void lcdLogo() {
  const static byte lcdLogoText[] PROGMEM = {0x00, 0x01, 0x20, 0x02, 0x03, 0x04, 0x05, 0x06, 0x03, 0x07};
  lcdDefChars(LCD_LOGO);
  lcd.clear();
  lcd.setCursor((LCD_COLS - sizeof(lcdLogoText)) / 2, 1);
  for (byte item = 0; item < sizeof(lcdLogoText); item++)
    lcd.write(pgm_read_byte(lcdLogoText + item));
  lcd.setCursor(LCD_COLS - strlen(VERSION) - 1, LCD_ROWS - 1);
  lcd.print("v");
  lcd.print(VERSION);
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
      lcdDefBig(lcdLogoShapes);
      lcdChars = lcdCharsType;
      break;
    case LCD_LGNUM:
    default:
      lcdDefBig(lcdLgNumShapes);
      lcdChars = lcdCharsType;
      break;
  }
}

/**
  Copy the source shape to destination shape and
  return the length divided by 4 (i.e. number of columns)

  @param dst the destination array
  @param src the source array
  @param len the length of the source array
  @return the number of columns (half the length of the source array)
*/
byte copyLgShape(byte * dst, const byte * src, size_t len) {
  memcpy_P(dst, src, len);
  return len / 4;
}

/**
  LCD Construct the large characters (on 4 lines)

  @param chr the character to define
  @param charShapes array of needed character shapes
  @return the number of display columns the character uses
*/
byte lcdLgConstruct(char chr, byte * charShapes) {
  byte charCols = 3;
  byte idx;

  // Find the character
  for (idx = 0; idx < 255; idx++) {
    if (lgNumChars[idx] == '\0')
      idx = 255;
    else if (lgNumChars[idx] == chr)
      break;
  }

  // FIXME
  if (idx >= 14 and idx <= 16)
    charCols = 1;
  else
    charCols = 3;

  return copyLgShape(charShapes, lgNum[idx], charCols * 4);
}

/**
  LCD Write one large custom charater (on 4 lines)

  @param chr the character to write
  @param col the column to write the character
*/
void lcdLgWrite(char chr, byte col, byte row = 0) {
  byte charShapes[32];
  byte charCols = lcdLgConstruct(chr, charShapes);
  for (byte line = 0; line < LCD_ROWS; line++) {
    lcd.setCursor(col, line + row);
    for (byte item = line * charCols; item < charCols * (line + 1); item++)
      lcd.write(byte(charShapes[item]));
  }
}

/**
  LCD Write large custom charaters (on 4 lines)

  @param text the text to write
  @param cols the columns to write each character
  @param type the character type to use
*/
void lcdLgPrint(const char *text, const byte * cols, byte row = 0, int type = LCD_LGNUM) {
  if (lcdChars != type) lcdDefChars(type);
  for (int i = 0; i <= sizeof(text); i++)
    lcdLgWrite(text[i], cols[i]);
}

/**
  LCD Display the current time with big characters
*/
bool lcdShowTime() {
  char buf[8] = "";
  // Get the date and time
  unsigned long utm = ntp.getSeconds();
  datetime_t dt = ntp.getDateTime(utm);
  // Check for DST and compute again if needed
  if (ntp.dstCheck(dt.yy, dt.ll, dt.dd, dt.hh))
    dt = ntp.getDateTime(utm + 3600);
  // Create the formatted time
  snprintf_P(buf, sizeof(buf), PSTR("%02d:%02d"), dt.hh, dt.mm);
  // Define the columns
  byte cols[] = {2, 6, 10, 12, 16};
#ifdef DEBUG
  Serial.print(F("SCR_CLK "));
  Serial.println(buf);
#endif
  lcd.clear();
  lcdLgPrint(buf, cols);
  return true;
}

/**
  LCD Display the sensor temperature with big characters
*/
bool lcdShowTemp() {
  if (dhtOK) {
    char buf[8] = "";
    // Create the formatted line
    snprintf_P(buf, sizeof(buf), PSTR("% 3d'c"), snsReport[SNS_ITP][0]);
    // Define the columns
    byte cols[] = {4, 8, 12, 16, 17};
#if defined(DEBUG)
    Serial.print("SCR_TEMP ");
    Serial.println(buf);
#endif
    lcd.clear();
    lcdLgPrint(buf, cols);
  }
  return dhtOK;
}

/**
  LCD Display the sensor humidity with big characters
*/
bool lcdShowHmdt() {
  if (dhtOK) {
    char buf[8] = "";
    // Create the formatted line
    snprintf_P(buf, sizeof(buf), PSTR("% 3d%%"), snsReport[SNS_IHM][0]);
    // Define the columns
    byte cols[] = {4, 8, 12, 16};
#if defined(DEBUG)
    Serial.print("SCR_HMDT ");
    Serial.println(buf);
#endif
    lcd.clear();
    lcdLgPrint(buf, cols);
  }
  return dhtOK;
}

/**
  LCD Display the WiFi parameters

  @param serial print on serial port too
*/
bool lcdShowWiFi(bool serial = true) {
  if (WiFi.isConnected()) {
    char ipbuf[16], gwbuf[16], nsbuf[16];

    charIP(WiFi.localIP(), ipbuf, sizeof(ipbuf), true);
    charIP(WiFi.gatewayIP(), gwbuf, sizeof(ipbuf), true);
    charIP(WiFi.dnsIP(), nsbuf, sizeof(ipbuf), true);

    if (serial) {
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
    };

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
bool lcdShowWeather(int report) {
  String tplUpLn, tplLwLn;
  if (wxReport[report][0][0] != '\0') {
    if (report == WX_NOW) {
      tplUpLn = "Now% 17s";
      tplLwLn = "";
    }
    else if (report == WX_TOD) {
      tplUpLn = "Today  % 13s";
      tplLwLn = "";
    }
    else if (report == WX_TON) {
      tplUpLn = "Tonight% 13s";
      tplLwLn = "";
    }
    else if (report == WX_TOM) {
      tplUpLn = "Tmrrow % 13s";
      tplLwLn = "";
    }
    else if (report == WX_DAT) {
      tplUpLn = "AfTmrw % 13s";
      tplLwLn = "";
    }
    else if (report == WX_SUN) {
      tplUpLn = "Sunrise % 12s";
      tplLwLn = "Sunset  % 12s";
    }
    else if (report == WX_MON) {
      tplUpLn = "Moon % 15s";
      tplLwLn = "";
    }
    else if (report == WX_BAR) {
      tplUpLn = "Baro% 16s";
      tplLwLn = "";
    }
    // Clear the screen
    lcd.clear();
    // Print the upper line
    lcd.setCursor(0, 0);
    if (tplUpLn != "")
      lcd.printf(tplUpLn.c_str(), wxReport[report][0]);
    else
      lcd.print(wxReport[report][0]);
#ifdef DEBUG
    Serial.print("LN 1: ");
    Serial.println(wxReport[report][0]);
#endif
    // Print the lower line
    lcd.setCursor(0, 1);
    if (tplLwLn != "")
      lcd.printf(tplLwLn.c_str(), wxReport[report][1]);
    else
      lcd.print(wxReport[report][1]);
#ifdef DEBUG
    Serial.print("LN 2: ");
    Serial.println(wxReport[report][1]);
#endif
    return true;
  }
  return false;
}

/**
  LCD Display the sensors

  @param sensor the sensor to display
*/
bool lcdShowSensor(int sensor) {
  char text[8] = "";

  if (snsReport[sensor][1] > 0) {
    if      (sensor == SNS_OTP) {
      sprintf(text, "% 3d'c", snsReport[sensor][0]);
      byte cols[] = {4, 8, 12, 16, 17};
#if defined(DEBUG)
      Serial.print("SCR_OTP ");
      Serial.println(text);
#endif
      lcd.clear();
      lcdLgPrint(text, cols);
    }
    else if (sensor == SNS_OHM) {
      sprintf(text, "% 4d%%", snsReport[sensor][0]);
      byte cols[] = {0, 4, 8, 12, 16};
#if defined(DEBUG)
      Serial.print("SCR_OHM ");
      Serial.println(text);
#endif
      lcd.clear();
      lcdLgPrint(text, cols);
    }
    else if (sensor == SNS_ODP) {
      sprintf(text, "% 3d'c", snsReport[sensor][0]);
      byte cols[] = {4, 8, 12, 16, 17};
#if defined(DEBUG)
      Serial.print("SCR_ODP ");
      Serial.println(text);
#endif
      lcd.clear();
      lcdLgPrint(text, cols);
    }
    else if (sensor == SNS_OPS) {
      sprintf(text, "% 4d", snsReport[sensor][0]);
      byte cols[] = {0, 4, 8, 12, 16};
#if defined(DEBUG)
      Serial.print("SCR_OPS ");
      Serial.println(text);
#endif
      lcd.clear();
      lcdLgPrint(text, cols);
    }
    if (snsReport[sensor][1] + snsTTL < millis() / 1000)
      snsReport[sensor][1] = 0;
    return true;
  }

  return false;
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
      result = lcdShowSensor(SNS_OTP);
      break;
    case SCR_OHM:
      result = lcdShowSensor(SNS_OHM);
      break;
    case SCR_ODP:
      result = lcdShowSensor(SNS_ODP);
      break;
    case SCR_OPS:
      result = lcdShowSensor(SNS_OPS);
      break;
    case SCR_NOW:
      result = lcdShowWeather(WX_NOW);
      break;
    case SCR_TOD:
      result = lcdShowWeather(WX_TOD);
      break;
    case SCR_TON:
      result = lcdShowWeather(WX_TON);
      break;
    case SCR_TOM:
      result = lcdShowWeather(WX_TOM);
      break;
    case SCR_DAT:
      result = lcdShowWeather(WX_DAT);
      break;
    case SCR_BAR:
      result = lcdShowWeather(WX_BAR);
      break;
    case SCR_SUN:
      result = lcdShowWeather(WX_SUN);
      break;
    case SCR_MON:
      result = lcdShowWeather(WX_MON);
      break;
    default:
      result = lcdShowTime();
      return -1;
  }
  if (result == true)
    return index + 1;
  else
    return 0;
}

/**
  Select the screen to display, in case of error, try the next screen.
*/
void lcdRotateScreens() {
  int nextIndex;
  do {
    nextIndex = lcdShowScreen(lcdIndex);
    if (nextIndex == -1) {
      // Default has been displayed
      lcdIndex = 1;
      nextIndex = 1;
    }
    else if (nextIndex == 0)
      // There was an error, try the next
      lcdIndex += 1;
    else
      // Successful display, set the next
      lcdIndex = nextIndex;
#if defined(DEBUG)
    Serial.print(F("SCR_NXT "));
    Serial.println(nextIndex);
#endif
  } while (nextIndex <= 0);
}

/**
  MQTT publishing wrapper, using strings, with retain flag on

  @param topic the MQTT topic
  @param payload the MQTT message to send to topic
  @return the publishig status
*/
boolean mqttPubRetain(const String & topic, const String & payload) {
  yield();
  return mqttClient.publish(topic.c_str(), payload.c_str(), true);
}

/**
  MQTT publishing wrapper, using strings

  @param topic the MQTT topic
  @param payload the MQTT message to send to topic
  @return the publishig status
*/
boolean mqttPub(const String & topic, const String & payload) {
  yield();
  return mqttClient.publish(topic.c_str(), payload.c_str(), false);
}

/**
  Publish char array to topic
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
  Publish char array to topic and retain
*/
boolean mqttPubRet(const char *payload, const char *lvl1, const char *lvl2 = NULL, const char *lvl3 = NULL, const boolean retain = true) {
  return mqttPub(payload, lvl1, lvl2, lvl3, retain);
}

/**
  Publish integer to topic
*/
boolean mqttPub(const int payload, const char *lvl1, const char *lvl2 = NULL, const char *lvl3 = NULL, const boolean retain = false) {
  char buf[8];
  sprintf(buf, "%d", payload);
  return mqttPub(buf, lvl1, lvl2, lvl3, retain);
}

/**
  Publish integer to topic and retain
*/
boolean mqttPubRet(const int payload, const char *lvl1, const char *lvl2 = NULL, const char *lvl3 = NULL, const boolean retain = true) {
  return mqttPub(payload, lvl1, lvl2, lvl3, retain);
}

/**
  Subscribe to topic/# or topic/subtopic/#
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
  char buf[32];
  strcpy(buf, mqttTopicRpt);
  strcat(buf, "/");
  strcat(buf, nodename);
  if (mqttClient.connect(mqttId, buf, 0, true, "offline")) {
    // Publish the "online" status
    mqttPubRet("online", buf);
    // Publish the connection report
    strcat(buf, "/wifi");
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
    mqttSubscribeAll(mqttTopicWx,  wxStation);  // Subscribe to weather topic
    mqttSubscribeAll(mqttTopicSns, "outdoor");  // Subscribe to outdoor sensors topic

    Serial.print(F("MQTT connected to "));
    Serial.print(mqttServer);
    Serial.print(F(" port "));
    Serial.println(mqttPort);
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
void mqttCallback(char *topic, byte * payload, unsigned int length) {
  // Make a limited copy of the payload and make sure it ends with \0
  char message[100];
  if (length > 100) length = 100;
  memcpy(message, payload, length);
  message[length] = '\0';

#ifdef DEBUG
  Serial.printf("MQTT %s: %s\r\n", topic, message);
#endif

  // Decompose the topic
  char *pRoot = NULL, *pTrunk = NULL, *pBranch = NULL;
  pRoot = topic;
  pTrunk = strchr(pRoot, '/');
  if (pTrunk != NULL)
    *pTrunk++ = '\0';
  pBranch = strchr(pTrunk, '/');
  if (pBranch != NULL)
    *pBranch++ = '\0';

  yield();

  // Dispatcher
  if (strcmp(pRoot, "wx") == 0 and strcmp(pTrunk, wxStation) == 0)
    wxProcess(pBranch, message);
  else if (strcmp(pRoot, "sensor") == 0 and strcmp(pTrunk, "outdoor") == 0)
    snsProcess(pBranch, message);
  else if (strcmp(pRoot, "command") == 0) {
    if (strcmp(pTrunk, nodename) == 0) {
      if (strcmp(pBranch, "restart") == 0)
        ESP.restart();
      else if (strcmp(pBranch, "beep") == 0) {
        // Just beep
        int freq = atoi(message);
        if (!freq) freq = 2000;
        tone(pinBeep, freq, 250);
      }
      else if (strcmp(pBranch, "light") == 0) {
        if (strcmp(message, "on") == 0) {
          // Lights on
          lcd.backlight();
          lcdLight = true;
          // Set again the timer to disable the backlight
          pirNextTime = millis() + pirDelay;
        }
        else if (strcmp(message, "off") == 0)
          // Expire the timer
          pirNextTime = millis();
      }
    }
  }
  yield();
}

/**
  Process the MQTT weather topics and messages and create weather reports

  @param report the weather report topic
  @param message the weather message
*/
void wxProcess(const char *report, char *message) {
  // Find the report index starting from the key
  int idxReport = 0;
  while (idxReport < WX_ALL)
    if (strcmp(report, wxRepKeys[idxReport++]) == 0)
      break;
  // Check if the index has been found
  idxReport--;
  if (idxReport >= 0 and idxReport < WX_ALL) {
    // Convert from UTF-8 or CP1252 to LCD charset
    strrpl(message, chrUtfDeg, chrLcdDeg);
    strrpl(message, chrWinDeg, chrLcdDeg);
    // Check the line separator
    char *sep = strstr(message, ", ");
    if (sep != NULL) {
      // Put \0 in the place of separator
      *sep = 0;
      // Store the two-line report
      strncpy(wxReport[idxReport][0], message, LCD_COLS);
      wxReport[idxReport][0][LCD_COLS] = '\0';
      strncpy(wxReport[idxReport][1], sep + 2, LCD_COLS);
      wxReport[idxReport][1][LCD_COLS] = '\0';
      // The 'tod' and 'ton' reports are mutually exclusive
      // FIXME
      //if      (idxReport == WX_TOD) wxClear(WX_TON);
      //else if (idxReport == WX_TON) wxClear(WX_TOD);
    }
  }
}

/**
  Clear a weather report

  @param report the report to be clean
*/
void wxClear(int report) {
  // Check if the index is valid
  if (report >= 0 and report < WX_ALL) {
    wxReport[report][0][0] = '\0';
    wxReport[report][1][0] = '\0';
  }
}

/**
  Process the sensor topics and messages and create sensor reports

  @param strSensor the sensor name
  @param strMessage the sensor value
*/
void snsProcess(const char *sensor, char *message) {
  int report = -1;
  if      (strcmp(sensor, "temperature") == 0)
    report = SNS_OTP;
  else if (strcmp(sensor, "humidity") == 0)
    report = SNS_OHM;
  else if (strcmp(sensor, "dewpoint") == 0)
    report = SNS_ODP;
  else if (strcmp(sensor, "sealevel") == 0)
    report = SNS_OPS;
  // Store the sensor data and age
  if (report != -1) {
    snsReport[report][0] = atoi(message);
    snsReport[report][1] = millis() / 1000;
  }
}

/**
  Read the DHT11 sensor

  @param temp temperature
  @param hmdt humidity
  @param drop drop the reading (read twice)
  @return success
*/
bool dhtRead(int *temp, int *hmdt, bool drop = false) {
  byte t = 0, h = 0;
  bool ok = false;
  if (drop)
    // Read and drop
    ok = dht.read(pinDHT, NULL, NULL, NULL);
  else {
    // Read and store
    if (dht.read(pinDHT, &t, &h, NULL) == 0) {
      if (temp) *temp = (int)t;
      if (hmdt) *hmdt = (int)h;
      ok = true;
    }
  }
#ifdef DEBUG
  if (!ok) Serial.println(F("Failed to read the DHT11 sensor"));
#endif
  return ok;
}

/**
  Feedback notification when SoftAP is started
*/
void wifiCallback (WiFiManager * wifiMgr) {
  // TODO
  String strMsg = "Connect to ";
  strMsg += wifiMgr->getConfigPortalSSID();
  Serial.println(strMsg);
  lcd.setCursor(0, 1);
  lcd.print(strMsg.c_str());
}

/**
  Try to connect to WiFi
*/
void wifiConnect(int timeout = 300) {
  // Set the host name
  WiFi.hostname(NODENAME);
  // Try to connect to WiFi
#ifdef WIFI_SSID
  Serial.print(F("WiFi connecting "));
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (!WiFi.isConnected()) {
    Serial.print(".");
    delay(1000);
  };
  Serial.println(F(" done."));
#else
  WiFiManager wifiManager;
  wifiManager.setTimeout(timeout);
  wifiManager.setAPCallback(wifiCallback);
  while (!wifiManager.autoConnect(NODENAME)) {
    // TODO
    String strMsg = "No WiFi network ";
    Serial.println(strMsg);
    lcd.setCursor(0, 1);
    lcd.print(strMsg.c_str());
    delay(1000);
  }
#endif
}

void setup() {
  // Init the serial interface
  Serial.begin(115200, SERIAL_8N1, SERIAL_TX_ONLY);
  Serial.println();
  Serial.print(NODENAME);
  Serial.print(F(" "));
  Serial.println(__DATE__);

  // For I2C, the ESP8266-1 module uses the pin 0 for SDA and 2 for SCL
#if defined(ARDUINO_ARCH_ESP8266)
  Wire.pins(SDA, SCL);
#endif

  // Beep
  pinMode(D5, OUTPUT);
  digitalWrite(D5, HIGH);
  delay(50);
  digitalWrite(D5, LOW);

  // LCD init
  lcdInit();

  // Try to connect
  wifiConnect();
  // Connected
  lcdShowWiFi(true);

  // OTA Update
  ArduinoOTA.setPort(otaPort);
  ArduinoOTA.setHostname(NODENAME);
  // ArduinoOTA.setPassword((const char *)"123");

  ArduinoOTA.onStart([]() {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("OTA");
    lcd.setCursor(LCD_COLS - 12, 0);
    lcd.print("[");
    lcd.setCursor(LCD_COLS - 1, 0);
    lcd.print("]");
    Serial.println(F("OTA Start"));
  });

  ArduinoOTA.onEnd([]() {
    Serial.println("\nOTA Finished");
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    int otaPrg = progress / (total / 10);
    if (otaProgress != otaPrg) {
      otaProgress = otaPrg;
      lcd.setCursor(LCD_COLS - 12 + otaProgress, 0);
      lcd.print("|");
#ifdef DEBUG
      Serial.printf("Progress: %u%%\r", otaProgress * 10);
#endif
    }
  });

  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR)
      Serial.println(F("Auth Failed"));
    else if (error == OTA_BEGIN_ERROR)
      Serial.println(F("Begin Failed"));
    else if (error == OTA_CONNECT_ERROR)
      Serial.println(F("Connect Failed"));
    else if (error == OTA_RECEIVE_ERROR)
      Serial.println(F("Receive Failed"));
    else if (error == OTA_END_ERROR)
      Serial.println(F("End Failed"));
  });

  ArduinoOTA.begin();
#ifdef DEBUG
  Serial.println(F("OTA Ready"));
#endif

  // Configure NTP
  ntp.init(NTP_SERVER);
  ntp.setTZ(NTP_TZ);

  // Start the MQTT client
  mqttClient.setServer(mqttServer, mqttPort);
  mqttClient.setCallback(mqttCallback);
  mqttNextTime = millis();
  yield();

  // DHT11
  if (dhtRead(NULL, NULL))
    Serial.println(F("DHT11 sensor detected"));
  else
    Serial.println(F("DHT11 sensor missing"));

  // Start the LCD timer
  lcdNextTime = millis() + lcdDelay;

  // PIR pin
  pinMode(pinPIR, INPUT);
  lcdLight = true;
  // Set the timer to disable the backlight
  pirNextTime = millis() + pirDelay;

  // Start the sensors timer
  snsNextTime = millis();
}

void loop() {
  // OTA
  ArduinoOTA.handle();
  yield();

  // Process incoming MQTT messages and maintain connection
  if (!mqttClient.loop())
    // Not connected, check if it's time to reconnect
    if (millis() >= mqttNextTime)
      // Try to reconnect every mqttDelay seconds
      if (!mqttReconnect()) mqttNextTime = millis() + mqttDelay;
  yield();

  // Rotate the LCD screens
  if (millis() >= lcdNextTime) {
    lcdRotateScreens();
    // Repeat after the delay
    lcdNextTime += lcdDelay;
  }

  // On motion detection, turn on the backlight and
  // display the first screen if not already on
  if (digitalRead(pinPIR) == 0) {
    // Check if the backlight has been turned off
    if (not lcdLight) {
      // Turn on the backlight
      lcd.backlight();
      lcdLight = true;
      // Jump back to screen zero and set the timer
      lcdIndex = SCR_CLK;
      lcdNextTime += lcdDelay;
      lcdRotateScreens();
      // TODO MQTT motion detection
    }
    // Extend the timer to disable the backlight
    pirNextTime = millis() + pirDelay;
#ifdef DEBUG
    Serial.println(F("Motion detected"));
#endif
  }

  // Check if motion detection has expired
  if (millis() >= pirNextTime and lcdLight) {
    // Turn off the backlight
    lcd.noBacklight();
    lcdLight = false;
  }

  // Read the sensors and publish the telemetry
  if (millis() >= snsNextTime) {
    // Get the temperature and humidity from DHT11
    if (dhtRead(&snsReport[SNS_ITP][0], &snsReport[SNS_IHM][0])) {
      int rdtime = millis() / 1000;
      snsReport[SNS_ITP][1] = rdtime;
      snsReport[SNS_IHM][1] = rdtime;
      dhtOK = true;
      // Compose and publish the telemetry
      char text[8] = "";
      snprintf_P(text, sizeof(text), PSTR("%d"), snsReport[SNS_ITP][0]);
      mqttPubRet(text, mqttTopicSns, "indoor", "temperature");
      snprintf_P(text, sizeof(text), PSTR("%d"), snsReport[SNS_IHM][0]);
      mqttPubRet(text, mqttTopicSns, "indoor", "humidity");
    }
    else
      dhtOK = false;

    // Publish the connection report
    char topic[32], buf[16];
    // Create the topic
    // TODO strcat_P
    strcpy(topic, mqttTopicRpt);
    strcat(topic, "/");
    strcat(topic, nodename);
    // Uptime
    char upt[32] = "";
    unsigned long ups = 0;
    ups = ntp.getUptime(upt, sizeof(upt));
    snprintf_P(buf, sizeof(buf), PSTR("%d"), ups);
    mqttPubRet(buf, topic, "uptime");
    mqttPubRet(upt, topic, "uptime", "text");
    // Free heap
    snprintf_P(buf, sizeof(buf), PSTR("%d"), ESP.getFreeHeap());
    mqttPubRet(buf, topic, "heap");
    // Power supply
    int vcc = ESP.getVcc();
    snprintf_P(buf, sizeof(buf), PSTR("%d.%d"), vcc / 1000, vcc % 1000);
    mqttPubRet(buf, topic, "vcc");
    // Add the WiFi topic and publish the RSSI value
    strcat_P(topic, PSTR("/wifi"));
    mqttPubRet(WiFi.RSSI(), topic, "rssi");

    // Repeat after the delay
    snsNextTime += snsDelay;
  }
}
