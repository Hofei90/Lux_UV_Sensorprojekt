//https://www.mouser.de/pdfdocs/designingveml7700.pdf

#include <Arduino.h>
#include "Adafruit_VEML7700.h"

#include <U8g2lib.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

#include <ESP8266WiFi.h>
#include <config.h>


U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
Adafruit_VEML7700 veml = Adafruit_VEML7700();

int G;
int IT;

float lux_g;
float white_g;
int als_g;


void read_gain(){
  Serial.print(F("Gain: "));
  switch (veml.getGain()) {
    case VEML7700_GAIN_1: Serial.println("1"); break;
    case VEML7700_GAIN_2: Serial.println("2"); break;
    case VEML7700_GAIN_1_4: Serial.println("1/4"); break;
    case VEML7700_GAIN_1_8: Serial.println("1/8"); break;
  }
}


void set_gain(int G){
  switch (G) {
    case 1: veml.setGain(VEML7700_GAIN_1_8); break;
    case 2: veml.setGain(VEML7700_GAIN_1_4); break;
    case 3: veml.setGain(VEML7700_GAIN_1); break;
    case 4: veml.setGain(VEML7700_GAIN_2); break;
  }
  read_gain();
}


void read_integration_time(){
  Serial.print(F("Integration Time (ms): "));
  switch (veml.getIntegrationTime()) {
    case VEML7700_IT_25MS: Serial.println("25"); break;
    case VEML7700_IT_50MS: Serial.println("50"); break;
    case VEML7700_IT_100MS: Serial.println("100"); break;
    case VEML7700_IT_200MS: Serial.println("200"); break;
    case VEML7700_IT_400MS: Serial.println("400"); break;
    case VEML7700_IT_800MS: Serial.println("800"); break;
  }
}


void set_integration_time(int IT){
  switch (IT) {
    case -2: veml.setIntegrationTime(VEML7700_IT_25MS); break;
    case -1: veml.setIntegrationTime(VEML7700_IT_50MS); break;
    case  0: veml.setIntegrationTime(VEML7700_IT_100MS); break;
    case  1: veml.setIntegrationTime(VEML7700_IT_200MS); break;
    case  2: veml.setIntegrationTime(VEML7700_IT_400MS); break;
    case  3: veml.setIntegrationTime(VEML7700_IT_800MS); break;
  }
  read_integration_time();
}


void connect_to_wlan() {
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
  delay(500);
  Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}


void setup() {
  u8g2.begin();
  u8g2.enableUTF8Print();		// enable UTF8 support for the Arduino print() function

  Serial.begin(115200);
  delay(100);
  Serial.println("Adafruit VEML7700 Test");

  if (!veml.begin()) {
    Serial.println("Sensor not found");
    while (1);
  }

  Serial.println("Sensor found");


  veml.powerSaveEnable(true);
  veml.setPowerSaveMode(VEML7700_POWERSAVE_MODE3);

  connect_to_wlan();
}


float calculation_lux(float lux){
  Serial.println("Calc Lux");
  float x = lux;
  Serial.print("Raw Lux: ");
  Serial.println(lux);
  float lux_calc = (0.000000003 * pow(x, 3)) + (0.000003 * pow(x, 2)) + (1.0564 * x) - 30.803;
  return lux_calc;
}


void messwerte_schreiben(bool calc){
  if (calc == true){
    lux_g = calculation_lux(veml.readLux());
  } else {
    lux_g = veml.readLux();
  }
  white_g = veml.readWhite();
  als_g = veml.readALS();
}


void veml7700_messung_starten(){
  // TODO: Sensor standby - was ist standby?
  veml.enable(false);
  set_integration_time(IT);
  set_gain(G);
  veml.enable(true);
  delay(3000);
  int als = veml.readALS();
  Serial.print("ALS: ");
  Serial.println(als);
  if (als <= 100) {
    G = G + 1;
    if (G > 4){
      G = 4;
    }
    Serial.print("G: ");
    Serial.println(G);
    if (G == 4) {
      IT = IT + 1;
      if (IT > 4){
        IT = 4;
      }
      if (IT == 4){
        messwerte_schreiben(false);
      } else {
        Serial.println("Starte 1");
        veml7700_messung_starten();
      }
    } else {
      Serial.println("Starte 2");
      veml7700_messung_starten();
    }
  } else {
    // rechter Bereich von Seite 24 pdf mouser
    if (als > 10000){
      IT = IT - 1;
      if (IT == -2){
        messwerte_schreiben(false);  // normal true lt. Datenblatt, hier kommen aber dann teilweise - Werte raus.
      } else {
        Serial.println("Starte 3");
        // Hier kommt mir die Zeichnung unpassend vor, es muss doch erst neu gemessen werden?!
        veml7700_messung_starten(); // Eigene Abänderung
      }
    } else {
      messwerte_schreiben(false);  // normal true lt. Datenblatt, hier kommen aber dann teilweise - Werte raus.
    }
  }    
}


void loop() {
  // VEML7700
  G = 1;
  IT = 0;
  veml7700_messung_starten();
  Serial.print("Lux: "); Serial.println(lux_g);
  Serial.print("White: "); Serial.println(white_g);
  Serial.print("Raw ALS: "); Serial.println(als_g);
  u8g2.setFont(u8g2_font_courR14_tf);  
  u8g2.setFontDirection(0);
  u8g2.firstPage();
  do {
    u8g2.setCursor(0, 15);
    u8g2.print("Lux: ");
    u8g2.print(lux_g);
    u8g2.setCursor(0, 30);
    u8g2.print("WS:  ");
    u8g2.println(white_g);
    u8g2.setCursor(0, 45);
    u8g2.print("ALS: ");
    u8g2.println(als_g);
  } while ( u8g2.nextPage() );
  veml.enable(false);
  delay(1000);
  // Später aktivieren wenn kein Display mehr im Einsatz ist und dafür das delay eine Zeile darüber entfernen
  // ESP.deepSleep(2E6);
}