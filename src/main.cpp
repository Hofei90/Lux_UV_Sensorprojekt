//https://www.mouser.de/pdfdocs/designingveml7700.pdf

#include <Arduino.h>
#include "Adafruit_VEML7700.h"
#include <SparkFun_VEML6075_Arduino_Library.h>

#include <U8g2lib.h>


#include <Wire.h>

#define SDA1 21
#define SCL1 22

#define SDA2 18
#define SCL2 19

#include <WiFi.h>
#include <PubSubClient.h>
#include <config.h>


WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;
const char* mqtt_server = "abc";

U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
Adafruit_VEML7700 veml = Adafruit_VEML7700();

int GAIN_VEML7700;
int IT_VEML7700;

float lux_g;
float white_g;
int als_g;


VEML6075 uv; // Create a VEML6075 object

// Calibration constants:
// Four gain calibration constants -- alpha, beta, gamma, delta -- can be used to correct the output in
// reference to a GOLDEN sample. The golden sample should be calibrated under a solar simulator.
// Setting these to 1.0 essentialy eliminates the "golden"-sample calibration
const float CALIBRATION_ALPHA_VIS = 1.0; // UVA / UVAgolden
const float CALIBRATION_BETA_VIS = 1.0;  // UVB / UVBgolden
const float CALIBRATION_GAMMA_IR = 1.0;  // UVcomp1 / UVcomp1golden
const float CALIBRATION_DELTA_IR = 1.0;  // UVcomp2 / UVcomp2golden

// Responsivity:
// Responsivity converts a raw 16-bit UVA/UVB reading to a relative irradiance (W/m^2).
// These values will need to be adjusted as either integration time or dynamic settings are modififed.
// These values are recommended by the "Designing the VEML6075 into an application" app note for 100ms IT
const float UVA_RESPONSIVITY_K1 = 0.001461; // UVAresponsivity
const float UVB_RESPONSIVITY_K2 = 0.002591; // UVBresponsivity

// UV coefficients:
// These coefficients
// These values are recommended by the "Designing the VEML6075 into an application" app note
const float UVA_VIS_COEF_A = 2.22; // a
const float UVA_IR_COEF_B = 1.33;  // b
const float UVB_VIS_COEF_C = 2.95; // c
const float UVB_IR_COEF_D = 1.74;  // d

float UVI_G;
float UVA_G;
float UVB_G;


const int solarPin = 32;
const int batteriePin = 33;
float batterie_spannung;
float solar_spannung;

void read_gain(){
  Serial.print(F("Gain: "));
  switch (veml.getGain()) {
    case VEML7700_GAIN_1: Serial.println("1"); break;
    case VEML7700_GAIN_2: Serial.println("2"); break;
    case VEML7700_GAIN_1_4: Serial.println("1/4"); break;
    case VEML7700_GAIN_1_8: Serial.println("1/8"); break;
  }
}


void set_gain(int GAIN_VEML7700){
  switch (GAIN_VEML7700) {
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


void set_integration_time(int IT_VEML7700){
  switch (IT_VEML7700) {
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


void scan1(){
Serial.println("Scanning I2C Addresses Channel 1");
uint8_t cnt=0;
for(uint8_t i=0;i<128;i++){
  Wire.beginTransmission(i);
  uint8_t ec=Wire.endTransmission(true);
  if(ec==0){
    if(i<16)Serial.print('0');
    Serial.print(i,HEX);
    cnt++;
  }
  else Serial.print("..");
  Serial.print(' ');
  if ((i&0x0f)==0x0f)Serial.println();
  }
Serial.print("Scan Completed, ");
Serial.print(cnt);
Serial.println(" I2C Devices found.");
}

void scan2(){
Serial.println("Scanning I2C Addresses Channel 2");
uint8_t cnt=0;
for(uint8_t i=0;i<128;i++){
  Wire1.beginTransmission(i);
  uint8_t ec=Wire1.endTransmission(true);
  if(ec==0){
    if(i<16)Serial.print('0');
    Serial.print(i,HEX);
    cnt++;
  }
  else Serial.print("..");
  Serial.print(' ');
  if ((i&0x0f)==0x0f)Serial.println();
  }
Serial.print("Scan Completed, ");
Serial.print(cnt);
Serial.println(" I2C Devices found.");
}


float calculation_lux(float lux){
  Serial.println("Calc Lux");
  float x = lux;
  Serial.print("Raw Lux: ");
  Serial.println(lux);
  float lux_calc = (1E-09 * pow(x, 3)) + (3E-06 * pow(x, 2)) + (1.0564 * x) - 30.803;
  if (lux_calc < lux){
    return lux;
  } else {
    return lux_calc;
  }
}


void veml7700_messwerte_schreiben(bool calc){
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
  set_integration_time(IT_VEML7700);
  set_gain(GAIN_VEML7700);
  veml.enable(true);
  delay(3000);
  int als = veml.readALS();
  Serial.print("ALS: ");
  Serial.println(als);
  if (als <= 100) {
    GAIN_VEML7700 = GAIN_VEML7700 + 1;
    if (GAIN_VEML7700 > 4){
      GAIN_VEML7700 = 4;
    }
    Serial.print("G: ");
    Serial.println(GAIN_VEML7700);
    if (GAIN_VEML7700 == 4) {
      IT_VEML7700 = IT_VEML7700 + 1;
      if (IT_VEML7700 > 4){
        IT_VEML7700 = 4;
      }
      if (IT_VEML7700 == 4){
        veml7700_messwerte_schreiben(false);
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
      IT_VEML7700 = IT_VEML7700 - 1;
      if (IT_VEML7700 == -2){
        veml7700_messwerte_schreiben(true);
      } else {
        Serial.println("Starte 3");
        // Hier kommt mir die Zeichnung unpassend vor, es muss doch erst neu gemessen werden?!
        veml7700_messung_starten(); // Eigene Abänderung
      }
    } else {
      veml7700_messwerte_schreiben(true);
    }
  }    
}

void veml6075_messung_schreiben(float uvi, float uva, float uvb){
  UVI_G = uvi;
  UVA_G = uva;
  UVB_G = uvb;
}

void veml6075_messung_starten(){
  uint16_t rawA, rawB, visibleComp, irComp;
  float UVAcalc, UVBcalc, uvia, uvib, uvi;

  // Read raw and compensation data from the sensor
  rawA = uv.rawUva();
  rawB = uv.rawUvb();
  visibleComp = uv.visibleCompensation();
  irComp = uv.irCompensation();

  // Calculate the simple UVIA and UVIB. These are used to calculate the UVI signal.
  UVAcalc = rawA - UVA_VIS_COEF_A * visibleComp - UVA_IR_COEF_B * irComp;
  UVBcalc = rawB - UVB_VIS_COEF_C * visibleComp - UVB_IR_COEF_D * irComp;

  // Convert raw UVIA and UVIB to values scaled by the sensor responsivity
  uvia = UVAcalc * UVA_RESPONSIVITY_K1; 
  uvib = UVBcalc * UVB_RESPONSIVITY_K2;

  // Use UVIA and UVIB to calculate the average UVI:
  uvi = (uvia + uvib) / 2.0;
  veml6075_messung_schreiben(uvi, uvia, uvib);
}


void werte_mqtt_senden(){
  client.loop();
  snprintf(msg, 50, "%f", lux_g);
  client.publish("sensoreinheit/1/lux", msg);
  snprintf(msg, 50, "%f", UVI_G);
  client.publish("sensoreinheit/1/uvi", msg);
  snprintf(msg, 50, "%f", UVA_G);
  client.publish("sensoreinheit/1/uva", msg);
  snprintf(msg, 50, "%f", UVB_G);
  client.publish("sensoreinheit/1/uvb", msg);
  snprintf(msg, 50, "%f", solar_spannung);
  client.publish("sensoreinheit/1/solarspannung", msg);
  snprintf(msg, 50, "%f", batterie_spannung);
  client.publish("sensoreinheit/1/batteriespannung", msg);
}


float adc_umrechner(float adc){
  float max_value = 3.3;
  float max_bit = 4095.0;
  Serial.print("max_value: "); Serial.println(max_value);
  Serial.print("max_bit:"); Serial.println(max_bit);
  Serial.print("adc: "); Serial.println(adc);
  float value = (float)adc / max_bit * max_value;
  Serial.print("value: "); Serial.println(value);
  return value;
}


void setup() {
  u8g2.begin();
  u8g2.enableUTF8Print();		// enable UTF8 support for the Arduino print() function

  Serial.begin(115200);
  Wire.begin(); 
  Wire1.begin(SDA2,SCL2,400000);
  delay(100);
  scan1();
  scan2();

  Serial.println("Adafruit VEML7700 Test");

  if (!veml.begin()) {
    Serial.println("Sensor not found");
    while (1);
  }  
  veml.powerSaveEnable(true);
  veml.setPowerSaveMode(VEML7700_POWERSAVE_MODE3);

  if (!uv.begin(Wire1)){
    Serial.println("Unable to communicate with VEML6075.");
    while (1)
      ;
  }
  // Integration time and high-dynamic values will change the UVA/UVB sensitivity. That means
  // new responsivity values will need to be measured for every combination of these settings.
  uv.setIntegrationTime(VEML6075::IT_100MS);
  uv.setHighDynamic(VEML6075::DYNAMIC_NORMAL);

  connect_to_wlan();
  client.setServer(mqtt_server, 1833);

}


void loop() {

  // VEML7700
  GAIN_VEML7700 = 1;
  IT_VEML7700 = 0;
  veml7700_messung_starten();
  veml6075_messung_starten();
  solar_spannung = adc_umrechner(analogRead(solarPin));
  batterie_spannung = adc_umrechner(analogRead(batteriePin));
  Serial.println("---------VEML7700---------");
  Serial.print("Lux: "); Serial.println(lux_g);
  Serial.print("White: "); Serial.println(white_g);
  Serial.print("Raw ALS: "); Serial.println(als_g);

  Serial.println("---------VEML6075---------");
  Serial.print("UV-Index: "); Serial.println(UVI_G);
  Serial.print("UVA: "); Serial.println(UVA_G);
  Serial.print("UVB: "); Serial.println(UVB_G);

  Serial.println("---------Spannungen---------");
  Serial.print("Batterie: "); Serial.print(batterie_spannung); Serial.println("V");
  Serial.print("Solar: "); Serial.print(solar_spannung); Serial.println("V");

  u8g2.setFont(u8g2_font_courR14_tf);  
  u8g2.setFontDirection(0);
  u8g2.firstPage();
  do {
    u8g2.setCursor(0, 15);
    u8g2.print("Lux: "); u8g2.print(lux_g);
    u8g2.setCursor(0, 30);
    u8g2.print("ALS: "); u8g2.println(als_g);
    u8g2.setCursor(0, 45);
    u8g2.print("UVI: "); u8g2.println(UVI_G);
    u8g2.setCursor(0, 60);
    u8g2.print("A:"); u8g2.print(UVA_G); u8g2.print("B:"); u8g2.println(UVB_G);
  } while ( u8g2.nextPage() );
  veml.enable(false);

  delay(2000);
  // Später aktivieren wenn kein Display mehr im Einsatz ist und dafür das delay eine Zeile darüber entfernen
  // ESP.deepSleep(2E6);
}