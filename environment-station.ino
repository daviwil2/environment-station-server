/*
   Usable pins used
     D1 I2C SCL
     D2 I2C SDA
     D3 -
     D4 not physically connected but used as a phantom TX for Software Serial
     D5 1 wire input for DHT22 temperature and humidity sensor
     D6 Software Serial RX
     D7 was genuine Software Serial TX but repurposed as GPIO pin to control power to atmosphere sensor

 * */

#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <NTPClient.h>        // https://github.com/arduino-libraries/NTPClient
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <PubSubClient.h>     // for MQTT communications

#include <BH1750.h>           // BH1750 ambient light
#include <LOLIN_HP303B.h>     // DPS310/HP303B barometric pressure, https://github.com/wemos/LOLIN_HP303B_Library
#include <DHT_U.h>            // DHT22 outdoor temperature and humidity
#include <DFRobot_VEML6075.h> // VEML6075 UVA/B
#include <DFRobot_MAX17043.h> // MAX17043 battery gauge 

#include "credentials.h"

// setup software serial on pin D6 (RX) for the air detector module
#define SWSerialRX D6
#define SWSerialTX D4 // D4 isn't physically conencted, this is just a dummy so SoftwareSerial instantiates correctly
#define LENG 31   //0x42 + 31 bytes equal to 32 bytes
unsigned char buf[LENG];
int PM01Value  = 0; //define PM1.0 value of the air detector module
int PM2_5Value = 0; //define PM2.5 value of the air detector module, <25 safe 24h per WHO 2018
int PM10Value  = 0; //define PM10  value of the air detector module, <50 safe 24h per WHO 2018
SoftwareSerial PMSerial(SWSerialRX, SWSerialTX); // RX, TX

// setup WiFi and MQTT; the constants are defined in the config.h file which is imported above
const char* ssid     = WIFI_SSID;
const char* password = WIFI_PASSWORD;
int         conn_tries = 0;
#define WIFI_RETRIES 10
WiFiClient espClient;           // define espClient to use WiFi
PubSubClient client(espClient); // use espClient as an MQTT client

// setup MQTT parameters
const char*   ROOT_TOPIC    = "environment-station/";
const char*   SUB_TOPIC     = "environment-station/JIC21V01/";
const bool    RETAINED      = true;
const int     BASE          = 10;
const char*   MQTT_SERVER   = "192.168.1.32";
const int     MQTT_PORT     = 1883;
char          topic[256];
char          payload[32];
bool          mqttResponse;

// setup NTP; functions are .getEpochTime() or .getFormattedTime()
WiFiUDP ntpUDP;
NTPClient ntpClient(ntpUDP, "0.ubnt.pool.ntp.org", 14400, 60000); // time server pool, offset (in seconds) and update interval (in milliseconds)

// setup sensors
#define DHTPIN D5
#define DHTTYPE DHT22
DHT_Unified dht(DHTPIN, DHTTYPE);           // DHT22 on pin D5 (1 wire)
DFRobot_VEML6075_IIC VEML6075(&Wire, 0x10); // VEML6075 on I2C address 0x10
BH1750 BH1750Sensor(0x23);                  // BH1750Sensor on I2C address 0x23
LOLIN_HP303B HP303BPressureSensor;          // HP303BPressureSensor on I2C address 0x77
DFRobot_MAX17043 batteryGauge;              // MAX17043 on I2C address 0x36

// SLEEP is in seconds, 60s * 30m = 1800s; can be manually booted for a read by pressing the reset button on the Wemos D1
#define SLEEP 1800
#define STARTUP_DELAY 10
#define POWER_5V_PIN D7

/* ********************************************************************************** */

void setup()
{

  // setup serial port for the serial monitor; even if not connected to USB (/dev/tty-usb...) this will still come ready so not hang
  Serial.begin(115200);
  while (!Serial) {
    delay(100); // wait 100ms = 0.1s before checking again
  }; // while

  Serial.println("\nSerial is available, starting");

  // D0 has WAKE feature, linked to RST with a resistor to allow awakening from deep sleep
  pinMode(D0, WAKEUP_PULLUP);

  // start air detector by turning on 5V power from the solar power manager
  pinMode(POWER_5V_PIN, OUTPUT); // we use this pin to control the 5V (orange wire)
  Serial.println("Turning on 5V power on solar power manager board (D7 pin HIGH)");
  digitalWrite(POWER_5V_PIN, HIGH);

  // setup the software serial port so we can read from the air detector module
  Serial.println("Starting software serial");
  PMSerial.begin(9600);
  PMSerial.setTimeout(1500);

  connectToWiFi(); // connect to the WiFi network
  reconnectMQTT(); // connect to the MQTT server

  // start/initialize sensors
  HP303BPressureSensor.begin(); // HP303B barometric pressure sensor
  BH1750Sensor.begin();         // BH1750 ambient light sensor
  dht.begin();                  // DHT22 temperature and pressure sensor
  VEML6075.begin();             // VEML6075 UVA/B sensor
  while (batteryGauge.begin() != 0) {
    delay(500); // wait 100ms = 0.1s before checking again
    Serial.println("MAX17043 battery gauge not found on I2C bus");
  }; // while

  Serial.println("Finished setup");

} // setup()

/* ********************************************************************************** */

void connectToWiFi() {

  // start WiFi and wait until we're connected
  WiFi.forceSleepWake();
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(100); // wait 100ms = 0.1s before checking again
  }; // while

  Serial.println("WiFi connected, starting NTP and MQTT");

  // start NTP client
  ntpClient.begin();

  // initialise MQTT client connection
  client.setServer(MQTT_SERVER, MQTT_PORT);

}; // connectToWiFi()

void reconnectMQTT() {

  // connect to MQTT: use WemosD1 as the client ID, username and password are NULL
  while (!client.connected()) {
    if (!client.connect("WemosD1", NULL, NULL) == true) {
      delay(100); // wait 100ms = 0.1s before checking again
    }; // if
    Serial.println("Connected to MQTT server as client \"WemosD1\"");
  }; // while

  // set the type of service for discovery purposes
  strcpy(topic, ROOT_TOPIC);
  strcat(topic, "type");
  mqttResponse = client.publish(topic, "environment-station", RETAINED); // this is published as retained message so is returned first, irrespective of whether data is being published on this topic

} // reconnectMQTT()

/**** read data from various sources e.g. sensors and send to the server over MQTT ****/

void sendCoreData() {

  Serial.println("Sending time");

  // time stamp with Unix epoch time
  strcpy(topic, SUB_TOPIC);
  strcat(topic, "timestamp");
  itoa(ntpClient.getEpochTime(), payload, BASE);
  mqttResponse = client.publish(topic, payload, RETAINED); // retained message so the topic remains visible to subscribers

  Serial.println("Sending battery data");

  // MAX17043 battery gauge

  float batteryVoltage = batteryGauge.readVoltage();
  if (batteryVoltage > 0) {
    strcpy(topic, SUB_TOPIC);
    strcat(topic, "battery_voltage");
    sprintf(payload, "%f", batteryVoltage / 1000);
    Serial.print("Battery voltage ");
    Serial.print(payload);
    Serial.println("V");
    mqttResponse = client.publish(topic, payload, RETAINED);
  } else {
    Serial.println("Battery voltage zero, either empty or an error reading");
  }; // if

  float batteryPercentage = batteryGauge.readPercentage();
  if (batteryPercentage > 0) {
    strcpy(topic, SUB_TOPIC);
    strcat(topic, "battery_percentage");
    sprintf(payload, "%f", batteryPercentage);
    Serial.print("Battery charge at ");
    Serial.print(payload);
    Serial.println("%");
    mqttResponse = client.publish(topic, payload, RETAINED);
  } else {
    Serial.println("Battery percentage zero, either empty or an error reading");
  }; // if

} // sendCoreData()

/*

   Air Detector / Atmospheric Sensor to read PM1.0, PM2.5 and PM10 values over Software Serial (PMSerial)

*/
void readAndSendAtmosphericSensorData() {

  Serial.print("Waiting ");
  Serial.print(STARTUP_DELAY);
  Serial.println(" seconds for atmosphere sensor to stabilise and give reliable data");
  delay(STARTUP_DELAY * 1000);

  Serial.println("Looking for atmospheric sensor data");

  if (PMSerial.find(0x42)) {
    PMSerial.readBytes(buf, LENG);
    if (buf[0] == 0x4d) {
      if (checkValue(buf, LENG)) {

        Serial.println("Valid atmospheric sensor data found, reading values");

        PM01Value  = readPM01(buf);  //count PM1.0 value of the air detector module
        PM2_5Value = readPM2_5(buf); //count PM2.5 value of the air detector module
        PM10Value  = readPM10(buf);  //count PM10 value of the air detector module

        Serial.println("Sending atmospheric sensor data to MQTT server");

        strcpy(topic, SUB_TOPIC);
        strcat(topic, "PM1.0");
        sprintf(payload, "%i", PM01Value);
        Serial.print("PM1.0 is ");
        Serial.print(PM01Value);
        Serial.println("ppm");
        mqttResponse = client.publish(topic, payload, RETAINED);

        strcpy(topic, SUB_TOPIC);
        strcat(topic, "PM2.5");
        sprintf(payload, "%i", PM2_5Value);
        Serial.print("PM2.5 is ");
        Serial.print(PM2_5Value);
        Serial.println("ppm");
        mqttResponse = client.publish(topic, payload, RETAINED);

        strcpy(topic, SUB_TOPIC);
        strcat(topic, "PM10");
        sprintf(payload, "%i", PM10Value);
        Serial.print("PM10 is ");
        Serial.print(PM10Value);
        Serial.println("ppm");
        mqttResponse = client.publish(topic, payload, RETAINED);

      } // if checkValue
    } // if buf[0]
  } else {
    Serial.println("Atmospheric sensor couldn\'t be found (leading 0x42 not found on PMSerial)");
  } // if PMserial

} // readAndSendAtmosphericSensorData()

char checkValue(unsigned char *thebuf, char leng) {

  char receiveflag = 0;
  int receiveSum = 0;

  for (int i = 0; i < (leng - 2); i++) {
    receiveSum = receiveSum + thebuf[i];
  }; // for
  receiveSum = receiveSum + 0x42;

  //check the serial data
  if (receiveSum == ((thebuf[leng - 2] << 8) + thebuf[leng - 1])) {
    receiveSum = 0;
    receiveflag = 1;
  };

  return receiveflag;

} // checkValue

int readPM01(unsigned char *thebuf) {
  int PM01Val;
  PM01Val = ((thebuf[3] << 8) + thebuf[4]); //count PM1.0 value of the air detector module
  return PM01Val;
} // int readPM01

int readPM2_5(unsigned char *thebuf) {
  int PM2_5Val;
  PM2_5Val = ((thebuf[5] << 8) + thebuf[6]); //count PM2.5 value of the air detector module
  return PM2_5Val;
} // int readPM2_5

int readPM10(unsigned char *thebuf) {
  int PM10Val;
  PM10Val = ((thebuf[7] << 8) + thebuf[8]); //count PM10 value of the air detector module
  return PM10Val;
} // int readPM10

// BH1750 (I2C) ambient light
void readAndSendAmbientLightData() {

  Serial.println("Reading and sending data for ambient light");

  // read data from BH1750Sensor
  float lux = BH1750Sensor.readLightLevel();
  strcpy(topic, SUB_TOPIC);
  strcat(topic, "light");
  sprintf(payload, "%f", lux);
  mqttResponse = client.publish(topic, payload, RETAINED);

} // readAndSendAmbientLightData()

// DPS310/HP303B (I2C) barometric pressure
void readAndSendBarometricPressureData() {

  Serial.println("Looking for barometric pressure data");

  // read data from HP303BPressureSensor; the sensor provides both barometric pressure and temperature sensors but we'll use only pressure
  int32_t pressure;
  int16_t oversampling = 7;
  int16_t ret;
  ret = HP303BPressureSensor.measurePressureOnce(pressure, oversampling);
  if (ret == 0) {
    Serial.println("Sending barometric pressure data to MQTT server");
    strcpy(topic, SUB_TOPIC);
    strcat(topic, "pressure");
    sprintf(payload, "%i", pressure / 100);
    mqttResponse = client.publish(topic, payload, RETAINED);
  } // if

} // readAndSendBarometricPressureData()

// DHT22 (1 wire) temperature and humidity sensor
void readAndSendTempAndHumidityData() {

  Serial.println("Looking for temperature and pressure data");

  // read data payload from DHT22 sensor
  sensors_event_t event;

  // temperature
  dht.temperature().getEvent(&event);
  strcpy(topic, SUB_TOPIC);
  strcat(topic, "temperature");
  if (!isnan(event.temperature)) {
    Serial.println("Sending temperature data to MQTT server");
    sprintf(payload, "%f", event.temperature);
  } else {
    Serial.println("Couldn\'t read temperature data");
    sprintf(payload, "%f", 0);
  }; // if
  mqttResponse = client.publish(topic, payload, RETAINED);

  // humidity
  dht.humidity().getEvent(&event);
  strcpy(topic, SUB_TOPIC);
  strcat(topic, "humidity");
  if (!isnan(event.relative_humidity)) {
    Serial.println("Sending humidity data to MQTT server");
    sprintf(payload, "%f", event.relative_humidity);
  } else {
    Serial.println("Couldn\'t read humidity data");
    sprintf(payload, "%f", 0);
  }; // if
  mqttResponse = client.publish(topic, payload, RETAINED);

} // readAndSendTempAndHumidityData()

void readAndSendUVData() {

  uint16_t    UvaRaw = VEML6075.readUvaRaw();         // read UVA raw
  uint16_t    UvbRaw = VEML6075.readUvbRaw();         // read UVB raw
  uint16_t    comp1Raw = VEML6075.readUvComp1Raw();   // read COMP1 raw
  uint16_t    comp2Raw = VEML6075.readUvComp2Raw();   // read COMP2 raw

  float       Uva = VEML6075.getUva();                // get UVA
  float       Uvb = VEML6075.getUvb();                // get UVB
  float       Uvi = VEML6075.getUvi(Uva, Uvb);        // get UV index

  Serial.println("Reading and sending data for UVA, UVB and UV Index");

  strcpy(topic, SUB_TOPIC);
  strcat(topic, "UVA");
  sprintf(payload, "%f", Uva);
  mqttResponse = client.publish(topic, payload, RETAINED);

  strcpy(topic, SUB_TOPIC);
  strcat(topic, "UVB");
  sprintf(payload, "%f", Uvb);
  mqttResponse = client.publish(topic, payload, RETAINED);

  strcpy(topic, SUB_TOPIC);
  strcat(topic, "UVIndex");
  sprintf(payload, "%f", Uvi);
  mqttResponse = client.publish(topic, payload, RETAINED);

} // readAndSendUVData()

/* ********************************************************************************** */

void loop() {

  Serial.print("\n"); // print a blank line as this is a new run of the loop

  // update the time from the NTP server pool
  ntpClient.update();

  // if not connected to the MQTT server call the reconnect function
  if (!client.connected()) {
    reconnectMQTT();
  }; // if

  sendCoreData();

  readAndSendAmbientLightData();
  readAndSendBarometricPressureData();
  readAndSendTempAndHumidityData();
  readAndSendUVData();
  readAndSendAtmosphericSensorData();

  Serial.println("Turning off 5V power on solar power manager board (D7 pin LOW)");
  digitalWrite(POWER_5V_PIN, LOW);
  delay(500); // wait 500ms for the pin to fall low

  // check for MQTT messages
  client.loop();

  // disconnect from MQTT
  client.disconnect();

  // pins GPIO16 (D0) to RST are bridged using a resistor to allow wake from deep sleep
  // so send the ESP8266 to sleep for the specified period

  Serial.print("Finished loop, going to sleep for ");
  Serial.print(SLEEP);
  Serial.println(" seconds");

  ESP.deepSleep(SLEEP * 1000000, WAKE_RF_DEFAULT); // sleep the device for 60 * 1M uS = 60s

  // delay(2500); // DEBUG! wait 2.5s

}
