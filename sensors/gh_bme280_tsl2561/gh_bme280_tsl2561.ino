/*
 * gh_bme_280_tsl2561.ino
 *
 * Home automation MQTT greenhouse state (temp / humidity / light) sensor using
 * BME280 and TSL2561 sensors on an ESP8266 host.
 *
 * Copyright (c) 2017, James Watts
 * All rights reserved, no unauthorized distribution or reproduction.
 *
 * See LICENSE.txt for details.
 */

#include <EEPROM.h>
#include <Wire.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>
#include <SparkFunTSL2561.h>
#include <SparkFunBME280.h>

/*
 * Configuration
 */
#define SLEEP_PERIOD 300
#define GPIO_LED 5
#define GPIO_ENABLE_LED true
#define EEPROM_CONFIG_ADDR 0

#define MQTT_TOPIC_TEMP "/home/gh1/temp"
#define MQTT_TOPIC_RELH "/home/gh1/relh"
#define MQTT_TOPIC_LUX  "/home/gh1/lux"
#define MQTT_RETRY 10

#define WIFI_APNAME "greenhouseS"
#define WIFI_APPASS "zNQHe3nQJW"
#define WIFI_IP 192,168,1,220
#define WIFI_GW 192,168,1,1
#define WIFI_SN 255,255,255,0

#define DEBUG_NOSLEEP false

#define BME280_ADDRESS 0x77

#define TSL2561_ADDRESS 0x39
#define TSL2561_GAIN 0
#define TSL2561_TIME 2

/*
 * Sensor value storage
 */
class SensorVals {
  public:
    float relh = 0;
    float tempf = 0;
    double lux = 0;
};

/*
 * MQTT Connection Info
 */
class MqttCreds {
  public:
    char server[40];
    char port[6];
    char client[15];
} mqtt_creds;

bool saveConfig = false;

/*
 * Sensors
 */
SFE_TSL2561 light;
BME280 air;
unsigned int integrationTime;

/*
 * MQTT Client
 */
WiFiClient espClient;
PubSubClient client(espClient);
char mqttMsg[20];

/*
 * Setup / Main Loop
 */
void setup() {

#if GPIO_ENABLE_LED
  pinMode(GPIO_LED, OUTPUT);
  digitalWrite(GPIO_LED, LOW);
#endif

  EEPROM.get(EEPROM_CONFIG_ADDR, mqtt_creds);

  if (setupWifi() && setupSensors()) {
    SensorVals sensorVals = readSensors();
    sendSensorVals(sensorVals);
  }

  sleepNow();
}

/*
 * Flash the onboard (or external) LED
 */
void flashLed(uint8_t count, uint8_t period = 100) {
#if GPIO_ENABLE_LED
  for (uint8_t i = 0; i < count; i++) {
    delay(period);
    digitalWrite(GPIO_LED, HIGH);
    delay(period);
    digitalWrite(GPIO_LED, LOW);
  }
#endif
}

/*
 * Read sensor values.
 */
SensorVals readSensors() {
  SensorVals sensorVals;
  unsigned int ld0, ld1;
  bool validRes = false;

  if (light.getData(ld0, ld1)) {
    validRes = light.getLux(TSL2561_GAIN, integrationTime, ld0, ld1, sensorVals.lux);
  }

  if (!validRes) {
    sensorVals.lux = 0;
    flashLed(8, 250);
  }

  sensorVals.tempf = air.readTempF();
  sensorVals.relh = air.readFloatHumidity();

  return(sensorVals);
}

/*
 * Send Sensor value MQTT packets.
 */
void sendSensorVals(SensorVals sensorVals) {
  uint8_t retries = 0;
  client.setServer(mqtt_creds.server, mqtt_creds.port);

  while ((!client.connected()) && (retries < MQTT_RETRY)) {
    if (client.connect(mqtt_creds.client)) {
      snprintf(mqttMsg, 20, "%0.2f", sensorVals.tempf);
      client.publish(MQTT_TOPIC_TEMP, mqttMsg);

      snprintf(mqttMsg, 20, "%0.2f", sensorVals.relh);
      client.publish(MQTT_TOPIC_RELH, mqttMsg);

      snprintf(mqttMsg, 20, "%0.2f", sensorVals.lux);
      client.publish(MQTT_TOPIC_LUX, mqttMsg);

      flashLed(2, 500);
    } else {
      retries++;
      delay(2000);
    }
  }

  client.loop();
}

/*
 * Setup I2c sensors.
 */
bool setupSensors() {
  unsigned char lightID;

  // Setup BME280
  air.settings.commInterface = I2C_MODE;
  air.settings.I2CAddress = BME280_ADDRESS;
  air.settings.runMode = 3;
  air.settings.tStandby = 0;
  air.settings.filter = 0;
  air.settings.tempOverSample = 1;
  air.settings.pressOverSample = 1;
  air.settings.humidOverSample = 1;

  delay(10);

  if (!air.begin()) {
    flashLed(20, 200);
    return(false);
  }

  delay(10);

  // Setup TSL2561
  light.begin(TSL2561_ADDRESS);

  if (!light.getID(lightID)) {
    flashLed(20, 200);
    return(false);
  }

  light.setTiming(TSL2561_GAIN, TSL2561_TIME, integrationTime);
  light.setPowerUp();

  delay(integrationTime);
  return(true);
}

/*
 * Setup Wifi Connection or config AP
 */
bool setupWifi() {
  WiFiManagerParameter mqtt_server(
    "server",
    "mqtt server",
    mqtt_creds.server,
    40
  );

  WiFiManagerParameter mqtt_port(
    "port",
    "mqtt port",
    mqtt_creds.port,
    6
  );

  WiFiManagerParameter mqtt_client(
    "client",
    "mqtt client ID",
    mqtt_creds.client,
    15
  );

  WiFiManager wifiManager;
  wifiManager.setSaveConfigCallback(saveConfigCallback);

#ifdef WIFI_IP
  wifiManager.setSTAStaticIPConfig(
    IPAddress(WIFI_IP),
    IPAddress(WIFI_GW),
    IPAddress(WIFI_SN)
  );
#endif

  wifiManager.addParameter(&mqtt_server);
  wifiManager.addParameter(&mqtt_port);
  wifiManager.addParameter(&mqtt_client);

  if (!wifiManager.autoConnect(WIFI_APNAME, WIFI_APPASS)) {
    flashLed(10);
    return(false);
  }

  flashLed(5,25);

  // Gather config values
  strncpy(mqtt_creds.server, mqtt_server.getValue(), 40);
  strncpy(mqtt_creds.port, mqtt_port.getValue(), 6);
  strncpy(mqtt_creds.client, mqtt_port.getValue(), 15);

  if (saveConfig) {
    EEPROM.put(EEPROM_CONFIG_ADDR, mqtt_creds);
  }

  return(true);
}

/*
 * Callback for WiFiManager config save.
 */
void saveConfigCallback() {
  saveConfig = true;
}

/*
 * Sleep (or restart) after sending.
 */
void sleepNow() {
#if DEBUG_NOSLEEP
  delay(5000);
  ESP.restart();
#else
  ESP.deepSleep(SLEEP_PERIOD * 1000000);
#endif
}

/*
 * Empty loop (sleep / reset during setup).
 */
void loop() { }

// vim:cin:ai:sts=2 sw=2 ft=cpp
