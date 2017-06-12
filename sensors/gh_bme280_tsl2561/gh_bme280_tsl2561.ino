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
#define SLEEP_PERIOD 600
#define GPIO_LED 5
#define GPIO_ENABLE_LED true
#define EEPROM_CONFIG_ADDR 0

#define MQTT_TOPIC_TEMP "/home/gh1/temp"
#define MQTT_TOPIC_RELH "/home/gh1/relh"
#define MQTT_TOPIC_LUX  "/home/gh1/lux"
#define MQTT_RETRY 10

#define WIFI_APNAME "homeGH1"
#define WIFI_APPASS "zNQHe3nQJW"
#define WIFI_IP 192,168,1,211
#define WIFI_GW 192,168,1,1
#define WIFI_SN 255,255,255,0

#define DEBUG false
#define DEBUG_NOSLEEP false

#define BME280_ADDRESS 0x77

#define TSL2561_ADDRESS 0x39
#define TSL2561_GAIN 1
#define TSL2561_TIME 0

/*
 * Debug logging
 */

#if DEBUG
#define DPRINT(...) Serial.print(__VA_ARGS__)
#define DPRINTL(...) Serial.println(__VA_ARGS__)
#else
#define DPRINT(...)
#define DPRINTL(...)
#endif

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
    char client[15];
    char portS[6];
    uint16_t port;
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

  Serial.begin(115200);
  EEPROM.begin(sizeof(MqttCreds) + 1);

#if GPIO_ENABLE_LED
  pinMode(GPIO_LED, OUTPUT);
  digitalWrite(GPIO_LED, LOW);
#endif

  loadMqttConfig();

  if (setupWifi() && setupSensors()) {
    SensorVals sensorVals = readSensors();
    sendSensorVals(sensorVals);
  }

  sleepNow();
}

/*
 * Load MQTT server config from EEPROM
 */
void loadMqttConfig() {
  EEPROM.get(EEPROM_CONFIG_ADDR, mqtt_creds);

  DPRINTL(F("Loaded MQTT config: "));
  DPRINT(F("Server: "));
  DPRINTL(mqtt_creds.server);
  DPRINT(F("Port: "));
  DPRINTL(mqtt_creds.port);
  DPRINT(F("Client ID: "));
  DPRINTL(mqtt_creds.client);
}

/*
 * Flash the onboard (or external) LED
 */
void flashLed(uint8_t count, uint16_t period = 100, uint8_t repeat = 1) {
#if GPIO_ENABLE_LED
  while (repeat > 0) {
    for (uint8_t i = 0; i < count; i++) {
      delay(period);
      digitalWrite(GPIO_LED, HIGH);
      delay(period);
      digitalWrite(GPIO_LED, LOW);
    }
    repeat--;

    if (repeat > 0) {
      delay(500);
    }
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
    DPRINTL(F("Invalid Lux reading."));
    flashLed(3);
  }

  sensorVals.tempf = air.readTempF();
  sensorVals.relh = air.readFloatHumidity();

  DPRINTL(F("Read sensor values:"));
  DPRINT(F("Temp: "));
  DPRINTL(sensorVals.tempf);
  DPRINT(F("RelH: "));
  DPRINTL(sensorVals.relh);
  DPRINT(F("Lux: "));
  DPRINTL(sensorVals.lux);

  return(sensorVals);
}

/*
 * Send Sensor value MQTT packets.
 */
void sendSensorVals(SensorVals sensorVals) {
  uint8_t retries = 0;
  client.setServer(mqtt_creds.server, mqtt_creds.port);

  DPRINT(F("Connecting to MQTT Server "));
  while ((!client.connected()) && (retries < MQTT_RETRY)) {
    if (client.connect(mqtt_creds.client)) {
      snprintf(mqttMsg, 20, "%d", lround(sensorVals.tempf));
      client.publish(MQTT_TOPIC_TEMP, mqttMsg);

      snprintf(mqttMsg, 20, "%d", lround(sensorVals.relh));
      client.publish(MQTT_TOPIC_RELH, mqttMsg);

      snprintf(mqttMsg, 20, "%d", lround(sensorVals.lux));
      client.publish(MQTT_TOPIC_LUX, mqttMsg);

      DPRINTL(F(". DONE"));
      flashLed(1, 750);
    } else {
      DPRINT(F("."));
      retries++;
      delay(2000);
    }
  }

  if (retries >= MQTT_RETRY) {
    DPRINTL(F("FAILED"));
    flashLed(5, 100, 3);
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
  air.settings.runMode = 1;
  air.settings.tStandby = 5;
  air.settings.filter = 0;
  air.settings.tempOverSample = 1;
  air.settings.pressOverSample = 1;
  air.settings.humidOverSample = 1;

  delay(10);

  if (!air.begin()) {
    DPRINTL(F("BME280 Setup Failed."));
    flashLed(5, 100, 5);
    return(false);
  }

  delay(10);

  // Setup TSL2561
  light.begin(TSL2561_ADDRESS);

  if (!light.getID(lightID)) {
    DPRINTL(F("TSL2561 Setup Failed."));
    flashLed(5, 100, 5);
    return(false);
  }

  light.setTiming(TSL2561_GAIN, TSL2561_TIME, integrationTime);
  light.setPowerUp();

  DPRINTL(F("Sensors configured, waiting for results."));

  delay(integrationTime);
  return(true);
}

/*
 * Setup Wifi Connection or config AP
 */
bool setupWifi() {
  DPRINTL(F("Configuring WiFi:"));
  WiFiManagerParameter mqtt_server(
    "server",
    "mqtt server",
    mqtt_creds.server,
    40
  );

  WiFiManagerParameter mqtt_port(
    "port",
    "mqtt port",
    mqtt_creds.portS,
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
    DPRINTL(F("WiFi connection failed."));
    flashLed(5, 100, 2);
    return(false);
  }

  // Gather config values
  if (saveConfig) {
    DPRINTL(F("Saving new MQTT config."));

    strncpy(mqtt_creds.server, mqtt_server.getValue(), 40);
    strncpy(mqtt_creds.client, mqtt_client.getValue(), 15);
    strncpy(mqtt_creds.portS, mqtt_port.getValue(), 6);
    mqtt_creds.port = atol(mqtt_creds.portS);

    EEPROM.put(EEPROM_CONFIG_ADDR, mqtt_creds);
    delay(10);
    EEPROM.commit();
  }

  DPRINTL(F("Gathered MQTT config: "));
  DPRINT(F("Server: "));
  DPRINTL(mqtt_creds.server);
  DPRINT(F("Port: "));
  DPRINTL(mqtt_creds.port);
  DPRINT(F("Client ID: "));
  DPRINTL(mqtt_creds.client);

  if (!strlen(mqtt_creds.server)) {
    DPRINTL(F("Unable to load MQTT server config, resetting."));
    wifiManager.resetSettings();
    delay(1000);
    ESP.restart();
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
  DPRINTL(F("Powering down sensors..."));
  light.setPowerDown();
  delay(10);

#if DEBUG_NOSLEEP
  DPRINTL(F("Restarting ESP..."));
  delay(5000);
  ESP.restart();
#else
  DPRINTL(F("Sleeping ESP..."));
  ESP.deepSleep(SLEEP_PERIOD * 1000000);
#endif
}

/*
 * Empty loop (sleep / reset during setup).
 */
void loop() { }

// vim:cin:ai:sts=2 sw=2 ft=cpp
