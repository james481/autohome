/*
 * door_vl6180.ino
 *
 * Home automation MQTT door state (lock, open / closed) sensor using a VL6180
 * TOF sensor on an ESP8266 host.
 *
 * Copyright (c) 2017, James Watts
 * All rights reserved, no unauthorized distribution or reproduction.
 *
 * See LICENSE.txt for details.
 */

#include <Wire.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <SparkFun_VL6180X.h>
#include "wifi_creds.h"
#include "mqtt_creds.h"

/*
 * Configuration
 */
#define LOCK_DISTANCE 40
#define LOCK_TOLERENCE 10

#define DEBUG true
#define DEBUGOUT Serial
#define DEBUG_NOSLEEP false

#define GPIO_DOOR_SWITCH 4
#define GPIO_LED 5
#define GPIO_ENABLE_LED true

#define SLEEP_PERIOD 60
#define MQTT_TOPIC_LOCK "/home/doors/front/lock"
#define MQTT_TOPIC_DOOR "/home/doors/front/state"
#define MQTT_RETRY 10
#define WIFI_RETRY 20

#define VL6180X_ADDRESS 0x29

/*
 * Lock / Door States
 */
enum LockState: uint8_t {
  unlocked = 0,
  locked   = 1,
  other    = 2,
};

enum DoorState: uint8_t {
  open   = 0,
  closed = 1,
};

/*
 * VL6180x Lock class
 */
class VL6180Lock: public VL6180x
{
  public:
    VL6180Lock(uint8_t address, uint8_t distance, uint8_t threshold): VL6180x(address) {
      _i2caddress = address;
      _distance = distance;
      _threshold = threshold;
    };

    // Check if the sensor has already been initialized / is operating.
    bool sensorRunning(void) {
      uint8_t regData = 255;
      regData = VL6180x_getRegister(VL6180X_SYSTEM_FRESH_OUT_OF_RESET);
      return(regData == 0);
    };

    // Init sensor settings and configure for discreet operation via interrupts.
    void sensorSettings(void) {
      if (sensorRunning()) {
        return;
      }

      VL6180xInit();

      // Configure disable interrupts.
      VL6180x_setRegister(VL6180X_SYSTEM_INTERRUPT_CONFIG_GPIO, 0x0);

      // Configure GPIO0 for interrupt / active high polarity.
      VL6180x_setRegister(VL6180X_SYSTEM_MODE_GPIO0, 0x30);

      // Sensor calibration and temperature
      VL6180x_setRegister(VL6180X_READOUT_AVERAGING_SAMPLE_PERIOD, 0x30);
      VL6180x_setRegister(VL6180X_SYSALS_ANALOGUE_GAIN, 0x46);
      VL6180x_setRegister(VL6180X_SYSRANGE_VHV_REPEAT_RATE, 0xFF);
      VL6180x_setRegister(VL6180X_SYSALS_INTEGRATION_PERIOD, 0x63);
      VL6180x_setRegister(VL6180X_SYSRANGE_VHV_RECALIBRATE, 0x01);
      VL6180x_setRegister(VL6180X_SYSRANGE_RANGE_CHECK_ENABLES, 0x11);
      VL6180x_setRegister16bit(VL6180X_SYSRANGE_EARLY_CONVERGENCE_ESTIMATE, 0x7B);
      VL6180x_setRegister(VL6180X_READOUT_AVERAGING_SAMPLE_PERIOD, 0x30);

      // Continuous ranging delay period (1 = 10ms) and timeout (1 = 1ms)
      VL6180x_setRegister(VL6180X_SYSRANGE_INTERMEASUREMENT_PERIOD, 0x64);
      VL6180x_setRegister(VL6180X_SYSRANGE_MAX_CONVERGENCE_TIME, 0x32);

      // History buffering
      VL6180x_setRegister(VL6180X_SYSTEM_HISTORY_CTRL, 0x05);

      // Reset flag
      VL6180x_setRegister(VL6180X_SYSTEM_FRESH_OUT_OF_RESET, 0x0);

      // Start continuous measurement.
      VL6180x_setRegister(VL6180X_SYSRANGE_START, 0x03);
    };

    // Get the last distance result from history buffer and
    // clear interrupts.
    uint8_t getLastDistance(void) {
      uint8_t regData;
      regData = VL6180x_getRegister(VL6180X_RESULT_HISTORY_BUFFER);
      VL6180x_setRegister(VL6180X_SYSTEM_INTERRUPT_CLEAR, 0x07);
      return(regData);
    };

    // Get current status / interrupt config
    LockState getConfigState(void) {
      uint8_t intConfig;
      LockState lockState;
      intConfig = VL6180x_getRegister(VL6180X_SYSTEM_INTERRUPT_CONFIG_GPIO);

      if (intConfig == 0x03) {
        lockState = locked;
      } else if (intConfig == 0x01) {
        lockState = unlocked;
      } else {
        lockState = other;
      }

      return(lockState);
    };

    // Config to monitor a new status change state.
    void setConfigState(LockState newState) {
      uint8_t intConfig, threshL, threshH;

      if (newState == getConfigState()) {
        VL6180x_setRegister(VL6180X_SYSTEM_INTERRUPT_CLEAR, 0x07);
        return;
      }

      // Stop continuous measurement.
      VL6180x_setRegister(VL6180X_SYSRANGE_START, 0x03);

      // Set interrupts to watch for lock / unlock distance change.
      switch (newState) {
        case locked:
          // Watch for change outside locked (+/- tolerence) window
          intConfig = 0x03;
          threshL = _distance - _threshold;
          threshH = _distance + _threshold;
          break;
        case unlocked:
          // Watch for change to less than locked + tolerence
          intConfig = 0x01;
          threshL = _distance + _threshold;
          threshH = 255;
          break;
        case other:
        default:
          // Watch for change to greater than locked - tolerence
          intConfig = 0x02;
          threshL = 0;
          threshH = _distance + _threshold;
      }

      delay(10);

      VL6180x_setRegister(VL6180X_SYSRANGE_THRESH_LOW, threshL);
      VL6180x_setRegister(VL6180X_SYSRANGE_THRESH_HIGH, threshH);
      VL6180x_setRegister(VL6180X_SYSTEM_INTERRUPT_CONFIG_GPIO, intConfig);
      VL6180x_setRegister(VL6180X_SYSTEM_INTERRUPT_CLEAR, 0x07);

      delay(10);

      // Start continuous measurement.
      VL6180x_setRegister(VL6180X_SYSRANGE_START, 0x03);
    };

  private:
    int _i2caddress;
    uint8_t _distance, _threshold;

    uint8_t VL6180x_getRegister(uint16_t registerAddr)
    {
      uint8_t data;

      Wire.beginTransmission(_i2caddress);
      Wire.write((registerAddr >> 8) & 0xFF);
      Wire.write(registerAddr & 0xFF);
      Wire.endTransmission(false);
      Wire.requestFrom(_i2caddress, 1);
      data = Wire.read();

      return data;
    }

    uint16_t VL6180x_getRegister16bit(uint16_t registerAddr)
    {
      uint8_t data_low;
      uint8_t data_high;
      uint16_t data;

      Wire.beginTransmission(_i2caddress);
      Wire.write((registerAddr >> 8) & 0xFF);
      Wire.write(registerAddr & 0xFF);
      Wire.endTransmission(false);

      Wire.requestFrom(_i2caddress, 2);
      data_high = Wire.read();
      data_low = Wire.read();
      data = (data_high << 8) | data_low;

      return data;
    }

    void VL6180x_setRegister(uint16_t registerAddr, uint8_t data)
    {
      Wire.beginTransmission(_i2caddress);
      Wire.write((registerAddr >> 8) & 0xFF);
      Wire.write(registerAddr & 0xFF);
      Wire.write(data);
      Wire.endTransmission();
    }

    void VL6180x_setRegister16bit(uint16_t registerAddr, uint16_t data)
    {
      Wire.beginTransmission(_i2caddress);
      Wire.write((registerAddr >> 8) & 0xFF);
      Wire.write(registerAddr & 0xFF);
      uint8_t temp;
      temp = (data >> 8) & 0xff;
      Wire.write(temp);
      temp = data & 0xff;
      Wire.write(temp);
      Wire.endTransmission();
    }
};

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];

VL6180Lock sensor(VL6180X_ADDRESS, LOCK_DISTANCE, LOCK_TOLERENCE);

/*
 * Setup / Main Loop
 */
void setup() {
  uint8_t distance;
  LockState lockState;
  DoorState doorState;

#if DEBUG
  DEBUGOUT.begin(115200);
  delay(1000);
  DEBUGOUT.println("ESP8266 powering up.");
#endif

  // Setup GPIO.
  pinMode(GPIO_DOOR_SWITCH, INPUT);

#if GPIO_ENABLE_LED
  pinMode(GPIO_LED, OUTPUT);
  digitalWrite(GPIO_LED, LOW);
#endif

  Wire.begin();
  delay(100);

  // Get current lock state from VL6180.
  if (!sensor.sensorRunning()) {

#if DEBUG
    DEBUGOUT.println("Configuring VL6180x.");
#endif

    // Configure sensor and start continuous measurement, then
    // wait a bit for it to start sampling.
    sensor.sensorSettings();
    delay(100);
  }

  distance = sensor.getLastDistance();
  lockState = getLockState(distance);

#if DEBUG
  DEBUGOUT.print("New Lock state detected: ");
  DEBUGOUT.print(lockState);
  DEBUGOUT.print(" from distance: ");
  DEBUGOUT.println(distance);
#endif

  // Get current door switch state.
  doorState = static_cast<DoorState>(digitalRead(GPIO_DOOR_SWITCH));

#if DEBUG
  DEBUGOUT.print("Door state detected: ");
  DEBUGOUT.println(doorState);
#endif

  sendStates(lockState, doorState);
  sensor.setConfigState(getLockState(sensor.getLastDistance()));

  // Go to sleep.
#if DEBUG
  DEBUGOUT.println("ESP8266 entering low power mode.");
#endif

#if DEBUG_NOSLEEP
  DEBUGOUT.println("Rebooting in 5 seconds.");
  delay(5000);
  ESP.restart();
#else
  ESP.deepSleep(SLEEP_PERIOD * 1000000);
#endif
}

/*
 * Determine the lock state from a sensor distance in mm.
 */
LockState getLockState(uint8_t distance) {
  LockState lockState;

  if (distance > (LOCK_DISTANCE + LOCK_TOLERENCE)) {
    // Door is unlocked.
    lockState = unlocked;
  } else if (distance < (LOCK_DISTANCE - LOCK_TOLERENCE)) {
    lockState = other;
  } else {
    lockState = locked;
  }

  return(lockState);
}

/*
 * Send the current lock / door states to MQTT server.
 */
void sendStates(LockState lockState, DoorState doorState) {
  uint8_t retries = 0;

  if (!setupWifi()) {
    return;
  }

  client.setServer(MQTT_SERVER, MQTT_PORT);

  // Loop until we're connected (or timeout).
  while ((!client.connected()) && (retries < MQTT_RETRY)) {

#if DEBUG
    DEBUGOUT.print("Attempting MQTT connection... ");
#endif

    // Attempt to connect
    if (client.connect(MQTT_CLIENT_ID, MQTT_CLIENT_USER, MQTT_CLIENT_PASS)) {

#if DEBUG
      DEBUGOUT.println("connected.");
#endif

      // Once connected, publish an announcement...
      snprintf(msg, 10, "%d", lockState);
      client.publish(MQTT_TOPIC_LOCK, msg);
      snprintf(msg, 10, "%d", doorState);
      client.publish(MQTT_TOPIC_DOOR, msg);

    } else {

#if DEBUG
      DEBUGOUT.print("failed, rc=");
      DEBUGOUT.println(client.state());
#endif
      // Wait before retrying.
      retries++;
      delay(2000);
    }
  }

  client.loop();
}

/*
 * Setup the Wifi network connection.
 */
bool setupWifi() {
  uint8_t retries = 0;

#if DEBUG
  DEBUGOUT.println();
  DEBUGOUT.print("Connecting to ");
  DEBUGOUT.println(WIFI_AP);
#endif

#if defined(WIFI_IP) && defined(WIFI_GW) && defined(WIFI_SN)
  // Set static IP / routing.
  IPAddress wifiip(WIFI_IP);
  IPAddress wifigw(WIFI_GW);
  IPAddress wifisn(WIFI_SN);
  WiFi.config(wifiip, wifigw, wifisn);
#endif

  WiFi.begin(WIFI_AP, WIFI_PASS);

  while ((WiFi.status() != WL_CONNECTED) && (retries < WIFI_RETRY)) {

#if GPIO_ENABLE_LED
    digitalWrite(GPIO_LED, HIGH);
    delay(250);
    digitalWrite(GPIO_LED, LOW);
    delay(250);
#else
    delay(500);
#endif

#if DEBUG
    DEBUGOUT.print(".");
#endif

    retries++;
  }

  if (retries >= WIFI_RETRY) {

#if DEBUG
    DEBUGOUT.println("");
    DEBUGOUT.println("Unable to connect to WiFi.");
#endif

#if GPIO_ENABLE_LED
    uint8_t flash = 0;
    while (flash <= 10) {
      delay(25);
      digitalWrite(GPIO_LED, HIGH);
      delay(25);
      digitalWrite(GPIO_LED, LOW);
      flash++;
    }
#endif

    return(false);
  }

#if DEBUG
  DEBUGOUT.println("");
  DEBUGOUT.println("WiFi connected");
  DEBUGOUT.println("IP address: ");
  DEBUGOUT.println(WiFi.localIP());
#endif

  return(true);
}

/*
 * Empty loop (sleep / reset during setup).
 */
void loop() { }

// vim:cin:ai:sts=2 sw=2 ft=cpp
