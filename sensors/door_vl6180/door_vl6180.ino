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
#define DEBUG true
#define DEBUGOUT Serial

#define GPIO_VL6180_IO0 13
#define GPIO_VL6180_IO1 12
#define GPIO_DOOR_SWITCH 4
#define GPIO_LED 5

#define MQTT_TOPIC_LOCK "/home/doors/front/lock"
#define MQTT_TOPIC_DOOR "/home/doors/front/state"

#define VL6180X_ADDRESS 0x29

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

VL6180xIdentification identification;
VL6180x sensor(VL6180X_ADDRESS);

void setup() {

#if DEBUG
  DEBUGOUT.begin(115200);
  delay(1000);
#endif

  pinMode(GPIO_LED, OUTPUT);
  pinMode(GPIO_DOOR_SWITCH, INPUT_PULLUP);
  setup_wifi();
  Wire.begin();
  delay(100);

  if (sensor.VL6180xInit() != 0){
    DEBUGOUT.println("FAILED TO INITALIZE");
  };

  sensor.VL6180xDefautSettings();
  delay(1000);

  client.setServer(MQTT_SERVER, MQTT_PORT);
}

void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  DEBUGOUT.println();
  DEBUGOUT.print("Connecting to ");
  DEBUGOUT.println(WIFI_AP);

  WiFi.begin(WIFI_AP, WIFI_PASS);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    DEBUGOUT.print(".");
  }

  DEBUGOUT.println("");
  DEBUGOUT.println("WiFi connected");
  DEBUGOUT.println("IP address: ");
  DEBUGOUT.println(WiFi.localIP());
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    DEBUGOUT.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(MQTT_CLIENT_ID, MQTT_CLIENT_USER, MQTT_CLIENT_PASS)) {
      DEBUGOUT.println("connected");
      // Once connected, publish an announcement...
      snprintf(msg, 10, "%d", sensor.getDistance());
      client.publish(MQTT_TOPIC_LOCK, msg);
      snprintf(msg, 10, "%d", digitalRead(GPIO_DOOR_SWITCH));
      client.publish(MQTT_TOPIC_DOOR, msg);
    } else {
      DEBUGOUT.print("failed, rc=");
      DEBUGOUT.print(client.state());
      DEBUGOUT.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
void loop() {

  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  long now = millis();
  if (now - lastMsg > 2000) {
    digitalWrite(GPIO_LED, HIGH);
    lastMsg = now;
    snprintf(msg, 4, "%d", sensor.getDistance());
    client.publish(MQTT_TOPIC_LOCK, msg);
    snprintf(msg, 10, "%d", digitalRead(GPIO_DOOR_SWITCH));
    client.publish(MQTT_TOPIC_DOOR, msg);
    delay(250);
    digitalWrite(GPIO_LED, LOW);
  }
}
// vim:cin:ai:sts=2 sw=2 ft=cpp
