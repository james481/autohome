/*
 * attiny85_watchdog.ino
 *
 * Home automation sensor / switch state watchdog using a ATTiny85 to
 monitor line states and wake an ESP8266 host on changes.
 *
 * Copyright (c) 2017, James Watts
 * All rights reserved, no unauthorized distribution or reproduction.
 *
 * See LICENSE.txt for details.
 */

#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <LowPower.h>

/*
 * Configuration
 */
#define GPIO_SENSOR_1 0
#define GPIO_SENSOR_2 1
#define GPIO_ESP_XPD 2
#define GPIO_ESP_RST 3

#define ESP_WAKE_PULSE 10

volatile bool pinState = false;
uint8_t espState = 0;

void setup() {
  // Disable ADC.
  ADCSRA &= ~(1<<ADEN);

  // Setup GPIO
  pinMode(GPIO_SENSOR_1, INPUT);
  pinMode(GPIO_SENSOR_2, INPUT);
  pinMode(GPIO_ESP_XPD, INPUT);
  pinMode(GPIO_ESP_RST, OUTPUT);
  digitalWrite(GPIO_ESP_RST, HIGH);

  // Enable pin change interrupts
  GIMSK |= _BV(PCIE);
  PCMSK = ((1 << GPIO_SENSOR_1) | (1 << GPIO_SENSOR_2) | (1 << GPIO_ESP_XPD));
}

void loop() {
  // Sleep until an interrupt.
  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);

  uint8_t newState = 0;
  bool wake = false;

  // Check if a sensor interrupt pin has changed state.
  if (pinState) {

    newState = digitalRead(GPIO_ESP_XPD);

    if (newState ^ espState) {
      if (newState == HIGH) {
        // Swallow sleep line change on XPD.
        espState = HIGH;
      } else {
        // Handle watchdog wakeup on XPD.
        espState = LOW;
        wake = true;
      }
    } else if (espState == HIGH) {
      wake = true;
    }

    if (wake) {
      // ESP8266 is asleep, wake it up by pulling reset low.
      digitalWrite(GPIO_ESP_RST, LOW);
      delay(ESP_WAKE_PULSE);
      digitalWrite(GPIO_ESP_RST, HIGH);
      delay(200);
    }

    pinState = false;
  }
}

ISR(PCINT0_vect) {
  pinState = true;
}

// vim:cin:ai:sts=2 sw=2 ft=cpp
