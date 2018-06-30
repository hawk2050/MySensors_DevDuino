
/**
 * The MySensors Arduino library handles the wireless radio link and protocol
 * between your home built sensors/actuators and HA controller of choice.
 * The sensors forms a self healing radio network with optional repeaters. Each
 * repeater and gateway builds a routing tables in EEPROM which keeps track of the
 * network topology allowing messages to be routed to nodes.
 *
 * Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
 * Copyright (C) 2013-2015 Sensnology AB
 * Full contributor list: https://github.com/mysensors/Arduino/graphs/contributors
 *
 * Documentation: http://www.mysensors.org
 * Support Forum: http://forum.mysensors.org
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 *******************************
 *
 * REVISION HISTORY
 * Version 1.0: Henrik EKblad
 * Version 1.1 - 2016-07-20: Converted to MySensors v2.0 and added various improvements - Torben Woltjen (mozzbozz)
 * 
 * DESCRIPTION
 * This sketch implements a humidity/temperature
 * sensor using a HTU21D connected via I2C to a V2.0 DevDuino
 *  
 * For more information, please visit:
 * http://www.mysensors.org/build/humidity
 * 
 *  MySensors library - http://www.mysensors.org/
 * DevDuino v2.0 - http://www.seeedstudio.com/wiki/DevDuino_Sensor_Node_V2.0_(ATmega_328)
 * nRF24L01+ spec - https://www.sparkfun.com/datasheets/Wireless/Nordic/nRF24L01P_Product_Specification_1_0.pdf
 * 
 * System clock is 16MHz
 */

// Enable debug prints
#define MY_DEBUG

// Enable and select radio type attached 
#define MY_RADIO_NRF24
//#define MY_RADIO_RFM69
//#define MY_RS485

#define NODE_ID 5
#define MY_RF24_CE_PIN 8
#define MY_RF24_CS_PIN 7
 
//#include <SPI.h>
#include <MySensors.h> 
#include <stdint.h>
#include <math.h> 
#include "SparkFunHTU21D.h"




// Sleep time between sensor updates (in milliseconds)
static const uint64_t UPDATE_INTERVAL = 100000;

// Force sending an update of the temperature after n sensor reads, so a controller showing the
// timestamp of the last update doesn't show something like 3 hours in the unlikely case, that
// the value didn't change since;
// i.e. the sensor would force sending an update every UPDATE_INTERVAL*FORCE_UPDATE_N_READS [ms]
static const uint8_t FORCE_UPDATE_N_READS = 3;

#define CHILD_ID_HUM 0
#define CHILD_ID_TEMP 1

#define CHILD_ID_VOLTAGE 2

float lastTemp;
float lastHum;
uint8_t nNoUpdatesTemp;
uint8_t nNoUpdatesHum;
bool metric = true;
uint8_t loopCount = 0;
uint8_t clockSwitchCount = 0;

MyMessage msgHum(CHILD_ID_HUM, V_HUM);
MyMessage msgTemp(CHILD_ID_TEMP, V_TEMP);
HTU21D myHTU21D;

MyMessage msgVolt(CHILD_ID_VOLTAGE, V_VOLTAGE); 

uint16_t measureBattery(bool);
uint8_t getBatteryPercent();
uint16_t readVcc();
void switchClock(unsigned char clk);
bool highfreq = true;


void presentation()  
{ 
  // Send the sketch version information to the gateway
  sendSketchInfo("DevDuino-temp-humidity-sensor", "1.1");
  
  // Register all sensors to gw (they will be created as child devices)
  present(CHILD_ID_HUM, S_HUM);
  present(CHILD_ID_TEMP, S_TEMP);
  present(CHILD_ID_VOLTAGE, S_CUSTOM);
  
  metric = getControllerConfig().isMetric;
}


void setup()
{
  analogReference(INTERNAL);
  myHTU21D.begin(); // set data pin of DHT sensor
}


void loop()      
{  
  loopCount++;
  clockSwitchCount++;
  bool forceTransmit = false;
  
  // When we wake up the 5th time after power on, switch to 1Mhz clock
  // This allows us to print debug messages on startup (as serial port is dependend on oscilator settings).
  if ( (clockSwitchCount == 5) && highfreq)
  {
    /* Switch to 1Mhz by setting clock prescaler to divide by 16 for the reminder of the sketch, 
     * to save power but more importantly to allow operation down to 1.8V
     * 
      */
    switchClock(1<<CLKPS2); 
  }

  measureBattery(forceTransmit);
  
  // Get temperature from HTU21D library
  float temperature = myHTU21D.readTemperature();

  if (isnan(temperature))
  {
    Serial.println("Failed reading temperature from HTU21D!");
  } 
  else if (temperature != lastTemp || nNoUpdatesTemp == FORCE_UPDATE_N_READS)
  {
    // Only send temperature if it changed since the last measurement or if we didn't send an update for n times
    lastTemp = temperature;

    // Reset no updates counter
    nNoUpdatesTemp = 0;
    send(msgTemp.set(temperature, 1));

    #ifdef MY_DEBUG
    Serial.print("T: ");
    Serial.println(temperature);
    #endif
  } 
  else
  {
    // Increase no update counter if the temperature stayed the same
    nNoUpdatesTemp++;
  }

  // Get humidity from HTU21D library
  float humidity = myHTU21D.readHumidity();
  if (isnan(humidity))
  {
    Serial.println("Failed reading humidity from HTU21D");
  }
  else if (humidity != lastHum || nNoUpdatesHum == FORCE_UPDATE_N_READS)
  {
    // Only send humidity if it changed since the last measurement or if we didn't send an update for n times
    lastHum = humidity;
    // Reset no updates counter
    nNoUpdatesHum = 0;
    send(msgHum.set(humidity, 1));
    
    #ifdef MY_DEBUG
    Serial.print("H: ");
    Serial.println(humidity);
    #endif
  }
  else
  {
    // Increase no update counter if the humidity stayed the same
    nNoUpdatesHum++;
  }

  // Sleep for a while to save energy
  sleep(UPDATE_INTERVAL); 
}

    
/**
* Measure remaining voltage in battery in millivolts
*
* From http://www.seeedstudio.com/wiki/DevDuino_Sensor_Node_V2.0_(ATmega_328)#Measurement_voltage_power
*/
uint16_t measureBattery(bool force)
{
  static uint16_t lastVcc = 0;
  
  if (force)
  {
    lastVcc = 0;
  }
  
  uint16_t thisVcc = readVcc();
  if(thisVcc != lastVcc || loopCount == 0)
  {
    send(msgVolt.set(readVcc(), 1));
    lastVcc = thisVcc;
  }
  return thisVcc;
  
}

uint16_t readVcc()
{
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
  ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
  ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif
  delay(75); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring
  uint8_t low = ADCL; // must read ADCL first - it then locks ADCH
  uint8_t high = ADCH; // unlocks both
  long result = (high<<8) | low;
  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  #if DEBUG_RCC
  Serial.print("Read Vcc = ");
  Serial.println(result);
  Serial.println('\r');
  #endif
  return (uint16_t)result; // Vcc in millivolts
  
}

void switchClock(unsigned char clk)
{
  cli();
  
  CLKPR = 1<<CLKPCE; // Set CLKPCE to enable clk switching
  CLKPR = clk;  
  sei();
  highfreq = false;
}