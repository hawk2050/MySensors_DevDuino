/*
* devduino-temp-hum-sensor.ino - Firmware for DevDuino v2.0 based temperature and humidity sensor Node with nRF24L01+ module
*
* Copyright 2014 Tomas Hozza <thozza@gmail.com>
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; either version 2 of the License.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, see <http://www.gnu.org/licenses/>.
*
* Authors:
* Tomas Hozza <thozza@gmail.com>
*
* MySensors library - http://www.mysensors.org/
* DevDuino v2.0 - http://www.seeedstudio.com/wiki/DevDuino_Sensor_Node_V2.0_(ATmega_328)
* nRF24L01+ spec - https://www.sparkfun.com/datasheets/Wireless/Nordic/nRF24L01P_Product_Specification_1_0.pdf
*
Hardware Connections (Breakoutboard to Arduino):
 -VCC = 3.3V
 -GND = GND
 -SDA = A4 (use inline 10k resistor if your board is 5V)
 -SCL = A5 (use inline 10k resistor if your board is 5V)

 System clock is 16MHz

 */
#include <MySensor.h>
#include <SPI.h>
#include <stdint.h>
#include <math.h>


#define API_v15


#include <Wire.h>
#include "HTU21D.h"

// FORCE_TRANSMIT_INTERVAL, this number of times of wakeup, the sensor is forced to report all values to 
// the controller
#define FORCE_TRANSMIT_INTERVAL 3 
#define SLEEP_TIME 300000
#define MAX_ATTACHED_DS18B20 2

//#define NODE_ID 5
#define NODE_ID 6

#define DEBUG_RCC 0

#define MCP9700_ENABLE 0
#define HTU21D_ENABLE 1

enum sensor_id
{
  CHILD_ID_LIGHT = 0,
  CHILD_ID_HTU21D_HUMIDITY,
  CHILD_ID_HTU21D_TEMP,
  CHILD_ID_DHT22_HUMIDITY,
  CHILD_ID_DHT22_TEMP,
  CHILD_ID_MCP9700_TEMP,
  CHILD_ID_DALLAS_TEMP_BASE,
  CHILD_ID_VOLTAGE = CHILD_ID_DALLAS_TEMP_BASE + MAX_ATTACHED_DS18B20
  
};



/***********************************/
/********* PIN DEFINITIONS *********/
/***********************************/
#define LED_pin 9

// Data wire is plugged into port 3 on the Arduino

#define RF24_CE_PIN 8
#define RF24_CS_PIN 7
#define MCP9700_PIN A3
/*****************************/
/********* FUNCTIONS *********/
/*****************************/
#if HTU21D_ENABLE
//Create an instance of the object
HTU21D myHumidity;
MyMessage msgHum(CHILD_ID_HTU21D_HUMIDITY, V_HUM);
MyMessage msgTemp(CHILD_ID_HTU21D_TEMP, V_TEMP);
#endif

#if MCP9700_ENABLE
float readMCP9700Temp();
MyMessage msgMCP9700Temp(CHILD_ID_MCP9700_TEMP, V_TEMP);
#endif

uint16_t measureBattery();
MyMessage msgVolt(CHILD_ID_VOLTAGE, V_VOLTAGE); 

uint8_t getBatteryPercent();
uint16_t readVcc();
void switchClock(unsigned char clk);
bool highfreq = true;

float lastTemperature;
boolean receivedConfig = false;
boolean metric = true; 
uint8_t loopCount = 0;
uint8_t clockSwitchCount = 0;
/************************************/
/********* GLOBAL VARIABLES *********/
/************************************/
#ifdef API_v15
MyTransportNRF24 transport(RF24_CE_PIN, RF24_CS_PIN, RF24_PA_LEVEL);
/*We're also tried to make the MySensors class hardware independent by introducing hardware profiles. 
 * They handle platform dependent things like sleeping, storage (EEPROM), watchdog, serial in- and output. 
 * Currently there is only one implementation for the ATMega328p (which also works fine for AtMega 2560)
 *Construct the class like this:
  */
MyHwATMega328 hw;
MySensor node(transport,hw);
#else
MySensor node(RF24_CE_PIN, RF24_CS_PIN);
#endif



/**********************************/
/********* IMPLEMENTATION *********/
/**********************************/
void setup()
{
  
  /*
  ** Auto Node numbering
  node.begin();
  */
  #ifdef NODE_ID
  node.begin(NULL,NODE_ID,false);
  #else
  node.begin(NULL,AUTO,false);
  #endif
  
  analogReference(INTERNAL);
  node.sendSketchInfo("devduino-temp-humidity-sensor", "0.4");
  
  node.present(CHILD_ID_VOLTAGE, S_CUSTOM);
  // Register all sensors to gateway (they will be created as child devices)
#if MCP9700_ENABLE
  node.present(CHILD_ID_MCP9700_TEMP, S_TEMP);
#endif

#if HTU21D_ENABLE
  myHumidity.begin();
  node.present(CHILD_ID_HTU21D_HUMIDITY, S_HUM);
  node.present(CHILD_ID_HTU21D_TEMP, S_TEMP);
#endif
  
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
  
  if (loopCount > FORCE_TRANSMIT_INTERVAL)
  { // force a transmission
    forceTransmit = true; 
    loopCount = 0;
  }
  // Process incoming messages (like config from server)
  node.process();
  measureBattery(forceTransmit);
  
   
  #if MCP9700_ENABLE
  readMCP9700Temp();
  #endif
  
  #if HTU21D_ENABLE
  readHTU21DTemperature(forceTransmit);
  readHTU21DHumidity(forceTransmit);  
  #endif
  
  node.sleep(SLEEP_TIME);
   
  
}

#if HTU21D_ENABLE
void readHTU21DTemperature(bool force)
{
  static float lastTemp = 0;
  
  if (force)
  {
   lastTemperature = -100;
  }
  float temp = myHumidity.readTemperature();
  
  if(lastTemp != temp)
  {
    node.send(msgTemp.set(temp,1));
    lastTemp = temp;
  }
}

void readHTU21DHumidity(bool force)
{
  static float lastHumidity = 0;
  
  if (force) 
  {
    lastHumidity = -100;
  }
  float humd = myHumidity.readHumidity();
  
  if(lastHumidity != humd)
  {
    node.send(msgHum.set(humd,1));
    lastHumidity = humd;
  }
}
#endif



/**
* Read the temperature from MCP9700
*/

#if MCP9700_ENABLE
float readMCP9700Temp() 
{

static float lastTemp = -200.0;
  float temp = analogRead(MCP9700_PIN)*3.3/1024.0;
  temp = temp - 0.5;
  temp = temp / 0.01;
  #if DEBUG_RCC
  Serial.print("Read Temp from MCP9700 = ");
  Serial.println(temp);
  Serial.println('\r');
  #endif
  if (temp != lastTemp || loopCount == 0)
  {
    node.send(msgMCP9700Temp.set(temp, 1));
    lastTemp = temp;
  }
  return temp;
  
}
#endif
/**
* Get the percentage of power in the battery
*/
uint8_t getBatteryPercent()
{
  static const float full_battery_v = 3169.0;
  float level = readVcc() / full_battery_v;
  uint8_t percent = level * 100;
  #if DEBUG_RCC
  Serial.print("Battery state = ");
  Serial.println(percent);
  Serial.println('\r');
  #endif
  node.sendBatteryLevel(percent);
  return percent;
}

uint16_t measureBattery(bool force)
{
  static uint16_t lastVcc = 0;
  
  if (force) {
    lastVcc = 0;
  }
  
  uint16_t thisVcc = readVcc();
  if(thisVcc != lastVcc || loopCount == 0)
  {
    node.send(msgVolt.set(readVcc(), 1));
    lastVcc = thisVcc;
  }
  return thisVcc;
  
}
/**
* Measure remaining voltage in battery in millivolts
*
* From http://www.seeedstudio.com/wiki/DevDuino_Sensor_Node_V2.0_(ATmega_328)#Measurement_voltage_power
*/

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

