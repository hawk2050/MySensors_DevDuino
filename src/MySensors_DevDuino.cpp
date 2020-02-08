
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

/*Makes this static so won't try and find another parent if communication with
 gateway fails*/
#define MY_PARENT_NODE_ID 0
#define MY_PARENT_NODE_IS_STATIC

/**
 * @def MY_TRANSPORT_WAIT_READY_MS
 * @brief Timeout in ms until transport is ready during startup, set to 0 for no timeout
 */
#define MY_TRANSPORT_WAIT_READY_MS (1000)

#define MY_NODE_ID 6
#define MY_RF24_CE_PIN 8
#define MY_RF24_CS_PIN 7
#define MY_RF24_CHANNEL 100
 
#include <MySensors.h> 
#include <stdint.h>
#include <math.h> 
#include "SparkFunHTU21D.h"
#include <BatterySense.hpp>

#define TEMP_HUM_SENSOR 1


// Sleep time between sensor updates (in milliseconds)
static const uint32_t DAY_UPDATE_INTERVAL_MS = 10000;

// Force sending an update of the temperature after n sensor reads, so a controller showing the
// timestamp of the last update doesn't show something like 3 hours in the unlikely case, that
// the value didn't change since;
// i.e. the sensor would force sending an update every UPDATE_INTERVAL*FORCE_UPDATE_N_READS [ms]
//static const uint8_t FORCE_UPDATE_N_READS = 3;

enum child_id_t
{
  CHILD_ID_HUMIDITY,
  CHILD_ID_TEMP,
  CHILD_ID_UV,
  CHILD_ID_VOLTAGE
};

float lastTemp;
float lastHum;

uint8_t loopCount = 0;
uint8_t clockSwitchCount = 0;

/*****************************/
/****** Sensor Objects *******/
/*****************************/

//Create an instance of the sensor objects
#if UV_SENSOR
const char sketchString[] = "mys_v11-uv";
UVSensor UV(MY_UVIS25_POWER_PIN); //Ultraviolet sensor
MyMessage msgUVindex(CHILD_ID_UV, V_UV);

#endif

#if TEMP_HUM_SENSOR
const char sketchString[] = "devDuino-temp_humid";
HTU21D myHTU21D;
MyMessage msgHum(CHILD_ID_HUMIDITY, V_HUM);
MyMessage msgTemp(CHILD_ID_TEMP, V_TEMP);
void readHTU21DTemperature(bool force);
void readHTU21DHumidity(bool force);
#endif

MyMessage msgVolt(CHILD_ID_VOLTAGE, V_VOLTAGE);
void switchClock(unsigned char clk);

/*Set true to have clock throttle back, or false to not throttle*/
bool throttlefreq = true;
bool cpu_is_throttled = false;

BatteryLevel batt;
/**********************************/
/********* IMPLEMENTATION *********/
/**********************************/
/*If you need to do initialization before the MySensors library starts up,
define a before() function */
void before()
{

}

/*To handle received messages, define the following function in your sketch*/
void receive(const MyMessage &message)
{
  /*Handle incoming message*/
}

/* If your node requests time using requestTime(). The following function is
used to pick up the response*/
void receiveTime(unsigned long ts)
{
}

/*You can still use setup() which is executed AFTER mysensors has been
initialised.*/
void setup()
{
  
  Wire.begin();
  #if UV_SENSOR
  UV.init();
  #endif
  #if TEMP_HUM_SENSOR
  myHTU21D.begin();
  #endif
  Serial.begin(9600);
  
}

void presentation()
{
   // Send the sketch version information to the gateway and Controller
  sendSketchInfo(sketchString, "0.6");
  // Register all sensors to gateway (they will be created as child devices)
  present(CHILD_ID_VOLTAGE, S_MULTIMETER);

#if UV_SENSOR
  present(CHILD_ID_UV, S_UV);
#endif

#if TEMP_HUM_SENSOR
  present(CHILD_ID_HUMIDITY, S_HUM);
  present(CHILD_ID_TEMP, S_TEMP);
#endif
   
}


void loop()
{

  uint32_t update_interval_ms = DAY_UPDATE_INTERVAL_MS;

  clockSwitchCount++;
  #if DEBUG_RCC
  Serial.print("clockSwitchCount = ");
  Serial.print(clockSwitchCount,DEC);
  Serial.println();
  #endif
  
  // When we wake up the 5th time after power on, switch to 4Mhz clock
  // This allows us to print debug messages on startup (as serial port is dependent on oscillator settings).
  if ( (clockSwitchCount == 5) && throttlefreq)
  {
    /* Switch to 4Mhz by setting clock prescaler to divide by 2 for the reminder of the sketch, 
     * to save power but more importantly to allow operation down to 1.8V
     * 
      */
    
    #if DEBUG_RCC
    Serial.print("Setting CPU Freq to 4MHz");
    Serial.println();
    #endif
    switchClock(0x02); // divide by 4, to give 4MHz on 16MHz, 3V3 Pro Mini
    cpu_is_throttled = true;
    throttlefreq = false;
  } //end if

  uint16_t battLevel = batt.getVoltage();
  send(msgVolt.set(battLevel,1));

#if TEMP_HUM_SENSOR
  readHTU21DTemperature(true);
  readHTU21DHumidity(true);
#endif

#if UV_SENSOR
  UV.read_sensor();

#if DEBUG_RCC
    Serial.print("UV reading is :");
    Serial.print(UV.get_uvi(), 1);
    Serial.println();
#endif
  
  send(msgUVindex.set(UV.get_uvi(),1));
#if DEBUG_RCC
    Serial.print("Power down UV sensor");
    Serial.println();
#endif
#endif /*UV_SENSOR*/

  sleep(update_interval_ms); 
  
  
} //end loop

void switchClock(unsigned char clk)
{
  cli();
  
  CLKPR = 1<<CLKPCE; // Set CLKPCE to enable clk switching
  CLKPR = clk;  
  sei();
  
}

#if TEMP_HUM_SENSOR
void readHTU21DTemperature(bool force)
{
  static float lastTemp = 0;

  if (force)
  {
   lastTemp = -100;
  }
  float temp = myHTU21D.readTemperature();

  if(lastTemp != temp)
  {
    send(msgTemp.set(temp,1));
    lastTemp = temp;
    #ifdef DEBUG_RCC
    Serial.print(" Temperature:");
    Serial.print(temp, 1);
    Serial.print("C");
    Serial.println();
    #endif
  }
}

void readHTU21DHumidity(bool force)
{
  static float lastHumidity = 0;

  if (force)
  {
    lastHumidity = -100;
  }
  float humd = myHTU21D.readHumidity();

  if(lastHumidity != humd)
  {
    send(msgHum.set(humd,1));
    lastHumidity = humd;
    #ifdef DEBUG_RCC
    Serial.print(" Humidity:");
    Serial.print(humd, 1);
    Serial.print("%");
    Serial.println();
    #endif
  }
}

#endif