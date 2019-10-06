/*
   MysCellar
   Copyright (C) 2015,2016,2017 Pierre Wieser <pwieser@trychlos.org>

   Description:
   Manages in one MySensors-compatible board following modules
   - flood detection
   - temperature measure
   - humidity measure
   - opening door detection.

   The MySensors Arduino library handles the wireless radio link and protocol
   between your home built sensors/actuators and HA controller of choice.
   The sensors forms a self healing radio network with optional repeaters. Each
   repeater and gateway builds a routing tables in EEPROM which keeps track of the
   network topology allowing messages to be routed to nodes.

   Flood detection:
   Based on rain sensor code by Reichenstein7 (thejamerson.com)
   http://www.instructables.com/id/Arduino-Modules-Rain-Sensor/
   Sends a boolean value to MySensors gateway

   Temperature/Humidity measures
   -----------------------------
   Based on humidity sensor by Henrik Ekblad <henrik.ekblad@mysensors.org>
   Copyright (C) 2013-2015 Sensnology AB
   http://www.mysensors.org/build/humidity
   Uses DHT22/AM2302 module

   Temperature and humidity measures:
   - measure (°C or %)
   - read sensor period (ms)
   - unchanged timeout (ms)

   Door opening detection
   ----------------------

   Radio module
   ------------
   Is implemented with a NRF24L01+ radio module

   Input/output messages
   ---------------------
    - Input, aka actions, aka messages received by this node from the gateway
    - Output, aka informations, aka messages sent by this node to the gateway

      cf. original, source and reference in build/Sheets.ods

      Sens  Name                  Id  Type          Nature  Command Message  Payload     Comment

      In  CHILD_MAIN               1  S_CUSTOM      Action  C_REQ  V_CUSTOM  1           reset EEPROM to default values
      In  CHILD_MAIN               1  S_CUSTOM      Action  C_REQ  V_CUSTOM  2           dump data
      In  CHILD_MAIN+1             2  S_CUSTOM      Action  C_SET  V_CUSTOM  <ms>        set periodic resend of the full data ; 0 to disable ; default=86400000 (24h)
      Out CHILD_MAIN+1             2  S_CUSTOM      Info           V_VAR1    <ms>        periodic resend period of the full data

      In  CHILD_ID_FLOOD          10  S_WATER_LEAK  Action  C_SET  V_CUSTOM  ARM=1|0     arm/unarm the sensor
      Out CHILD_ID_FLOOD          10  S_WATER_LEAK  Info           V_ARMED   true|false  whether the flood alarm is armed
      Out CHILD_ID_FLOOD+1        11  S_WATER_LEAK  Info           V_TRIPPED true|false  whether the flood alarm is tripped (false if not armed)
      In  CHILD_ID_FLOOD+2        12  S_WATER_LEAK  Action  C_SET  V_CUSTOM  <ms>        set max send frequency ; def_max_frequency_timeout = 120000 (2mn) ; 0 for stop
      Out CHILD_ID_FLOOD+2        12  S_WATER_LEAK  Info           V_VAR1    <ms>        current max send frequency for the flood alarm
      In  CHILD_ID_FLOOD+3        13  S_WATER_LEAK  Action  C_SET  V_CUSTOM  <ms>        set unchanged send timeout ; def_unchanged_timeout = 3600000 (1h)
      Out CHILD_ID_FLOOD+3        13  S_WATER_LEAK  Info           V_VAR1    <ms>        current unchanged send timeout for the flood alarm

      Out CHILD_ID_RAIN           20  S_RAIN        Info           V_RAIN    <num>       rain analogic value in [0..1024]
      In  CHILD_ID_RAIN+1         21  S_RAIN        Action  C_SET  V_CUSTOM  <ms>        set max send frequency ; def_max_frequency_timeout = 120000 (2mn) ; 0 for stop
      Out CHILD_ID_RAIN+1         21  S_RAIN        Info           V_VAR1    <ms>        current max send frequency for rain
      In  CHILD_ID_RAIN+2         22  S_RAIN        Action  C_SET  V_CUSTOM  <ms>        set unchanged send timeout ; def_unchanged_timeout = 3600000 (1h)
      Out CHILD_ID_RAIN+2         22  S_RAIN        Info           V_VAR1    <ms>        current unchanged send timeout for rain

      Out CHILD_ID_TEMPERATURE    30  S_TEMP        Info           V_TEMP    <num>       temperature
      In  CHILD_ID_TEMPERATURE+1  31  S_TEMP        Action  C_SET  V_CUSTOM  <ms>        set max send frequency ; def_max_frequency_timeout = 120000 (2mn) ; 0 for stop
      Out CHILD_ID_TEMPERATURE+1  31  S_TEMP        Info           V_VAR1    <ms>        current max send frequency for temperature
      In  CHILD_ID_TEMPERATURE+2  32  S_TEMP        Action  C_SET  V_CUSTOM  <ms>        set unchanged send timeout ; def_unchanged_timeout = 3600000 (1h)
      Out CHILD_ID_TEMPERATURE+2  32  S_TEMP        Info           V_VAR1    <ms>        current unchanged send timeout for temperature

      Out CHILD_ID_HUMIDITY       40  S_HUM         Info           V_HUM     <num>       humidity
      In  CHILD_ID_HUMIDITY+1     41  S_HUM         Action  C_SET  V_CUSTOM  <ms>        set max send frequency ; def_max_frequency_timeout = 120000 (2mn) ; 0 for stop
      Out CHILD_ID_HUMIDITY+1     41  S_HUM         Info           V_VAR1    <ms>        current max send frequency for humidity
      In  CHILD_ID_HUMIDITY+2     42  S_HUM         Action  C_SET  V_CUSTOM  <ms>        set unchanged send timeout ; def_unchanged_timeout = 3600000 (1h)
      Out CHILD_ID_HUMIDITY+2     42  S_HUM         Info           V_VAR1    <ms>        current unchanged send timeout for humidity

      Out CHILD_ID_DOOR           50  S_DOOR        Info           V_ARMED   true|false  whether the door opening detection is armed (always TRUE for now)
      Out CHILD_ID_DOOR+1         51  S_DOOR        Info           V_TRIPPED true|false  whether the door is opened (false if not armed)
      In  CHILD_ID_DOOR+2         52  S_DOOR        Action  C_SET  V_CUSTOM  <ms>        set max send frequency ; def_max_frequency_timeout = 120000 (2mn) ; 0 for stop
      Out CHILD_ID_DOOR+2         52  S_DOOR        Info           V_VAR1    <ms>        current max send frequency for door opening detection
      In  CHILD_ID_DOOR+3         53  S_DOOR        Action  C_SET  V_CUSTOM  <ms>        set unchanged send timeout ; def_unchanged_timeout = 3600000 (1h)
      Out CHILD_ID_DOOR+3         53  S_DOOR        Info           V_VAR1    <ms>        current unchanged send timeout for door opening detection

   This program is free software; you can redistribute it and/or
   modify it under the terms of the GNU General Public License
   version 2 as published by the Free Software Foundation.

   NOTE: this sketch has been written for MySensor v 2.1 library.

   pwi 2017- 3-25 activate repeater feature
   pwi 2017- 4- 2 use pwiTimer + parms are updatable
   pwi 2017- 5-10 better manage post-flood
   pwi 2017- 5-15 review flood subsystem
   pwi 2017- 5-22 fix flood rearming
   pwi 2019- 5-16 improve comments
   pwi 2019- 5-17 v7.0-2019
                    introduce the pwiSensor and pwiAlarm classes
   pwi 2019- 5-19 v7.1-2019
                    review the messaging vision to have one child per information message
                    remove the pwiAlarm class (no grace delay) to take place in the Nano
                    use patched MySensors 2.1.1 to publish the library version
   pwi 2019- 5-19 v7.2-2019
                    fix lighting of flood armed LED at startup
   pwi 2019- 5-25 v7.3-2019
                    auto-send the full configuration
                    numeric measures are not compared by float, but rather by int
   pwi 2019- 6- 1 v7.4-2019
                    review the sensors node ids
                    fix the temperature and hygrometry measures
                    add a version number if the EEPROM structure
   pwi 2019- 6- 1 v7.4.1-2019
                    fix weird behavior of switch statement
   pwi 2019- 6- 1 v7.4.2-2019
                    fix unexpected automatic creation of a surnumerous sensor in Jeedom

   pwi 2019- 6- 1 v7.5-2019
                    implement periodic auto dump
  Sketch uses 28522 bytes (92%) of program storage space. Maximum is 30720 bytes.
  Global variables use 1721 bytes (84%) of dynamic memory, leaving 327 bytes for local variables. Maximum is 2048 bytes.

   pwi 2019- 9- 8 v7.6-2019
                    rename Eeprom variables to min_period, max_period
                    remove alarm test code
                    update to pwiPrivate 190902
                    update to pwiCommon 190903
                    no more use any signing code
   Sketch uses 25904 bytes (84%) of program storage space. Maximum is 30720 bytes.
Global variables use 892 bytes (43%) of dynamic memory, leaving 1156 bytes for local variables. Maximum is 2048 bytes.

 * pwi 2019-10- 6 v7.7-2019
 *                  fix flood messages at setup
 *                  fix receiving code
 * Sketch uses 25962 bytes (84%) of program storage space. Maximum is 30720 bytes.
Global variables use 874 bytes (42%) of dynamic memory, leaving 1174 bytes for local variables. Maximum is 2048 bytes.
*/

// uncomment for debugging this sketch
#define SKETCH_DEBUG

static char const sketchName[] PROGMEM    = "mysCellar";
static char const sketchVersion[] PROGMEM = "7.7-2019";

/* The MySensors part */
#define MY_NODE_ID 4
#define MY_DEBUG
#define MY_REPEATER_FEATURE
#define MY_RADIO_NRF24
#define MY_RF24_PA_LEVEL RF24_PA_HIGH
//#define MY_SIGNING_SOFT
//#define MY_SIGNING_SOFT_RANDOMSEED_PIN 7
//#include <pwi_myhmac.h>
#include <pwi_myrf24.h>
#include <MySensors.h>

/* The four sensors + the main one
   - main let us acts on global configuration
   - alert sensors react to the alert timer (send at least one message if unchanged)
   - measure sensors react to measure timers (have a min frequency and a max frequency)
*/
enum {
  CHILD_MAIN            =  1,
  CHILD_ID_FLOOD        = 10,
  CHILD_ID_RAIN         = 20,
  CHILD_ID_TEMPERATURE  = 30,
  CHILD_ID_HUMIDITY     = 40,
  CHILD_ID_DOOR         = 50
};

MyMessage msg;

/*
 * Declare our classes
 */
#include <pwiCommon.h>
#include <pwiSensor.h>
#include <pwiTimer.h>
#include "eeprom.h"

sEeprom eeprom;
pwiTimer autodump_timer;

/* ****************************************************************************
   Flood detection is provided by a rain sensor which itself provides
    both digital and analog values:
    - digital value is used as an armable alert by CHILD_ID_FLOOD
    - analog value is managed as a standard measure by CHILD_ID_RAIN.
*/
// rain sensor analog output
#define RAIN_ANALOGINPUT       (A7)
#define RAIN_ANALOG_MIN        (0)            // flooded
#define RAIN_ANALOG_MAX        (1023)         // dry
// rain sensor digital output
#define FLOOD_DIGITALINPUT      (4)
#define FLOOD_DIGITAL_FLOODED   (LOW)
#define FLOOD_DIGITAL_DRY       (HIGH)

#define FLOOD_TRIPPED_LED       (5)
#define FLOOD_ARMED_LED         (6)

bool floodMeasureCb( void *user_data = NULL );
void floodSendCb( void *user_data = NULL );

pwiSensor flood_sensor( CHILD_ID_FLOOD, FLOOD_DIGITALINPUT);
bool      flood_tripped = false;

void floodPresentation()
{
    //                                           1234567890123456789012345
    present( CHILD_ID_FLOOD,   S_WATER_LEAK, F( "Flood detection" ));
    present( CHILD_ID_FLOOD+1, S_WATER_LEAK, F( "Flood alarm tripped" ));
    present( CHILD_ID_FLOOD+2, S_WATER_LEAK, F( "Flood min period" ));
    present( CHILD_ID_FLOOD+3, S_WATER_LEAK, F( "Flood max period" ));
}

void floodSetup()
{
    digitalWrite( FLOOD_ARMED_LED, LOW );
    pinMode( FLOOD_ARMED_LED, OUTPUT );
    digitalWrite( FLOOD_TRIPPED_LED, LOW );
    pinMode( FLOOD_TRIPPED_LED, OUTPUT );

    flood_sensor.setup( eeprom.flood_min_period, eeprom.flood_max_period, floodMeasureCb, floodSendCb );
    floodArmedSet( eeprom.flood_armed );
    flood_sensor.measureAndSend( true );
}

/* Regarding the flood detection, we are only interested by the digital value
   Sends a change as soon as the measure changes.
   Does not send anything if the alarm is not armed.
*/
bool floodMeasureCb( void *user_data )
{
    bool changed = false;

    if( eeprom.flood_armed ){
        bool cur_flooded = ( digitalRead( FLOOD_DIGITALINPUT ) == FLOOD_DIGITAL_FLOODED );
        if( cur_flooded != flood_tripped ){
            floodTrippedSet( cur_flooded );
            changed = true;
        }
    }

    return ( changed );
}

void floodSendCb( void *user_data )
{
    msg.clear();
    send( msg.setSensor( CHILD_ID_FLOOD ).setType( V_ARMED ).set( eeprom.flood_armed ));
    msg.clear();
    send( msg.setSensor( CHILD_ID_FLOOD + 1 ).setType( V_TRIPPED ).set( flood_tripped ));
}

void floodArmedSet( bool armed )
{
    bool changed = ( eeprom.flood_armed != armed );
    if( changed ){
        eeprom.flood_armed = armed;
        eepromWrite( eeprom, saveState );
        floodTrippedSet( false );
    }
    digitalWrite( FLOOD_ARMED_LED, eeprom.flood_armed ? HIGH:LOW );
}

void floodArmedSet( const char *payload )
{
    if( strncmp( payload, "ARM=", 4 ) != 0 ){
        Serial.print( F( "floodArmedSet() payload=" ));
        Serial.print( payload );
        Serial.println( F( " invalide; 'ARM=1|0' expected" ));
        return;
    }
    int num = atoi( payload+4 );
    bool armed = ( num > 0 );
    floodArmedSet( armed );
}

void floodMaxPeriodSend()
{
    msg.clear();
    send( msg.setSensor( CHILD_ID_FLOOD+3 ).setType( V_VAR1 ).set( eeprom.flood_max_period ));
}

void floodMaxPeriodSet( unsigned long ms )
{
    eeprom.flood_max_period = ms;
    eepromWrite( eeprom, saveState );
    flood_sensor.setMaxPeriod( ms );
}

void floodMinPeriodSend()
{
    msg.clear();
    send( msg.setSensor( CHILD_ID_FLOOD+2 ).setType( V_VAR1 ).set( eeprom.flood_min_period ));
}

void floodMinPeriodSet( unsigned long ms )
{
    eeprom.flood_min_period = ms;
    eepromWrite( eeprom, saveState );
    flood_sensor.setMinPeriod( ms );
}

/* Set the tripped status of the flood detection.
    + send the corresponding state to the controller
    On tripped:
    - start the rearm timer
*/
void floodTrippedSet( bool tripped )
{
    flood_tripped = tripped;
    digitalWrite( FLOOD_TRIPPED_LED, tripped ? HIGH:LOW );
}

/* ****************************************************************************
   Dealing with analog part of the rain sensor.
*/
bool rainMeasureCb( void *user_data = NULL );
void rainSendCb( void *user_data = NULL );

pwiSensor rain_sensor( CHILD_ID_RAIN, RAIN_ANALOGINPUT );
int       rain_last = 0;

void rainPresentation()
{
    //                                    1234567890123456789012345
    present( CHILD_ID_RAIN,   S_RAIN, F( "Rain sensor" ));
    present( CHILD_ID_RAIN+1, S_RAIN, F( "Rain min period" ));
    present( CHILD_ID_RAIN+2, S_RAIN, F( "Rain max period" ));
}

void rainSetup()
{
    rain_sensor.setup( eeprom.rain_min_period, eeprom.rain_max_period, rainMeasureCb, rainSendCb );
    rain_sensor.measureAndSend( true );
}

bool rainMeasureCb( void *user_data )
{
    bool changed = false;
    int cur_rain = analogRead( RAIN_ANALOGINPUT );

    if( cur_rain != rain_last ){
        rain_last = cur_rain;
        changed = true;
    }

    return ( changed );
}

void rainSendCb( void *user_data )
{
    msg.clear();
    send( msg.setSensor( CHILD_ID_RAIN ).setType( V_RAIN ).set( rain_last ));
#ifdef SKETCH_DEBUG
    Serial.print( F( "[rainSendCb] rain=" ));
    Serial.print( rain_last );
    uint8_t range = map( rain_last, RAIN_ANALOG_MIN, RAIN_ANALOG_MAX, 0, 3 );
    Serial.print( F( ", mapped=" ));
    Serial.print( range );
    switch ( range ) {
      case 0:    // Sensor getting wet
        Serial.println( F( " (flood)" ));
        break;
      case 1:    // Sensor getting wet
        Serial.println( F( " (raining)" ));
        break;
      case 2:    // Sensor dry
        Serial.println( F( " (wet)" ));
        break;
      case 3:    // Sensor dry
        Serial.println( F( " (dry)" ));
        break;
    }
#endif
}

void rainMaxPeriodSend()
{
    msg.clear();
    send( msg.setSensor( CHILD_ID_RAIN+2 ).setType( V_VAR1 ).set( eeprom.rain_max_period ));
}

void rainMaxPeriodSet( unsigned long ms )
{
    eeprom.rain_max_period = ms;
    eepromWrite( eeprom, saveState );
    rain_sensor.setMaxPeriod( ms );
}

void rainMinPeriodSend()
{
    msg.clear();
    send( msg.setSensor( CHILD_ID_RAIN+1 ).setType( V_VAR1 ).set( eeprom.rain_min_period ));
}

void rainMinPeriodSet( unsigned long ms )
{
    eeprom.rain_min_period = ms;
    eepromWrite( eeprom, saveState );
    rain_sensor.setMinPeriod( ms );
}

/* ****************************************************************************
   Temperature/Humidity DHT22/AM2302 sensors
   There is only one physical sensor module for the two measures.
*/
#include <DHT.h>
DHT dht;

// AM2302 DAT digital output
#define TEMPHUM_DIGITALINPUT    (3)

bool tempMeasureCb( void *user_data = NULL );
void tempSendCb( void *user_data = NULL );

pwiSensor temp_sensor( CHILD_ID_TEMPERATURE, TEMPHUM_DIGITALINPUT );
int       temp_last = 0;    // store the temperature multiplied by 10

void tempPresentation()
{
    //                                           1234567890123456789012345
    present( CHILD_ID_TEMPERATURE,   S_TEMP, F( "Temperature sensor" ));
    present( CHILD_ID_TEMPERATURE+1, S_TEMP, F( "Temperature min period" ));
    present( CHILD_ID_TEMPERATURE+2, S_TEMP, F( "Temperature max period" ));
}

/* Read the temperature as a float.
   If changed or @forceSend is %true, then send the temperature value to the controller.
*/
void tempSetup()
{
    // temperature/humidity
    dht.setup( TEMPHUM_DIGITALINPUT, DHT::AM2302 );

    temp_sensor.setup( eeprom.temp_min_period, eeprom.temp_max_period, tempMeasureCb, tempSendCb );
    temp_sensor.measureAndSend( true );
}

bool tempMeasureCb( void *user_data )
{
    bool changed = false;

    // Get temperature from DHT library
    float ftemp = dht.getTemperature();
    if( isnan( ftemp )){
        Serial.println( F( "[tempMeasureCb] failed reading temperature from DHT" ));
    } else {
        int itemp = (int) 10 * ftemp;
        if( itemp != temp_last ){
            temp_last = itemp;
            changed = true;
        }
    }

    return ( changed );
}

void tempSendCb( void *user_data )
{
    float ftemp = temp_last / 10.0;
    msg.clear();
    send( msg.setSensor( CHILD_ID_TEMPERATURE ).setType( V_TEMP ).set( ftemp, 1 ));
#ifdef SKETCH_DEBUG
    Serial.print( F( "[tempSendCb] temp=" ));
    Serial.print( ftemp, 1 );
    Serial.println( F( "°C" ));
#endif
}

void tempMaxPeriodSend()
{
    msg.clear();
    send( msg.setSensor( CHILD_ID_TEMPERATURE+2 ).setType( V_VAR1 ).set( eeprom.temp_max_period ));
}

void tempMaxPeriodSet( unsigned long ms )
{
    eeprom.temp_max_period = ms;
    eepromWrite( eeprom, saveState );
    temp_sensor.setMaxPeriod( ms );
}

void tempMinPeriodSend()
{
    msg.clear();
    send( msg.setSensor( CHILD_ID_TEMPERATURE+1 ).setType( V_VAR1 ).set( eeprom.temp_min_period ));
}

void tempMinPeriodSet( unsigned long ms )
{
    eeprom.temp_min_period = ms;
    eepromWrite( eeprom, saveState );
    temp_sensor.setMinPeriod( ms );
}

/* ****************************************************************************
   Read the humidity as a float.
   If changed or @forceSend is %true, then send the humidity value to the controller.
*/
bool humMeasureCb( void *user_data = NULL );
void humSendCb( void *user_data = NULL );

pwiSensor hum_sensor( CHILD_ID_HUMIDITY, TEMPHUM_DIGITALINPUT );
int       hum_last = 0;   // store the measure multiplied by 10

void humPresentation()
{
    //                                       1234567890123456789012345
    present( CHILD_ID_HUMIDITY,   S_HUM, F( "Humidity sensor" ));
    present( CHILD_ID_HUMIDITY+1, S_HUM, F( "Humidity min period" ));
    present( CHILD_ID_HUMIDITY+2, S_HUM, F( "Humidity max period" ));
}

void humSetup()
{
    hum_sensor.setup( eeprom.hum_min_period, eeprom.hum_max_period, humMeasureCb, humSendCb );
    hum_sensor.measureAndSend( true );
}

bool humMeasureCb( void *user_data )
{
    bool changed = false;

    // Get temperature from DHT library
    float fhum = dht.getHumidity();
    if( isnan( fhum )){
        Serial.println( F( "[humMeasureCb] failed reading humidity from DHT" ));
    } else {
        int ihum = ( int ) 10*fhum;
        if( ihum != hum_last ){
            hum_last = ihum;
            changed = true;
        }
    }

    return ( changed );
}

void humSendCb( void *user_data )
{
    float fhum = hum_last / 10.0 ;
    msg.clear();
    send( msg.setSensor( CHILD_ID_HUMIDITY ).setType( V_HUM ).set( fhum, 1 ));
#ifdef SKETCH_DEBUG
    Serial.print( F( "[humSendCb] humidity=" ));
    Serial.print( fhum, 1 );
    Serial.println( F( "%" ));
#endif
}

void humMaxPeriodSend()
{
    msg.clear();
    send( msg.setSensor( CHILD_ID_HUMIDITY+2 ).setType( V_VAR1 ).set( eeprom.hum_max_period ));
}

void humMaxPeriodSet( unsigned long ms )
{
    eeprom.hum_max_period = ms;
    eepromWrite( eeprom, saveState );
    hum_sensor.setMaxPeriod( ms );
}

void humMinPeriodSend()
{
    msg.clear();
    send( msg.setSensor( CHILD_ID_HUMIDITY+1 ).setType( V_VAR1 ).set( eeprom.hum_min_period ));
}

void humMinPeriodSet( unsigned long ms )
{
    eeprom.hum_min_period = ms;
    eepromWrite( eeprom, saveState );
    hum_sensor.setMinPeriod( ms );
}

/* ****************************************************************************
   Opening door detection
   The sensor is provided by the Telemecanique door switch.
   This is a double-relais platine, with a NO and a NC contacts.
   The NC contact is used by origin for lights.
   We are using here the NO contact for the door opening detector:
   - door closed : contact is closed
   - door opened: contact is opened.
*/
#define DOOR_OPENING_INPUT      (A0)           // use analog pin as a digital input
#define DOOR_OPENING_LED        (A1)

bool doorMeasureCb( void *user_data = NULL );
void doorSendCb( void *user_data = NULL );

pwiSensor door_sensor( CHILD_ID_DOOR, DOOR_OPENING_INPUT);
bool      door_opened = false;

void doorPresentation()
{
    //                                    1234567890123456789012345
    present( CHILD_ID_DOOR,   S_DOOR, F( "Door opening detection" ));
    present( CHILD_ID_DOOR+1, S_DOOR, F( "Door alarm tripped" ));
    present( CHILD_ID_DOOR+2, S_DOOR, F( "Door min period" ));
    present( CHILD_ID_DOOR+3, S_DOOR, F( "Door max period" ));
}

void doorSetup()
{
    pinMode( DOOR_OPENING_INPUT, INPUT );
    digitalWrite( DOOR_OPENING_LED, LOW );
    pinMode( DOOR_OPENING_LED, OUTPUT );

    door_sensor.setup( eeprom.door_min_period, eeprom.door_max_period, doorMeasureCb, doorSendCb );
    door_sensor.measureAndSend( true );
}

bool doorMeasureCb( void *user_data )
{
    bool changed = false;
    bool cur_opened = ( digitalRead( DOOR_OPENING_INPUT ) == HIGH );

    if ( cur_opened != door_opened ) {
      door_opened = cur_opened;
      digitalWrite( DOOR_OPENING_LED, door_opened ? HIGH : LOW );
      changed = true;
    }

    return ( changed );
}

void doorSendCb( void *user_data )
{
    msg.clear();
    send( msg.setSensor( CHILD_ID_DOOR ).setType( V_ARMED ).set( eeprom.door_armed ));
    msg.clear();
    send( msg.setSensor( CHILD_ID_DOOR+1 ).setType( V_TRIPPED ).set( door_opened ));
#ifdef SKETCH_DEBUG
    Serial.print( F( "[doorSendCb] armed=" ));
    Serial.print( eeprom.door_armed ? "True":"False" );
    Serial.print( F( ", opened=" ));
    Serial.println( door_opened ? "True":"False" );
#endif
}

void doorMaxPeriodSend()
{
    msg.clear();
    send( msg.setSensor( CHILD_ID_DOOR+3 ).setType( V_VAR1 ).set( eeprom.door_max_period ));
}

void doorMaxPeriodSet( unsigned long ms )
{
    eeprom.door_max_period = ms;
    eepromWrite( eeprom, saveState );
    door_sensor.setMaxPeriod( ms );
}

void doorMinPeriodSend()
{
    msg.clear();
    send( msg.setSensor( CHILD_ID_DOOR+2 ).setType( V_VAR1 ).set( eeprom.door_min_period ));
}

void doorMinPeriodSet( unsigned long ms )
{
    eeprom.door_min_period = ms;
    eepromWrite( eeprom, saveState );
    door_sensor.setMinPeriod( ms );
}

/* **************************************************************************************
 *  mainSensor
 */

void mainAutoDumpCb( void*empty );

void mainPresentation()
{
    present( CHILD_MAIN+1, S_CUSTOM, F( "Config dump" ));
}

void mainSetup()
{
    autodump_timer.setup( "AutoDumpTimer", eeprom.auto_dump_timeout, false, ( pwiTimerCb ) mainAutoDumpCb );
    autodump_timer.start();
}

void mainAutoDumpCb( void*empty )
{
    dumpData();
}

void mainAutoDumpSend()
{
    uint8_t sensor_id = CHILD_MAIN+1;
    uint8_t msg_type = V_VAR1;
    unsigned long payload = eeprom.auto_dump_timeout;
#ifdef SKETCH_DEBUG
    Serial.print( F( "[mainAutoDumpSend] sensor=" ));
    Serial.print( sensor_id );
    Serial.print( F( ", type=" ));
    Serial.print( msg_type );
    Serial.print( F( ", payload=" ));
    Serial.println( payload );
#endif
    msg.clear();
    send( msg.setSensor( sensor_id ).setType( msg_type ).set( payload ));
}

void mainAutoDumpSet( unsigned long ms )
{
#ifdef SKETCH_DEBUG
    Serial.print( F( "[mainAutoDumpSet] ms=" ));
    Serial.println( ms );
#endif
    eeprom.auto_dump_timeout = ms;
    eepromWrite( eeprom, saveState );
}

/* **************************************************************************************
 *  MAIN CODE
 */

// As of MySensors v2.x, presentation() is called before setup().
void presentation()
{
#ifdef SKETCH_DEBUG
    Serial.println( F( "presentation()" ));
#endif
    mainPresentation();
    floodPresentation();
    rainPresentation();
    tempPresentation();
    humPresentation();
    doorPresentation();
}

void setup()
{
#ifdef SKETCH_DEBUG
    Serial.begin( 115200 );
    Serial.println( F( "setup()" ));
#endif

    // sketch presentation
    sendSketchInfo( PGMSTR( sketchName ), PGMSTR( sketchVersion ));

    // library version
    msg.clear();
    mSetCommand( msg, C_INTERNAL );
    sendAsIs( msg.setSensor( 255 ).setType( I_VERSION ).set( MYSENSORS_LIBRARY_VERSION ));

    //eepromReset( eeprom, saveState );
    eepromRead( eeprom, loadState, saveState );
    eepromDump( eeprom );

    mainSetup();
    floodSetup();
    rainSetup();
    tempSetup();
    humSetup();
    doorSetup();
}

void loop()
{
    pwiTimer::Loop();
    wait( 1000 );
}

void receive(const MyMessage &message)
{
    uint8_t cmd = message.getCommand();

    char payload[MAX_PAYLOAD+1];
    memset( payload, '\0', sizeof( payload ));
    message.getString( payload );

#ifdef SKETCH_DEBUG
    Serial.print( F( "[receive] sensor=" ));
    Serial.print( message.sensor );
    Serial.print( F( ", type=" ));
    Serial.print( message.type );
    Serial.print( F( ", cmd=" ));
    Serial.print( cmd );
    Serial.print( F( ", payload='" ));
    Serial.print( payload );
    Serial.println( F( "'" ));
#endif

    // all received messages should be V_CUSTOM
    if( message.type != V_CUSTOM ){
#ifdef SKETCH_DEBUG
        Serial.println( F( "[receive] message cancelled as should be V_CUSTOM" ));
#endif
        return;
    }

    if( cmd == C_REQ ){
          uint8_t ureq = strlen( payload ) > 0 ? atoi( payload ) : 0;
#ifdef SKETCH_DEBUG
          Serial.print( F( "[receive] C_REQ: ureq=" ));
          Serial.println( ureq );
#endif
          switch( message.sensor ){
              case CHILD_MAIN:
                  switch ( ureq ) {
                    case 1:
                        eepromReset( eeprom, saveState );
                        break;
                    case 2:
                        dumpData();
                        break;
                  }
                  break;
          }

    } else if( cmd == C_SET ){
        unsigned long ulong = strlen( payload ) > 0 ? atol( payload ) : 0;
#ifdef SKETCH_DEBUG
        Serial.print( F( "[receive] C_SET: ulong=" ));
        Serial.println( ulong );
#endif
        switch( message.sensor ){
            case CHILD_MAIN+1:
                mainAutoDumpSet( ulong );
                mainAutoDumpSend();
                break;
            case CHILD_ID_FLOOD:
                floodArmedSet( payload );
                break;
            case CHILD_ID_FLOOD+2:
                floodMinPeriodSet( ulong );
                floodMinPeriodSend();
                break;
            case CHILD_ID_FLOOD+3:
                floodMaxPeriodSet( ulong );
                floodMaxPeriodSend();
                break;
            case CHILD_ID_RAIN+1:
                rainMinPeriodSet( ulong );
                rainMinPeriodSend();
                break;
            case CHILD_ID_RAIN+2:
                rainMaxPeriodSet( ulong );
                rainMaxPeriodSend();
                break;
            case CHILD_ID_TEMPERATURE+1:
                tempMinPeriodSet( ulong );
                tempMinPeriodSend();
                break;
            case CHILD_ID_TEMPERATURE+2:
                tempMaxPeriodSet( ulong );
                tempMaxPeriodSend();
                break;
            case CHILD_ID_HUMIDITY+1:
                humMinPeriodSet( ulong );
                humMinPeriodSend();
                break;
            case CHILD_ID_HUMIDITY+2:
                humMaxPeriodSet( ulong );
                humMaxPeriodSend();
                break;
            case CHILD_ID_DOOR+2:
                doorMinPeriodSet( ulong );
                doorMinPeriodSend();
                break;
            case CHILD_ID_DOOR+3:
                doorMaxPeriodSet( ulong );
                doorMaxPeriodSend();
                break;
        }
    }
}

void dumpData( void )
{
    mainAutoDumpSend();
  
    floodSendCb();
    floodMinPeriodSend();
    floodMaxPeriodSend();
  
    rainSendCb();
    rainMinPeriodSend();
    rainMaxPeriodSend();
  
    tempSendCb();
    tempMinPeriodSend();
    tempMaxPeriodSend();
  
    humSendCb();
    humMinPeriodSend();
    humMaxPeriodSend();
  
    doorSendCb();
    doorMinPeriodSend();
    doorMaxPeriodSend();
}

