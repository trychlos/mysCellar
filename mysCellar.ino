/*
 * MysCellar
 * Copyright (C) 2015,2016,2017 Pierre Wieser <pwieser@trychlos.org>
 *
 * Description:
 * Manages in one MySensors-compatible board following modules
 * - flood detection
 * - temperature measure
 * - humidity measure
 * - opening door detection.
 *
 * The MySensors Arduino library handles the wireless radio link and protocol
 * between your home built sensors/actuators and HA controller of choice.
 * The sensors forms a self healing radio network with optional repeaters. Each
 * repeater and gateway builds a routing tables in EEPROM which keeps track of the
 * network topology allowing messages to be routed to nodes.
 *
 * Flood detection:
 * Based on rain sensor code by Reichenstein7 (thejamerson.com)
 * http://www.instructables.com/id/Arduino-Modules-Rain-Sensor/
 * Sends a boolean value to MySensors gateway
 *
 * Temperature/Humidity measures
 * -----------------------------
 * Based on humidity sensor by Henrik Ekblad <henrik.ekblad@mysensors.org>
 * Copyright (C) 2013-2015 Sensnology AB
 * http://www.mysensors.org/build/humidity
 * Uses DHT22/AM2302 module
 *
 * Temperature and humidity measures:
 * - measure (°C or %)
 * - read sensor period (ms)
 * - unchanged timeout (ms)
 *   though rather useless as measures change every time we are reading the sensors
 *
 * Door opening detection
 * ----------------------
 *
 * Radio module
 * ------------
 * Is implemented with a NRF24L01+ radio module
 *
 * Input/output messages
 * ---------------------
 *  - Input, aka actions, aka messages received by this node from the gateway
 *  - Output, aka informations, aka messages sent by this node to the gateway
 *
 *    cf. original in build/Sheet.ods
 *
      Sens  Name                  Id  Type          Nature  Command Message Payload     Comment
      
      In  CHILD_MAIN               1  S_CUSTOM      Action  C_REQ V_CUSTOM  1           reset EEPROM to default values
      In  CHILD_MAIN               1  S_CUSTOM      Action  C_REQ V_CUSTOM  2           dump full configuration and status, all current infos are sent

      In  CHILD_ID_FLOOD          10  S_WATER_LEAK  Action  C_SET V_CUSTOM  1=<ms>      set max send frequency ; def_max_frequency_timeout = 120000 (2mn) ; 0 for stop
      In  CHILD_ID_FLOOD          10  S_WATER_LEAK  Action  C_SET V_CUSTOM  2=<ms>      set unchanged send timeout ; def_unchanged_timeout = 3600000 (1h)
      In  CHILD_ID_FLOOD          10  S_WATER_LEAK  Action  C_SET V_CUSTOM  ARM=1|0     arm/unarm the sensor
      Out CHILD_ID_FLOOD          10  S_WATER_LEAK  Info          V_ARMED   true|false  whether the flood alarm is armed
      Out CHILD_ID_FLOOD+1        11  S_WATER_LEAK  Info          V_TRIPPED true|false  whether the flood alarm is tripped (false if not armed)
      Out CHILD_ID_FLOOD+2        12  S_WATER_LEAK  Info          V_VAR1    <ms>        current max send frequency for the flood alarm
      Out CHILD_ID_FLOOD+3        13  S_WATER_LEAK  Info          V_VAR2    <ms>        current unchanged send timeout for the flood alarm

      In  CHILD_ID_RAIN           20  S_RAIN        Action  C_SET V_CUSTOM  1=<ms>      set max send frequency ; def_max_frequency_timeout = 120000 (2mn) ; 0 for stop
      In  CHILD_ID_RAIN           20  S_RAIN        Action  C_SET V_CUSTOM  2=<ms>      set unchanged send timeout ; def_unchanged_timeout = 3600000 (1h)
      Out CHILD_ID_RAIN           20  S_RAIN        Info          V_RAIN    <num>       rain analogic value in [0..1024]
      Out CHILD_ID_RAIN+1         21  S_RAIN        Info          V_VAR1    <ms>        current max send frequency for rain
      Out CHILD_ID_RAIN+2         22  S_RAIN        Info          V_VAR2    <ms>        current unchanged send timeout for rain
 
      In  CHILD_ID_TEMPERATURE    30  S_TEMP        Action  C_SET V_CUSTOM  1=<ms>      set max send frequency ; def_max_frequency_timeout = 120000 (2mn) ; 0 for stop
      In  CHILD_ID_TEMPERATURE    30  S_TEMP        Action  C_SET V_CUSTOM  2=<ms>      set unchanged send timeout ; def_unchanged_timeout = 3600000 (1h)
      Out CHILD_ID_TEMPERATURE    30  S_TEMP        Info          V_TEMP    <num>       temperature
      Out CHILD_ID_TEMPERATURE+1  31  S_TEMP        Info          V_VAR1    <ms>        current max send frequency for temperature
      Out CHILD_ID_TEMPERATURE+2  32  S_TEMP        Info          V_VAR2    <ms>        current unchanged send timeout for temperature

      In  CHILD_ID_HUMIDITY       40  S_HUM         Action  C_SET V_CUSTOM  1=<ms>      set max send frequency ; def_max_frequency_timeout = 120000 (2mn) ; 0 for stop
      In  CHILD_ID_HUMIDITY       40  S_HUM         Action  C_SET V_CUSTOM  2=<ms>      set unchanged send timeout ; def_unchanged_timeout = 3600000 (1h)
      Out CHILD_ID_HUMIDITY       40  S_HUM         Info          V_HUM     <num>       humidity
      Out CHILD_ID_HUMIDITY+1     41  S_HUM         Info          V_VAR1    <ms>        current max send frequency for humidity
      Out CHILD_ID_HUMIDITY+2     42  S_HUM         Info          V_VAR2    <ms>        current unchanged send timeout for humidity

      In  CHILD_ID_DOOR           50  S_DOOR        Action  C_SET V_CUSTOM  1=<ms>      set max send frequency ; def_max_frequency_timeout = 120000 (2mn) ; 0 for stop
      In  CHILD_ID_DOOR           50  S_DOOR        Action  C_SET V_CUSTOM  2=<ms>      set unchanged send timeout ; def_unchanged_timeout = 3600000 (1h)
      Out CHILD_ID_DOOR           50  S_DOOR        Info          V_ARMED   true|false  whether the door opening detection is armed
      Out CHILD_ID_DOOR+1         51  S_DOOR        Info          V_TRIPPED true|false  whether the door is opened (false if not armed)
      Out CHILD_ID_DOOR+2         52  S_DOOR        Info          V_VAR1    <ms>        current max send frequency for door opening detection
      Out CHILD_ID_DOOR+3         53  S_DOOR        Info          V_VAR2    <ms>        current unchanged send timeout for door opening detection
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * NOTE: this sketch has been written for MySensor v 2.1 library.
 * 
 * pwi 2017- 3-25 activate repeater feature
 * pwi 2017- 4- 2 use pwiTimer + parms are updatable
 * pwi 2017- 5-10 better manage post-flood
 * pwi 2017- 5-15 review flood subsystem
 * pwi 2017- 5-22 fix flood rearming
 * pwi 2019- 5-16 improve comments
 * pwi 2019- 5-17 v7.0-2019
 *                  introduce the pwiSensor and pwiAlarm classes
 * pwi 2019- 5-19 v7.1-2019
 *                  review the messaging vision to have one child per information message
 *                  remove the pwiAlarm class (no grace delay) to take place in the Nano
 *                  use patched MySensors 2.1.1 to publish the library version
 * pwi 2019- 5-19 v7.2-2019
 *                  fix lighting of flood armed LED ar startup
 *
  Sketch uses 28142 bytes (91%) of program storage space. Maximum is 30720 bytes.
  Global variables use 1794 bytes (87%) of dynamic memory, leaving 254 bytes for local variables. Maximum is 2048 bytes.
 */

// uncomment for debugging this sketch
#define DEBUG_ENABLED

// uncomment for debugging EEPROM read/writes
#define EEPROM_DEBUG

static const char * const thisSketchName    = "mysCellar";
static const char * const thisSketchVersion = "7.2-2019";

/*
 * The current configuration
 * Read from/written to the EEPROM
 */
typedef struct {
    /* a 'PWI' null-terminated string which marks the structure as initialized */
    char mark[4];
    /* flood detection
       also keep here the permanent status of flood alarm armed (not modified on tripped)
       also keep the last tripped status (in case of a reset) */
    uint8_t       flood_armed;
    unsigned long flood_max_frequency_timeout;
    unsigned long flood_unchanged_timeout;
    unsigned long flood_grace_delay;
    unsigned long flood_advert_period;
    /* flood measure */
    unsigned long rain_max_frequency_timeout;
    unsigned long rain_unchanged_timeout;
    /* temperature measure */
    unsigned long temp_max_frequency_timeout;
    unsigned long temp_unchanged_timeout;
    /* humidity measure */
    unsigned long hum_max_frequency_timeout;
    unsigned long hum_unchanged_timeout;
    /* door opening detection */
    uint8_t       door_armed;
    unsigned long door_max_frequency_timeout;
    unsigned long door_unchanged_timeout;
    unsigned long door_grace_delay;
    unsigned long door_advert_period;
}
  sEeprom;

sEeprom eeprom;
  
static unsigned long def_max_frequency_timeout = 120000;        // 2 mn
static unsigned long def_unchanged_timeout = 3600000;           // 1 h
static unsigned long def_grace_delay = 0;
static unsigned long def_advert_period = 0;

/* The MySensors part */
#define MY_NODE_ID 4
#define MY_DEBUG
#define MY_REPEATER_FEATURE
#define MY_RADIO_NRF24
#define MY_RF24_CHANNEL 103
#define MY_RF24_PA_LEVEL RF24_PA_MIN
#define MY_SIGNING_SOFT
#define MY_SIGNING_SOFT_RANDOMSEED_PIN 7
#define MY_SOFT_HMAC_KEY 0xe5,0xc5,0x36,0xd8,0x4b,0x45,0x49,0x25,0xaa,0x54,0x3b,0xcc,0xf4,0xcb,0xbb,0xb7,0x77,0xa4,0x80,0xae,0x83,0x75,0x40,0xc2,0xb1,0xcb,0x72,0x50,0xaa,0x8a,0xf1,0x6d
#include <MySensors.h>

/* The four sensors + the main one
 * - main let us acts on global configuration
 * - alert sensors react to the alert timer (send at least one message if unchanged)
 * - measure sensors react to measure timers (have a min frequency and a max frequency)
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
#include "pwi_sensor.h"
#include "pwi_timer.h"

/* ****************************************************************************
 * Flood detection is provided by a rain sensor which itself provides
 *  both digital and analog values:
 *  - digital value is used as an armable alert by CHILD_ID_FLOOD
 *  - analog value is managed as a standard measure by CHILD_ID_RAIN.
 */
// rain sensor analog output
#define FLOOD_ANALOGINPUT       (A7)
#define FLOOD_ANALOG_MIN        (0)            // flooded
#define FLOOD_ANALOG_MAX        (1023)         // dry
// rain sensor digital output
#define FLOOD_DIGITALINPUT      (4)
#define FLOOD_DIGITAL_FLOODED   (LOW)
#define FLOOD_DIGITAL_DRY       (HIGH)

#define FLOOD_TRIPPED_LED       (5)
#define FLOOD_ARMED_LED         (6)

bool floodMeasureCb( void *user_data=NULL );
void floodSendCb( void *user_data=NULL );
#ifdef ALARM_GRACE_DELAY
void floodAdvertCb( void *user_data=NULL );
void floodAlertCb( void *user_data=NULL );
#endif

pwiSensor flood_sensor;
bool      flood_tripped = false;

void floodPresentation()  
{
    flood_sensor.present( CHILD_ID_FLOOD, S_WATER_LEAK, "Flood detection" );
    present( CHILD_ID_FLOOD+1, S_WATER_LEAK, "Flood detection. Tripped" );
    present( CHILD_ID_FLOOD+2, S_WATER_LEAK, "Flood detection. Min period" );
    present( CHILD_ID_FLOOD+3, S_WATER_LEAK, "Flood detection. Max period" );
#ifdef ALARM_GRACE_DELAY
    present( CHILD_ID_FLOOD+4, S_WATER_LEAK, "Flood detection. Grace delay" );
    present( CHILD_ID_FLOOD+5, S_WATER_LEAK, "Flood detection. Advertising period" );
    present( CHILD_ID_FLOOD+6, S_WATER_LEAK, "Flood detection. Remaining grace delay" );
#endif
}

void floodSetup()  
{
    digitalWrite( FLOOD_ARMED_LED, LOW );
    pinMode( FLOOD_ARMED_LED, OUTPUT );
    digitalWrite( FLOOD_TRIPPED_LED, LOW );
    pinMode( FLOOD_TRIPPED_LED, OUTPUT );

    flood_sensor.setup( eeprom.flood_unchanged_timeout, eeprom.flood_max_frequency_timeout, floodMeasureCb, floodSendCb );
    floodSetArmed( eeprom.flood_armed );
}

/* Regarding the flood detection, we are only interested by the digital value
 * Sends a change as soon as the measure changes.
 * Does not send anything if the alarm is not armed.
 */
bool floodMeasureCb( void *user_data )
{
    bool changed = false;
    if( eeprom.flood_armed ){
        bool cur_flooded = ( digitalRead( FLOOD_DIGITALINPUT ) == FLOOD_DIGITAL_FLOODED );
        if( cur_flooded != flood_tripped ){
            floodSetTripped( cur_flooded );
            changed = true;
        }
    }
    return( changed );
}

void floodSendCb( void *user_data )
{
    msg.clear();
    send( msg.setSensor( CHILD_ID_FLOOD ).setType( V_ARMED ).set( eeprom.flood_armed ));
    msg.clear();
    send( msg.setSensor( CHILD_ID_FLOOD+1 ).setType( V_TRIPPED ).set( flood_tripped ));
}

void floodSetArmed( bool armed )
{
    eeprom.flood_armed = armed;
    eepromWrite( eeprom );
    digitalWrite( FLOOD_ARMED_LED, eeprom.flood_armed ? HIGH:LOW );
    floodSetTripped( false );
}

/* Set the tripped status of the flood detection.
 *  + send the corresponding state to the controller
 *  On tripped:
 *  - start the rearm timer
 */
void floodSetTripped( bool tripped )
{
    flood_tripped = tripped;
    digitalWrite( FLOOD_TRIPPED_LED, tripped ? HIGH:LOW );
}

void floodSetMaxFrequency( unsigned long ms )
{
    eeprom.flood_max_frequency_timeout = ms;
    eepromWrite( eeprom );
    flood_sensor.setMinPeriod( ms );
    floodSendMaxFrequency();
}

void floodSendMaxFrequency()
{
    msg.clear();
    send( msg.setSensor( CHILD_ID_FLOOD+2 ).setType( V_VAR1 ).set( eeprom.flood_max_frequency_timeout ));
}

void floodSetUnchangedTimeout( unsigned long ms )
{
    eeprom.flood_unchanged_timeout = ms;
    eepromWrite( eeprom );
    flood_sensor.setMaxPeriod( ms );
    floodSendUnchangedTimeout();
}

void floodSendUnchangedTimeout()
{
    msg.clear();
    send( msg.setSensor( CHILD_ID_FLOOD+3 ).setType( V_VAR2 ).set( eeprom.flood_unchanged_timeout ));
}

#ifdef ALARM_GRACE_DELAY
/* Set the armed status of the flood detection
 *  + send the corresponding state to the controller
 *  + reinitialize the tripped status
 *    thus also sending the new tripped status
 */
void floodSetGraceDelay( unsigned long ms )
{
    eeprom.flood_grace_delay = ms;
    eepromWrite( eeprom );
    flood_sensor.setGraceDelay( ms );
    floodSendGraceDelay();
}

void floodSendGraceDelay()
{
    msg.clear();
    send( msg.setSensor( CHILD_ID_FLOOD+4 ).setType( V_VAR3 ).set( eeprom.flood_grace_delay ));
}

void floodSetAdvertPeriod( unsigned long ms )
{
    eeprom.flood_advert_period = ms;
    eepromWrite( eeprom );
    flood_sensor.setAdvertisingPeriod( ms );
    floodSendAdvertPeriod();
}

void floodSendAdvertPeriod()
{
    msg.clear();
    send( msg.setSensor( CHILD_ID_FLOOD+5 ).setType( V_VAR4 ).set( eeprom.flood_grace_delay ));
}

void floodSendRemainingDelay()
{
    msg.clear();
    send( msg.setSensor( CHILD_ID_FLOOD+6 ).setType( V_VAR5 ).set( flood_sensor.getRemainingDelay()));
}
#endif

void floodReceive( const char *payload )
{
    uint8_t ureq = strlen( payload ) > 0 ? atoi( payload ) : 0;
    unsigned long ulong = strlen( payload ) > 2 ? atol( payload+2 ) : 0;
    switch( ureq ){
        case 1:
            floodSetMaxFrequency( ulong );
            break;
        case 2:
            floodSetUnchangedTimeout( ulong );
            break;
#ifdef ALARM_GRACE_DELAY
        case 3:
            floodSetGraceDelay( ulong );
            break;
        case 5:
            floodSetAdvertPeriod( ulong );
            break;
#endif
        default:
            flood_sensor.setArmed( payload );
            floodSetArmed( flood_sensor.isArmed());
            floodSendCb();
            break;
    }
}

/* ****************************************************************************
 * Dealing with analog part of the rain sensor.
 */
bool rainMeasureCb( void *user_data=NULL );
void rainSendCb( void *user_data=NULL );

pwiSensor rain_sensor;
int       rain_last = 0;

void rainPresentation()  
{
    rain_sensor.present( CHILD_ID_RAIN, S_RAIN, "Rain sensor" );
    present( CHILD_ID_RAIN+1, S_RAIN, "Rain sensor. Min period" );
    present( CHILD_ID_RAIN+2, S_RAIN, "Rain sensor. Max period" );
}

void rainSetup()  
{
    rain_sensor.setup( eeprom.rain_unchanged_timeout, eeprom.rain_max_frequency_timeout, rainMeasureCb, rainSendCb );
}

bool rainMeasureCb( void *user_data )
{
    bool changed = false;
    int cur_rain = analogRead( FLOOD_ANALOGINPUT );

    if( cur_rain != rain_last ){
        rain_last = cur_rain;
        changed = true;
    }
    return( changed );
}

void rainSendCb( void *user_data )
{
    msg.clear();
    send( msg.setSensor( CHILD_ID_RAIN ).setType( V_RAIN ).set( rain_last ));
#ifdef DEBUG_ENABLED
    Serial.print( F( "[rainSendCb] rain=" ));
    Serial.print( rain_last );
    uint8_t range = map( rain_last, FLOOD_ANALOG_MIN, FLOOD_ANALOG_MAX, 0, 3 );
    Serial.print( F( ", mapped=" ));
    Serial.print( range );
    switch( range ){
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

void rainSetMaxFrequency( unsigned long ms )
{
    eeprom.rain_max_frequency_timeout = ms;
    eepromWrite( eeprom );
    rain_sensor.setMinPeriod( ms );
    rainSendMaxFrequency();
}

void rainSendMaxFrequency()
{
    msg.clear();
    send( msg.setSensor( CHILD_ID_RAIN+1 ).setType( V_VAR1 ).set( eeprom.rain_max_frequency_timeout ));
}

void rainSetUnchangedTimeout( unsigned long ms )
{
    eeprom.rain_unchanged_timeout = ms;
    eepromWrite( eeprom );
    rain_sensor.setMaxPeriod( ms );
    rainSendUnchangedTimeout();
}

void rainSendUnchangedTimeout()
{
    msg.clear();
    send( msg.setSensor( CHILD_ID_RAIN+2 ).setType( V_VAR2 ).set( eeprom.rain_unchanged_timeout ));
}

void rainReceive( const char *payload )
{
    uint8_t ureq = strlen( payload ) > 0 ? atoi( payload ) : 0;
    unsigned long ulong = strlen( payload ) > 2 ? atol( payload+2 ) : 0;
    switch( ureq ){
        case 1:
            rainSetMaxFrequency( ulong );
            break;
        case 2:
            rainSetUnchangedTimeout( ulong );
            break;
    }
}

/* ****************************************************************************
 * Temperature/Humidity DHT22/AM2302 sensors
 * There is only one physical sensor module for the two measures.
 */
#include <DHT.h>  
DHT dht;

// AM2302 DAT digital output
#define TEMPHUM_DIGITALINPUT    (3)

bool tempMeasureCb( void *user_data=NULL );
void tempSendCb( void *user_data=NULL );

pwiSensor temp_sensor;
float     temp_last = 0.0;

void tempPresentation()  
{
    temp_sensor.present( CHILD_ID_TEMPERATURE, S_TEMP, "Temperature sensor" );
    present( CHILD_ID_TEMPERATURE+1, S_TEMP, "Temperature sensor. Min period" );
    present( CHILD_ID_TEMPERATURE+2, S_TEMP, "Temperature sensor. Max period" );
}

/* Read the temperature as a float.
 * If changed or @forceSend is %true, then send the temperature value to the controller.
 */
void tempSetup()  
{
    // temperature/humidity
    dht.setup( TEMPHUM_DIGITALINPUT, DHT::AM2302 ); 

    temp_sensor.setup( eeprom.temp_unchanged_timeout, eeprom.temp_max_frequency_timeout, tempMeasureCb, tempSendCb );
}

bool tempMeasureCb( void *user_data )
{
    bool changed = false;

    // Get temperature from DHT library
    float cur_temp = dht.getTemperature();
    if( isnan( cur_temp )){
        Serial.println( F( "[tempOnMeasureCb] failed reading temperature from DHT" ));
    } else if( cur_temp != temp_last ){
        temp_last = cur_temp;
        changed = true;
    }

    return( changed );
}

void tempSendCb( void *user_data )
{
    msg.clear();
    send( msg.setSensor( CHILD_ID_TEMPERATURE ).setType( V_TEMP ).set( temp_last, 1 ));
#ifdef DEBUG_ENABLED
    Serial.print( F( "[tempSendCb] temp=" ));
    Serial.print( temp_last,1 );
    Serial.println( F( "°C" ));
#endif
}

void tempSetMaxFrequency( unsigned long ms )
{
    eeprom.temp_max_frequency_timeout = ms;
    eepromWrite( eeprom );
    temp_sensor.setMinPeriod( ms );
    tempSendMaxFrequency();
}

void tempSendMaxFrequency()
{
    msg.clear();
    send( msg.setSensor( CHILD_ID_TEMPERATURE+1 ).setType( V_VAR1 ).set( eeprom.temp_max_frequency_timeout ));
}

void tempSetUnchangedTimeout( unsigned long ms )
{
    eeprom.temp_unchanged_timeout = ms;
    eepromWrite( eeprom );
    temp_sensor.setMaxPeriod( ms );
    tempSendUnchangedTimeout();
}

void tempSendUnchangedTimeout()
{
    msg.clear();
    send( msg.setSensor( CHILD_ID_TEMPERATURE+2 ).setType( V_VAR2 ).set( eeprom.temp_unchanged_timeout ));
}

void tempReceive( const char *payload )
{
    uint8_t ureq = strlen( payload ) > 0 ? atoi( payload ) : 0;
    unsigned long ulong = strlen( payload ) > 2 ? atol( payload+2 ) : 0;
    switch( ureq ){
        case 1:
            tempSetMaxFrequency( ulong );
            break;
        case 2:
            tempSetUnchangedTimeout( ulong );
            break;
    }
}

/* ****************************************************************************
 * Read the humidity as a float.
 * If changed or @forceSend is %true, then send the humidity value to the controller.
 */
bool humMeasureCb( void *user_data=NULL );
void humSendCb( void *user_data=NULL );

pwiSensor hum_sensor;
float     hum_last = 0.0;

void humPresentation()  
{
    hum_sensor.present( CHILD_ID_HUMIDITY, S_HUM, "Humidity sensor" );
    present( CHILD_ID_HUMIDITY+1, S_HUM, "Humidity sensor. Min period" );
    present( CHILD_ID_HUMIDITY+2, S_HUM, "Humidity sensor. Max period" );
}

void humSetup()  
{
    hum_sensor.setup( eeprom.hum_unchanged_timeout, eeprom.hum_max_frequency_timeout, humMeasureCb, humSendCb );
}

bool humMeasureCb( void *user_data )
{
    bool changed = false;

    // Get temperature from DHT library
    float cur_hum = dht.getHumidity();
    if( isnan( cur_hum )){
        Serial.println( F( "[humMeasureCb] failed reading humidity from DHT" ));
    } else if( cur_hum != hum_last ){
        hum_last = cur_hum;
        changed = true;
    }

    return( changed );
}

void humSendCb( void *user_data )
{
    msg.clear();
    send( msg.setSensor( CHILD_ID_HUMIDITY ).setType( V_HUM ).set( hum_last, 1 ));
#ifdef DEBUG_ENABLED
    Serial.print( F( "[humSendCb] humidity=" ));
    Serial.print( hum_last, 1 );
    Serial.println( F( "%" ));
#endif
}

void humSetMaxFrequency( unsigned long ms )
{
    eeprom.hum_max_frequency_timeout = ms;
    eepromWrite( eeprom );
    hum_sensor.setMinPeriod( ms );
    humSendMaxFrequency();
}

void humSendMaxFrequency()
{
    msg.clear();
    send( msg.setSensor( CHILD_ID_HUMIDITY+1 ).setType( V_VAR1 ).set( eeprom.hum_max_frequency_timeout ));
}

void humSetUnchangedTimeout( unsigned long ms )
{
    eeprom.hum_unchanged_timeout = ms;
    eepromWrite( eeprom );
    hum_sensor.setMaxPeriod( ms );
    humSendUnchangedTimeout();
}

void humSendUnchangedTimeout()
{
    msg.clear();
    send( msg.setSensor( CHILD_ID_HUMIDITY+2 ).setType( V_VAR2 ).set( eeprom.hum_unchanged_timeout ));
}

void humReceive( const char *payload )
{
    uint8_t ureq = strlen( payload ) > 0 ? atoi( payload ) : 0;
    unsigned long ulong = strlen( payload ) > 2 ? atol( payload+2 ) : 0;
    switch( ureq ){
        case 1:
            humSetMaxFrequency( ulong );
            break;
        case 2:
            humSetUnchangedTimeout( ulong );
            break;
    }
}

/* ****************************************************************************
 * Opening door detection
 * The sensor is provided by the Telemecanique door switch.
 * This is a double-relais platine, with a NO and a NC contacts.
 * The NC contact is used by origin for lights.
 * We are using here the NO contact for the door opening detector:
 * - door closed : contact is closed
 * - door opened: contact is opened.
 */
#define OPENING_INPUT           (A0)           // use analog pin as a digital input
#define OPENING_LED             (A1)

bool doorMeasureCb( void *user_data=NULL );
void doorSendCb( void *user_data=NULL );

pwiSensor door_sensor;
bool      door_opened = false;

void doorPresentation()  
{
    door_sensor.present( CHILD_ID_DOOR, S_DOOR, "Door opening detection" );
    present( CHILD_ID_DOOR+1, S_DOOR, "Door opening detection. Tripped" );
    present( CHILD_ID_DOOR+2, S_DOOR, "Door opening detection. Min period" );
    present( CHILD_ID_DOOR+3, S_DOOR, "Door opening detection. Max period" );
#ifdef ALARM_GRACE_DELAY
    present( CHILD_ID_DOOR+4, S_DOOR, "Door opening detection. Grace delay" );
    present( CHILD_ID_DOOR+5, S_DOOR, "Door opening detection. Advertising period" );
    present( CHILD_ID_DOOR+6, S_DOOR, "Door opening detection. Remaining grace delay" );
#endif
}

void doorSetup()  
{
    pinMode( OPENING_INPUT, INPUT );
    digitalWrite( OPENING_LED, LOW );
    pinMode( OPENING_LED, OUTPUT );

    door_sensor.setup( eeprom.door_unchanged_timeout, eeprom.door_max_frequency_timeout, doorMeasureCb, doorSendCb );
}

bool doorMeasureCb( void *user_data )
{
    bool changed = false;
    bool cur_opened = ( digitalRead( OPENING_INPUT ) == HIGH );

    if( cur_opened != door_opened ){
        door_opened = cur_opened;
        digitalWrite( OPENING_LED, door_opened ? HIGH:LOW );
        changed = true;
    }

    return( changed );
}

void doorSendCb( void *user_data )
{
    msg.clear();
    send( msg.setSensor( CHILD_ID_DOOR ).setType( V_ARMED ).set( eeprom.door_armed ));
    msg.clear();
    send( msg.setSensor( CHILD_ID_DOOR+1 ).setType( V_TRIPPED ).set( door_opened ));
#ifdef DEBUG_ENABLED
    Serial.print( F( "[doorSendCb] opened=" ));
    Serial.println( door_opened ? "True":"False" );
#endif
}

void doorSetMaxFrequency( unsigned long ms )
{
    eeprom.door_max_frequency_timeout = ms;
    eepromWrite( eeprom );
    door_sensor.setMinPeriod( ms );
    doorSendMaxFrequency();
}

void doorSendMaxFrequency()
{
    msg.clear();
    send( msg.setSensor( CHILD_ID_DOOR+2 ).setType( V_VAR1 ).set( eeprom.door_max_frequency_timeout ));
}

void doorSetUnchangedTimeout( unsigned long ms )
{
    eeprom.door_unchanged_timeout = ms;
    eepromWrite( eeprom );
    door_sensor.setMaxPeriod( ms );
    doorSendUnchangedTimeout();
}

void doorSendUnchangedTimeout()
{
    msg.clear();
    send( msg.setSensor( CHILD_ID_DOOR+3 ).setType( V_VAR2 ).set( eeprom.door_unchanged_timeout ));
}

#ifdef ALARM_GRACE_DELAY
void doorSetArmed( bool armed )
{
    eeprom.door_armed = armed;
    eepromWrite( eeprom );
    door_sensor.setArmed( armed );
}

void doorSetGraceDelay( unsigned long ms )
{
    eeprom.door_grace_delay = ms;
    eepromWrite( eeprom );
    door_sensor.setGraceDelay( ms );
    doorSendGraceDelay();
}

void doorSendGraceDelay()
{
    msg.clear();
    send( msg.setSensor( CHILD_ID_DOOR+4 ).setType( V_VAR3 ).set( eeprom.door_grace_delay ));
}

void doorSetAdvertPeriod( unsigned long ms )
{
    eeprom.door_advert_period = ms;
    eepromWrite( eeprom );
    door_sensor.setAdvertisingPeriod( ms );
    doorSendAdvertPeriod();
}

void doorSendAdvertPeriod()
{
    msg.clear();
    send( msg.setSensor( CHILD_ID_DOOR+5 ).setType( V_VAR4 ).set( eeprom.door_advert_period ));
}

void doorSendRemainingDelay()
{
    msg.clear();
    send( msg.setSensor( CHILD_ID_DOOR+6 ).setType( V_VAR5 ).set( door_sensor.getRemainingDelay()));
}
#endif

void doorReceive( const char *payload )
{
    uint8_t ureq = strlen( payload ) > 0 ? atoi( payload ) : 0;
    unsigned long ulong = strlen( payload ) > 2 ? atol( payload+2 ) : 0;
    switch( ureq ){
        case 1:
            doorSetMaxFrequency( ulong );
            break;
        case 2:
            doorSetUnchangedTimeout( ulong );
            break;
#ifdef ALARM_GRACE_DELAY
        case 3:
            doorSetGraceDelay( ulong );
            break;
        case 4:
            doorSetAdvertPeriod( ulong );
            break;
        default:
            door_sensor.setArmed( payload );
            doorSetArmed( door_sensor.isArmed());
            break;
#endif
    }
}

/* **************************************************************************************
 *  MAIN CODE
 */

// As of MySensors v2.x, presentation() is called before setup().
void presentation()
{
#ifdef DEBUG_ENABLED
    Serial.println( F( "presentation()" ));
#endif
    // sketch presentation
    sendSketchInfo( thisSketchName, thisSketchVersion );

    // sensors presentation
    // do not present the action-only sensor
    //present( CHILD_MAIN, S_CUSTOM, "Commands Target" );
    floodPresentation();
    rainPresentation();
    tempPresentation();
    humPresentation();
    doorPresentation();
}

void setup()  
{
#ifdef DEBUG_ENABLED
    Serial.begin( 115200 );
    Serial.println( F( "setup()" ));
#endif

    eepromRead( eeprom );
    floodSetup();
    rainSetup();
    tempSetup();
    humSetup();
    doorSetup();
    
    pwiTimer::Dump();
    dumpConfiguration();

    // library version
    msg.clear();
    mSetCommand( msg, C_INTERNAL );
    sendAsIs( msg.setSensor( 255 ).setType( I_VERSION ).set( MYSENSORS_LIBRARY_VERSION ));
}

void loop()
{
    pwiTimer::Loop();
}

/* **************************************************************************************
 *  receive
 */
void receive(const MyMessage &message)
{
    char payload[2*MAX_PAYLOAD+1];
    uint8_t cmd = message.getCommand();
    uint8_t ureq;

    memset( payload, '\0', sizeof( payload ));
    message.getString( payload );
#ifdef HAVE_DEBUG
    Serial.print( F( "receive() payload='" )); Serial.print( payload ); Serial.println( "'" ); 
#endif

    // all received messages should be V_CUSTOM
    if( message.type != V_CUSTOM ){
        return;
    }

    if( message.sensor == CHILD_MAIN && cmd == C_REQ ){
        ureq = strlen( payload ) > 0 ? atoi( payload ) : 0;
        switch( ureq ){
            case 1:
                eepromReset( eeprom );
                break;
            case 2:
                dumpConfiguration();
                break;
        }
        return;
    }

    if( cmd == C_SET ){
        switch( message.sensor ){
            case CHILD_ID_FLOOD:
                floodReceive( payload );
                break;
            case CHILD_ID_RAIN:
                rainReceive( payload );
                break;
            case CHILD_ID_TEMPERATURE:
                tempReceive( payload );
                break;
            case CHILD_ID_HUMIDITY:
                humReceive( payload );
                break;
            case CHILD_ID_DOOR:
                doorReceive( payload );
                break;
        }
    }
}

void dumpConfiguration( void )
{
    floodSendCb();
    floodSendMaxFrequency();
    floodSendUnchangedTimeout();
#ifdef ALARM_GRACE_DELAY
    floodSendGraceDelay();
    floodSendAdvertPeriod();
    floodSendRemainingDelay();
#endif

    rainMeasureCb();
    rainSendCb();
    rainSendMaxFrequency();
    rainSendUnchangedTimeout();

    tempMeasureCb();
    tempSendCb();
    tempSendMaxFrequency();
    tempSendUnchangedTimeout();

    humMeasureCb();
    humSendCb();
    humSendMaxFrequency();
    humSendUnchangedTimeout();

    doorSendCb();
    doorSendMaxFrequency();
    doorSendUnchangedTimeout();
#ifdef ALARM_GRACE_DELAY
    doorSendGraceDelay();
    doorSendAdvertPeriod();
    doorSendRemainingDelay();
#endif
}

/**
 * eepromDump:
 * @sdata: the sEeprom data structure to be dumped.
 *
 * Dump the sEeprom struct content.
 */
void eepromDump( sEeprom &sdata )
{
#ifdef EEPROM_DEBUG
    Serial.print( F( "[eepromDump] mark='" ));                       Serial.print( sdata.mark ); Serial.println( "'" );
    Serial.print( F( "[eepromDump] flood_armed=" ));                 Serial.println( sdata.flood_armed ? "True":"False" );
    Serial.print( F( "[eepromDump] flood_max_frequency_timeout=" )); Serial.println( sdata.flood_max_frequency_timeout );
    Serial.print( F( "[eepromDump] flood_unchanged_timeout=" ));     Serial.println( sdata.flood_unchanged_timeout );
    Serial.print( F( "[eepromDump] flood_grace_delay=" ));           Serial.println( sdata.flood_grace_delay );
    Serial.print( F( "[eepromDump] flood_advert_period=" ));         Serial.println( sdata.flood_advert_period );
    Serial.print( F( "[eepromDump] rain_max_frequency_timeout=" ));  Serial.println( sdata.rain_max_frequency_timeout );
    Serial.print( F( "[eepromDump] rain_unchanged_timeout=" ));      Serial.println( sdata.rain_unchanged_timeout );
    Serial.print( F( "[eepromDump] temp_max_frequency_timeout=" ));  Serial.println( sdata.temp_max_frequency_timeout );
    Serial.print( F( "[eepromDump] temp_unchanged_timeout=" ));      Serial.println( sdata.temp_unchanged_timeout );
    Serial.print( F( "[eepromDump] hum_max_frequency_timeout=" ));   Serial.println( sdata.hum_max_frequency_timeout );
    Serial.print( F( "[eepromDump] hum_unchanged_timeout=" ));       Serial.println( sdata.hum_unchanged_timeout );
    Serial.print( F( "[eepromDump] door_armed=" ));                  Serial.println( sdata.door_armed ? "True":"False" );
    Serial.print( F( "[eepromDump] door_max_frequency_timeout=" ));  Serial.println( sdata.door_max_frequency_timeout );
    Serial.print( F( "[eepromDump] door_unchanged_timeout=" ));      Serial.println( sdata.door_unchanged_timeout );
    Serial.print( F( "[eepromDump] door_grace_delay=" ));            Serial.println( sdata.door_grace_delay );
    Serial.print( F( "[eepromDump] door_advert_period=" ));          Serial.println( sdata.door_advert_period );
#endif
}

/**
 * eepromRead:
 * @sdata: the sEeprom data structure to be filled.
 *
 * Read the data from the EEPROM.
 */
void eepromRead( sEeprom &sdata )
{
#ifdef EEPROM_DEBUG
    Serial.println( F( "[eepromRead]" ));
#endif
    memset( &sdata, '\0', sizeof( sdata ));
    uint16_t i;
    for( i=0 ; i<sizeof( sdata ); ++i ){
        (( uint8_t * ) &sdata )[i] = loadState(( uint8_t ) i );
    }
    // initialize with default values if found zero
    if( sdata.mark[0] != 'P' || sdata.mark[1] != 'W' || sdata.mark[2] != 'I' || sdata.mark[3] != 0 ){
        eepromReset( sdata );
    } else {
        eepromDump( sdata );
    }
}

/**
 * eepromReset:
 * @sdata: the sEeprom data structure to be filled.
 *
 * Reset the EEPROM to its default values.
 */
void eepromReset( sEeprom &sdata )
{
#ifdef EEPROM_DEBUG
    Serial.println( F( "[eepromReset]" ));
#endif
    memset( &sdata, '\0', sizeof( sdata ));
    strcpy( sdata.mark, "PWI" );
    sdata.flood_armed = true;
    sdata.flood_max_frequency_timeout = def_max_frequency_timeout;
    sdata.flood_unchanged_timeout = def_unchanged_timeout;
    sdata.flood_grace_delay = def_grace_delay;
    sdata.flood_advert_period = def_advert_period;
    sdata.rain_max_frequency_timeout = def_max_frequency_timeout;
    sdata.rain_unchanged_timeout = def_unchanged_timeout;
    sdata.temp_max_frequency_timeout = def_max_frequency_timeout;
    sdata.temp_unchanged_timeout = def_unchanged_timeout;
    sdata.hum_max_frequency_timeout = def_max_frequency_timeout;
    sdata.hum_unchanged_timeout = def_unchanged_timeout;
    sdata.door_armed = true;
    sdata.door_max_frequency_timeout = def_max_frequency_timeout;
    sdata.door_unchanged_timeout = def_unchanged_timeout;
    sdata.door_grace_delay = def_grace_delay;
    sdata.door_advert_period = def_advert_period;
    eepromWrite( sdata );
}

/**
 * eepromWrite:
 * @sdata: the sEeprom data structure to be written.
 *
 * Write the data to the EEPROM.
 */
void eepromWrite( sEeprom &sdata )
{
#ifdef EEPROM_DEBUG
    Serial.println( F( "[eepromWrite]" ));
#endif
    uint16_t i;
    for( i=0 ; i<sizeof( sdata ) ; ++i ){
        saveState( i, (( uint8_t * ) &sdata )[i] );
    }
    eepromDump( sdata );
}

