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
 *    Sens  Child                 Id  Cmd    Message    Payload     Comment
 *    ----  --------------------  --  -----  ---------  ----------  ------------------------------------------------------------------------------
 *    In    CHILD_MAIN_0           0  C_SET  V_CUSTOM   1           reset EEPROM
 *    In    CHILD_MAIN_0           0  C_SET  V_CUSTOM   2;<ms>      set max frequency for all status, def_max_frequency_timeout = 120000 (2mn)
 *    In    CHILD_MAIN_0           0  C_SET  V_CUSTOM   3;<ms>      set unchanged timeout for all status, def_unchanged_timeout = 3600000 (1h)
 *    In    CHILD_MAIN_0           0  C_SET  V_CUSTOM   4;<ms>      set timeout for alerts, def_alert_timeout = 250ms
 *    In    CHILD_MAIN_0           0  C_REQ  V_CUSTOM   1           dump configuration
 *    In    CHILD_ID_FLOOD_0      10  C_SET  V_ARMED    0|1         arm (payload>0) / unarm (payload=0) the flood alarm
 *    In    CHILD_ID_FLOOD_0      10  C_SET  V_CUSTOM   1;<ms>      set the rearm delay of the flood alarm, def_flood_rearm_delay = 43200000 (12h)
 *    Out   CHILD_MAIN_0           0         V_VAR1     <ms>        dump configuration: max frequency
 *    Out   CHILD_MAIN_1           1         V_VAR1     <ms>        dump configuration: unchanged timeout
 *    Out   CHILD_MAIN_2           2         V_VAR1     <ms>        dump configuration: alert timeout
 *    Out   CHILD_ID_FLOOD_0      10         V_VAR1     true|false  dump configuration: whether the flood alarm is armed (EEPROM status)
 *    Out   CHILD_ID_FLOOD_1      11         V_VAR1     true|false  dump configuration: whether the flood alarm is armed (current status)
 *    Out   CHILD_ID_FLOOD_2      12         V_TRIPPED  true|false  whether the flood alarm is tripped
 *    Out   CHILD_ID_FLOOD_3      13         V_VAR1     <ms>        dump configuration: rearm delay of the flood alarm
 *    Out   CHILD_ID_RAIN         20         V_RAIN     <num>       rain analogic value
 *    Out   CHILD_ID_TEMPERATURE  21         V_TEMP     <num>       temperature
 *    Out   CHILD_ID_HUMIDITY     22         V_HUM      <num>       humidity
 *    Out   CHILD_ID_DOOR         23         V_DOOR     true|false  whether the door is opened
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
 */

// uncomment for debugging this sketch
#define DEBUG_ENABLED

// uncomment for debugging EEPROM read/writes
#define EEPROM_DEBUG

static const char * const thisSketchName    = "mysCellar";
static const char * const thisSketchVersion = "6.1.2017";

/*
 * The current configuration
 * Read from/written to the EEPROM
 */
typedef struct {
    /* a 'PWI' null-terminated string which marks the structure as initialized */
    char mark[4];
    unsigned long alert_timeout;
    unsigned long max_frequency_timeout;
    unsigned long unchanged_timeout;
    /* the permanent status of flood alarm armed (not modified on tripped) */
    uint8_t       flood_armed;
    unsigned long flood_rearm_delay;
}
  sEeprom;

sEeprom eeprom;
  
/* 
 * Declare the timers
 */
#include "pwi_timer.h"

static unsigned long def_alert_timeout = 250;                   // this is the alarm reaction time
static unsigned long def_max_frequency_timeout = 120000;        // 2 mn
static unsigned long def_unchanged_timeout = 3600000;           // 1 h
static unsigned long def_flood_rearm_delay = 43200000;          // 12h

/* The MySensors part */
//#define MY_DEBUG
#define MY_REPEATER_FEATURE
#define MY_RADIO_NRF24
#define MY_RF24_CHANNEL 103
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
    CHILD_MAIN_0 = 0,
    CHILD_MAIN_1,
    CHILD_MAIN_2,
    CHILD_ID_FLOOD_0 = 10,
    CHILD_ID_FLOOD_1,
    CHILD_ID_FLOOD_2,
    CHILD_ID_FLOOD_3,
    CHILD_ID_FLOOD_4,
    CHILD_ID_RAIN = 20,
    CHILD_ID_TEMPERATURE,
    CHILD_ID_HUMIDITY,
    CHILD_ID_DOOR,
};

MyMessage msg;

/*
 * Flood detection is provided by a rain sensor which itself provides
 *  both digital and analog values:
 *  - digital value is used as an armable alert.
 *  - analog value is managed as a standard measure.
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

bool st_flood_armed = false;                   // flood armed
bool st_flood_tripped = false;                 // flood detected

pwiTimer st_flood_remaining_timer;
pwiTimer st_flood_alert_timer;
pwiTimer st_flood_unchanged_timer;
pwiTimer st_flood_rearm_timer;

pwiTimer st_rain_mf_timer;
pwiTimer st_rain_unchanged_timer;

/* Set the armed status of the flood detection
 *  - either because a flood has been detected, and only set a current armed status
 *  - or because the configuration has been requested to change, and then also set
 *    the permanent armed status in the eeprom.
 */
void floodSetArmed( bool armed, bool bPermanent=true, bool bSend=true )
{
    st_flood_armed = armed;
    digitalWrite( FLOOD_ARMED_LED, st_flood_armed ? HIGH:LOW );

    if( bPermanent ){
        floodSetTripped( false );
        eeprom.flood_armed = st_flood_armed;
        eeprom_write( eeprom );
    }

    if( bSend ){
        floodSendArmed();
    }

    st_flood_remaining_timer.stop();
}

void floodSendArmed( void )
{
    msg.clear();
    send( msg.setSensor( CHILD_ID_FLOOD_1 ).setType( V_VAR1 ).set( st_flood_armed ));
    msg.clear();
    send( msg.setSensor( CHILD_ID_FLOOD_0 ).setType( V_VAR1 ).set( eeprom.flood_armed ));
}

/* Set the tripped status of the flood detection.
 *  Send the corresponding message to the controller.
 *  On tripped:
 *  - unarms the alarm
 *  - start the rearm timer
 *  - start a timer to advertise the controller of the remaining on MaxFrequency
 */
void floodSetTripped( bool tripped )
{
    st_flood_tripped = tripped;
    digitalWrite( FLOOD_TRIPPED_LED, st_flood_tripped ? HIGH:LOW );

    // if flood sensor is tripped, then disable the alert
    // and start the timer to rearm it later
    if( tripped ){
        floodSetArmed( false, false );
        st_flood_rearm_timer.start();
        st_flood_remaining_timer.start();
    }
}

/* Regarding the flood detection, we are only interested by the digital value
 * Sends a change as soon as the measure changes.
 * Does not send anything if the alarm is not armed.
 */
void floodReadAndSend( bool forceSend )
{
    if( st_flood_armed ){
        bool changed = false;
        bool cur_flooded = ( digitalRead( FLOOD_DIGITALINPUT ) == FLOOD_DIGITAL_FLOODED );
        if( cur_flooded != st_flood_tripped ){
            floodSetTripped( cur_flooded );
            changed = true;
        }
        if( changed || forceSend ){
            msg.clear();
            send( msg.setSensor( CHILD_ID_FLOOD_2 ).setType( V_TRIPPED ).set( st_flood_tripped ));
            st_flood_unchanged_timer.restart();
#ifdef DEBUG_ENABLED
            Serial.print( F( "[floodReadAndSend] forceSend=" ));
            Serial.print( forceSend ? "True":"False" );
            Serial.print( F( ", changed=" ));
            Serial.print( changed ? "True":"False" );
            Serial.print( F( ", flooded=" ));
            Serial.println( st_flood_tripped ? "True":"False" );
#endif
        }
    }
}

void floodOnAlertCb( void *empty )
{
    floodReadAndSend( false );
}

void floodOnUnchangedCb( void *empty )
{
    floodReadAndSend( true );
    floodSendArmed();
}

void floodOnRearmCb( void *empty )
{
    floodSetArmed( true, false );
}

void floodOnRemainingCb( void *empty )
{
    msg.clear();
    send( msg.setSensor( CHILD_ID_FLOOD_4 ).setType( V_VAR1 ).set( st_flood_rearm_timer.getRemaining()));
}

/* Dealing with analog part of the rain sensor.
 */
void rainReadAndSend( bool forceSend )
{
    static int last_rain = 0;
    bool changed = false;

    int cur_rain = analogRead( FLOOD_ANALOGINPUT );
    if( cur_rain != last_rain ){
        last_rain = cur_rain;
        changed = true;
    }

    if( changed || forceSend ){
        msg.clear();
        send( msg.setSensor( CHILD_ID_RAIN ).setType( V_RAIN ).set( last_rain ));
        st_rain_unchanged_timer.restart();
#ifdef DEBUG_ENABLED
        Serial.print( F( "[rainReadAndSend] forceSend=" ));
        Serial.print( forceSend ? "True":"False" );
        Serial.print( F( ", changed=" ));
        Serial.print( changed ? "True":"False" );
        Serial.print( F( ", rain=" ));
        Serial.print( last_rain );
        uint8_t range = map( last_rain, FLOOD_ANALOG_MIN, FLOOD_ANALOG_MAX, 0, 3 );
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
}

void rainOnMaxFrequencyCb( void *empty )
{
    rainReadAndSend( false );
}

void onRainUnchangedCb( void *empty )
{
    rainReadAndSend( true );
}

/*
 * Temperature/Humidity DHT22/AM2302 sensors
 * There is only one physical sensor module for the two measures.
 */
#include <DHT.h>  
DHT dht;

// AM2302 DAT digital output
#define TEMPHUM_DIGITALINPUT    (3)

pwiTimer st_temp_mf_timer;
pwiTimer st_temp_unchanged_timer;

pwiTimer st_hum_mf_timer;
pwiTimer st_hum_unchanged_timer;

/* Read the temperature as a float.
 * If changed or @forceSend is %true, then send the temperature value to the controller.
 */
void tempReadAndSend( bool forceSend )
{
    static float last_temp = 0.0;
    bool changed = false;

    // Get temperature from DHT library
    float cur_temp = dht.getTemperature();
    if( isnan( cur_temp )){
        Serial.println( F( "[tempReadAndSend] failed reading temperature from DHT" ));
    } else if( cur_temp != last_temp ){
        last_temp = cur_temp;
        changed = true;
    }

    if( changed || forceSend ){
        msg.clear();
        send( msg.setSensor( CHILD_ID_TEMPERATURE ).setType( V_TEMP ).set( last_temp, 1 ));
        st_temp_unchanged_timer.restart();
#ifdef DEBUG_ENABLED
        Serial.print( F( "[tempReadAndSend] forceSend=" ));
        Serial.print( forceSend ? "True":"False" );
        Serial.print( F( ", changed=" ));
        Serial.print( changed ? "True":"False" );
        Serial.print( F( ", temp=" ));
        Serial.print( last_temp );
        Serial.println( F( "°C" ));
#endif
    }
}

void tempOnMaxFrequencyCb( void *empty )
{
    tempReadAndSend( false );
}

void tempOnUnchangedCb( void *empty )
{
    tempReadAndSend( true );
}

/* Read the humidity as a float.
 * If changed or @forceSend is %true, then send the humidity value to the controller.
 */
void humReadAndSend( bool forceSend )
{
    static float last_hum = 0.0;
    bool changed = false;

    // Get temperature from DHT library
    float cur_hum = dht.getHumidity();
    if( isnan( cur_hum )){
        Serial.println( F( "[humReadAndSend] failed reading humidity from DHT" ));
    } else if( cur_hum != last_hum ){
        last_hum = cur_hum;
        changed = true;
    }

    if( changed || forceSend ){
        msg.clear();
        send( msg.setSensor( CHILD_ID_HUMIDITY ).setType( V_HUM ).set( last_hum, 1 ));
        st_hum_unchanged_timer.restart();
#ifdef DEBUG_ENABLED
        Serial.print( F( "[humReadAndSend] forceSend=" ));
        Serial.print( forceSend ? "True":"False" );
        Serial.print( F( ", changed=" ));
        Serial.print( changed ? "True":"False" );
        Serial.print( F( ", hum=" ));
        Serial.print( last_hum );
        Serial.println( F( "%" ));
#endif
    }
}

void humOnMaxFrequencyCb( void *empty )
{
    humReadAndSend( false );
}

void humOnUnchangedCb( void *empty )
{
    humReadAndSend( true );
}

/*
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

pwiTimer st_door_alert_timer;
pwiTimer st_door_unchanged_timer;

void doorReadAndSend( bool forceSend )
{
    static bool last_opened = false;
    bool changed = false;

    bool cur_opened = ( digitalRead( OPENING_INPUT ) == HIGH );
    if( cur_opened != last_opened ){
        last_opened = cur_opened;
        changed = true;
        digitalWrite( OPENING_LED, last_opened ? HIGH:LOW );
    }

    if( changed || forceSend ){
        msg.clear();
        send( msg.setSensor( CHILD_ID_DOOR ).setType( V_TRIPPED ).set( last_opened ));
        st_door_unchanged_timer.restart();
#ifdef DEBUG_ENABLED
        Serial.print( F( "[doorReadAndSend] forceSend=" ));
        Serial.print( forceSend ? "True":"False" );
        Serial.print( F( ", changed=" ));
        Serial.print( changed ? "True":"False" );
        Serial.print( F( ", opened=" ));
        Serial.println( last_opened ? "True":"False" );
#endif
    }
}

void doorOnAlertCb( void *empty )
{
    doorReadAndSend( false );
}

void doorOnUnchangedCb( void *empty )
{
    doorReadAndSend( true );
}

/* **************************************************************************************
 *  MAIN CODE
 */
void presentation()
{
#ifdef DEBUG_ENABLED
    Serial.println( F( "[presentation]" ));
#endif
    sendSketchInfo( thisSketchName, thisSketchVersion );
    present( CHILD_MAIN_0,         S_CUSTOM,     "General Commands Target" );
    present( CHILD_MAIN_1,         S_CUSTOM,     "UnchangedTimeout" );
    present( CHILD_MAIN_2,         S_CUSTOM,     "AlarmsReactionTime" );
    present( CHILD_ID_FLOOD_0,     S_CUSTOM,     "Flood Commands Target" );
    present( CHILD_ID_FLOOD_1,     S_WATER_LEAK, "Flood detection armed (current status)" );
    present( CHILD_ID_FLOOD_2,     S_WATER_LEAK, "Flood detected" );
    present( CHILD_ID_FLOOD_3,     S_WATER_LEAK, "RearmDelay" );
    present( CHILD_ID_FLOOD_4,     S_WATER_LEAK, "RemainingDelayBeforeRearm" );
    present( CHILD_ID_RAIN,        S_RAIN,       "Rain sensor" );
    present( CHILD_ID_TEMPERATURE, S_TEMP,       "Temperature measure" );
    present( CHILD_ID_HUMIDITY,    S_HUM,        "Humidity measure" );
    present( CHILD_ID_DOOR,        S_DOOR,       "Door opening detection" );
}

void setup()  
{
#ifdef DEBUG_ENABLED
    Serial.begin( 115200 );
    Serial.println( F( "[setup]" ));
#endif

    eeprom_read( eeprom );

    // flood detection
    digitalWrite( FLOOD_TRIPPED_LED, LOW );
    pinMode( FLOOD_TRIPPED_LED, OUTPUT );
    digitalWrite( FLOOD_ARMED_LED, LOW );
    pinMode( FLOOD_ARMED_LED, OUTPUT );

    st_flood_remaining_timer.set( "FloodRemainingTimer", eeprom.max_frequency_timeout, false, floodOnRemainingCb );
    st_flood_alert_timer.start( "FloodAlertTimer", eeprom.alert_timeout, false, floodOnAlertCb, NULL, false );
    st_flood_unchanged_timer.set( "FloodUnchangedTimer", eeprom.unchanged_timeout, false, floodOnUnchangedCb );
    st_flood_rearm_timer.set( "FloodRearmTimer", eeprom.flood_rearm_delay, true, floodOnRearmCb );
    floodSetArmed( eeprom.flood_armed, false, false );

    st_rain_mf_timer.start( "RainRateMaxFrequencyTimer", eeprom.max_frequency_timeout, false, rainOnMaxFrequencyCb );
    st_rain_unchanged_timer.set( "RainRateUnchangedTimer", eeprom.unchanged_timeout, false, onRainUnchangedCb );

    // temperature/humidity
    dht.setup( TEMPHUM_DIGITALINPUT, DHT::AM2302 ); 

    st_temp_mf_timer.start( "TempMaxFrequencyTimer", eeprom.max_frequency_timeout, false, tempOnMaxFrequencyCb );
    st_temp_unchanged_timer.set( "TempUnchangedTimer", eeprom.unchanged_timeout, false, tempOnUnchangedCb );

    st_hum_mf_timer.start( "HumidityMaxFrequencyTimer", eeprom.max_frequency_timeout, false, humOnMaxFrequencyCb );
    st_hum_unchanged_timer.set( "HumidityUnchangedTimer", eeprom.unchanged_timeout, false, humOnUnchangedCb );

    // opening door
    pinMode( OPENING_INPUT, INPUT );
    digitalWrite( OPENING_LED, LOW );
    pinMode( OPENING_LED, OUTPUT );

    st_door_alert_timer.start( "DoorAlertTimer", eeprom.alert_timeout, false, doorOnAlertCb, NULL, false );
    st_door_unchanged_timer.set( "DoorUnchangedTimer", eeprom.unchanged_timeout, false, doorOnUnchangedCb );
    
    pwiTimer::Dump();

    // send all informations a first time
    dumpConfiguration();
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
    uint8_t req;
    int entier;
    unsigned long ulong;

    memset( payload, '\0', sizeof( payload ));
    message.getString( payload );
#ifdef HAVE_DEBUG
    Serial.print( F( "[receive] payload='" )); Serial.print( payload ); Serial.println( "'" ); 
#endif

    switch( message.sensor ){
        case CHILD_MAIN_0:
            switch( cmd ){
                case C_SET:
                    switch( message.type ){
                        case V_CUSTOM:
                            req = strlen( payload ) > 0 ? atoi( payload ) : 0;
                            switch( req ){
                                case 1:
                                    eeprom_reset( eeprom );
                                    break;
                                case 2:
                                    ulong = strlen( payload ) > 2 ? atol( payload+2 ) : 0;
                                    if( ulong > 0 ){
                                        eeprom.max_frequency_timeout = ulong;
                                        eeprom_write( eeprom );
                                        st_flood_remaining_timer.setDelay( ulong );
                                        st_rain_mf_timer.setDelay( ulong );
                                        st_temp_mf_timer.setDelay( ulong );
                                        st_hum_mf_timer.setDelay( ulong );
                                    }
                                    break;
                                case 3:
                                    ulong = strlen( payload ) > 2 ? atol( payload+2 ) : 0;
                                    if( ulong > 0 ){
                                        eeprom.unchanged_timeout = ulong;
                                        eeprom_write( eeprom );
                                        st_flood_unchanged_timer.setDelay( ulong );
                                        st_rain_unchanged_timer.setDelay( ulong );
                                        st_temp_unchanged_timer.setDelay( ulong );
                                        st_hum_unchanged_timer.setDelay( ulong );
                                        st_door_unchanged_timer.setDelay( ulong );
                                    }
                                    break;
                                case 4:
                                    ulong = strlen( payload ) > 2 ? atol( payload+2 ) : 0;
                                    if( ulong > 0 ){
                                        eeprom.alert_timeout = ulong;
                                        eeprom_write( eeprom );
                                        st_flood_alert_timer.setDelay( ulong );
                                        st_door_alert_timer.setDelay( ulong );
                                    }
                                    break;
                            }
                            break;
                    }
                    break;
                case C_REQ:
                    switch( message.type ){
                        case V_CUSTOM:
                            req = strlen( payload ) > 0 ? atoi( payload ) : 0;
                            switch( req ){
                                case 1:
                                    dumpConfiguration();
                                    break;
                            }
                            break;
                    }
                    break;
            }
            break;
        case CHILD_ID_FLOOD_0:
            switch( cmd ){
                case C_SET:
                    switch( message.type ){
                        case V_ARMED:
                            entier = strlen( payload ) > 0 ? atoi( payload ) : 0;
                            floodSetArmed( entier > 0 );
                            break;
                        case V_CUSTOM:
                            req = strlen( payload ) > 0 ? atoi( payload ) : 0;
                            switch( req ){
                                case 1:
                                    ulong = strlen( payload ) > 2 ? atol( payload+2 ) : 0;
                                    if( ulong > 0 ){
                                        eeprom.flood_rearm_delay = ulong;
                                        eeprom_write( eeprom );
                                        st_flood_rearm_timer.setDelay( ulong );
                                    }
                                    break;
                            }
                            break;
                    }
            }
    }
}

void dumpConfiguration( void )
{
    // send global configuration
    msg.clear();
    send( msg.setSensor( CHILD_MAIN_0 ).setType( V_VAR1 ).set( eeprom.max_frequency_timeout ));
    msg.clear();
    send( msg.setSensor( CHILD_MAIN_1 ).setType( V_VAR1 ).set( eeprom.unchanged_timeout ));
    msg.clear();
    send( msg.setSensor( CHILD_MAIN_2 ).setType( V_VAR1 ).set( eeprom.alert_timeout ));

    // send sensors status
    floodSendArmed();
    floodReadAndSend( true );
    msg.clear();
    send( msg.setSensor( CHILD_ID_FLOOD_3 ).setType( V_VAR1 ).set( eeprom.flood_rearm_delay ));

    rainReadAndSend( true );
    tempReadAndSend( true );
    humReadAndSend( true );
    doorReadAndSend( true );
}

/**
 * eeprom_dump:
 * @sdata: the sEeprom data structure to be dumped.
 *
 * Dump the sEeprom struct content.
 */
void eeprom_dump( sEeprom &sdata )
{
#ifdef EEPROM_DEBUG
    Serial.print( F( "[eeprom_dump] mark='" )); Serial.print( sdata.mark ); Serial.println( "'" );
    Serial.print( F( "[eeprom_dump] alert_timeout=" )); Serial.println( sdata.alert_timeout );
    Serial.print( F( "[eeprom_dump] max_frequency_timeout=" )); Serial.println( sdata.max_frequency_timeout );
    Serial.print( F( "[eeprom_dump] unchanged_timeout=" )); Serial.println( sdata.unchanged_timeout );
    Serial.print( F( "[eeprom_dump] flood_armed=" )); Serial.println( sdata.flood_armed ? "True":"False" );
    Serial.print( F( "[eeprom_dump] flood_rearm_delay=" )); Serial.println( sdata.flood_rearm_delay );
#endif
}

/**
 * eeprom_read:
 * @sdata: the sEeprom data structure to be filled.
 *
 * Read the data from the EEPROM.
 */
void eeprom_read( sEeprom &sdata )
{
#ifdef EEPROM_DEBUG
    Serial.println( F( "[eeprom_read]" ));
#endif
    memset( &sdata, '\0', sizeof( sdata ));
    uint16_t i;
    for( i=0 ; i<sizeof( sdata ); ++i ){
        (( uint8_t * ) &sdata )[i] = loadState(( uint8_t ) i );
    }
    // initialize with default values if found zero
    if( sdata.mark[0] != 'P' || sdata.mark[1] != 'W' || sdata.mark[2] != 'I' || sdata.mark[3] != 0 ){
        eeprom_reset( sdata );
    } else {
        eeprom_dump( sdata );
    }
}

/**
 * eeprom_reset:
 * @sdata: the sEeprom data structure to be filled.
 *
 * Reset the EEPROM to its default values.
 */
void eeprom_reset( sEeprom &sdata )
{
#ifdef EEPROM_DEBUG
    Serial.println( F( "[eeprom_reset]" ));
#endif
    memset( &sdata, '\0', sizeof( sdata ));
    strcpy( sdata.mark, "PWI" );
    sdata.alert_timeout = def_alert_timeout;
    sdata.max_frequency_timeout = def_max_frequency_timeout;
    sdata.unchanged_timeout = def_unchanged_timeout;
    sdata.flood_armed = true;
    sdata.flood_rearm_delay = def_flood_rearm_delay;
    eeprom_write( sdata );
}

/**
 * eeprom_write:
 * @sdata: the sEeprom data structure to be written.
 *
 * Write the data to the EEPROM.
 */
void eeprom_write( sEeprom &sdata )
{
#ifdef EEPROM_DEBUG
    Serial.println( F( "[eeprom_write]" ));
#endif
    uint16_t i;
    for( i=0 ; i<sizeof( sdata ) ; ++i ){
        saveState( i, (( uint8_t * ) &sdata )[i] );
    }
    eeprom_dump( sdata );
}

