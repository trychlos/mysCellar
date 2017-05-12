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
 * Temperature/Humidity measures:
 * Based on humidity sensor by Henrik Ekblad <henrik.ekblad@mysensors.org>
 * Copyright (C) 2013-2015 Sensnology AB
 * http://www.mysensors.org/build/humidity
 * Uses DHT22/AM2302 module
 *
 * Radio module:
 * Is implemented with a NRF24L01+ radio module
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
 */

// uncomment for debugging this sketch
#define DEBUG_ENABLED

static const char * const thisSketchName    = "mysCellar";
static const char * const thisSketchVersion = "5.2.2017";

/* 
 * Declare the timers
 */
#include "pwi_timer.h"

static unsigned long st_main_timeout = 250;
pwiTimer main_timer;

static unsigned long st_max_frequency_timeout = 60000;       // 1 mn
pwiTimer temphum_max_timer;
pwiTimer flood_max_timer;

static unsigned long st_unchanged_timeout = 3600000;         // 1 h
pwiTimer flood_unchanged_timer;
pwiTimer temp_unchanged_timer;
pwiTimer hum_unchanged_timer;
pwiTimer door_unchanged_timer;

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
    CHILD_MAIN = 0,
    CHILD_ID_FLOOD = 10,
    CHILD_ID_TEMPERATURE,
    CHILD_ID_HUMIDITY,
    CHILD_ID_DOOR,
    CHILD_ID_FLOOD_AN,
};

MyMessage msgVar1( 0, V_VAR1 );
MyMessage msgVar2( 0, V_VAR2 );

/*
 * Flood detection
 */
// rain sensor analog output
#define FLOOD_ANALOGINPUT       (A7)
#define FLOOD_ANALOG_MIN        (0)            // flooded
#define FLOOD_ANALOG_MAX        (1023)         // dry
// rain sensor digital output
#define FLOOD_DIGITALINPUT      (4)
#define FLOOD_DIGITAL_FLOODED   (LOW)
#define FLOOD_DIGITAL_DRY       (HIGH)

#define FLOOD_LED               (5)

bool st_flooded = false;
int st_flood_an = 0;

// message type
MyMessage msgFlood( CHILD_ID_FLOOD, V_TRIPPED );
MyMessage msgFloodAn( CHILD_ID_FLOOD_AN, V_VAR1 );

/* regarding the flood detection, we are only interested by the digital value
 * analog value is only used during development for debugging help
 * 
 * Sends a change as soon as the measure changes, but not more than 1/mn
 * 
 * Returns: %TRUE if a change must be sent
 */
bool readFlood( void )
{
    bool prev_flood = st_flooded;
    int prev_floodan = st_flood_an;
    bool changed = false;
    st_flooded = ( digitalRead( FLOOD_DIGITALINPUT ) == FLOOD_DIGITAL_FLOODED );
    st_flood_an = analogRead( FLOOD_ANALOGINPUT );
    if(( st_flooded != prev_flood || st_flood_an != prev_floodan ) && !flood_max_timer.isStarted()){
        changed = true;
        digitalWrite( FLOOD_LED, st_flooded ? HIGH : LOW );
    }
    return( changed );
}

void sendFlood( void )
{
    send( msgFlood.set( st_flooded ));
    send( msgFloodAn.set( st_flood_an ));

    flood_max_timer.start();
    flood_unchanged_timer.restart();

#ifdef DEBUG_ENABLED
    Serial.print( F( "[sendFlood] analogValue=" ));
    Serial.print( st_flood_an );
    Serial.print( F( ", digitalValue=" ));
    Serial.print( st_flooded ? "High":"Low" );
    Serial.print( F( " => flooded=" ));
    Serial.println( st_flooded ? "True":"False" );

    uint8_t range = map( st_flood_an, FLOOD_ANALOG_MIN, FLOOD_ANALOG_MAX, 0, 3 );
    Serial.print( F( "[sendFlood] mapped=" ));
    Serial.print( range );
    switch( range ){
        case 0:    // Sensor getting wet
            Serial.println( F( " => Flood" ));
            break;
        case 1:    // Sensor getting wet
            Serial.println( F( " => Raining" ));
            break;
        case 2:    // Sensor dry
            Serial.println( F( " => Wet" ));
            break;
        case 3:    // Sensor dry
            Serial.println( F( " => Dry" ));
            break;
    }
#endif
}

void onFloodUnchangedCb( void *empty )
{
    sendFlood();
}

/*
 * Temperature/Humidity DHT22/AM2302 sensors
 * As there is only one sensor for the two measures, we take the two measures
 * together, and send the two messages together also.
 */
#include <DHT.h>  
DHT dht;

// AM2302 DAT digital output
#define TEMPHUM_DIGITALINPUT    (3)

float st_temp = 0;
float st_temp_prev = 0;
float st_hum = 0;
float st_hum_prev = 0;

// message type
MyMessage msgHum(  CHILD_ID_HUMIDITY,    V_HUM );
MyMessage msgTemp( CHILD_ID_TEMPERATURE, V_TEMP );

/* temperature and humidity measures actually return the last value
 * cached by the library; the library takes care itself of respecting
 * the minimal sampling period
 */
void readTemp( void )
{
      // Get temperature from DHT library
      float temp = dht.getTemperature();
      if( isnan( temp )){
          Serial.println( F( "[readTemp] failed reading temperature from DHT" ));
      } else {
          st_temp = temp;
      }
}

void readHum( void )
{
      // Get temperature from DHT library
      float hum = dht.getHumidity();
      if( isnan( hum )){
          Serial.println( F( "[readTemp] failed reading humidity from DHT" ));
      } else {
          st_hum = hum;
      }
}

/* This is triggered by the max_frequency timer
 *  Only send the measures if they have changed
 */
void sendTempHumCb( void *empty )
{
    sendTempHum( false );
}

void onTempHumUnchangedCb( void *empty )
{
    sendTempHum( true );
}

void sendTempHum( bool force )
{
    if( force || st_temp != st_temp_prev ){
        send( msgTemp.set( st_temp, 1 ));
        st_temp_prev = st_temp;
        temp_unchanged_timer.restart();
    }
    if( force || st_hum != st_hum_prev ){
        send( msgHum.set( st_hum, 1 ));
        st_hum_prev = st_hum;
        hum_unchanged_timer.restart();
    }
    temphum_max_timer.restart();

#ifdef DEBUG_ENABLED
    Serial.print( F( "[sendTempHum] temp=" ));
    Serial.print( st_temp );
    Serial.print( F( "°C, hum=" ));
    Serial.print( st_hum );
    Serial.println( F( "%" ));
#endif
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

// message type
MyMessage msgDoor( CHILD_ID_DOOR, V_TRIPPED );

bool st_opened = false;

bool readDoor( void  )
{
    bool prev_opened = st_opened;
    bool changed = false;
    st_opened = ( digitalRead( OPENING_INPUT ) == HIGH );
    if( st_opened != prev_opened ){
        changed = true;
        digitalWrite( OPENING_LED, st_opened ? HIGH : LOW );
#ifdef DEBUG_ENABLED
        Serial.print( F( "[readDoor] opened=" ));
        Serial.println( st_opened ? "True":"False" );
#endif
    }
    return( changed );
}

void sendDoor( void )
{
    send( msgDoor.set( st_opened ));
    door_unchanged_timer.restart();
    
#ifdef DEBUG_ENABLED
    Serial.print( F( "[sendDoor] " ));
    Serial.println( st_opened ? F( "door is opened" ) : F( "door is closed" ));
#endif
}

void onDoorUnchangedCb( void *empty )
{
    sendDoor();
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
    present( CHILD_MAIN, S_CUSTOM );
    present( CHILD_ID_FLOOD,       S_WATER_LEAK, "Flood detection" );
    present( CHILD_ID_FLOOD_AN,    S_CUSTOM,     "Analog flood detection" );
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

    // flood detection
    digitalWrite( FLOOD_LED, LOW );
    pinMode( FLOOD_LED, OUTPUT );
    
    // temperature/humidity
    dht.setup( TEMPHUM_DIGITALINPUT, DHT::AM2302 ); 
    
    // opening door
    pinMode( OPENING_INPUT, INPUT );
    digitalWrite( OPENING_LED, LOW );
    pinMode( OPENING_LED, OUTPUT );

    main_timer.start( "MainTimer", st_main_timeout, false, MainLoopCb, NULL );
    main_timer.setDebug( false );
    
    flood_max_timer.set( "AnalogFloodMaxFrequencyTimer", st_max_frequency_timeout, true, NULL, NULL );
    temphum_max_timer.start( "TempmHumMaxFrequencyTimer", st_max_frequency_timeout, false, sendTempHumCb, NULL );

    // unchanged_timer's have 'once=true' because they are restarted when sending a message anyway
    flood_unchanged_timer.start( "FloodUnchangedTimer", st_unchanged_timeout, true, onFloodUnchangedCb, NULL );
    temp_unchanged_timer.start( "TemperatureUnchangedTimer", st_unchanged_timeout, true, onTempHumUnchangedCb, NULL );
    hum_unchanged_timer.start( "HumidityUnchangedTimer", st_unchanged_timeout, true, onTempHumUnchangedCb, NULL );
    door_unchanged_timer.start( "DoorUnchangedTimer", st_unchanged_timeout, true, onDoorUnchangedCb, NULL );
    
    pwiTimer::Dump();
}

void loop()
{
    pwiTimer::Loop();
}

/* **************************************************************************************
 *  MainLoopCb
 *  Is called on each timeout of the main_timer (setup to 250ms)
 *  Take the measure, but do not send
 */
void MainLoopCb( void *empty )
{
    // send measures on max_frequency timeout only
    readTemp();
    readHum();
    
    // send alerts as soon as they are detected
    if( readFlood()){
        sendFlood();
    }
    if( readDoor()){
        sendDoor();
    }
}

/* **************************************************************************************
 *  receive
 */
void receive(const MyMessage &message)
{
    char payload[2*MAX_PAYLOAD+1];
    uint8_t cmd = message.getCommand();
    uint8_t req;

    memset( payload, '\0', sizeof( payload ));
    message.getString( payload );
#ifdef HAVE_DEBUG
    Serial.print( F( "[receive] payload='" )); Serial.print( payload ); Serial.println( "'" ); 
#endif

    if( message.sensor == CHILD_MAIN ){
        switch( cmd ){
            case C_SET:
                req = strlen( payload ) > 0 ? atoi( payload ) : 0;
                switch( req ){
                    case 1:
                        st_max_frequency_timeout = strlen( payload ) > 2 ? atol( payload+2 ) : 0;
                        temphum_max_timer.set( st_max_frequency_timeout );
                        break;
                    case 2:
                        st_unchanged_timeout = strlen( payload ) > 2 ? atol( payload+2 ) : 0;
                        flood_unchanged_timer.set( st_unchanged_timeout );
                        temp_unchanged_timer.set( st_unchanged_timeout );
                        hum_unchanged_timer.set( st_unchanged_timeout );
                        door_unchanged_timer.set( st_unchanged_timeout );
                        break;
                }
                break;
            case C_REQ:
                req = strlen( payload ) > 0 ? atoi( payload ) : 0;
                switch( req ){
                    case 1:
                        dumpConfiguration();
                        break;
                }
                break;
        }
    }
}

void dumpConfiguration( void )
{
    send( msgVar1.setSensor( CHILD_MAIN ).set( st_max_frequency_timeout ));
    send( msgVar2.setSensor( CHILD_MAIN ).set( st_unchanged_timeout ));
    sendFlood();
    sendTempHum( true );
    sendDoor();
}

