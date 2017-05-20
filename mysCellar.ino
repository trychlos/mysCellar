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
 * pwi 2017- 5-15 review flood subsystem
 */

// uncomment for debugging this sketch
#define DEBUG_ENABLED

// uncomment for debugging EEPROM read/writes
#define EEPROM_DEBUG

static const char * const thisSketchName    = "mysCellar";
static const char * const thisSketchVersion = "6.0.2017";

/*
 * The current configuration
 * Read from/Written to the EEPROM
 */
typedef struct {
    /* a 'PWI' string with a null-terminating byte which marks the structure as initialized */
    char mark[4];
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

static unsigned long st_main_timeout = 250;
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
    CHILD_MAIN = 0,
    CHILD_ID_FLOOD = 10,
    CHILD_ID_TEMPERATURE,
    CHILD_ID_HUMIDITY,
    CHILD_ID_DOOR,
    CHILD_ID_RAIN,
};

MyMessage msg;

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

#define FLOOD_TRIPPED_LED       (5)
#define FLOOD_ARMED_LED         (6)

bool st_flood_armed = false;                   // flood armed
bool st_flood_tripped = false;                 // flood detected
pwiTimer st_flood_unchanged_timer;
pwiTimer st_flood_rearm_timer;

int st_rain = 0;
pwiTimer st_rain_mf_timer;
pwiTimer st_rain_unchanged_timer;

/* Set the tripped status of the flood detection
 * Update the EEPROM accordingly.
 */
void floodSetTripped( bool tripped )
{
    st_flood_tripped = tripped;
    digitalWrite( FLOOD_TRIPPED_LED, st_flood_tripped ? HIGH:LOW );
}

/* Set the armed status of the flood detection
 *  - either because a flood has been detected, and only set a current armed status
 *  - or because the configuration has been requested to change, and then also set
 *    the permanent armed status in the eeprom.
 */
void floodSetArmed( bool armed, bool bPermanent=true )
{
    st_flood_armed = armed;
    
    digitalWrite( FLOOD_ARMED_LED, st_flood_armed ? HIGH:LOW );

    if( bPermanent ){
        floodSetTripped( false );
        eeprom.flood_armed = st_flood_armed;
        eeprom_write( eeprom );
        sendFloodArmed();
    }
}

void onFloodRearmCb( void *empty )
{
#ifdef DEBUG_ENABLED
    Serial.println( F( "[onFloodRearmCb]" ));
#endif
    floodSetArmed( true, false );
}

void sendFloodArmed( void )
{
    msg.clear();
    send( msg.setSensor( CHILD_ID_FLOOD ).setType( V_ARMED ).set( eeprom.flood_armed ));
}

/* Regarding the flood detection, we are only interested by the digital value
 * Sends a change as soon as the measure changes, but not more than 1/mn
 * 
 * Returns: %TRUE if a change must be sent
 */
bool readFlood( void )
{
    bool flooded = false;
    bool changed = false;

    if( st_flood_armed ){
        bool prev_flood = st_flood_tripped;
        flooded = ( digitalRead( FLOOD_DIGITALINPUT ) == FLOOD_DIGITAL_FLOODED );
        changed |= ( prev_flood != flooded );
    }
    if( changed ){
        floodSetTripped( flooded );
    }
    
    return( changed );
}

void sendFlood( void )
{
    msg.clear();
    send( msg.setSensor( CHILD_ID_FLOOD ).setType( V_TRIPPED ).set( st_flood_tripped ));
    st_flood_unchanged_timer.restart();

    // if flood sensor is tripped, then disable the alert
    // and restart the timer to rearm
    if( st_flood_tripped ){
        floodSetArmed( false, false );
        st_flood_rearm_timer.start();
    }

#ifdef DEBUG_ENABLED
    Serial.print( F( "[sendFlood] flood=" ));
    Serial.println( st_flood_tripped ? "True":"False" );
#endif
}

void onFloodUnchangedCb( void *empty )
{
    sendFlood();
}

/* Dealing with analog part of the rain sensor
 * 
 * Returns: %TRUE if a change must be sent
 */
bool readRain( void )
{
    bool changed = false;

    if( !st_rain_mf_timer.isStarted()){
        int prev_rain = st_rain;
        st_rain = analogRead( FLOOD_ANALOGINPUT );
        changed |= ( prev_rain != st_rain );
    }

    return( changed );
}

void sendRain( void )
{
    msg.clear();
    send( msg.setSensor( CHILD_ID_RAIN ).setType( V_RAIN ).set( st_rain ));
    st_rain_mf_timer.restart();
    st_rain_unchanged_timer.restart();

#ifdef DEBUG_ENABLED
    Serial.print( F( "[sendRain] analogValue=" ));
    Serial.print( st_rain );
    uint8_t range = map( st_rain, FLOOD_ANALOG_MIN, FLOOD_ANALOG_MAX, 0, 3 );
    Serial.print( F( ", mapped=" ));
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

void onRainUnchangedCb( void *empty )
{
    sendRain();
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

pwiTimer st_temp_mf_timer;
pwiTimer st_temp_unchanged_timer;
float st_temp = 0;

pwiTimer st_hum_mf_timer;
pwiTimer st_hum_unchanged_timer;
float st_hum = 0;

/* temperature and humidity measures actually return the last value
 * cached by the library; the library takes care itself of respecting
 * the minimal sampling period
 * 
 * Returns: %TRUE if changed.
 */
bool readTemp( void )
{
    bool changed = false;

    if( !st_temp_mf_timer.isStarted()){
        float prev_temp = st_temp;
        // Get temperature from DHT library
        float temp = dht.getTemperature();
        if( isnan( temp )){
            Serial.println( F( "[readTemp] failed reading temperature from DHT" ));
        } else {
            st_temp = temp;
            changed |= ( prev_temp != st_temp );
        }
    }

    return( changed );
}

void sendTemp( void )
{
    msg.clear();
    send( msg.setSensor( CHILD_ID_TEMPERATURE ).setType( V_TEMP ).set( st_temp, 1 ));
    st_temp_mf_timer.restart();
    st_temp_unchanged_timer.restart();

#ifdef DEBUG_ENABLED
    Serial.print( F( "[sendTemp] temp=" ));
    Serial.print( st_temp );
    Serial.println( F( "°C" ));
#endif
}

void onTempUnchangedCb( void *empty )
{
    sendTemp();
}

bool readHum( void )
{
    bool changed = false;

    if( !st_hum_mf_timer.isStarted()){
        float prev_hum = st_hum;
        // Get temperature from DHT library
        float hum = dht.getHumidity();
        if( isnan( hum )){
            Serial.println( F( "[readTemp] failed reading humidity from DHT" ));
        } else {
            st_hum = hum;
            changed |= ( prev_hum != st_hum );
        }
    }

    return( changed );
}

void sendHum( void )
{
    msg.clear();
    send( msg.setSensor( CHILD_ID_HUMIDITY ).setType( V_HUM ).set( st_hum, 1 ));
    st_hum_mf_timer.restart();
    st_hum_unchanged_timer.restart();

#ifdef DEBUG_ENABLED
    Serial.print( F( "[sendHum] hum=" ));
    Serial.print( st_hum );
    Serial.println( F( "%" ));
#endif
}

void onHumUnchangedCb( void *empty )
{
    sendHum();
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

pwiTimer st_door_unchanged_timer;
bool st_opened = false;

bool readDoor( void  )
{
    bool changed = false;
    bool prev_opened = st_opened;
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
    msg.clear();
    send( msg.setSensor( CHILD_ID_DOOR ).setType( V_TRIPPED ).set( st_opened ));
    st_door_unchanged_timer.restart();
#ifdef DEBUG_ENABLED
    Serial.print( F( "[sendDoor] opened=" ));
    Serial.println( st_opened ? "True":"False" );
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
    present( CHILD_ID_RAIN,        S_RAIN,       "Rain sensor" );
    present( CHILD_ID_TEMPERATURE, S_TEMP,       "Temperature measure" );
    present( CHILD_ID_HUMIDITY,    S_HUM,        "Humidity measure" );
    present( CHILD_ID_DOOR,        S_DOOR,       "Door opening detection" );
}

void setup()  
{
    static pwiTimer main_timer;
    
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

    st_flood_unchanged_timer.set( "RainFloodUnchangedTimer", eeprom.unchanged_timeout, false, onFloodUnchangedCb );
    st_flood_rearm_timer.set( "RainFloodRearmTimer", eeprom.flood_rearm_delay, true, onFloodRearmCb );
    floodSetArmed( eeprom.flood_armed, false );

    st_rain_mf_timer.set( "RainRateMaxFrequencyTimer", eeprom.max_frequency_timeout, true );
    st_rain_unchanged_timer.set( "RainRateUnchangedTimer", eeprom.unchanged_timeout, false, onRainUnchangedCb );

    // temperature/humidity
    dht.setup( TEMPHUM_DIGITALINPUT, DHT::AM2302 ); 

    st_temp_mf_timer.set( "TempMaxFrequencyTimer", eeprom.max_frequency_timeout, true );
    st_temp_unchanged_timer.set( "TempUnchangedTimer", eeprom.unchanged_timeout, false, onTempUnchangedCb );

    st_hum_mf_timer.set( "HumidityMaxFrequencyTimer", eeprom.max_frequency_timeout, true );
    st_hum_unchanged_timer.set( "HumidityUnchangedTimer", eeprom.unchanged_timeout, false, onHumUnchangedCb );

    // opening door
    pinMode( OPENING_INPUT, INPUT );
    digitalWrite( OPENING_LED, LOW );
    pinMode( OPENING_LED, OUTPUT );

    st_door_unchanged_timer.set( "DoorUnchangedTimer", eeprom.unchanged_timeout, false, onDoorUnchangedCb );

    // main timer
    main_timer.start( "MainTimer", st_main_timeout, false, MainLoopCb, NULL, false );
    
    pwiTimer::Dump();
}

void loop()
{
    pwiTimer::Loop();
}

/* **************************************************************************************
 *  MainLoopCb
 *  Is called on each timeout of the main_timer (setup to 250ms)
 *  Take the measure, and send if changed
 *  - each measure has its own 'max_frequency' timer which prevents it to be sent too often
 *  - alarms are sent as soon as they are detected.
 */
void MainLoopCb( void *empty )
{
    if( readRain()){
        sendRain();
    }
    if( readTemp()){
        sendTemp();
    }
    if( readHum()){
        sendHum();
    }
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
    int entier;
    unsigned long ulong;

    memset( payload, '\0', sizeof( payload ));
    message.getString( payload );
#ifdef HAVE_DEBUG
    Serial.print( F( "[receive] payload='" )); Serial.print( payload ); Serial.println( "'" ); 
#endif

    switch( message.sensor ){
        case CHILD_MAIN:
            switch( message.type ){
                case V_CUSTOM:
                    switch( cmd ){
                        case C_SET:
                            req = strlen( payload ) > 0 ? atoi( payload ) : 0;
                            switch( req ){
                                case 1:
                                    ulong = strlen( payload ) > 2 ? atol( payload+2 ) : 0;
                                    if( ulong > 0 ){
                                        eeprom.max_frequency_timeout = ulong;
                                        eeprom_write( eeprom );
                                        st_rain_mf_timer.setDelay( ulong );
                                        st_temp_mf_timer.setDelay( ulong );
                                        st_hum_mf_timer.setDelay( ulong );
                                    }
                                    break;
                                case 2:
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
                                case 3:
                                    eeprom_reset( eeprom );
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
                    break;
            }
            break;
            
        case CHILD_ID_FLOOD:
            switch( message.type ){
                case V_ARMED:
                    switch( cmd ){
                        case C_SET:
                            entier = strlen( payload ) > 0 ? atoi( payload ) : 0;
                            floodSetArmed( entier > 0 );
                            break;
                    }
                    break;
                case V_CUSTOM:
                    switch( cmd ){
                        case C_SET:
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
                    break;
            }
            break;
    }
}

void dumpConfiguration( void )
{
    msg.clear();
    send( msg.setSensor( CHILD_MAIN ).setType( V_VAR1 ).set( eeprom.max_frequency_timeout ));
    msg.clear();
    send( msg.setSensor( CHILD_MAIN ).setType( V_VAR2 ).set( eeprom.unchanged_timeout ));
    sendFloodArmed();
    if( eeprom.flood_armed ){
        sendFlood();
    }
    msg.clear();
    send( msg.setSensor( CHILD_ID_FLOOD ).setType( V_VAR1 ).set( eeprom.flood_rearm_delay ));
    msg.clear();
    send( msg.setSensor( CHILD_ID_FLOOD ).setType( V_VAR2 ).set( st_flood_armed ));
    sendRain();
    sendTemp();
    sendHum();
    sendDoor();
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

