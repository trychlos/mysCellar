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
 */
//#define MY_DEBUG
#define MY_REPEATER_FEATURE
#define MY_RADIO_NRF24
#define MY_RF24_CHANNEL 103
#define MY_SIGNING_SOFT
#define MY_SIGNING_SOFT_RANDOMSEED_PIN 7
#define MY_SOFT_HMAC_KEY 0xe5,0xc5,0x36,0xd8,0x4b,0x45,0x49,0x25,0xaa,0x54,0x3b,0xcc,0xf4,0xcb,0xbb,0xb7,0x77,0xa4,0x80,0xae,0x83,0x75,0x40,0xc2,0xb1,0xcb,0x72,0x50,0xaa,0x8a,0xf1,0x6d
#include <MySensors.h>

#include <DHT.h>  

/*
 * Four sensors here, each one having its own:
 * - minPeriod: the minimum period, i.e. the maximum frequency, i.e. the maximum
 *   messaging rate: does not send more than one measure per 'minPeriod'
 *   (does not flood the gateway)
 * - maxPeriod: the maximum period, i.e. the minimum frequency, i.e. the minimum
 *   messaging rate: at least send one measure every 'maxPeriod'
 *   (say "I'm alive").
 *
 * Measures are done at the maximum frequency. Results are sent if a change is
 * detected, or the minimum frequency is reached.
 *
 * As a consequence, the maximum reactivity of a sensor is influenced by the
 * minimum period: after a measure has been sent at 't', any change before
 * 't+minPeriod' will be silently ignored.
 * The best reactivity is reached when a change occurs before 't+maxPeriod'.
 */
enum {
    CHILD_ID_HUMIDITY = 0,
    CHILD_ID_TEMPERATURE,
    CHILD_ID_FLOOD,
    CHILD_ID_OPENING
};

/*
 * Flood detection
 */
// rain sensor analog output
#define FLOOD_ANALOGINPUT       (A7)
#define FLOOD_ANALOG_MIN        (0)            // flooded
#define FLOOD_ANALOG_MAX        (1024)         // dry
// rain sensor digital output
#define FLOOD_DIGITALINPUT      (4)
#define FLOOD_DIGITAL_FLOODED   (LOW)
#define FLOOD_DIGITAL_DRY       (HIGH)
// message type
MyMessage msgFlood( CHILD_ID_FLOOD, V_TRIPPED );

/*
 * Temperature/Humidity DHT22/AM2302 sensors
 * Definitions
 */
// AM2302 DAT digital output
#define TEMPHUM_DIGITALINPUT    (3)
// message type
MyMessage msgHum(  CHILD_ID_HUMIDITY,    V_HUM );
MyMessage msgTemp( CHILD_ID_TEMPERATURE, V_TEMP );
DHT dht;

/*
 * Opening door detection
 * We are using here the NO contact for the door opening detector:
 * - door closed : contact is closed
 * - door opened: contact is opened.
 */
#define OPENING_INPUT           (A0)           // use analog pin as a digital input
// message type
MyMessage msgOpening( CHILD_ID_OPENING, V_TRIPPED );

/*
 * MySensors gateway
 */
boolean metric = true;

// set to 1 to debug the read/send functions
static const int debug_read = 0;
static const int debug_send = 0;

/*
 * Code helpers...
 *
 * Note:
 * the Arduino compiler is not able to compile a declaration as:
 *     typedef struct _sDefines sDefines;
 *     struct _sDefines { ... };
 * and then use sDefines pointer as a variable type in function declaration.
 * We are so stucked to use 'void *' pointers.
 */
enum {
    TYPE_BOOLEAN = 0,
    TYPE_FLOAT
};

typedef struct {
    int           childId;
    unsigned long minPeriod;
    unsigned long maxPeriod;
    int           typeData;
    boolean       bValue;
    float         fValue;
    unsigned long lastSent;
    boolean      ( *fnRead )( void * );
    void         ( *fnSend )( void * );
}
    sDefines;

boolean read_float( void *p, float data, const char *msgerr )
{
    if( isnan( data )){
        Serial.println( msgerr );
        return( false );
    }
    boolean changed = false;
    sDefines *pDef = ( sDefines * ) p;
    if( data != pDef->fValue ){
        changed = true;
        pDef->fValue = data;
    }
    return( changed );
}

boolean read_boolean( void *p, boolean data )
{
    boolean changed = false;
    sDefines *pDef = ( sDefines * ) p;
    if( data != pDef->bValue ){
        changed = true;
        pDef->bValue = data;
    }
    return( changed );
}


void send_float( void *p, MyMessage &msg, const char *label )
{
    sDefines *pDef = ( sDefines * ) p;
    send( msg.set( pDef->fValue, 1 ));
    if( debug_send ){
        Serial.print( label );
        Serial.println( pDef->fValue );
    }
}

/*
 * temperature and humidity measures actually return the last value
 * cached by the library; the library takes care itself of respecting
 * the minimal sampling period
 */
boolean read_temperature( void *p )
{
    if( debug_read ){
        Serial.print( "read_temperature: " );
        Serial.println( millis());
    }
    return( read_float( p, dht.getTemperature(), PSTR( "Failed reading temperature from DHT" )));
}

void send_temperature( void *p )
{
    send_float( p, msgTemp, PSTR( "Temperature: " ));
}

boolean read_humidity( void *p )
{
    if( debug_read ){
        Serial.print( "read_humidity: " );
        Serial.println( millis());
    }
    return( read_float( p, dht.getHumidity(), PSTR( "Failed reading humidity from DHT" )));
}

void send_humidity( void *p )
{
    send_float( p, msgHum, PSTR( "Humidity: " ));
}
/*
 * regarding the flood detection, we are only interested by the digital value
 * analog value is only used during development for debugging help
 */
boolean read_flood( void *p )
{
    if( debug_read ){
        Serial.print( "read_flood: " );
        Serial.println( millis());
    }
    return( read_boolean( p, digitalRead( FLOOD_DIGITALINPUT ) == FLOOD_DIGITAL_FLOODED ));
}

void send_flood( void *p )
{
    sDefines *pDef = ( sDefines * ) p;
    int av = analogRead( FLOOD_ANALOGINPUT );
    int range = map( av, FLOOD_ANALOG_MIN, FLOOD_ANALOG_MAX, 0, 3 );
    send( msgFlood.set(( uint8_t ) pDef->bValue ));
    if( debug_send ){
        Serial.println( "RainSensor:" );
        Serial.print( "  analogValue=" );
        Serial.println( av );
        Serial.print( "  mapped=" );
        Serial.println( range );
        switch( range ){
            case 0:    // Sensor getting wet
                Serial.println("  => Flood");
                break;
            case 1:    // Sensor getting wet
                Serial.println("  => Rain Warning");
                break;
            case 2:    // Sensor dry - To shut this up delete the " Serial.println("Not Raining"); " below.
                Serial.println("  => Not Raining");
                break;
        }
        Serial.print( "  digitalValue=" );
        Serial.println( pDef->bValue ? "HIGH":"LOW" );
        Serial.print( "  => Flooded=" );
        Serial.println( pDef->bValue ? "True":"False" );
    }
}

/*
 * opening door
 * The sensor is provided by the Telemecanique door switch.
 * This is a double-relais platine, with a NO and a NC contacts.
 * The NC contact is used by origin for lights.
 * We are using here the NO contact for the door opening detector:
 * - door closed : contact is closed
 * - door opened: contact is opened.
 */
boolean read_opening( void *p )
{
    if( debug_read ){
        Serial.print( "read_opening: " );
        Serial.println( millis());
    }
    return( read_boolean( p, digitalRead( OPENING_INPUT ) == HIGH ));
}

void send_opening( void *p )
{
    sDefines *pDef = ( sDefines * ) p;
    send( msgOpening.set(( uint8_t ) pDef->bValue ));
    if( debug_send ){
        Serial.println( PSTR( "Opening detection:" ));
        Serial.println( pDef->bValue ? PSTR( "  Door is opened" ) : PSTR( "  Door is closed" ));
    }
}

sDefines aDefines[] = {
    { CHILD_ID_TEMPERATURE, 5000, 60000, TYPE_FLOAT,   false, 0, 0, read_temperature, send_temperature },
    { CHILD_ID_HUMIDITY,    5000, 60000, TYPE_FLOAT,   false, 0, 0, read_humidity,    send_humidity },
    { CHILD_ID_FLOOD,      10000, 60000, TYPE_BOOLEAN, false, 0, 0, read_flood,       send_flood },
    { CHILD_ID_OPENING,      500, 60000, TYPE_BOOLEAN, false, 0, 0, read_opening,     send_opening },
    { -1 },
};

void presentation()
{
    // Send the Sketch Version Information to the Gateway
    sendSketchInfo( "mysCellar", "4.2017" );

    // Register all sensors to gw (they will be created as child devices)
    present( CHILD_ID_HUMIDITY,    S_HUM,        "Humidity measure" );
    present( CHILD_ID_TEMPERATURE, S_TEMP,       "Temperature measure" );
    present( CHILD_ID_FLOOD,       S_WATER_LEAK, "Flood detection" );
    present( CHILD_ID_OPENING,     S_DOOR,       "Opening detection" );
}

void setup()  
{
    dht.setup( TEMPHUM_DIGITALINPUT, DHT::AM2302 ); 

    // useless in my own case
    //metric = gw.getConfig().isMetric;
    
    // setup an analog pin as a digital input
    pinMode( OPENING_INPUT, INPUT );
}

/*
 */
void loop()      
{
    unsigned long thisLoop = millis();
    unsigned long thisDelay;
    boolean changed;
    
    //Serial.print( "thisLoop=" );
    //Serial.println( thisLoop );
    for( int i=0 ; aDefines[i].childId >= 0 ; ++i ){
        thisDelay = thisLoop - aDefines[i].lastSent;
        if( thisDelay > aDefines[i].minPeriod ){
            changed = aDefines[i].fnRead( &aDefines[i] );
            if( changed || thisDelay > aDefines[i].maxPeriod ){
                aDefines[i].lastSent = thisLoop;
                aDefines[i].fnSend( &aDefines[i] );
            }
        }
    }

    wait( 100 );
}

