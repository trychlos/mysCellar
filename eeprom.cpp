#include "eeprom.h"

/* **************************************************************************************
 *  EEPROM management
 *  
 * pwi 2019- 6- 1 v1 creation
 * pwi 2019- 9- 6 rename structure members
 */

// uncomment for debugging eeprom functions
#define EEPROM_DEBUG

/**
 * eepromDump:
 */
void eepromDump( sEeprom &data )
{
#ifdef EEPROM_DEBUG
    Serial.print( F( "[eepromDump] mark='" ));               Serial.print( data.mark ); Serial.println( F( "'" ));
    Serial.print( F( "[eepromDump] version=" ));             Serial.println( data.version );
    Serial.print( F( "[eepromDump] flood_armed=" ));         Serial.println( data.flood_armed ? "True" : "False" );
    Serial.print( F( "[eepromDump] flood_min_period=" ));    Serial.println( data.flood_min_period );
    Serial.print( F( "[eepromDump] flood_max_period=" ));    Serial.println( data.flood_max_period );
    Serial.print( F( "[eepromDump] flood_grace_delay=" ));   Serial.println( data.flood_grace_delay );
    Serial.print( F( "[eepromDump] flood_advert_period=" )); Serial.println( data.flood_advert_period );
    Serial.print( F( "[eepromDump] rain_min_period=" ));     Serial.println( data.rain_min_period );
    Serial.print( F( "[eepromDump] rain_max_period=" ));     Serial.println( data.rain_max_period );
    Serial.print( F( "[eepromDump] temp_min_period=" ));     Serial.println( data.temp_min_period );
    Serial.print( F( "[eepromDump] temp_max_period=" ));     Serial.println( data.temp_max_period );
    Serial.print( F( "[eepromDump] hum_min_period=" ));      Serial.println( data.hum_min_period );
    Serial.print( F( "[eepromDump] hum_max_period=" ));      Serial.println( data.hum_max_period );
    Serial.print( F( "[eepromDump] door_armed=" ));          Serial.println( data.door_armed ? "True" : "False" );
    Serial.print( F( "[eepromDump] door_min_period=" ));     Serial.println( data.door_min_period );
    Serial.print( F( "[eepromDump] door_max_period=" ));     Serial.println( data.door_max_period );
    Serial.print( F( "[eepromDump] door_grace_delay=" ));    Serial.println( data.door_grace_delay );
    Serial.print( F( "[eepromDump] door_advert_period=" ));  Serial.println( data.door_advert_period );
    Serial.print( F( "[eepromDump] auto_dump_timeout=" ));   Serial.println( data.auto_dump_timeout );
#endif
}

/**
 * eepromRead:
 */
void eepromRead( sEeprom &data, pEepromRead pfnRead, pEepromWrite pfnWrite )
{
    for( uint8_t i=0 ; i<sizeof( sEeprom ); ++i ){
        (( uint8_t * ) &data )[i] = pfnRead( i );
    }
    // initialize with default values if mark not found
    if( data.mark[0] != 'P' || data.mark[1] != 'W' || data.mark[2] != 'I' || data.mark[3] != 0 ){
        eepromReset( data, pfnWrite );
    }
}

/**
 * eepromReset:
 */
void eepromReset( sEeprom &data, pEepromWrite pfnWrite )
{
    unsigned long def_min_period = 120000;        // 2 mn
    unsigned long def_max_period = 3600000;           // 1 h
#ifdef EEPROM_DEBUG
    Serial.println( F( "[eepromReset]" ));
#endif
    memset( &data, '\0', sizeof( sEeprom ));
    strcpy( data.mark, "PWI" );
    data.version = EEPROM_VERSION;
  
    data.flood_armed = true;
    data.flood_min_period = def_min_period;
    data.flood_max_period = def_max_period;
    data.flood_grace_delay = 0;
    data.flood_advert_period = 0;
  
    data.rain_min_period = def_min_period;
    data.rain_max_period = def_max_period;
  
    data.temp_min_period = def_min_period;
    data.temp_max_period = def_max_period;
  
    data.hum_min_period = def_min_period;
    data.hum_max_period = def_max_period;
  
    data.door_armed = true;
    data.door_min_period = 200;
    data.door_max_period = def_max_period;
    data.door_grace_delay = 0;
    data.door_advert_period = 0;
  
    data.auto_dump_timeout = 86400000;              // 24h
  
    eepromWrite( data, pfnWrite );
}

/**
 * eepromWrite:
 */
void eepromWrite( sEeprom &data, pEepromWrite pfnWrite )
{
#ifdef EEPROM_DEBUG
    Serial.println( F( "[eepromWrite]" ));
#endif
    for( uint8_t i=0 ; i<sizeof( sEeprom ); ++i ){
        pfnWrite( i, (( uint8_t * ) &data )[i] );
    }
}

