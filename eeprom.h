#ifndef __EEPROM_H__
#define __EEPROM_H__

#include <Arduino.h>

/* **************************************************************************************
 *  EEPROM description
 *  This is the data structure saved in the EEPROM.
 * 
 *  MySensors leave us with 256 bytes to save configuration data in the EEPROM.
 *  - the mark takes 4 bytes
 *  - 252/4 = 63 bytes per module
 * 
 * pwi 2019- 6- 1 v1 creation
 * pwi 2019- 9- 6 rename structure members
 */
#define EEPROM_VERSION  1

typedef uint8_t pEepromRead( uint8_t );
typedef void    pEepromWrite( uint8_t, uint8_t );

typedef struct {
    /* a 'PWI' null-terminated string which marks the structure as initialized */
    char          mark[4];
    uint8_t       version;      // 1
    /* flood detection
       also keep here the permanent status of flood alarm armed (not modified on tripped)
       also keep the last tripped status (in case of a reset) */
    uint8_t       flood_armed;
    unsigned long flood_min_period;
    unsigned long flood_max_period;
    unsigned long flood_grace_delay;
    unsigned long flood_advert_period;
    /* flood measure */
    unsigned long rain_min_period;
    unsigned long rain_max_period;
    /* temperature measure */
    unsigned long temp_min_period;
    unsigned long temp_max_period;
    /* humidity measure */
    unsigned long hum_min_period;
    unsigned long hum_max_period;
    /* door opening detection */
    uint8_t       door_armed;
    unsigned long door_min_period;
    unsigned long door_max_period;
    unsigned long door_grace_delay;
    unsigned long door_advert_period;
    /* autosend the full configuration */
    unsigned long auto_dump_timeout;
}
 sEeprom;

void eepromDump( sEeprom &data );
void eepromRead( sEeprom &data, pEepromRead pfnRead, pEepromWrite pfnWrite );
void eepromReset( sEeprom &data, pEepromWrite pfn );
void eepromWrite( sEeprom &data, pEepromWrite pfn );

#endif // __EEPROM_H__

