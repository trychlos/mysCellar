#ifndef __PWI_SENSOR_H__
#define __PWI_SENSOR_H__

#include <Arduino.h>
#include "pwi_timer.h"

/*
 * A base, but usable, class for any measurement sensor.
 *
 * A measurement sensor defines :
 * - the max frequency at which send the changes of the measure through the
 *   'min_period_ms' timer
 * - the min frequency at which even an unchanged measure must be sent through
 *   the 'max_period_ms' timer
 *   this timer is started at each change of the measure
 * - appropriate callbacks to :
 *   > take the measure
 *   > send the measure.
 *
 * pwi 2019- 5-18 v1 creation
 * pwi 2019- 5-18 v2 the sensor can be armed/unarmed
 */

typedef void ( *pwiSendCb )( void * );
typedef bool ( *pwiMeasureCb )( void * );

class pwiSensor {
    public:
                      pwiSensor();
		    uint8_t       getId();
        bool          isArmed();
        void          present( uint8_t id, uint8_t type, const char *label );
        void          setArmed( bool armed );
        void          setArmed( const char *payload );
        void          setMaxPeriod( unsigned long delay_ms );
        void          setMinPeriod( unsigned long delay_ms );
        void          setSendOnChange( bool autosend );
        void          setup( unsigned long max_period_ms, unsigned long min_period_ms, pwiMeasureCb measureCb, pwiSendCb sendCb, void *user_data=NULL );

    private:
		    uint8_t       id;
		    uint8_t       type;
        const char   *label;
        bool          armed;
        unsigned long max_period_ms;
		    pwiTimer      max_timer;
        unsigned long min_period_ms;
    		pwiTimer      min_timer;
    		pwiSendCb     sendCb;
    		pwiMeasureCb  measureCb;
        void         *user_data;
        bool          send_on_change;

        static void   onMaxPeriodCb( pwiSensor *node);
        static void   onMinPeriodCb( pwiSensor *node);
};

#endif // __PWI_SENSOR_H__

