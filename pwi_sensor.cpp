#include "pwi_sensor.h"
/*
 * pwi 2019- 5-18 v1 creation
 * pwi 2019- 5-19 v2 add debug messages
 *                   fix MySensors present() scope
 * pwi 2019- 5-20 v3 stop the timers on unarming
 */

#include <core/MySensorsCore.h>

// uncomment to debugging this file
#define SENSOR_DEBUG

/**
 * pwiSensor::pwiSensor:
 * 
 * Constructor.
 *
 * Public
 */
pwiSensor::pwiSensor( void )
{
    this->id = 0;
    this->type = 0;
    this->label = NULL;
    this->armed = true;
    this->min_period_ms = 0;
    this->max_period_ms = 0;
	  this->sendCb = NULL;
  	this->measureCb = NULL;
    this->send_on_change = true;
}

/**
 * pwiSensor::getId:
 * 
 * Returns: the node identifier.
 *
 * Public
 */
uint8_t pwiSensor::getId()
{
    return( this->id );
}

/**
 * pwiSensor::isArmed:
 * 
 * Returns: %TRUE if the sensor is armed.
 * 
 * If the sensor is not armed, no measure is taken.
 * The messages sent are so ARMED=false, TRIPPED=false.
 *
 * Public
 */
bool pwiSensor::isArmed()
{
    return( this->armed );
}

/**
 * pwiSensor::present:
 *
 * Present the node to the controller.
 *
 * Public
 */
void pwiSensor::present( uint8_t id, uint8_t type, const char *label )
{
#ifdef SENSOR_DEBUG
    Serial.print( F( "[pwiSensor::present] id=" ));
    Serial.print( id );
    Serial.print( F( ", type=" ));
    Serial.print( type );
    Serial.print( F( ", label=" ));
    Serial.println( label );
#endif
	  this->id = id;
	  this->type = type;
	  this->label = label;
	  ::present( this->id, this->type, this->label );
}

/**
 * pwiSensor::setArmed:
 * 
 * Arm/unarm the sensor.
 *
 * Public
 */
void pwiSensor::setArmed( bool armed )
{
    this->armed = armed;
    if( !armed ){
        this->min_timer.stop();
        this->max_timer.stop();
    }
}

/**
 * pwiSensor::setArmed:
 *
 * Arm the sensor depending of a received message which should be 'ARM=1|0'
 *
 * Public
 */
void pwiSensor::setArmed( const char *payload )
{
    if( !strncmp( payload, "ARM=", 4 )){
        bool armed = ( bool )( atoi( payload+4 ));
        this->setArmed( armed );
    }
}

/**
 * pwiSensor::setMaxPeriod:
 * 
 * Position the min frequency at which the measure has to be sent.
 * This corresponds to a sort of heartbeat for the sensor.
 *
 * Public
 */
void pwiSensor::setMaxPeriod( unsigned long delay_ms )
{
    if( delay_ms > this->min_period_ms ){
		    this->max_period_ms = delay_ms;
    		this->max_timer.setDelay( delay_ms );
    		if( this->max_timer.isStarted()){
    			  this->max_timer.restart();
    		}
  	}
}

/**
 * pwiSensor::setMinPeriod:
 * 
 * Position the max frequency at which the measure has to be made.
 * If zero, then stop the timer.
 *
 * Public
 */
void pwiSensor::setMinPeriod( unsigned long delay_ms )
{
    if( delay_ms < this->max_period_ms ){
        this->min_period_ms = delay_ms;
        this->min_timer.setDelay( delay_ms );
        if( this->min_period_ms == 0 ){
            this->min_timer.stop();
        } else if( this->min_timer.isStarted()){
            this->min_timer.restart();
        }
    }
}

/**
 * pwiSensor::setSendOnChange:
 * 
 * Whether a message should be sent to the controller each time the measure changes.
 * This is the default behavior for all our sensor nodes.
 * But, this should not be the case for alarm nodes, where the message must only be
 * send after the expiration of the grace period.
 *
 * Public
 */
void pwiSensor::setSendOnChange( bool autosend )
{
    this->send_on_change = autosend;
}

/**
 * pwiSensor::setup:
 * 
 * Public
 */
void pwiSensor::setup( unsigned long max_period_ms, unsigned long min_period_ms, pwiMeasureCb measureCb, pwiSendCb sendCb, void *user_data )
{
#ifdef SENSOR_DEBUG
    Serial.print( F( "[pwiSensor::setup] id=" ));
    Serial.print( this->id );
    Serial.print( F( ", min_period_ms=" ));
    Serial.print( min_period_ms );
    Serial.print( F( ", max_period_ms=" ));
    Serial.println( max_period_ms );
#endif
    this->measureCb = measureCb;
    this->sendCb = sendCb;
    this->user_data = user_data;

    max_timer.set( "MaxPeriodTimer", max_period_ms, true, pwiSensor::onMaxPeriodCb, this );
    this->setMaxPeriod( max_period_ms );

    min_timer.set( "MinPeriodTimer", min_period_ms, false, pwiSensor::onMinPeriodCb, this );
    this->setMinPeriod( min_period_ms );
    min_timer.start();
}

/**
 * pwiSensor::onMaxPeriodCb:
 * 
 * Callback to handle the maximum period (the heartbeat).
 * Refresh the measure before sending the result.
 * 
 * Private Static
 */
void pwiSensor::onMaxPeriodCb( pwiSensor *node )
{
#ifdef SENSOR_DEBUG
    Serial.print( F( "[pwiSensor::onMaxPeriodCb] node_id=" ));
    Serial.println( node->getId());
#endif
    if( node->armed ){
        if( node->sendCb ){
            node->sendCb( node->user_data );
            node->max_timer.restart();
        }
    }
}

/**
 * pwiSensor::onMinPeriodCb:
 * 
 * Callback to handle the minimum period (the max frequency of the measure).
 * If the sensor is not armed, then no measure is taken.
 * 
 * Private Static
 */
void pwiSensor::onMinPeriodCb( pwiSensor *node )
{
#ifdef SENSOR_DEBUG
    Serial.print( F( "[pwiSensor::onMinPeriodCb] node_id=" ));
    Serial.println( node->getId());
#endif
    if( node->armed ){
        if( node->measureCb ){
            if( node->measureCb( node->user_data )){
                if( node->sendCb && node->send_on_change ){
                    node->sendCb( node->user_data );
                    node->max_timer.restart();
                }
            }
        }
    }
}

