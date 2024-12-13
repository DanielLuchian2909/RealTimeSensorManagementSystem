/**
 ********************************************************************************
 * @file        EnvironmentalSensor.h
 * @author      Daniel Luchian
 * @brief       Header file for the ClassName class
 ********************************************************************************
 */

#ifndef __ENVIRONMENTALSENSOR_H__
#define __ENVIRONMENTALSENSOR_H__

/************************************
 * INCLUDES
 ************************************/
extern "C"
{
	#include "bme280.h"
}

/************************************
 * MACROS AND DEFINES
 ************************************/

/************************************
 * TYPEDEFS
 ************************************/

/************************************
 * EXPORTED VARIABLES
 ************************************/

/************************************
 * GLOBAL FUNCTION PROTOTYPES
 ************************************/

/************************************
 * CLASS DECLARATION
 ************************************/
class EnvironmentalSensor {
public:
	EnvironmentalSensor();
	virtual ~EnvironmentalSensor();
};

#endif // __ENVIRONMENTALSENSOR_H__
