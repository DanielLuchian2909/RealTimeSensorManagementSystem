/**
 ********************************************************************************
 * @file        cinterface_env_sensor.cpp
 * @author      Daniel Luchian
 * @brief       CInterface for the EnvironmentalSensor class
 ********************************************************************************
 */

/************************************
 * INCLUDES
 ************************************/
#include "base_types.h"
#include "env_sensor.hpp"

/************************************
 * EXTERN VARIABLES
 ************************************/

/************************************
 * PRIVATE MACROS AND DEFINES
 ************************************/
#define CINTERFACE_BME280_ERROR 0

/************************************
 * PRIVATE TYPEDEFS
 ************************************/
//Structure containing a pointer to an environmental sensor class for C-interfacing
typedef struct EnvSensorHandle
{
	EnvironmentalSensor* pcMyEnvSensorClass;
} EnvSensorHandle;

/************************************
 * STATIC VARIABLES
 ************************************/

/************************************
 * STATIC FUNCTIONS
 ************************************/

/************************************
 * GLOBAL FUNCTIONS
 ************************************/
//---------------------------------------------------------------------
extern "C"
EnvSensorHandle* //Returns a pointer to an EnvSensorHandle
cInterfaceCreateEnvironmentalSensor() //C style function to create the sensor object
{
	//Creates a new EnvSensorHandle and Environmental Sensor Class then links them
    return new EnvSensorHandle{ new EnvironmentalSensor() };
}

//---------------------------------------------------------------------
extern "C"
void
cInterfaceDestroyEnvironmentalSensor( //C style function to destroy the senseor object and handle
		EnvSensorHandle* psEnvSensorHandle_) //A pointer to an environmental sensor handle
{
	if (psEnvSensorHandle_ && (psEnvSensorHandle_->pcMyEnvSensorClass) ) //If the sensor exists delete it and its handle
	{
		delete psEnvSensorHandle_->pcMyEnvSensorClass;
		delete psEnvSensorHandle_;
	}
}

//---------------------------------------------------------------------
extern "C"
void cInterfaceReadSensorData( //C style function to read the sensor data
		EnvSensorHandle* psEnvSensorHandle_, //A pointer to to an environmental sensor handle
		ReadMode eReadMode_, //The sensor read mode
		EnvSensorSelect eEnvSensorSelect_) //The sensor(s) to read
{
	//If the handle exists and it has a sensor, read the sensors data
	if (psEnvSensorHandle_ && (psEnvSensorHandle_->pcMyEnvSensorClass) )
	{
		psEnvSensorHandle_->pcMyEnvSensorClass->readSensorData(eReadMode_, eEnvSensorSelect_);
	}
}

//---------------------------------------------------------------------
extern "C"
BOOLE //Return true if the sensor was successfully resetted, false otherwise
cInterfaceSoftReset( //C style function to soft reset the sensor
		EnvSensorHandle* psEnvSensorHandle_) //A pointer to to an environmental sensor handle
{
	//If the handle exists and it has a sensor, soft reset the sensor
	if (psEnvSensorHandle_ && (psEnvSensorHandle_->pcMyEnvSensorClass) )
	{
		return (psEnvSensorHandle_->pcMyEnvSensorClass->softReset());
	}
	return FALSE;
}

//---------------------------------------------------------------------
extern "C"
BOOLE //Return true if the sensor was successfully set, false otherwise
cInterfaceSetSensorMode( //C style function to set the sensor mode
		EnvSensorHandle* psEnvSensorHandle_, //A pointer to to an environmental sensor handle
		UCHAR ucSensorMode_) //Sensor mode to set
{
	//If the handle exists and it has a sensor, set the sensor mode
	if (psEnvSensorHandle_ && (psEnvSensorHandle_->pcMyEnvSensorClass) )
	{
		return (psEnvSensorHandle_->pcMyEnvSensorClass->setSensorMode(ucSensorMode_));
	}
	return FALSE;
}

//---------------------------------------------------------------------
extern "C"
BOOLE //Return true if the sensor was successfully set, false otherwise
cInterfaceSetSensorSettings( //C style function to set the sensor mode
		EnvSensorHandle* psEnvSensorHandle_, //A pointer to to an environmental sensor handle
		void* psNewSensorSettings_, //Sensor settings to set
		UCHAR ucDesiredSettings_) //The desired settings to change
{
	//If the handle exists and it has a sensor, set the sensor settings
	if (psEnvSensorHandle_ && (psEnvSensorHandle_->pcMyEnvSensorClass) )
	{
		return (psEnvSensorHandle_->pcMyEnvSensorClass->setSensorSettings(psNewSensorSettings_, ucDesiredSettings_));
	}
	return FALSE;
}

//---------------------------------------------------------------------
extern "C"
const bme280_data* //A pointer to the data (immutable)
cInterfaceGetSensorData( //C style function to get the sensor data
		EnvSensorHandle* psEnvSensorHandle_) //A pointer to to an environmental sensor handle
{
	return (psEnvSensorHandle_->pcMyEnvSensorClass->getSensorData());
}

//---------------------------------------------------------------------
extern "C"
UCHAR //Returns the sensor chip ID
cInterfaceGetChipID( //C style getter for the sensor chip ID
		EnvSensorHandle* psEnvSensorHandle_) //A pointer to to an environmental sensor handle
{
	//If the handle exists and it has a sensor, get the chip id
	if (psEnvSensorHandle_ && (psEnvSensorHandle_->pcMyEnvSensorClass) )
	{
		return (psEnvSensorHandle_->pcMyEnvSensorClass->getChipID());
	}
	return CINTERFACE_BME280_ERROR;
}

//---------------------------------------------------------------------
extern "C"
UCHAR //Returns the sensor chip ID
cInterfaceGetSensorMode( //C style getter for the sensor mode
		EnvSensorHandle* psEnvSensorHandle_) //A pointer to to an environmental sensor handle
{
	//If the handle exists and it has a sensor, get the sensor mode
	if (psEnvSensorHandle_ && (psEnvSensorHandle_->pcMyEnvSensorClass) )
	{
		return (psEnvSensorHandle_->pcMyEnvSensorClass->getSensorMode());
	}
	return CINTERFACE_BME280_ERROR;
}

//---------------------------------------------------------------------
extern "C"
const  bme280_settings* //Return a constant pointer to a BME280 sensor settings structure
cInterfaceGetSensorSettings( //C style getter for the sensor settings
		EnvSensorHandle* psEnvSensorHandle_) //A pointer to to an environmental sensor handle
{
	//If the handle exists and it has a sensor, get the address of the sensor settings
	if (psEnvSensorHandle_ && (psEnvSensorHandle_->pcMyEnvSensorClass) )
	{
		return (psEnvSensorHandle_->pcMyEnvSensorClass->getSensorSettings());
	}
}

//---------------------------------------------------------------------
extern "C"
UCHAR //Returns the standby time
cInterfaceGetSensorStandbyTime( //C style getter for the sensor standby time
		EnvSensorHandle* psEnvSensorHandle_) //A pointer to to an environmental sensor handle
{
	//If the handle exists and it has a sensor, get the standby time
	if (psEnvSensorHandle_ && (psEnvSensorHandle_->pcMyEnvSensorClass) )
	{
		return (psEnvSensorHandle_->pcMyEnvSensorClass->getStandbyTime());
	}
	return CINTERFACE_BME280_ERROR; //TODO Better resolution for non-exisiting EnvSensor case
}

//---------------------------------------------------------------------
extern "C"
UINT //Returns the max delay between measurements
cInterfaceGetMaxDelay( //C style gtter for sensor maximum delay
		EnvSensorHandle* psEnvSensorHandle_) //A pointer to to an environmental sensor handle
{
	//If the handle exists and it has a sensor, get the maximum delay
	if (psEnvSensorHandle_ && (psEnvSensorHandle_->pcMyEnvSensorClass) )
	{
		return (psEnvSensorHandle_->pcMyEnvSensorClass->getMaxDelay());
	}
	return CINTERFACE_BME280_ERROR; //TODO Better resolution for non-exisiting EnvSensor case
}

/************************************
 * PUBLIC METHODS
 ************************************/

/************************************
 * PRIVATE HELPER FUNCTIONS
 ************************************/
