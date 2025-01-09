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
#include "baseTypes.h" //Basic type definitions and utilities

/************************************
 * MACROS AND DEFINES
 ************************************/

/************************************
 * TYPEDEFS
 ************************************/
typedef struct bme280_dev bme280_dev;           	  //Forward declaration of the bme280_dev structure
typedef struct bme280_data bme280_data;         	  //Forward declaration of the bme280_data structure
typedef struct EnvSensorHandle EnvSensorHandle; 	  //Forward declaration of the EnvSensorHandle structure
typedef struct bme280_settings bme280_settings; 	  //Forward declaration of the bme280_settings structure
typedef struct bme280_uncomp_data bme280_uncomp_data; //Forward declaration of the bme280_uncomp_data structure

//Enum for the different read modes
typedef enum
{
	READ_MODE_SINGLE,
	READ_MODE_POLLING,
	READ_MODE_CONTINUOUS,
	NUM_READ_MODES //Must be last
} ReadMode;

//Enum to select sensors(s) (values based off of bme280 library)
typedef enum
{
    ENVSENSOR_SELECT_TEMPERATURE = 1,
    ENVSENSOR_SELECT_HUMIDITY = 1 << 1,
    ENVSENSOR_SELECT_PRESSURE = 1 << 2,
    ENVSENSOR_SELECT_ALL = ENVSENSOR_SELECT_TEMPERATURE | ENVSENSOR_SELECT_HUMIDITY | ENVSENSOR_SELECT_PRESSURE
} EnvSensorSelect;
/************************************
 * EXPORTED VARIABLES
 ************************************/

/************************************
 * GLOBAL FUNCTION PROTOTYPES
 ************************************/
//Functions for the c-interface of the class
#ifdef __cplusplus
extern "C" {
#endif

EnvSensorHandle* cInterfaceCreateEnvironmentalSensor(); //Create a new sensor object
void cInterfaceDestroyEnvironmentalSensor(EnvSensorHandle* psEnvSensorHandle_); //Destroy the sensor object
void cInterfaceReadSensorData(EnvSensorHandle* psEnvSensorHandle_, ReadMode eReadMode_, EnvSensorSelect eEnvSensorSelect_); //Read the sensor data
const bme280_data* cInterfaceGetSensorData(EnvSensorHandle* psEnvSensorHandle_); //Get the sensor data

#ifdef __cplusplus
}
#endif

/************************************
 * CLASS DEFINITION
 ************************************/
#ifdef __cplusplus
class EnvironmentalSensor
{

public:

	//Constructor + Destructor
	EnvironmentalSensor();
	virtual ~EnvironmentalSensor();

	//Sensor Operation
	void readSensorData(ReadMode eReadMode_, EnvSensorSelect eEnvSensorSelect_);
	CHAR softReset();
	//CHAR compensateData(); //TODO Implement

	//Setters
	BOOLE setSensorMode(UCHAR ucSensorMode_);
	BOOLE setSensorSettings(void* psNewSensorSettings, UCHAR ucDesiredSettings_);
	BOOLE setStandByTime(UCHAR ucStandByTime_);

	//Getters
	const bme280_data* getSensorData() const;
	UCHAR getChipID() const;
	UCHAR getSensorMode() const;
	const bme280_settings* getSensorSettings() const;
	UCHAR getStandbyTime() const;
	UINT getMaxDelay() const;

private:

	//Sensor Configuration Functions
	BOOLE initSensor();
	void configureI2c();
	BOOLE calculateMaxMeasurementDelay(UINT* puiMaxDelay_); //TODO Implement

	//Class properties
	bme280_dev* psMyEnvSensor;
	bme280_data* psMyCurrentSensorData;
	bme280_settings* psMyCurrentSensorSettings;
	UCHAR ucMySensorMode;
	UINT uiMyMaxDelay;

};

#endif // __cplusplus

#endif // __ENVIRONMENTALSENSOR_H__
