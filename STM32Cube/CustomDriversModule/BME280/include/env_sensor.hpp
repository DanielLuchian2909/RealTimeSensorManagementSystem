/**
 ********************************************************************************
 * @file        env_sensor.hpp
 * @author      Daniel Luchian
 * @brief       Header file for the EnvironmentalSensor class
 ********************************************************************************
 */

#ifndef __ENV_SENSOR_HPP__
#define __ENV_SENSOR_HPP__

/************************************
 * INCLUDES
 ************************************/
#include "base_types.h" //Basic type definitions and utilities

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
BOOLE cInterfaceSoftReset(EnvSensorHandle* psEnvSensorHandle_);//Soft reset the sensor
BOOLE cInterfaceSetSensorMode(EnvSensorHandle* psEnvSensorHandle_, UCHAR ucSensorMode_); //Set sensor mode
BOOLE cInterfaceSetSensorSettings( EnvSensorHandle* psEnvSensorHandle_,void* psNewSensorSettings_, UCHAR ucDesiredSettings_); //Set sensor settings
const bme280_data* cInterfaceGetSensorData(EnvSensorHandle* psEnvSensorHandle_); //Get the sensor data
UCHAR cInterfaceGetChipID(EnvSensorHandle* psEnvSensorHandle_); //Get the chip idUCHAR //Returns the sensor chip ID
UCHAR cInterfaceGetSensorMode(EnvSensorHandle* psEnvSensorHandle_); //Get sensor mode
const  bme280_settings* cInterfaceGetSensorSettings(EnvSensorHandle* psEnvSensorHandle_); //Get sensor settings
UCHAR cInterfaceGetSensorStandbyTime(EnvSensorHandle* psEnvSensorHandle_); //Get sensor standy time
UINT cInterfaceGetMaxDelay(EnvSensorHandle* psEnvSensorHandle_); //Get sensor max delay

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
	BOOLE softReset();

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

#endif // __ENV_SENSOR_HPP__
