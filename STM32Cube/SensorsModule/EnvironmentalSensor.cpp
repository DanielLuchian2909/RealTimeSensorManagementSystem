/**
 ********************************************************************************
 * @file        EnvironmentalSensor.cpp
 * @author      Daniel Luchian
 * @brief       Implementation file for the ClassName class
 ********************************************************************************
 */

/************************************
 * INCLUDES
 ************************************/
#include "EnvironmentalSensor.h"
#include <cstdlib>

//Declaration of C Headers/Libraries
extern "C"
{
	#include "bme280.h" 			//C Library for the sensor
	#include "i2c.h" 				//STM32 generated i2c interface
	#include "tim.h" 				//STM32 generated timer interface
	#include "stm32f4xx_hal_i2c.h"  //STM32 generated i2c API
}

/************************************
 * EXTERN VARIABLES
 ************************************/
extern I2C_HandleTypeDef hi2c1; //Handle for I2C interface 1
extern TIM_HandleTypeDef htim1; //Handle for timer 1

/************************************
 * PRIVATE MACROS AND DEFINES
 ************************************/

/************************************
 * PRIVATE TYPEDEFS
 ************************************/
//Structure containing a pointer to an environmental sensor class for C-interfacing
typedef struct EnvSensorHandle
{
	EnvironmentalSensor* pcMyEnvSensor;
} EnvSensorHandle;

/************************************
 * STATIC VARIABLES
 ************************************/

/************************************
 * STATIC FUNCTIONS
 ************************************/
static BME280_INTF_RET_TYPE	Bme280I2cRead(UCHAR ucRegAddr_, UCHAR *ucRegData_, UINT uiLen_, void *pvIntf_); //The I2C read function for the BME280 sensor
static BME280_INTF_RET_TYPE Bme280I2cWrite(UCHAR ucRegAddr_, const UCHAR *ucRegData_, UINT uiLen_, void* pvIntf_); //The I2C write function for the BME280 sensor
static void Bme280Delay(UINT uiPeriod_, void* pvIntf_); //Microsecond delay function for the BME280 sensor

//-----------------------------------------------------------------------
static BME280_INTF_RET_TYPE	//Returns the last read/write error code
Bme280I2cRead(			    //The I2C read function for the BME280 sensor
		UCHAR ucRegAddr_,	//The address of the register to read from
		UCHAR *ucRegData_,  //A pointer to the where the read data will be stored
		UINT uiLen_,	    //Number of bytes of data to be read
		void *pvIntf_)      //A pointer to the I2C interface
{
	//Setting the address of the BME280 device (last bit is implicity set to 0 for write mode)
	USHORT usDeviceAddr = BME280_I2C_ADDR_PRIM << 1;

	//Transmit data (register to read from) in blocking mode
	//(blocking mode for now, until buffer is handled better)
	if (HAL_I2C_Master_Transmit(&hi2c1, usDeviceAddr, &ucRegAddr_, sizeof(ucRegAddr_), HAL_MAX_DELAY)
			!= HAL_OK)
	{
		return BME280_INTF_RET_FAIL;
	}

	//Swith address to read mode
	usDeviceAddr |= 1;

	//Receive data from the sensor
	if (HAL_I2C_Master_Receive(&hi2c1, usDeviceAddr, ucRegData_, uiLen_, HAL_MAX_DELAY)
			!= HAL_OK)
	{
		return BME280_INTF_RET_FAIL;
	}

	return BME280_INTF_RET_SUCCESS;
}

//-----------------------------------------------------------------------
static BME280_INTF_RET_TYPE 	 //Returns the last read/write error code
Bme280I2cWrite( 				 //The i2c write function for the BME280 sensor
		UCHAR ucRegAddr_,		 //The address of the register to write to
		const UCHAR *ucRegData_, //A pointer to the data to write to that register
		UINT uiLen_,			 //The length (in bytes) of the data
		void* pvIntf_)			 //Function pointer (not used)
{
		//Setting the address of the BME280 device (last bit is implicity set to 0 for write mode)
		USHORT usDeviceAddr = BME280_I2C_ADDR_PRIM << 1;

		//Creating the data buffer
		UCHAR ucDataBuffer[1 + uiLen_]; //Buffer of length 1 byte for register address + data length

		//Set the first byte to the register address to write to
		ucDataBuffer[0] = ucRegAddr_;

		//Fill the buffer with the data to write to the register
		for (UINT i = 0; i < uiLen_; i++)
		{
			ucDataBuffer[i + 1] = ucRegData_[i];
		}

		//Transmit data in blocking mode (blocking mode for now, until buffer is handled better)
		if (HAL_I2C_Master_Transmit(&hi2c1, usDeviceAddr, ucDataBuffer, sizeof(ucDataBuffer), HAL_MAX_DELAY)
				!= HAL_OK)
		{
			return BME280_INTF_RET_FAIL; //Handle error
		}

		return BME280_INTF_RET_SUCCESS;
}

//-----------------------------------------------------------------------
static void
Bme280Delay( //Delay function in microseconds
		UINT uiPeriod_, //Delay time in microseconds
		void* pvIntf_)  //Function pointer (not used)
{

	UINT uiMilliSeconds = 0;
	UINT uiMicroSeconds = uiPeriod_;

    //If the delay requested is more than the timer period break the delay down into the milli and micro components
    if (uiPeriod_ > htim1.Instance->ARR)
    {
    	uiMilliSeconds = uiPeriod_ / 1000;
    	uiMicroSeconds = uiPeriod_ % 1000;
    }

    //Delay the milliseconds component first
    for (UINT i = 0; i < uiMilliSeconds; i++)
    {
    	htim1.Instance->CNT = 0; //Reset the timer
    	while (htim1.Instance->CNT < 1000)
    	{
    		//Wait for timer to count one milliseconds
    	}
    }

    //Reset the timer
    htim1.Instance->CNT = 0;

    //Delay the microseconds component
    while (htim1.Instance->CNT < uiPeriod_)
    {
        //Wait for timer to count set microseconds
    }
}

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
	if (psEnvSensorHandle_) //If the sensor exists delete it and its handle
	{
		delete psEnvSensorHandle_->pcMyEnvSensor;
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
	//If the handle exists and it has a sensor read the sensors data
	if (psEnvSensorHandle_ && (psEnvSensorHandle_->pcMyEnvSensor) )
	{
		psEnvSensorHandle_->pcMyEnvSensor->readSensorData(eReadMode_, eEnvSensorSelect_);
	}
}

//---------------------------------------------------------------------
extern "C"
const bme280_data* //A pointer to the data (immutable)
cInterfaceGetSensorData( //C style function to get the sensor data
		EnvSensorHandle* psEnvSensorHandle_) //A pointer to to an environmental sensor handle
{
	return (psEnvSensorHandle_->pcMyEnvSensor->getSensorData());
}

/************************************
 * CONSTRUCTORS AND DESTRUCTOR
 ************************************/
//-----------------------------------------------------------------------
EnvironmentalSensor::EnvironmentalSensor() //Default constructor
{
	//Create a new environment sensor structure
	psMyEnvSensor = new bme280_dev;

	//Create a new environmental sensor data structure
	psMyCurrentSensorData = new  bme280_data;

	//Create a new environmental sensor settings structure
	psMyCurrentSensorSettings = new bme280_settings;

	psMyEnvSensor->intf = BME280_I2C_INTF;

	//Configure I2C as the default communication
	configureI2c();

	//Set the delay function
	psMyEnvSensor->delay_us = Bme280Delay;

	//Call the BME280 Api to initialize the sensor
	bme280_init(psMyEnvSensor);

	//Configure default sensor settings
	psMyCurrentSensorSettings->osr_p = BME280_OVERSAMPLING_1X;
	psMyCurrentSensorSettings->osr_t = BME280_OVERSAMPLING_1X;
	psMyCurrentSensorSettings->osr_h = BME280_OVERSAMPLING_1X;
	psMyCurrentSensorSettings->filter = BME280_FILTER_COEFF_2;
	psMyCurrentSensorSettings->standby_time = BME280_STANDBY_TIME_0_5_MS;
	bme280_set_sensor_settings(BME280_SEL_ALL_SETTINGS, psMyCurrentSensorSettings, psMyEnvSensor);

	//Set the sensor to the default mode
	ucMySensorMode = BME280_POWERMODE_NORMAL;
	bme280_set_sensor_mode(ucMySensorMode, psMyEnvSensor);

	//Calculate the measurement time in microseconds (based on sensor settings)
	bme280_cal_meas_delay(&uiMyMaxDelay, psMyCurrentSensorSettings);
}

//-----------------------------------------------------------------------
EnvironmentalSensor::~EnvironmentalSensor()//Destructor
{
	delete psMyEnvSensor; 		  		//De-alloacte the environmental sensor structure
	delete psMyCurrentSensorData; 	    //De-allocate the environmental sensor data structure
	delete psMyCurrentSensorSettings;	//De-allocate the environmental sensor settings structure
}

/************************************
 * PUBLIC METHODS
 ************************************/
//-----------------------------------------------------------------------
void
EnvironmentalSensor::readSensorData( //Read the current sensor data
		ReadMode eReadMode_, //The mode to read the sensor data
		EnvSensorSelect eEnvSensorSelect_) //The sensor(s) to read
{
	//Switch based on which read mode is selected
	switch (eReadMode_)
	{

	//Single read mode case
	case (READ_MODE_SINGLE):
			//Get the sensor data for the select sensor
			bme280_get_sensor_data(eEnvSensorSelect_, psMyCurrentSensorData, psMyEnvSensor);
			break;

	//TODO Implement remaining read modes

	//Default to single shot reading
	default:
		bme280_get_sensor_data(eEnvSensorSelect_, psMyCurrentSensorData, psMyEnvSensor);
		break;
	}
}

//-----------------------------------------------------------------------
CHAR //Returns true if the reset did not fail, otherwise false
EnvironmentalSensor::softReset() //Function to reset the sensor
{
	//Soft rest the sensor through the BME280 library
	return (bme280_soft_reset(psMyEnvSensor));
}

//-----------------------------------------------------------------------
BOOLE //Returns true if the sensor mode is successfully set, otherwise false
EnvironmentalSensor::setSensorMode( //Setting the sensor mode
		UCHAR ucSensorMode_) //Sensor mode to set
{
	//If the sensor mode is successfully set return true  and update the sensor mode, otherwise false
	if (bme280_set_sensor_mode(ucSensorMode_, psMyEnvSensor) == BME280_OK)
	{
		ucMySensorMode = ucSensorMode_;
		return TRUE;
	}
	return FALSE;
}

//-----------------------------------------------------------------------
BOOLE //Returns true if the sensor mode is successfully set, otherwise false
EnvironmentalSensor::setSensorSettings( //Setting the sensor mode
		void* psNewSensorSettings_, //Sensor settings to set
		UCHAR ucDesiredSettings) //The desired settings to change
{
	//Backup the current sensor settings
	bme280_settings sBackupSettings = *psMyCurrentSensorSettings;

	//Cast imputed settings to BME280 settings type and update the class sensor settings
	bme280_settings* psNewSensorSettings = static_cast<bme280_settings*>(psNewSensorSettings_);
	*psMyCurrentSensorSettings = *psNewSensorSettings;

	//If the settings are not successfully set don't change the sensor settings and return false, otherwise true
	if (bme280_set_sensor_settings(ucDesiredSettings, psMyCurrentSensorSettings, psMyEnvSensor) != BME280_OK)
	{
		*psMyCurrentSensorSettings = sBackupSettings;
		return FALSE;
	}
	return TRUE;
}

//-----------------------------------------------------------------------
const bme280_data* //Returns the address of where the bme280_data for this class is store
EnvironmentalSensor::getSensorData() const //Getter for the sensor data
{
    return psMyCurrentSensorData;
}

//-----------------------------------------------------------------------
UCHAR //Returns the sensor chip ID
EnvironmentalSensor::getChipID() const //Getter for the sensor chip id
{
	return psMyEnvSensor->chip_id;
}

//-----------------------------------------------------------------------
UCHAR //Return the sensor mode
EnvironmentalSensor::getSensorMode() const
{
	//Temporary sensor mode buffer
	UCHAR ucSensorMode;

	//If the sensor mode is successfully fetched return it, otherwise return 0x04 (not a valid mode)
	if ( bme280_get_sensor_mode(&ucSensorMode, psMyEnvSensor) == 0)
	{
		return ucSensorMode;
	}
	return BME280_POWERMODE_INVALID;
}

//-----------------------------------------------------------------------
const  bme280_settings* //Return a constant reference to a BME280 sensor settings structure
EnvironmentalSensor::getSensorSettings() const //Getter for the sensor settings
{

}

//-----------------------------------------------------------------------
UCHAR //Returns the standby time
EnvironmentalSensor::getStandbyTime() const //Getter for sensor standby time
{
	return psMyCurrentSensorSettings->standby_time;
}

//-----------------------------------------------------------------------
UINT //Returns the max delay between measurements
EnvironmentalSensor::getMaxDelay() const //Getter for sensor standby time
{
	return uiMyMaxDelay;
}

/************************************
 * PRIVATE HELPER FUNCTIONS
 ************************************/
//-----------------------------------------------------------------------
void
EnvironmentalSensor::configureI2c() //Function responsible for configuring I2C for the sensor
{
	psMyEnvSensor->read = Bme280I2cRead;
	psMyEnvSensor->write= Bme280I2cWrite;
	psMyEnvSensor->intf_ptr = &hi2c1;
}
