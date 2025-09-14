/**
 ********************************************************************************
 * @file  sensor_rw_demo.c
 * @author Daniel Luchian
 * @brief RTOS scheduler implementation for managing thread execution
 ********************************************************************************
 */

/************************************
 * INCLUDES
 ************************************/
#include "sensor_rw_demo.h"
#include "lk_kernel.h"
#include "lk_mutex.h"
#include "env_sensor.hpp"

/************************************
 * EXTERN VARIABLES
 ************************************/

/************************************
 * PRIVATE MACROS AND DEFINES
 ************************************/

/************************************
 * PRIVATE TYPEDEFS
 ************************************/
typedef struct sensor_data_alias_t
{
UINT pressure_;
INT temperature_;
UINT humidity_;
} sensor_data_alias_t;

typedef struct
{
	EnvSensorHandle* sensor_;
	sensor_data_alias_t sensor_data_;
	mutex_t mutex_;
} sensor_thread_args_t;

/************************************
 * STATIC VARIABLES
 ************************************/

/************************************
 * GLOBAL VARIABLES
 ************************************/

/************************************
 * STATIC FUNCTION PROTOTYPES
 ************************************/
void readSensorThread(void* args);
void writeSensorThread(void* args);

/************************************
 * STATIC FUNCTIONS
 ************************************/
//-----------------------------------------------------------------------
void readSensorThread(void* args) // Thread function to read BME280 sensor data
{
	sensor_thread_args_t* inputs = (sensor_thread_args_t*)args;

	// Main Thread Loop
	while (1)
	{
		lk_mutexLock(&inputs->mutex_);
		cInterfaceReadSensorData(inputs->sensor_, READ_MODE_CONTINUOUS, ENVSENSOR_SELECT_ALL);
		lk_mutexUnlock(&inputs->mutex_);

		lk_delay(500);
	}
}

//-----------------------------------------------------------------------
void writeSensorThread(void* args) // Thread function to write sensor data over UART
{
	sensor_thread_args_t* inputs = (sensor_thread_args_t*)args;

	// Main Thread Loop
	while (1)
	{
		lk_mutexLock(&inputs->mutex_);
		inputs->sensor_data_ = *((sensor_data_alias_t*)(cInterfaceGetSensorData(inputs->sensor_)));
		printf("\r\nPressure=%u kPa, Temperature=%i C, Humidity=%u %%RH",
		       inputs->sensor_data_.pressure_,
			   inputs->sensor_data_.temperature_,
			   inputs->sensor_data_.humidity_);
		lk_mutexUnlock(&inputs->mutex_);

		lk_delay(500);
	}
}

/************************************
 * GLOBAL FUNCTIONS
 ************************************/
//-----------------------------------------------------------------------
void runSensorRWDemo()
{
	// Initialize thread arguments
	sensor_thread_args_t* args = malloc(sizeof(sensor_thread_args_t));
	lk_mutexInit(&args->mutex_);
	args->sensor_ = cInterfaceCreateEnvironmentalSensor();

	// Start RTOS
    lk_kernelInit();
    lk_createThread(readSensorThread, args);
    lk_createThread(writeSensorThread, args);
    lk_kernelStart();
}
