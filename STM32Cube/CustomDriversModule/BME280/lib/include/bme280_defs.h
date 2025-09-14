/**
* Copyright (c) 2020 Bosch Sensortec GmbH. All rights reserved.
*
* BSD-3-Clause
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* @file       bme280_defs.h
* @date       2020-12-17
* @version    v3.5.1
*
*/

#ifndef __BME280_DEFS_H__
#define __BME280_DEFS_H__

/********************************************************/
/* header includes */
#include "base_types.h"

//Macro fixes

/********************************************************/
/*! @name       Common macros               */
/********************************************************/

/**@}*/
/**\name C standard macros */
#ifndef NULL
#ifdef __cplusplus
#define NULL         0
#else
#define NULL         ((void *) 0)
#endif
#endif

/******************************************************************************/
/*! @name        Compiler switch macros Definitions                */
/******************************************************************************/
#define BME280_32BIT_ENABLE
#ifndef BME280_64BIT_ENABLE /*< Check if 64-bit integer (using BME280_64BIT_ENABLE) is enabled */
#ifndef BME280_32BIT_ENABLE /*< Check if 32-bit integer (using BME280_32BIT_ENABLE) is enabled */
#ifndef BME280_DOUBLE_ENABLE /*< If any of the integer data types not enabled then enable BME280_DOUBLE_ENABLE */
#define BME280_DOUBLE_ENABLE
#endif
#endif
#endif

/******************************************************************************/
/*! @name        General Macro Definitions                */
/******************************************************************************/


/*!
 * BME280_INTF_RET_TYPE is the read/write interface return type which can be overwritten by the build system.
 */
#ifndef BME280_INTF_RET_TYPE
#define BME280_INTF_RET_TYPE                      CHAR
#endif

/*!
 * The last error code from read/write interface is stored in the device structure as intf_rslt.
 */
#ifndef BME280_INTF_RET_SUCCESS
#define BME280_INTF_RET_SUCCESS                   (CHAR)(0)
#endif

#ifndef BME280_INTF_RET_FAIL
#define BME280_INTF_RET_FAIL                   	  (CHAR)(1)
#endif

/*! @name API success code */
#define BME280_OK                                 (CHAR)(0)

/*! @name API error codes */
#define BME280_E_NULL_PTR                         (CHAR)(-1)
#define BME280_E_COMM_FAIL                        (CHAR)(-2)
#define BME280_E_INVALID_LEN                      (CHAR)(-3)
#define BME280_E_DEV_NOT_FOUND                    (CHAR)(-4)
#define BME280_E_SLEEP_MODE_FAIL                  (CHAR)(-5)
#define BME280_E_NVM_COPY_FAILED                  (CHAR)(-6)

/*! @name API warning codes */
#define BME280_W_INVALID_OSR_MACRO                (CHAR)(1)

/*! @name BME280 chip identifier */
#define BME280_CHIP_ID                            (UCHAR)(0x60)

/*! @name I2C addresses */
#define BME280_I2C_ADDR_PRIM                      (UCHAR)(0x76)
#define BME280_I2C_ADDR_SEC                       (UCHAR)(0x77)

/*! @name Register Address */
#define BME280_REG_CHIP_ID                        (UCHAR)(0xD0)
#define BME280_REG_RESET                          (UCHAR)(0xE0)
#define BME280_REG_TEMP_PRESS_CALIB_DATA          (UCHAR)(0x88)
#define BME280_REG_HUMIDITY_CALIB_DATA            (UCHAR)(0xE1)
#define BME280_REG_CTRL_HUM                       (UCHAR)(0xF2)
#define BME280_REG_STATUS                         (UCHAR)(0xF3)
#define BME280_REG_PWR_CTRL                       (UCHAR)(0xF4)
#define BME280_REG_CTRL_MEAS                      (UCHAR)(0xF4)
#define BME280_REG_CONFIG                         (UCHAR)(0xF5)
#define BME280_REG_DATA                           (UCHAR)(0xF7)

/*! @name Macros related to size */
#define BME280_LEN_TEMP_PRESS_CALIB_DATA          (UCHAR)(26)
#define BME280_LEN_HUMIDITY_CALIB_DATA            (UCHAR)(7)
#define BME280_LEN_P_T_H_DATA                     (UCHAR)(8)

/*! @name Sensor power modes */
#define BME280_POWERMODE_SLEEP                    (UCHAR)(0x00)
#define BME280_POWERMODE_FORCED                   (UCHAR)(0x01)
#define BME280_POWERMODE_NORMAL                   (UCHAR)(0x03)
#define BME280_POWERMODE_INVALID				  (UCHAR)(0x05)

#define BME280_SENSOR_MODE_MSK                    (UCHAR)(0x03)
#define BME280_SENSOR_MODE_POS                    (UCHAR)(0x00)

/*! @name Soft reset command */
#define BME280_SOFT_RESET_COMMAND                 (UCHAR)(0xB6)

#define BME280_STATUS_IM_UPDATE                   (UCHAR)(0x01)
#define BME280_STATUS_MEAS_DONE                   (UCHAR)(0x08)

/*! @name Sensor component selection macros
 * These values are internal for API implementation. Don't relate this to
 * data sheet.
 */
#define BME280_PRESS                              (UCHAR)(1)
#define BME280_TEMP                               (UCHAR)(1 << 1)
#define BME280_HUM                                (UCHAR)(1 << 2)
#define BME280_ALL                                (UCHAR)(0x07)

/*! @name Settings selection macros */
#define BME280_SEL_OSR_PRESS                      (UCHAR)(1)
#define BME280_SEL_OSR_TEMP                       (UCHAR)(1 << 1)
#define BME280_SEL_OSR_HUM                        (UCHAR)(1 << 2)
#define BME280_SEL_FILTER                         (UCHAR)(1 << 3)
#define BME280_SEL_STANDBY                        (UCHAR)(1 << 4)
#define BME280_SEL_ALL_SETTINGS                   (UCHAR)(0x1F)

/*! @name Oversampling macros */
#define BME280_NO_OVERSAMPLING                    (UCHAR)(0x00)
#define BME280_OVERSAMPLING_1X                    (UCHAR)(0x01)
#define BME280_OVERSAMPLING_2X                    (UCHAR)(0x02)
#define BME280_OVERSAMPLING_4X                    (UCHAR)(0x03)
#define BME280_OVERSAMPLING_8X                    (UCHAR)(0x04)
#define BME280_OVERSAMPLING_16X                   (UCHAR)(0x05)
#define BME280_OVERSAMPLING_MAX                   (UCHAR)(16)

#define BME280_CTRL_HUM_MSK                       (UCHAR)(0x07)
#define BME280_CTRL_HUM_POS                       (UCHAR)(0x00)
#define BME280_CTRL_PRESS_MSK                     (UCHAR)(0x1C)
#define BME280_CTRL_PRESS_POS                     (UCHAR)(0x02)
#define BME280_CTRL_TEMP_MSK                      (UCHAR)(0xE0)
#define BME280_CTRL_TEMP_POS                      (UCHAR)(0x05)

/*! @name Measurement delay calculation macros  */
#define BME280_MEAS_OFFSET                        (USHORT)(1250)
#define BME280_MEAS_DUR                           (USHORT)(2300)
#define BME280_PRES_HUM_MEAS_OFFSET               (USHORT)(575)
#define BME280_MEAS_SCALING_FACTOR                (USHORT)(1000)
#define BME280_STARTUP_DELAY                      (USHORT)(2000)

/*! @name Length macros */
#define BME280_MAX_LEN                            (UCHAR)(10)

/*! @name Standby duration selection macros */
#define BME280_STANDBY_TIME_0_5_MS                (0x00)
#define BME280_STANDBY_TIME_62_5_MS               (0x01)
#define BME280_STANDBY_TIME_125_MS                (0x02)
#define BME280_STANDBY_TIME_250_MS                (0x03)
#define BME280_STANDBY_TIME_500_MS                (0x04)
#define BME280_STANDBY_TIME_1000_MS               (0x05)
#define BME280_STANDBY_TIME_10_MS                 (0x06)
#define BME280_STANDBY_TIME_20_MS                 (0x07)

#define BME280_STANDBY_MSK                        (UCHAR)(0xE0)
#define BME280_STANDBY_POS                        (UCHAR)(0x05)

/*! @name Bit shift macros */
#define BME280_12_BIT_SHIFT                       (UCHAR)(12)
#define BME280_8_BIT_SHIFT                        (UCHAR)(8)
#define BME280_4_BIT_SHIFT                        (UCHAR)(4)

/*! @name Filter coefficient selection macros */
#define BME280_FILTER_COEFF_OFF                   (0x00)
#define BME280_FILTER_COEFF_2                     (0x01)
#define BME280_FILTER_COEFF_4                     (0x02)
#define BME280_FILTER_COEFF_8                     (0x03)
#define BME280_FILTER_COEFF_16                    (0x04)

#define BME280_FILTER_MSK                         (UCHAR)(0x1C)
#define BME280_FILTER_POS                         (UCHAR)(0x02)

/*! @name Macro to combine two 8 bit data's to form a 16 bit data */
#define BME280_CONCAT_BYTES(msb, lsb)             (((USHORT)msb << 8) | (USHORT)lsb)

/*! @name Macro to SET and GET BITS of a register */
#define BME280_SET_BITS(reg_data, bitname, data) \
    ((reg_data & ~(bitname##_MSK)) | \
     ((data << bitname##_POS) & bitname##_MSK))

#define BME280_SET_BITS_POS_0(reg_data, bitname, data) \
    ((reg_data & ~(bitname##_MSK)) | \
     (data & bitname##_MSK))

#define BME280_GET_BITS(reg_data, bitname)        ((reg_data & (bitname##_MSK)) >> \
                                                   (bitname##_POS))
#define BME280_GET_BITS_POS_0(reg_data, bitname)  (reg_data & (bitname##_MSK))

/********************************************************/

/*!
 * @brief Interface selection Enums
 */
typedef enum{
    /*! SPI interface */
    BME280_SPI_INTF,
    /*! I2C interface */
    BME280_I2C_INTF

}bme280_intf;

/******************************************************************************/
/*! @name           Function Pointers                             */
/******************************************************************************/

/*!
 * @brief Bus communication function pointer which should be mapped to
 * the platform specific read functions of the user
 *
 * @param[in] reg_addr       : Register address from which data is read.
 * @param[out] reg_data      : Pointer to data buffer where read data is stored.
 * @param[in] len            : Number of bytes of data to be read.
 * @param[in, out] intf_ptr  : Void pointer that can enable the linking of descriptors
 *                             for interface related call backs.
 *
 * @retval   0 -> Success.
 * @retval Non zero value -> Fail.
 *
 */
typedef BME280_INTF_RET_TYPE (*bme280_read_fptr_t)(
		UCHAR reg_addr,
		UCHAR *reg_data,
		UINT len,
		void *intf_ptr);

/*!
 * @brief Bus communication function pointer which should be mapped to
 * the platform specific write functions of the user
 *
 * @param[in] reg_addr      : Register address to which the data is written.
 * @param[in] reg_data      : Pointer to data buffer in which data to be written
 *                            is stored.
 * @param[in] len           : Number of bytes of data to be written.
 * @param[in, out] intf_ptr : Void pointer that can enable the linking of descriptors
 *                            for interface related call backs
 *
 * @retval   0   -> Success.
 * @retval Non zero value -> Fail.
 *
 */
typedef BME280_INTF_RET_TYPE (*bme280_write_fptr_t)(
		UCHAR reg_addr,
		const UCHAR *reg_data,
		UINT len,
		void *intf_ptr);

/*!
 * @brief Delay function pointer which should be mapped to
 * delay function of the user
 *
 * @param[in] period              : Delay in microseconds.
 * @param[in, out] intf_ptr       : Void pointer that can enable the linking of descriptors
 *                                  for interface related call backs
 *
 */
typedef void (*bme280_delay_us_fptr_t)(
		UINT period,
		void *intf_ptr);

/******************************************************************************/
/*!  @name         Structure Declarations                             */
/******************************************************************************/

/*!
 * @brief Calibration data
 */
typedef struct bme280_calib_data
{
    /*! Calibration coefficient for the temperature sensor */
    USHORT dig_t1;

    /*! Calibration coefficient for the temperature sensor */
    SHORT dig_t2;

    /*! Calibration coefficient for the temperature sensor */
    SHORT dig_t3;

    /*! Calibration coefficient for the pressure sensor */
    USHORT dig_p1;

    /*! Calibration coefficient for the pressure sensor */
    SHORT dig_p2;

    /*! Calibration coefficient for the pressure sensor */
    SHORT dig_p3;

    /*! Calibration coefficient for the pressure sensor */
    SHORT dig_p4;

    /*! Calibration coefficient for the pressure sensor */
    SHORT dig_p5;

    /*! Calibration coefficient for the pressure sensor */
    SHORT dig_p6;

    /*! Calibration coefficient for the pressure sensor */
    SHORT dig_p7;

    /*! Calibration coefficient for the pressure sensor */
    SHORT dig_p8;

    /*! Calibration coefficient for the pressure sensor */
    SHORT dig_p9;

    /*! Calibration coefficient for the humidity sensor */
    UCHAR dig_h1;

    /*! Calibration coefficient for the humidity sensor */
    SHORT dig_h2;

    /*! Calibration coefficient for the humidity sensor */
    UCHAR dig_h3;

    /*! Calibration coefficient for the humidity sensor */
    SHORT dig_h4;

    /*! Calibration coefficient for the humidity sensor */
    SHORT dig_h5;

    /*! Calibration coefficient for the humidity sensor */
    CHAR dig_h6;

    /*! Variable to store the intermediate temperature coefficient */
    INT t_fine;
}bme280_calib_data;

/*!
 * @brief bme280 sensor structure which comprises of temperature, pressure and
 * humidity data
 */
//#ifdef BME280_DOUBLE_ENABLE
//typedef struct bme280_data
//{
    /*! Compensated pressure */
 //   DOUBLE pressure;

    /*! Compensated temperature */
  //  DOUBLE temperature;

    /*! Compensated humidity */
  //  DOUBLE humidity;
//}bme280_data;

//#else
typedef struct bme280_data
{
    /*! Compensated pressure */
    UINT pressure;

    /*! Compensated temperature */
    INT temperature;

    /*! Compensated humidity */
    UINT humidity;
}bme280_data;
//#endif /*! BME280_USE_FLOATING_POINT */

/*!
 * @brief bme280 sensor structure which comprises of uncompensated temperature,
 * pressure and humidity data
 */
typedef struct bme280_uncomp_data
{
    /*! Un-compensated pressure */
    UINT pressure;

    /*! Un-compensated temperature */
    UINT temperature;

    /*! Un-compensated humidity */
    UINT humidity;
}bme280_uncomp_data;

/*!
 * @brief bme280 sensor settings structure which comprises of mode,
 * oversampling and filter settings.
 */
typedef struct bme280_settings
{
    /*! Pressure oversampling */
    UCHAR osr_p;

    /*! Temperature oversampling */
    UCHAR osr_t;

    /*! Humidity oversampling */
    UCHAR osr_h;

    /*! Filter coefficient */
    UCHAR filter;

    /*! Standby time */
    UCHAR standby_time;
}bme280_settings;

/*!
 * @brief bme280 device structure
 */
typedef struct bme280_dev
{
    /*! Chip Id */
    UCHAR chip_id;

    /*! Interface Selection
     * For SPI, intf = BME280_SPI_INTF
     * For I2C, intf = BME280_I2C_INTF
     */
    bme280_intf intf;

    /*!
     * The interface pointer is used to enable the user
     * to link their interface descriptors for reference during the
     * implementation of the read and write interfaces to the
     * hardware.
     */
    void *intf_ptr;

    /*! Variable to store result of read/write function */
    BME280_INTF_RET_TYPE intf_rslt;

    /*! Read function pointer */
    bme280_read_fptr_t read;

    /*! Write function pointer */
    bme280_write_fptr_t write;

    /*! Delay function pointer */
    bme280_delay_us_fptr_t delay_us;

    /*! Trim data */
    bme280_calib_data calib_data;

}bme280_dev;

#endif /* _BME280_DEFS_H */
