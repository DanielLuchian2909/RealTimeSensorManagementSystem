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
* @file       bme280.c
* @date       2020-12-17
* @version    v3.5.1
*
*/

/*! @file bme280.c
 * @brief Sensor driver for BME280 sensor
 */
#include "bme280.h"

/**\name Internal macros */
/* To identify osr settings selected by user */
#define OVERSAMPLING_SETTINGS    (UCHAR)(0x07)

/* To identify filter and standby settings selected by user */
#define FILTER_STANDBY_SETTINGS  (UCHAR)(0x18)

/*!
 * @brief This internal API puts the device to sleep mode.
 *
 * @param[in] dev : ure instance of bme280_dev.
 *
 * @return Result of API execution status.
 *
 * @retval   0 -> Success.
 * @retval > 0 -> Warning.
 * @retval < 0 -> Fail.
 *
 */
static CHAR put_device_to_sleep(bme280_dev *dev);

/*!
 * @brief This internal API writes the power mode in the sensor.
 *
 * @param[in] dev         : ure instance of bme280_dev.
 * @param[in] sensor_mode : Variable which contains the power mode to be set.
 *
 * @return Result of API execution status.
 *
 * @retval   0 -> Success.
 * @retval > 0 -> Warning.
 * @retval < 0 -> Fail.
 *
 */
static CHAR write_power_mode(UCHAR sensor_mode, bme280_dev *dev);

/*!
 * @brief This internal API is used to validate the device pointer for
 * null conditions.
 *
 * @param[in] dev : ure instance of bme280_dev.
 *
 * @return Result of API execution status
 *
 * @retval   0 -> Success.
 * @retval > 0 -> Warning.
 * @retval < 0 -> Fail.
 *
 */
static CHAR null_ptr_check(const bme280_dev *dev);

/*!
 * @brief This internal API interleaves the register address between the
 * register data buffer for burst write operation.
 *
 * @param[in] reg_addr   : Contains the register address array.
 * @param[out] temp_buff : Contains the temporary buffer to store the
 * register data and register address.
 * @param[in] reg_data   : Contains the register data to be written in the
 * temporary buffer.
 * @param[in] len        : No of bytes of data to be written for burst write.
 *
 */
static void interleave_reg_addr(const UCHAR *reg_addr, UCHAR *temp_buff, const UCHAR *reg_data, UINT len);

/*!
 * @brief This internal API reads the calibration data from the sensor, parse
 * it and store in the device ure.
 *
 * @param[in] dev : ure instance of bme280_dev.
 *
 * @return Result of API execution status
 *
 * @retval   0 -> Success.
 * @retval > 0 -> Warning.
 * @retval < 0 -> Fail.
 *
 */
static CHAR get_calib_data(bme280_dev *dev);

/*!
 *  @brief This internal API is used to parse the temperature and
 *  pressure calibration data and store it in the device ure.
 *
 *  @param[out] dev     : ure instance of bme280_dev to store the calib data.
 *  @param[in] reg_data : Contains the calibration data to be parsed.
 *
 */
static void parse_temp_press_calib_data(const UCHAR *reg_data, bme280_dev *dev);

/*!
 *  @brief This internal API is used to parse the humidity calibration data
 *  and store it in device ure.
 *
 *  @param[out] dev     : ure instance of bme280_dev to store the calib data.
 *  @param[in] reg_data : Contains calibration data to be parsed.
 *
 */
static void parse_humidity_calib_data(const UCHAR *reg_data, bme280_dev *dev);

/*!
 * @brief This internal API is used to identify the settings which the user
 * wants to modify in the sensor.
 *
 * @param[in] sub_settings     : Contains the settings subset to identify particular
 * group of settings which the user is interested to change.
 * @param[in] desired_settings : Contains the user specified settings.
 *
 * @return Indicates whether user is interested to modify the settings which
 * are related to sub_settings.
 * @return True -> User wants to modify this group of settings
 * @return False -> User does not want to modify this group of settings
 *
 */
static UCHAR are_settings_changed(UCHAR sub_settings, UCHAR desired_settings);

/*!
 * @brief This API sets the humidity over sampling settings of the sensor.
 *
 * @param[in] dev      : ure instance of bme280_dev.
 * @param[in] settings : Pointer variable which contains the settings to
 * be set in the sensor.
 *
 * @return Result of API execution status
 *
 * @retval   0 -> Success.
 * @retval > 0 -> Warning.
 * @retval < 0 -> Fail.
 *
 */
static CHAR set_osr_humidity_settings(const bme280_settings *settings, bme280_dev *dev);

/*!
 * @brief This internal API sets the oversampling settings for pressure,
 * temperature and humidity in the sensor.
 *
 * @param[in] desired_settings : Variable used to select the settings which
 * are to be set.
 * @param[in] settings         : Pointer variable which contains the settings to
 * be set in the sensor.
 * @param[in] dev              : ure instance of bme280_dev.
 *
 * @return Result of API execution status
 *
 * @retval   0 -> Success.
 * @retval > 0 -> Warning.
 * @retval < 0 -> Fail.
 *
 */
static CHAR set_osr_settings(UCHAR desired_settings, const bme280_settings *settings,
                             bme280_dev *dev);

/*!
 * @brief This API sets the pressure and/or temperature oversampling settings
 * in the sensor according to the settings selected by the user.
 *
 * @param[in] dev : ure instance of bme280_dev.
 * @param[in] desired_settings: variable to select the pressure and/or
 * temperature oversampling settings.
 * @param[in] settings : Pointer variable which contains the settings to
 * be set in the sensor.
 *
 * @return Result of API execution status
 *
 * @retval   0 -> Success.
 * @retval > 0 -> Warning.
 * @retval < 0 -> Fail.
 *
 */
static CHAR set_osr_press_temp_settings(UCHAR desired_settings,
                                        const bme280_settings *settings,
                                        bme280_dev *dev);

/*!
 * @brief This internal API fills the pressure oversampling settings provided by
 * the user in the data buffer so as to write in the sensor.
 *
 * @param[in] settings : Pointer variable which contains the settings to
 * be set in the sensor.
 * @param[out] reg_data : Variable which is filled according to the pressure
 * oversampling data provided by the user.
 *
 */
static void fill_osr_press_settings(UCHAR *reg_data, const bme280_settings *settings);

/*!
 * @brief This internal API fills the temperature oversampling settings provided
 * by the user in the data buffer so as to write in the sensor.
 *
 * @param[in] settings : Pointer variable which contains the settings to
 * be set in the sensor.
 * @param[out] reg_data : Variable which is filled according to the temperature
 * oversampling data provided by the user.
 *
 */
static void fill_osr_temp_settings(UCHAR *reg_data, const bme280_settings *settings);

/*!
 * @brief This internal API sets the filter and/or standby duration settings
 * in the sensor according to the settings selected by the user.
 *
 * @param[in] dev : ure instance of bme280_dev.
 * @param[in] settings : Pointer variable which contains the settings to
 * be set in the sensor.
 * @param[in] settings : ure instance of bme280_settings.
 *
 * @return Result of API execution status
 *
 * @retval   0 -> Success.
 * @retval > 0 -> Warning.
 * @retval < 0 -> Fail.
 *
 */
static CHAR set_filter_standby_settings(UCHAR desired_settings,
                                        const bme280_settings *settings,
                                        bme280_dev *dev);

/*!
 * @brief This internal API fills the filter settings provided by the user
 * in the data buffer so as to write in the sensor.
 *
 * @param[in] settings : Pointer variable which contains the settings to
 * be set in the sensor.
 * @param[out] reg_data : Variable which is filled according to the filter
 * settings data provided by the user.
 *
 */
static void fill_filter_settings(UCHAR *reg_data, const bme280_settings *settings);

/*!
 * @brief This internal API fills the standby duration settings provided by the
 * user in the data buffer so as to write in the sensor.
 *
 * @param[in] settings : Pointer variable which contains the settings to
 * be set in the sensor.
 * @param[out] reg_data : Variable which is filled according to the standby
 * settings data provided by the user.
 *
 */
static void fill_standby_settings(UCHAR *reg_data, const bme280_settings *settings);

/*!
 * @brief This internal API parse the oversampling(pressure, temperature
 * and humidity), filter and standby duration settings and store in the
 * device ure.
 *
 * @param[in] settings : Pointer variable which contains the settings to
 * be get in the sensor.
 * @param[in] reg_data : Register data to be parsed.
 *
 */
static void parse_device_settings(const UCHAR *reg_data, bme280_settings *settings);

/*!
 * @brief This API is used to parse the pressure, temperature and
 * humidity data and store it in the bme280_uncomp_data ure instance.
 *
 * @param[in] reg_data     : Contains register data which needs to be parsed
 * @param[out] uncomp_data : Contains the uncompensated pressure, temperature and humidity data
 */
static void parse_sensor_data(const UCHAR *reg_data, bme280_uncomp_data *uncomp_data);

/*!
 * @brief This internal API reloads the already existing device settings in the
 * sensor after soft reset.
 *
 * @param[in] dev : ure instance of bme280_dev.
 * @param[in] settings : Pointer variable which contains the settings to
 * be set in the sensor.
 *
 * @return Result of API execution status
 *
 * @retval   0 -> Success.
 * @retval > 0 -> Warning.
 * @retval < 0 -> Fail.
 *
 */
static CHAR reload_device_settings(const bme280_settings *settings, bme280_dev *dev);

#ifdef BME280_DOUBLE_ENABLE

/*!
 * @brief This internal API is used to compensate the raw pressure data and
 * return the compensated pressure data in double data type.
 *
 * @param[in] uncomp_data : Contains the uncompensated pressure data.
 * @param[in] calib_data  : Pointer to the calibration data ure.
 *
 * @return Compensated pressure data in double.
 *
 */
static double compensate_pressure(const bme280_uncomp_data *uncomp_data,
                                  const bme280_calib_data *calib_data);

/*!
 * @brief This internal API is used to compensate the raw humidity data and
 * return the compensated humidity data in double data type.
 *
 * @param[in] uncomp_data : Contains the uncompensated humidity data.
 * @param[in] calib_data  : Pointer to the calibration data ure.
 *
 * @return Compensated humidity data in double.
 *
 */
static double compensate_humidity(const bme280_uncomp_data *uncomp_data,
                                  const bme280_calib_data *calib_data);

/*!
 * @brief This internal API is used to compensate the raw temperature data and
 * return the compensated temperature data in double data type.
 *
 * @param[in] uncomp_data : Contains the uncompensated temperature data.
 * @param[in] calib_data  : Pointer to calibration data ure.
 *
 * @return Compensated temperature data in double.
 *
 */
static double compensate_temperature(const bme280_uncomp_data *uncomp_data,
                                      bme280_calib_data *calib_data);

#else

/*!
 * @brief This internal API is used to compensate the raw temperature data and
 * return the compensated temperature data in integer data type.
 *
 * @param[in] uncomp_data : Contains the uncompensated temperature data.
 * @param[in] calib_data  : Pointer to calibration data ure.
 *
 * @return Compensated temperature data in integer.
 *
 */
static INT compensate_temperature(const bme280_uncomp_data *uncomp_data,
                                       bme280_calib_data *calib_data);

/*!
 * @brief This internal API is used to compensate the raw pressure data and
 * return the compensated pressure data in integer data type.
 *
 * @param[in] uncomp_data : Contains the uncompensated pressure data.
 * @param[in] calib_data  : Pointer to the calibration data ure.
 *
 * @return Compensated pressure data in integer.
 *
 */
static UINT compensate_pressure(const  bme280_uncomp_data *uncomp_data,
                                    const  bme280_calib_data *calib_data);

/*!
 * @brief This internal API is used to compensate the raw humidity data and
 * return the compensated humidity data in integer data type.
 *
 * @param[in] uncomp_data : Contains the uncompensated humidity data.
 * @param[in] calib_data  : Pointer to the calibration data ure.
 *
 * @return Compensated humidity data in integer.
 *
 */
static UINT compensate_humidity(const  bme280_uncomp_data *uncomp_data,
                                    const  bme280_calib_data *calib_data);

#endif

/****************** Global Function Definitions *******************************/

/*!
 *  @brief This API is the entry point.
 *  It reads the chip-id and calibration data from the sensor.
 */
CHAR bme280_init( bme280_dev *dev)
{
    CHAR rslt;
    UCHAR chip_id = 0;

    /* Read the chip-id of bme280 sensor */
    rslt = bme280_get_regs(BME280_REG_CHIP_ID, &chip_id, 1, dev);

    /* Check for chip id validity */
    if (rslt == BME280_OK)
    {
        if (chip_id == BME280_CHIP_ID)
        {
            dev->chip_id = chip_id;

            /* Reset the sensor */
            rslt = bme280_soft_reset(dev);

            if (rslt == BME280_OK)
            {
                /* Read the calibration data */
                rslt = get_calib_data(dev);
            }
        }
        else
        {
            rslt = BME280_E_DEV_NOT_FOUND;
        }
    }

    return rslt;
}

/*!
 * @brief This API reads the data from the given register address of the sensor.
 */
CHAR bme280_get_regs(UCHAR reg_addr, UCHAR *reg_data, UINT len,  bme280_dev *dev)
{
    CHAR rslt;

    /* Check for null pointer in the device ure */
    rslt = null_ptr_check(dev);

    if ((rslt == BME280_OK) && (reg_data != NULL))
    {
        /* If interface selected is SPI */
        if (dev->intf != BME280_I2C_INTF)
        {
            reg_addr = reg_addr | 0x80;
        }

        /* Read the data */
        dev->intf_rslt = dev->read(reg_addr, reg_data, len, dev->intf_ptr);

        /* Check for communication error */
        if (dev->intf_rslt != BME280_INTF_RET_SUCCESS)
        {
            rslt = BME280_E_COMM_FAIL;
        }
    }
    else
    {
        rslt = BME280_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API writes the given data to the register address
 * of the sensor.
 */
CHAR bme280_set_regs(UCHAR *reg_addr, const UCHAR *reg_data, UINT len,  bme280_dev *dev)
{
    CHAR rslt;
    UCHAR temp_buff[20]; /* Typically not to write more than 10 registers */
    UINT temp_len;
    UINT reg_addr_cnt;

    if (len > BME280_MAX_LEN)
    {
        len = BME280_MAX_LEN;
    }

    /* Check for null pointer in the device ure */
    rslt = null_ptr_check(dev);

    /* Check for arguments validity */
    if ((rslt == BME280_OK) && (reg_addr != NULL) && (reg_data != NULL))
    {
        if (len != 0)
        {
            temp_buff[0] = reg_data[0];

            /* If interface selected is SPI */
            if (dev->intf != BME280_I2C_INTF)
            {
                for (reg_addr_cnt = 0; reg_addr_cnt < len; reg_addr_cnt++)
                {
                    reg_addr[reg_addr_cnt] = reg_addr[reg_addr_cnt] & 0x7F;
                }
            }

            /* Burst write mode */
            if (len > 1)
            {
                /* Interleave register address w.r.t data for
                 * burst write
                 */
                interleave_reg_addr(reg_addr, temp_buff, reg_data, len);
                temp_len = ((len * 2) - 1);
            }
            else
            {
                temp_len = len;
            }

            dev->intf_rslt = dev->write(reg_addr[0], temp_buff, temp_len, dev->intf_ptr);

            /* Check for communication error */
            if (dev->intf_rslt != BME280_INTF_RET_SUCCESS)
            {
                rslt = BME280_E_COMM_FAIL;
            }
        }
        else
        {
            rslt = BME280_E_INVALID_LEN;
        }
    }
    else
    {
        rslt = BME280_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API sets the oversampling, filter and standby duration
 * (normal mode) settings in the sensor.
 */
CHAR bme280_set_sensor_settings(UCHAR desired_settings,
                                  const  bme280_settings *settings,
                                   bme280_dev *dev)
{
    CHAR rslt;
    UCHAR sensor_mode;

    if (settings != NULL)
    {
        rslt = bme280_get_sensor_mode(&sensor_mode, dev);

        if ((rslt == BME280_OK) && (sensor_mode != BME280_POWERMODE_SLEEP))
        {
            rslt = put_device_to_sleep(dev);
        }

        if (rslt == BME280_OK)
        {
            /* Check if user wants to change oversampling
             * settings
             */
            if (are_settings_changed(OVERSAMPLING_SETTINGS, desired_settings))
            {
                rslt = set_osr_settings(desired_settings, settings, dev);
            }

            /* Check if user wants to change filter and/or
             * standby settings
             */
            if ((rslt == BME280_OK) && are_settings_changed(FILTER_STANDBY_SETTINGS, desired_settings))
            {
                rslt = set_filter_standby_settings(desired_settings, settings, dev);
            }
        }
    }
    else
    {
        rslt = BME280_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API gets the oversampling, filter and standby duration
 * (normal mode) settings from the sensor.
 */
CHAR bme280_get_sensor_settings( bme280_settings *settings,  bme280_dev *dev)
{
    CHAR rslt;
    UCHAR reg_data[4];

    if (settings != NULL)
    {
        rslt = bme280_get_regs(BME280_REG_CTRL_HUM, reg_data, 4, dev);

        if (rslt == BME280_OK)
        {
            parse_device_settings(reg_data, settings);
        }
    }
    else
    {
        rslt = BME280_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API sets the power mode of the sensor.
 */
CHAR bme280_set_sensor_mode(UCHAR sensor_mode,  bme280_dev *dev)
{
    CHAR rslt;
    UCHAR last_set_mode;

    rslt = bme280_get_sensor_mode(&last_set_mode, dev);

    /* If the sensor is not in sleep mode put the device to sleep
     * mode
     */
    if ((rslt == BME280_OK) && (last_set_mode != BME280_POWERMODE_SLEEP))
    {
        rslt = put_device_to_sleep(dev);
    }

    /* Set the power mode */
    if (rslt == BME280_OK)
    {
        rslt = write_power_mode(sensor_mode, dev);
    }

    return rslt;
}

/*!
 * @brief This API gets the power mode of the sensor.
 */
CHAR bme280_get_sensor_mode(UCHAR *sensor_mode,  bme280_dev *dev)
{
    CHAR rslt;

    if (sensor_mode != NULL)
    {
        /* Read the power mode register */
        rslt = bme280_get_regs(BME280_REG_PWR_CTRL, sensor_mode, 1, dev);

        /* Assign the power mode to variable */
        *sensor_mode = BME280_GET_BITS_POS_0(*sensor_mode, BME280_SENSOR_MODE);
    }
    else
    {
        rslt = BME280_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API performs the soft reset of the sensor.
 */
CHAR bme280_soft_reset( bme280_dev *dev)
{
    CHAR rslt;
    UCHAR reg_addr = BME280_REG_RESET;
    UCHAR status_reg = 0;
    UCHAR try_run = 5;

    /* 0xB6 is the soft reset command */
    UCHAR soft_rst_cmd = BME280_SOFT_RESET_COMMAND;

    /* Write the soft reset command in the sensor */
    rslt = bme280_set_regs(&reg_addr, &soft_rst_cmd, 1, dev);

    if (rslt == BME280_OK)
    {
        /* If NVM not copied yet, Wait for NVM to copy */
        do
        {
            /* As per data sheet - Table 1, startup time is 2 ms. */
            dev->delay_us(BME280_STARTUP_DELAY, dev->intf_ptr);
            rslt = bme280_get_regs(BME280_REG_STATUS, &status_reg, 1, dev);

        } while ((rslt == BME280_OK) && (try_run--) && (status_reg & BME280_STATUS_IM_UPDATE));

        if (status_reg & BME280_STATUS_IM_UPDATE)
        {
            rslt = BME280_E_NVM_COPY_FAILED;
        }
    }

    return rslt;
}

/*!
 * @brief This API reads the pressure, temperature and humidity data from the
 * sensor, compensates the data and store it in the bme280_data structure
 * instance passed by the user.
 */
CHAR bme280_get_sensor_data(UCHAR sensor_comp,  bme280_data *comp_data,  bme280_dev *dev)
{
    CHAR rslt;

    /* Array to store the pressure, temperature and humidity data read from
     * the sensor
     */
    UCHAR reg_data[BME280_LEN_P_T_H_DATA] = { 0 };
    bme280_uncomp_data uncomp_data = { 0 };

    if (comp_data != NULL)
    {
        /* Read the pressure and temperature data from the sensor */
        rslt = bme280_get_regs(BME280_REG_DATA, reg_data, BME280_LEN_P_T_H_DATA, dev);

        if (rslt == BME280_OK)
        {
            /* Parse the read data from the sensor */
            parse_sensor_data(reg_data, &uncomp_data);

            /* Compensate the pressure and/or temperature and/or
             * humidity data from the sensor
             */
            rslt = bme280_compensate_data(sensor_comp, &uncomp_data, comp_data, &dev->calib_data);
        }
    }
    else
    {
        rslt = BME280_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API is used to compensate the pressure and/or
 * temperature and/or humidity data according to the component selected
 * by the user.
 */
CHAR bme280_compensate_data(UCHAR sensor_comp,
                              const  bme280_uncomp_data *uncomp_data,
                               bme280_data *comp_data,
                               bme280_calib_data *calib_data)
{
    CHAR rslt = BME280_OK;

    if ((uncomp_data != NULL) && (comp_data != NULL) && (calib_data != NULL))
    {
        /* Initialize to zero */
        comp_data->temperature = 0;
        comp_data->pressure = 0;
        comp_data->humidity = 0;

        /* If pressure or temperature component is selected */
        if (sensor_comp & (BME280_PRESS | BME280_TEMP | BME280_HUM))
        {
            /* Compensate the temperature data */
            comp_data->temperature = compensate_temperature(uncomp_data, calib_data);
        }

        if (sensor_comp & BME280_PRESS)
        {
            /* Compensate the pressure data */
            comp_data->pressure = compensate_pressure(uncomp_data, calib_data);
        }

        if (sensor_comp & BME280_HUM)
        {
            /* Compensate the humidity data */
            comp_data->humidity = compensate_humidity(uncomp_data, calib_data);
        }
    }
    else
    {
        rslt = BME280_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API is used to calculate the maximum delay in milliseconds required for the
 * temperature/pressure/humidity(whichever are enabled) measurement to complete.
 */
CHAR bme280_cal_meas_delay(UINT *max_delay, const  bme280_settings *settings)
{
    CHAR rslt = BME280_OK;
    UCHAR temp_osr;
    UCHAR pres_osr;
    UCHAR hum_osr;

    /* Array to map OSR config register value to actual OSR */
    UCHAR osr_sett_to_act_osr[] = { 0, 1, 2, 4, 8, 16 };

    if ((settings != NULL) && (max_delay != NULL))
    {
        /* Mapping osr settings to the actual osr values e.g. 0b101 -> osr X16 */
        if (settings->osr_t <= BME280_OVERSAMPLING_16X)
        {
            temp_osr = osr_sett_to_act_osr[settings->osr_t];
        }
        else
        {
            temp_osr = BME280_OVERSAMPLING_MAX;
        }

        if (settings->osr_p <= BME280_OVERSAMPLING_16X)
        {
            pres_osr = osr_sett_to_act_osr[settings->osr_p];
        }
        else
        {
            pres_osr = BME280_OVERSAMPLING_MAX;
        }

        if (settings->osr_h <= BME280_OVERSAMPLING_16X)
        {
            hum_osr = osr_sett_to_act_osr[settings->osr_h];
        }
        else
        {
            hum_osr = BME280_OVERSAMPLING_MAX;
        }

        (*max_delay) =
            (UINT)((BME280_MEAS_OFFSET + (BME280_MEAS_DUR * temp_osr) +
                        ((BME280_MEAS_DUR * pres_osr) + BME280_PRES_HUM_MEAS_OFFSET) +
                        ((BME280_MEAS_DUR * hum_osr) + BME280_PRES_HUM_MEAS_OFFSET)));
    }
    else
    {
        rslt = BME280_E_NULL_PTR;
    }

    return rslt;
}

/****************************************************************************/
/**\name                        INTERNAL APIs                               */

/*!
 * @brief This internal API sets the oversampling settings for pressure,
 * temperature and humidity in the sensor.
 */
static CHAR set_osr_settings(UCHAR desired_settings, const  bme280_settings *settings,  bme280_dev *dev)
{
    CHAR rslt = BME280_W_INVALID_OSR_MACRO;

    if (desired_settings & BME280_SEL_OSR_HUM)
    {
        rslt = set_osr_humidity_settings(settings, dev);
    }

    if (desired_settings & (BME280_SEL_OSR_PRESS | BME280_SEL_OSR_TEMP))
    {
        rslt = set_osr_press_temp_settings(desired_settings, settings, dev);
    }

    return rslt;
}

/*!
 * @brief This API sets the humidity oversampling settings of the sensor.
 */
static CHAR set_osr_humidity_settings(const  bme280_settings *settings,  bme280_dev *dev)
{
    CHAR rslt;
    UCHAR ctrl_hum;
    UCHAR ctrl_meas;
    UCHAR reg_addr = BME280_REG_CTRL_HUM;

    ctrl_hum = settings->osr_h & BME280_CTRL_HUM_MSK;

    /* Write the humidity control value in the register */
    rslt = bme280_set_regs(&reg_addr, &ctrl_hum, 1, dev);

    /* Humidity related changes will be only effective after a
     * write operation to ctrl_meas register
     */
    if (rslt == BME280_OK)
    {
        reg_addr = BME280_REG_CTRL_MEAS;
        rslt = bme280_get_regs(reg_addr, &ctrl_meas, 1, dev);

        if (rslt == BME280_OK)
        {
            rslt = bme280_set_regs(&reg_addr, &ctrl_meas, 1, dev);
        }
    }

    return rslt;
}

/*!
 * @brief This API sets the pressure and/or temperature oversampling settings
 * in the sensor according to the settings selected by the user.
 */
static CHAR set_osr_press_temp_settings(UCHAR desired_settings,
                                          const  bme280_settings *settings,
                                           bme280_dev *dev)
{
    CHAR rslt;
    UCHAR reg_addr = BME280_REG_CTRL_MEAS;
    UCHAR reg_data;

    rslt = bme280_get_regs(reg_addr, &reg_data, 1, dev);

    if (rslt == BME280_OK)
    {
        if (desired_settings & BME280_SEL_OSR_PRESS)
        {
            fill_osr_press_settings(&reg_data, settings);
        }

        if (desired_settings & BME280_SEL_OSR_TEMP)
        {
            fill_osr_temp_settings(&reg_data, settings);
        }

        /* Write the oversampling settings in the register */
        rslt = bme280_set_regs(&reg_addr, &reg_data, 1, dev);
    }

    return rslt;
}

/*!
 * @brief This internal API sets the filter and/or standby duration settings
 * in the sensor according to the settings selected by the user.
 */
static CHAR set_filter_standby_settings(UCHAR desired_settings,
                                          const  bme280_settings *settings,
                                           bme280_dev *dev)
{
    CHAR rslt;
    UCHAR reg_addr = BME280_REG_CONFIG;
    UCHAR reg_data;

    rslt = bme280_get_regs(reg_addr, &reg_data, 1, dev);

    if (rslt == BME280_OK)
    {
        if (desired_settings & BME280_SEL_FILTER)
        {
            fill_filter_settings(&reg_data, settings);
        }

        if (desired_settings & BME280_SEL_STANDBY)
        {
            fill_standby_settings(&reg_data, settings);
        }

        /* Write the oversampling settings in the register */
        rslt = bme280_set_regs(&reg_addr, &reg_data, 1, dev);
    }

    return rslt;
}

/*!
 * @brief This internal API fills the filter settings provided by the user
 * in the data buffer so as to write in the sensor.
 */
static void fill_filter_settings(UCHAR *reg_data, const  bme280_settings *settings)
{
    *reg_data = BME280_SET_BITS(*reg_data, BME280_FILTER, settings->filter);
}

/*!
 * @brief This internal API fills the standby duration settings provided by
 * the user in the data buffer so as to write in the sensor.
 */
static void fill_standby_settings(UCHAR *reg_data, const  bme280_settings *settings)
{
    *reg_data = BME280_SET_BITS(*reg_data, BME280_STANDBY, settings->standby_time);
}

/*!
 * @brief This internal API fills the pressure oversampling settings provided by
 * the user in the data buffer so as to write in the sensor.
 */
static void fill_osr_press_settings(UCHAR *reg_data, const  bme280_settings *settings)
{
    *reg_data = BME280_SET_BITS(*reg_data, BME280_CTRL_PRESS, settings->osr_p);
}

/*!
 * @brief This internal API fills the temperature oversampling settings
 * provided by the user in the data buffer so as to write in the sensor.
 */
static void fill_osr_temp_settings(UCHAR *reg_data, const  bme280_settings *settings)
{
    *reg_data = BME280_SET_BITS(*reg_data, BME280_CTRL_TEMP, settings->osr_t);
}

/*!
 * @brief This internal API parse the oversampling(pressure, temperature
 * and humidity), filter and standby duration settings and store in the
 * bme280_settings ure.
 */
static void parse_device_settings(const UCHAR *reg_data,  bme280_settings *settings)
{
    settings->osr_h = BME280_GET_BITS_POS_0(reg_data[0], BME280_CTRL_HUM);
    settings->osr_p = BME280_GET_BITS(reg_data[2], BME280_CTRL_PRESS);
    settings->osr_t = BME280_GET_BITS(reg_data[2], BME280_CTRL_TEMP);
    settings->filter = BME280_GET_BITS(reg_data[3], BME280_FILTER);
    settings->standby_time = BME280_GET_BITS(reg_data[3], BME280_STANDBY);
}

/*!
 *  @brief This API is used to parse the pressure, temperature and
 *  humidity data and store it in the bme280_uncomp_data ure instance.
 */
static void parse_sensor_data(const UCHAR *reg_data,  bme280_uncomp_data *uncomp_data)
{
    /* Variables to store the sensor data */
    UINT data_xlsb;
    UINT data_lsb;
    UINT data_msb;

    /* Store the parsed register values for pressure data */
    data_msb = (UINT)reg_data[0] << BME280_12_BIT_SHIFT;
    data_lsb = (UINT)reg_data[1] << BME280_4_BIT_SHIFT;
    data_xlsb = (UINT)reg_data[2] >> BME280_4_BIT_SHIFT;
    uncomp_data->pressure = data_msb | data_lsb | data_xlsb;

    /* Store the parsed register values for temperature data */
    data_msb = (UINT)reg_data[3] << BME280_12_BIT_SHIFT;
    data_lsb = (UINT)reg_data[4] << BME280_4_BIT_SHIFT;
    data_xlsb = (UINT)reg_data[5] >> BME280_4_BIT_SHIFT;
    uncomp_data->temperature = data_msb | data_lsb | data_xlsb;

    /* Store the parsed register values for humidity data */
    data_msb = (UINT)reg_data[6] << BME280_8_BIT_SHIFT;
    data_lsb = (UINT)reg_data[7];
    uncomp_data->humidity = data_msb | data_lsb;
}

/*!
 * @brief This internal API writes the power mode in the sensor.
 */
static CHAR write_power_mode(UCHAR sensor_mode,  bme280_dev *dev)
{
    CHAR rslt;
    UCHAR reg_addr = BME280_REG_PWR_CTRL;

    /* Variable to store the value read from power mode register */
    UCHAR sensor_mode_reg_val;

    /* Read the power mode register */
    rslt = bme280_get_regs(reg_addr, &sensor_mode_reg_val, 1, dev);

    /* Set the power mode */
    if (rslt == BME280_OK)
    {
        sensor_mode_reg_val = BME280_SET_BITS_POS_0(sensor_mode_reg_val, BME280_SENSOR_MODE, sensor_mode);

        /* Write the power mode in the register */
        rslt = bme280_set_regs(&reg_addr, &sensor_mode_reg_val, 1, dev);
    }

    return rslt;
}

/*!
 * @brief This internal API puts the device to sleep mode.
 */
static CHAR put_device_to_sleep( bme280_dev *dev)
{
    CHAR rslt;
    UCHAR reg_data[4];
     bme280_settings settings;

    rslt = bme280_get_regs(BME280_REG_CTRL_HUM, reg_data, 4, dev);

    if (rslt == BME280_OK)
    {
        parse_device_settings(reg_data, &settings);
        rslt = bme280_soft_reset(dev);

        if (rslt == BME280_OK)
        {
            rslt = reload_device_settings(&settings, dev);
        }
    }

    return rslt;
}

/*!
 * @brief This internal API reloads the already existing device settings in
 * the sensor after soft reset.
 */
static CHAR reload_device_settings(const  bme280_settings *settings,  bme280_dev *dev)
{
    CHAR rslt;

    rslt = set_osr_settings(BME280_SEL_ALL_SETTINGS, settings, dev);

    if (rslt == BME280_OK)
    {
        rslt = set_filter_standby_settings(BME280_SEL_ALL_SETTINGS, settings, dev);
    }

    return rslt;
}

#ifdef BME280_DOUBLE_ENABLE

/*!
 * @brief This internal API is used to compensate the raw temperature data and
 * return the compensated temperature data in double data type.
 */
static double compensate_temperature(const  bme280_uncomp_data *uncomp_data,  bme280_calib_data *calib_data)
{
    double var1;
    double var2;
    double temperature;
    double temperature_min = -40;
    double temperature_max = 85;

    var1 = (((double)uncomp_data->temperature) / 16384.0 - ((double)calib_data->dig_t1) / 1024.0);
    var1 = var1 * ((double)calib_data->dig_t2);
    var2 = (((double)uncomp_data->temperature) / 131072.0 - ((double)calib_data->dig_t1) / 8192.0);
    var2 = (var2 * var2) * ((double)calib_data->dig_t3);
    calib_data->t_fine = (INT)(var1 + var2);
    temperature = (var1 + var2) / 5120.0;

    if (temperature < temperature_min)
    {
        temperature = temperature_min;
    }
    else if (temperature > temperature_max)
    {
        temperature = temperature_max;
    }

    return temperature;
}

/*!
 * @brief This internal API is used to compensate the raw pressure data and
 * return the compensated pressure data in double data type.
 */
static double compensate_pressure(const  bme280_uncomp_data *uncomp_data,
                                  const  bme280_calib_data *calib_data)
{
    double var1;
    double var2;
    double var3;
    double pressure;
    double pressure_min = 30000.0;
    double pressure_max = 110000.0;

    var1 = ((double)calib_data->t_fine / 2.0) - 64000.0;
    var2 = var1 * var1 * ((double)calib_data->dig_p6) / 32768.0;
    var2 = var2 + var1 * ((double)calib_data->dig_p5) * 2.0;
    var2 = (var2 / 4.0) + (((double)calib_data->dig_p4) * 65536.0);
    var3 = ((double)calib_data->dig_p3) * var1 * var1 / 524288.0;
    var1 = (var3 + ((double)calib_data->dig_p2) * var1) / 524288.0;
    var1 = (1.0 + var1 / 32768.0) * ((double)calib_data->dig_p1);

    /* Avoid exception caused by division by zero */
    if (var1 > (0.0))
    {
        pressure = 1048576.0 - (double) uncomp_data->pressure;
        pressure = (pressure - (var2 / 4096.0)) * 6250.0 / var1;
        var1 = ((double)calib_data->dig_p9) * pressure * pressure / 2147483648.0;
        var2 = pressure * ((double)calib_data->dig_p8) / 32768.0;
        pressure = pressure + (var1 + var2 + ((double)calib_data->dig_p7)) / 16.0;

        if (pressure < pressure_min)
        {
            pressure = pressure_min;
        }
        else if (pressure > pressure_max)
        {
            pressure = pressure_max;
        }
    }
    else /* Invalid case */
    {
        pressure = pressure_min;
    }

    return pressure;
}

/*!
 * @brief This internal API is used to compensate the raw humidity data and
 * return the compensated humidity data in double data type.
 */
static double compensate_humidity(const  bme280_uncomp_data *uncomp_data,
                                  const  bme280_calib_data *calib_data)
{
    double humidity;
    double humidity_min = 0.0;
    double humidity_max = 100.0;
    double var1;
    double var2;
    double var3;
    double var4;
    double var5;
    double var6;

    var1 = ((double)calib_data->t_fine) - 76800.0;
    var2 = (((double)calib_data->dig_h4) * 64.0 + (((double)calib_data->dig_h5) / 16384.0) * var1);
    var3 = uncomp_data->humidity - var2;
    var4 = ((double)calib_data->dig_h2) / 65536.0;
    var5 = (1.0 + (((double)calib_data->dig_h3) / 67108864.0) * var1);
    var6 = 1.0 + (((double)calib_data->dig_h6) / 67108864.0) * var1 * var5;
    var6 = var3 * var4 * (var5 * var6);
    humidity = var6 * (1.0 - ((double)calib_data->dig_h1) * var6 / 524288.0);

    if (humidity > humidity_max)
    {
        humidity = humidity_max;
    }
    else if (humidity < humidity_min)
    {
        humidity = humidity_min;
    }

    return humidity;
}

#else

/*!
 * @brief This internal API is used to compensate the raw temperature data and
 * return the compensated temperature data in integer data type.
 */
static INT compensate_temperature(const  bme280_uncomp_data *uncomp_data,
                                       bme280_calib_data *calib_data)
{
    INT var1;
    INT var2;
    INT temperature;
    INT temperature_min = -4000;
    INT temperature_max = 8500;

    var1 = (INT)((uncomp_data->temperature / 8) - ((INT)calib_data->dig_t1 * 2));
    var1 = (var1 * ((INT)calib_data->dig_t2)) / 2048;
    var2 = (INT)((uncomp_data->temperature / 16) - ((INT)calib_data->dig_t1));
    var2 = (((var2 * var2) / 4096) * ((INT)calib_data->dig_t3)) / 16384;
    calib_data->t_fine = var1 + var2;
    temperature = (calib_data->t_fine * 5 + 128) / 256;

    if (temperature < temperature_min)
    {
        temperature = temperature_min;
    }
    else if (temperature > temperature_max)
    {
        temperature = temperature_max;
    }

    return temperature;
}
#ifndef BME280_32BIT_ENABLE /* 64 bit compensation for pressure data */

/*!
 * @brief This internal API is used to compensate the raw pressure data and
 * return the compensated pressure data in integer data type with higher
 * accuracy.
 */
static UINT compensate_pressure(const  bme280_uncomp_data *uncomp_data,
                                    const  bme280_calib_data *calib_data)
{
    int64_t var1;
    int64_t var2;
    int64_t var3;
    int64_t var4;
    UINT pressure;
    UINT pressure_min = 3000000;
    UINT pressure_max = 11000000;

    var1 = ((int64_t)calib_data->t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)calib_data->dig_p6;
    var2 = var2 + ((var1 * (int64_t)calib_data->dig_p5) * 131072);
    var2 = var2 + (((int64_t)calib_data->dig_p4) * 34359738368);
    var1 = ((var1 * var1 * (int64_t)calib_data->dig_p3) / 256) + ((var1 * ((int64_t)calib_data->dig_p2) * 4096));
    var3 = ((int64_t)1) * 140737488355328;
    var1 = (var3 + var1) * ((int64_t)calib_data->dig_p1) / 8589934592;

    /* To avoid divide by zero exception */
    if (var1 != 0)
    {
        var4 = 1048576 - uncomp_data->pressure;
        var4 = (((var4 * INT64_C(2147483648)) - var2) * 3125) / var1;
        var1 = (((int64_t)calib_data->dig_p9) * (var4 / 8192) * (var4 / 8192)) / 33554432;
        var2 = (((int64_t)calib_data->dig_p8) * var4) / 524288;
        var4 = ((var4 + var1 + var2) / 256) + (((int64_t)calib_data->dig_p7) * 16);
        pressure = (UINT)(((var4 / 2) * 100) / 128);

        if (pressure < pressure_min)
        {
            pressure = pressure_min;
        }
        else if (pressure > pressure_max)
        {
            pressure = pressure_max;
        }
    }
    else
    {
        pressure = pressure_min;
    }

    return pressure;
}
#else /* 32 bit compensation for pressure data */

/*!
 * @brief This internal API is used to compensate the raw pressure data and
 * return the compensated pressure data in integer data type.
 */
static UINT compensate_pressure(const  bme280_uncomp_data *uncomp_data,
                                    const  bme280_calib_data *calib_data)
{
    INT var1;
    INT var2;
    INT var3;
    INT var4;
    UINT var5;
    UINT pressure;
    UINT pressure_min = 30000;
    UINT pressure_max = 110000;

    var1 = (((INT)calib_data->t_fine) / 2) - (INT)64000;
    var2 = (((var1 / 4) * (var1 / 4)) / 2048) * ((INT)calib_data->dig_p6);
    var2 = var2 + ((var1 * ((INT)calib_data->dig_p5)) * 2);
    var2 = (var2 / 4) + (((INT)calib_data->dig_p4) * 65536);
    var3 = (calib_data->dig_p3 * (((var1 / 4) * (var1 / 4)) / 8192)) / 8;
    var4 = (((INT)calib_data->dig_p2) * var1) / 2;
    var1 = (var3 + var4) / 262144;
    var1 = (((32768 + var1)) * ((INT)calib_data->dig_p1)) / 32768;

    /* Avoid exception caused by division by zero */
    if (var1)
    {
        var5 = (UINT)((UINT)1048576) - uncomp_data->pressure;
        pressure = ((UINT)(var5 - (UINT)(var2 / 4096))) * 3125;

        if (pressure < 0x80000000)
        {
            pressure = (pressure << 1) / ((UINT)var1);
        }
        else
        {
            pressure = (pressure / (UINT)var1) * 2;
        }

        var1 = (((INT)calib_data->dig_p9) * ((INT)(((pressure / 8) * (pressure / 8)) / 8192))) / 4096;
        var2 = (((INT)(pressure / 4)) * ((INT)calib_data->dig_p8)) / 8192;
        pressure = (UINT)((INT)pressure + ((var1 + var2 + calib_data->dig_p7) / 16));

        if (pressure < pressure_min)
        {
            pressure = pressure_min;
        }
        else if (pressure > pressure_max)
        {
            pressure = pressure_max;
        }
    }
    else
    {
        pressure = pressure_min;
    }

    return pressure;
}
#endif

/*!
 * @brief This internal API is used to compensate the raw humidity data and
 * return the compensated humidity data in integer data type.
 */
static UINT compensate_humidity(const  bme280_uncomp_data *uncomp_data,
                                    const  bme280_calib_data *calib_data)
{
    INT var1;
    INT var2;
    INT var3;
    INT var4;
    INT var5;
    UINT humidity;
    UINT humidity_max = 102400;

    var1 = calib_data->t_fine - ((INT)76800);
    var2 = (INT)(uncomp_data->humidity * 16384);
    var3 = (INT)(((INT)calib_data->dig_h4) * 1048576);
    var4 = ((INT)calib_data->dig_h5) * var1;
    var5 = (((var2 - var3) - var4) + (INT)16384) / 32768;
    var2 = (var1 * ((INT)calib_data->dig_h6)) / 1024;
    var3 = (var1 * ((INT)calib_data->dig_h3)) / 2048;
    var4 = ((var2 * (var3 + (INT)32768)) / 1024) + (INT)2097152;
    var2 = ((var4 * ((INT)calib_data->dig_h2)) + 8192) / 16384;
    var3 = var5 * var2;
    var4 = ((var3 / 32768) * (var3 / 32768)) / 128;
    var5 = var3 - ((var4 * ((INT)calib_data->dig_h1)) / 16);
    var5 = (var5 < 0 ? 0 : var5);
    var5 = (var5 > 419430400 ? 419430400 : var5);
    humidity = (UINT)(var5 / 4096);

    if (humidity > humidity_max)
    {
        humidity = humidity_max;
    }

    return humidity;
}
#endif

/*!
 * @brief This internal API reads the calibration data from the sensor, parse
 * it and store in the device ure.
 */
static CHAR get_calib_data(bme280_dev *dev)
{
    CHAR rslt;
    UCHAR reg_addr = BME280_REG_TEMP_PRESS_CALIB_DATA;

    /* Array to store calibration data */
    UCHAR calib_data[BME280_LEN_TEMP_PRESS_CALIB_DATA] = { 0 };

    /* Read the calibration data from the sensor */
    rslt = bme280_get_regs(reg_addr, calib_data, BME280_LEN_TEMP_PRESS_CALIB_DATA, dev);

    if (rslt == BME280_OK)
    {
        /* Parse temperature and pressure calibration data and store
         * it in device ure
         */
        parse_temp_press_calib_data(calib_data, dev);
        reg_addr = BME280_REG_HUMIDITY_CALIB_DATA;

        /* Read the humidity calibration data from the sensor */
        rslt = bme280_get_regs(reg_addr, calib_data, BME280_LEN_HUMIDITY_CALIB_DATA, dev);

        if (rslt == BME280_OK)
        {
            /* Parse humidity calibration data and store it in
             * device ure
             */
            parse_humidity_calib_data(calib_data, dev);
        }
    }

    return rslt;
}

/*!
 * @brief This internal API interleaves the register address between the
 * register data buffer for burst write operation.
 */
static void interleave_reg_addr(const UCHAR *reg_addr, UCHAR *temp_buff, const UCHAR *reg_data, UINT len)
{
    UINT index;

    for (index = 1; index < len; index++)
    {
        temp_buff[(index * 2) - 1] = reg_addr[index];
        temp_buff[index * 2] = reg_data[index];
    }
}

/*!
 *  @brief This internal API is used to parse the temperature and
 *  pressure calibration data and store it in device ure.
 */
static void parse_temp_press_calib_data(const UCHAR *reg_data, bme280_dev *dev)
{
     bme280_calib_data *calib_data = &dev->calib_data;

    calib_data->dig_t1 = BME280_CONCAT_BYTES(reg_data[1], reg_data[0]);
    calib_data->dig_t2 = (SHORT)BME280_CONCAT_BYTES(reg_data[3], reg_data[2]);
    calib_data->dig_t3 = (SHORT)BME280_CONCAT_BYTES(reg_data[5], reg_data[4]);
    calib_data->dig_p1 = BME280_CONCAT_BYTES(reg_data[7], reg_data[6]);
    calib_data->dig_p2 = (SHORT)BME280_CONCAT_BYTES(reg_data[9], reg_data[8]);
    calib_data->dig_p3 = (SHORT)BME280_CONCAT_BYTES(reg_data[11], reg_data[10]);
    calib_data->dig_p4 = (SHORT)BME280_CONCAT_BYTES(reg_data[13], reg_data[12]);
    calib_data->dig_p5 = (SHORT)BME280_CONCAT_BYTES(reg_data[15], reg_data[14]);
    calib_data->dig_p6 = (SHORT)BME280_CONCAT_BYTES(reg_data[17], reg_data[16]);
    calib_data->dig_p7 = (SHORT)BME280_CONCAT_BYTES(reg_data[19], reg_data[18]);
    calib_data->dig_p8 = (SHORT)BME280_CONCAT_BYTES(reg_data[21], reg_data[20]);
    calib_data->dig_p9 = (SHORT)BME280_CONCAT_BYTES(reg_data[23], reg_data[22]);
    calib_data->dig_h1 = reg_data[25];
}

/*!
 *  @brief This internal API is used to parse the humidity calibration data
 *  and store it in device ure.
 */
static void parse_humidity_calib_data(const UCHAR *reg_data, bme280_dev *dev)
{
    bme280_calib_data *calib_data = &dev->calib_data;
    SHORT dig_h4_lsb;
    SHORT dig_h4_msb;
    SHORT dig_h5_lsb;
    SHORT dig_h5_msb;

    calib_data->dig_h2 = (SHORT)BME280_CONCAT_BYTES(reg_data[1], reg_data[0]);
    calib_data->dig_h3 = reg_data[2];
    dig_h4_msb = (SHORT)(CHAR)reg_data[3] * 16;
    dig_h4_lsb = (SHORT)(reg_data[4] & 0x0F);
    calib_data->dig_h4 = dig_h4_msb | dig_h4_lsb;
    dig_h5_msb = (SHORT)(CHAR)reg_data[5] * 16;
    dig_h5_lsb = (SHORT)(reg_data[4] >> 4);
    calib_data->dig_h5 = dig_h5_msb | dig_h5_lsb;
    calib_data->dig_h6 = (CHAR)reg_data[6];
}

/*!
 * @brief This internal API is used to identify the settings which the user
 * wants to modify in the sensor.
 */
static UCHAR are_settings_changed(UCHAR sub_settings, UCHAR desired_settings)
{
    UCHAR settings_changed = FALSE;

    if (sub_settings & desired_settings)
    {
        /* User wants to modify this particular settings */
        settings_changed = TRUE;
    }
    else
    {
        /* User don't want to modify this particular settings */
        settings_changed = FALSE;
    }

    return settings_changed;
}

/*!
 * @brief This internal API is used to validate the device ure pointer for
 * null conditions.
 */
static CHAR null_ptr_check(const bme280_dev *dev)
{
    CHAR rslt;

    if ((dev == NULL) || (dev->read == NULL) || (dev->write == NULL) || (dev->delay_us == NULL))
    {
        /* Device ure pointer is not valid */
        rslt = BME280_E_NULL_PTR;
    }
    else
    {
        /* Device Structure is fine */
        rslt = BME280_OK;
    }

    return rslt;
}
