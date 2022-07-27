/**
 * Copyright (c) 2015 - present LibDriver All rights reserved
 * 
 * The MIT License (MIT)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE. 
 *
 * @file      driver_mpu6050_basic.h
 * @brief     driver mpu6050 basic header file
 * @version   1.0.0
 * @author    Shifeng Li
 * @date      2022-06-30
 *
 * <h3>history</h3>
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/06/30  <td>1.0      <td>Shifeng Li  <td>first upload
 * </table>
 */

#ifndef DRIVER_MPU6050_BASIC_H
#define DRIVER_MPU6050_BASIC_H

#include "driver_mpu6050_interface.h"

#ifdef __cplusplus
extern "C"{
#endif

/**
 * @defgroup mpu6050_example_driver mpu6050 example driver function
 * @brief    mpu6050 example driver modules
 * @ingroup  mpu6050_driver
 * @{
 */

/**
 * @brief mpu6050 basic example default definition
 */
#define MPU6050_BASIC_DEFAULT_CLOCK_SOURCE                   MPU6050_CLOCK_SOURCE_PLL_X_GYRO           /**< gyro pll x */
#define MPU6050_BASIC_DEFAULT_RATE                           50                                        /**< 50Hz */
#define MPU6050_BASIC_DEFAULT_LOW_PASS_FILTER                MPU6050_LOW_PASS_FILTER_3                 /**< low pass filter 3 */
#define MPU6050_BASIC_DEFAULT_CYCLE_WAKE_UP                  MPU6050_BOOL_FALSE                        /**< disable cycle wake up */
#define MPU6050_BASIC_DEFAULT_WAKE_UP_FREQUENCY              MPU6050_WAKE_UP_FREQUENCY_1P25_HZ         /**< 1.25Hz */
#define MPU6050_BASIC_DEFAULT_INTERRUPT_PIN_LEVEL            MPU6050_PIN_LEVEL_LOW                     /**< low level */
#define MPU6050_BASIC_DEFAULT_INTERRUPT_PIN_TYPE             MPU6050_PIN_TYPE_PUSH_PULL                /**< push pull */
#define MPU6050_BASIC_DEFAULT_ACCELEROMETER_RANGE            MPU6050_ACCELEROMETER_RANGE_2G            /**< 2g */
#define MPU6050_BASIC_DEFAULT_GYROSCOPE_RANGE                MPU6050_GYROSCOPE_RANGE_2000DPS           /**< 2000dps */
#define MPU6050_BASIC_DEFAULT_INTERRUPT_MOTION               MPU6050_BOOL_FALSE                        /**< disable motion */
#define MPU6050_BASIC_DEFAULT_INTERRUPT_FIFO_OVERFLOW        MPU6050_BOOL_FALSE                        /**< disable fifo overflow */
#define MPU6050_BASIC_DEFAULT_INTERRUPT_DMP                  MPU6050_BOOL_FALSE                        /**< disable dmp */
#define MPU6050_BASIC_DEFAULT_INTERRUPT_I2C_MAST             MPU6050_BOOL_FALSE                        /**< disable i2c master */
#define MPU6050_BASIC_DEFAULT_INTERRUPT_DATA_READY           MPU6050_BOOL_FALSE                        /**< disable data ready */
#define MPU6050_BASIC_DEFAULT_INTERRUPT_LATCH                MPU6050_BOOL_TRUE                         /**< enable latch */
#define MPU6050_BASIC_DEFAULT_INTERRUPT_READ_CLEAR           MPU6050_BOOL_TRUE                         /**< enable interrupt read clear */
#define MPU6050_BASIC_DEFAULT_EXTERN_SYNC                    MPU6050_EXTERN_SYNC_INPUT_DISABLED        /**< extern sync input disable */
#define MPU6050_BASIC_DEFAULT_FSYNC_INTERRUPT                MPU6050_BOOL_FALSE                        /**< disable fsync interrupt */
#define MPU6050_BASIC_DEFAULT_FSYNC_INTERRUPT_LEVEL          MPU6050_PIN_LEVEL_LOW                     /**< low level */
#define MPU6050_BASIC_DEFAULT_IIC_MASTER                     MPU6050_BOOL_FALSE                        /**< disable iic master */
#define MPU6050_BASIC_DEFAULT_IIC_BYPASS                     MPU6050_BOOL_FALSE                        /**< disable iic bypass */

/**
 * @brief     basic example init
 * @param[in] addr_pin is the iic device address
 * @return    status code
 *            - 0 success
 *            - 1 init failed
 * @note      none
 */
uint8_t mpu6050_basic_init(mpu6050_address_t addr_pin);

/**
 * @brief  basic example deinit
 * @return status code
 *         - 0 success
 *         - 1 deinit failed
 * @note   none
 */
uint8_t mpu6050_basic_deinit(void);

/**
 * @brief      basic example read
 * @param[out] *g points to a converted data buffer
 * @param[out] *dps points to a converted data buffer
 * @return     status code
 *             - 0 success
 *             - 1 read failed
 * @note       none
 */
uint8_t mpu6050_basic_read(float g[3], float dps[3]);

/**
 * @brief      basic example read temperature
 * @param[out] *degrees points to a converted data buffer
 * @return     status code
 *             - 0 success
 *             - 1 read temperature failed
 * @note       none
 */
uint8_t mpu6050_basic_read_temperature(float *degrees);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif
