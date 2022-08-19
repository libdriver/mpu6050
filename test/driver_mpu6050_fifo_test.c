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
 * @file      driver_mpu6050_fifo_test.c
 * @brief     driver mpu6050 fifo test source file
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

#include "driver_mpu6050_fifo_test.h"

static mpu6050_handle_t gs_handle;        /**< mpu6050 handle */
static int16_t gs_accel_raw[80][3];       /**< accel raw */
static float gs_accel_g[80][3];           /**< accel g */
static int16_t gs_gyro_raw[80][3];        /**< gyro raw */
static float gs_gyro_dps[80][3];          /**< gyro dps */
static uint16_t gs_len;                   /**< len */

/**
 * @brief  fifo test irq
 * @return status code
 *         - 0 success
 *         - 1 run failed
 * @note   none
 */
uint8_t mpu6050_fifo_test_irq_handler(void)
{
    if (mpu6050_irq_handler(&gs_handle) != 0)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

/**
 * @brief     fifo test
 * @param[in] addr is the iic device address
 * @param[in] times is the test times
 * @return    status code
 *            - 0 success
 *            - 1 test failed
 * @note      none
 */
uint8_t mpu6050_fifo_test(mpu6050_address_t addr, uint32_t times)
{
    uint8_t res;
    uint32_t i;
    mpu6050_info_t info;
    
    /* link interface function */
    DRIVER_MPU6050_LINK_INIT(&gs_handle, mpu6050_handle_t);
    DRIVER_MPU6050_LINK_IIC_INIT(&gs_handle, mpu6050_interface_iic_init);
    DRIVER_MPU6050_LINK_IIC_DEINIT(&gs_handle, mpu6050_interface_iic_deinit);
    DRIVER_MPU6050_LINK_IIC_READ(&gs_handle, mpu6050_interface_iic_read);
    DRIVER_MPU6050_LINK_IIC_WRITE(&gs_handle, mpu6050_interface_iic_write);
    DRIVER_MPU6050_LINK_DELAY_MS(&gs_handle, mpu6050_interface_delay_ms);
    DRIVER_MPU6050_LINK_DEBUG_PRINT(&gs_handle, mpu6050_interface_debug_print);
    DRIVER_MPU6050_LINK_RECEIVE_CALLBACK(&gs_handle, mpu6050_interface_receive_callback);
    
    /* get information */
    res = mpu6050_info(&info);
    if (res != 0)
    {
        mpu6050_interface_debug_print("mpu6050: get info failed.\n");
       
        return 1;
    }
    else
    {
        /* print chip info */
        mpu6050_interface_debug_print("mpu6050: chip is %s.\n", info.chip_name);
        mpu6050_interface_debug_print("mpu6050: manufacturer is %s.\n", info.manufacturer_name);
        mpu6050_interface_debug_print("mpu6050: interface is %s.\n", info.interface);
        mpu6050_interface_debug_print("mpu6050: driver version is %d.%d.\n", info.driver_version / 1000, (info.driver_version % 1000) / 100);
        mpu6050_interface_debug_print("mpu6050: min supply voltage is %0.1fV.\n", info.supply_voltage_min_v);
        mpu6050_interface_debug_print("mpu6050: max supply voltage is %0.1fV.\n", info.supply_voltage_max_v);
        mpu6050_interface_debug_print("mpu6050: max current is %0.2fmA.\n", info.max_current_ma);
        mpu6050_interface_debug_print("mpu6050: max temperature is %0.1fC.\n", info.temperature_max);
        mpu6050_interface_debug_print("mpu6050: min temperature is %0.1fC.\n", info.temperature_min);
    }
    
    /* start fifo test */
    mpu6050_interface_debug_print("mpu6050: start fifo test.\n");
    
    /* set the addr pin */
    res = mpu6050_set_addr_pin(&gs_handle, addr);
    if (res != 0)
    {
        mpu6050_interface_debug_print("mpu6050: set addr pin failed.\n");
       
        return 1;
    }
    
    /* init */
    res = mpu6050_init(&gs_handle);
    if (res != 0)
    {
        mpu6050_interface_debug_print("mpu6050: init failed.\n");
       
        return 1;
    }
    
    /* delay 100 ms */
    mpu6050_interface_delay_ms(100);
    
    /* disable sleep */
    res = mpu6050_set_sleep(&gs_handle, MPU6050_BOOL_FALSE);
    if (res != 0)
    {
        mpu6050_interface_debug_print("mpu6050: set sleep failed.\n");
        (void)mpu6050_deinit(&gs_handle);
       
        return 1;
    }
    
    /* set pll x */
    res = mpu6050_set_clock_source(&gs_handle, MPU6050_CLOCK_SOURCE_PLL_X_GYRO);
    if (res != 0)
    {
        mpu6050_interface_debug_print("mpu6050: set clock source failed.\n");
        (void)mpu6050_deinit(&gs_handle);
       
        return 1;
    }
    
    /* set 50Hz */
    res = mpu6050_set_sample_rate_divider(&gs_handle, 1000 / (50 - 1));
    if (res != 0)
    {
        mpu6050_interface_debug_print("mpu6050: set sample rate divider failed.\n");
        (void)mpu6050_deinit(&gs_handle);
       
        return 1;
    }
    
    /* ±2g */
    res = mpu6050_set_accelerometer_range(&gs_handle, MPU6050_ACCELEROMETER_RANGE_2G);
    if (res != 0)
    {
        mpu6050_interface_debug_print("mpu6050: set accelerometer range failed.\n");
        (void)mpu6050_deinit(&gs_handle);
       
        return 1;
    }
    
    /* ±2000dps */
    res = mpu6050_set_gyroscope_range(&gs_handle, MPU6050_GYROSCOPE_RANGE_2000DPS);
    if (res != 0)
    {
        mpu6050_interface_debug_print("mpu6050: set gyroscope range failed.\n");
        (void)mpu6050_deinit(&gs_handle);
       
        return 1;
    }
    
    /* set low pass filter 3 */
    res = mpu6050_set_low_pass_filter(&gs_handle, MPU6050_LOW_PASS_FILTER_3);
    if (res != 0)
    {
        mpu6050_interface_debug_print("mpu6050: set low pass filter failed.\n");
        (void)mpu6050_deinit(&gs_handle);
       
        return 1;
    }
    
    /* enable temperature sensor */
    res = mpu6050_set_temperature_sensor(&gs_handle, MPU6050_BOOL_TRUE);
    if (res != 0)
    {
        mpu6050_interface_debug_print("mpu6050: set temperature sensor failed.\n");
        (void)mpu6050_deinit(&gs_handle);
       
        return 1;
    }
    
    /* disable cycle wake up */
    res = mpu6050_set_cycle_wake_up(&gs_handle, MPU6050_BOOL_FALSE);
    if (res != 0)
    {
        mpu6050_interface_debug_print("mpu6050: set cycle wake up failed.\n");
        (void)mpu6050_deinit(&gs_handle);
       
        return 1;
    }
    
    /* set wake up frequency 1.25Hz */
    res = mpu6050_set_wake_up_frequency(&gs_handle, MPU6050_WAKE_UP_FREQUENCY_1P25_HZ);
    if (res != 0)
    {
        mpu6050_interface_debug_print("mpu6050: set wake up frequency failed.\n");
        (void)mpu6050_deinit(&gs_handle);
       
        return 1;
    }
    
    /* enable acc x */
    res = mpu6050_set_standby_mode(&gs_handle, MPU6050_SOURCE_ACC_X, MPU6050_BOOL_FALSE);
    if (res != 0)
    {
        mpu6050_interface_debug_print("mpu6050: set standby mode failed.\n");
        (void)mpu6050_deinit(&gs_handle);
       
        return 1;
    }
    
    /* enable acc y */
    res = mpu6050_set_standby_mode(&gs_handle, MPU6050_SOURCE_ACC_Y, MPU6050_BOOL_FALSE);
    if (res != 0)
    {
        mpu6050_interface_debug_print("mpu6050: set standby mode failed.\n");
        (void)mpu6050_deinit(&gs_handle);
       
        return 1;
    }
    
    /* enable acc z */
    res = mpu6050_set_standby_mode(&gs_handle, MPU6050_SOURCE_ACC_Z, MPU6050_BOOL_FALSE);
    if (res != 0)
    {
        mpu6050_interface_debug_print("mpu6050: set standby mode failed.\n");
        (void)mpu6050_deinit(&gs_handle);
       
        return 1;
    }
    
    /* enable gyro x */
    res = mpu6050_set_standby_mode(&gs_handle, MPU6050_SOURCE_GYRO_X, MPU6050_BOOL_FALSE);
    if (res != 0)
    {
        mpu6050_interface_debug_print("mpu6050: set standby mode failed.\n");
        (void)mpu6050_deinit(&gs_handle);
       
        return 1;
    }
    
    /* enable gyro y */
    res = mpu6050_set_standby_mode(&gs_handle, MPU6050_SOURCE_GYRO_Y, MPU6050_BOOL_FALSE);
    if (res != 0)
    {
        mpu6050_interface_debug_print("mpu6050: set standby mode failed.\n");
        (void)mpu6050_deinit(&gs_handle);
       
        return 1;
    }
    
    /* enable gyro z */
    res = mpu6050_set_standby_mode(&gs_handle, MPU6050_SOURCE_GYRO_Z, MPU6050_BOOL_FALSE);
    if (res != 0)
    {
        mpu6050_interface_debug_print("mpu6050: set standby mode failed.\n");
        (void)mpu6050_deinit(&gs_handle);
       
        return 1;
    }
    
    /* disable gyroscope x test */
    res = mpu6050_set_gyroscope_test(&gs_handle, MPU6050_AXIS_X, MPU6050_BOOL_FALSE);
    if (res != 0)
    {
        mpu6050_interface_debug_print("mpu6050: set gyroscope test failed.\n");
        (void)mpu6050_deinit(&gs_handle);
       
        return 1;
    }
    
    /* disable gyroscope y test */
    res = mpu6050_set_gyroscope_test(&gs_handle, MPU6050_AXIS_Y, MPU6050_BOOL_FALSE);
    if (res != 0)
    {
        mpu6050_interface_debug_print("mpu6050: set gyroscope test failed.\n");
        (void)mpu6050_deinit(&gs_handle);
       
        return 1;
    }
    
    /* disable gyroscope z test */
    res = mpu6050_set_gyroscope_test(&gs_handle, MPU6050_AXIS_Z, MPU6050_BOOL_FALSE);
    if (res != 0)
    {
        mpu6050_interface_debug_print("mpu6050: set gyroscope test failed.\n");
        (void)mpu6050_deinit(&gs_handle);
       
        return 1;
    }
    
    /* disable accelerometer x test */
    res = mpu6050_set_accelerometer_test(&gs_handle, MPU6050_AXIS_X, MPU6050_BOOL_FALSE);
    if (res != 0)
    {
        mpu6050_interface_debug_print("mpu6050: set accelerometer test failed.\n");
        (void)mpu6050_deinit(&gs_handle);
       
        return 1;
    }
    
    /* disable accelerometer y test */
    res = mpu6050_set_accelerometer_test(&gs_handle, MPU6050_AXIS_Y, MPU6050_BOOL_FALSE);
    if (res != 0)
    {
        mpu6050_interface_debug_print("mpu6050: set accelerometer test failed.\n");
        (void)mpu6050_deinit(&gs_handle);
       
        return 1;
    }
    
    /* disable accelerometer z test */
    res = mpu6050_set_accelerometer_test(&gs_handle, MPU6050_AXIS_Z, MPU6050_BOOL_FALSE);
    if (res != 0)
    {
        mpu6050_interface_debug_print("mpu6050: set accelerometer test failed.\n");
        (void)mpu6050_deinit(&gs_handle);
       
        return 1;
    }
    
    /* disable temp fifo */
    res = mpu6050_set_fifo_enable(&gs_handle, MPU6050_FIFO_TEMP, MPU6050_BOOL_FALSE);
    if (res != 0)
    {
        mpu6050_interface_debug_print("mpu6050: set fifo enable failed.\n");
        (void)mpu6050_deinit(&gs_handle);
       
        return 1;
    }
    
    /* enable xg fifo */
    res = mpu6050_set_fifo_enable(&gs_handle, MPU6050_FIFO_XG, MPU6050_BOOL_TRUE);
    if (res != 0)
    {
        mpu6050_interface_debug_print("mpu6050: set fifo enable failed.\n");
        (void)mpu6050_deinit(&gs_handle);
       
        return 1;
    }
    
    /* enable yg fifo */
    res = mpu6050_set_fifo_enable(&gs_handle, MPU6050_FIFO_YG, MPU6050_BOOL_TRUE);
    if (res != 0)
    {
        mpu6050_interface_debug_print("mpu6050: set fifo enable failed.\n");
        (void)mpu6050_deinit(&gs_handle);
       
        return 1;
    }
    
    /* enable zg fifo */
    res = mpu6050_set_fifo_enable(&gs_handle, MPU6050_FIFO_ZG, MPU6050_BOOL_TRUE);
    if (res != 0)
    {
        mpu6050_interface_debug_print("mpu6050: set fifo enable failed.\n");
        (void)mpu6050_deinit(&gs_handle);
       
        return 1;
    }
    
    /* enable accel fifo */
    res = mpu6050_set_fifo_enable(&gs_handle, MPU6050_FIFO_ACCEL, MPU6050_BOOL_TRUE);
    if (res != 0)
    {
        mpu6050_interface_debug_print("mpu6050: set fifo enable failed.\n");
        (void)mpu6050_deinit(&gs_handle);
       
        return 1;
    }
    
    /* enable fifo */
    res = mpu6050_set_fifo(&gs_handle, MPU6050_BOOL_TRUE);
    if (res != 0)
    {
        mpu6050_interface_debug_print("mpu6050: set fifo failed.\n");
        (void)mpu6050_deinit(&gs_handle);
       
        return 1;
    }
    
    /* set interrupt level low */
    res = mpu6050_set_interrupt_level(&gs_handle, MPU6050_PIN_LEVEL_LOW);
    if (res != 0)
    {
        mpu6050_interface_debug_print("mpu6050: set interrupt level failed.\n");
        (void)mpu6050_deinit(&gs_handle);
       
        return 1;
    }
    
    /* push pull */
    res = mpu6050_set_interrupt_pin_type(&gs_handle, MPU6050_PIN_TYPE_PUSH_PULL);
    if (res != 0)
    {
        mpu6050_interface_debug_print("mpu6050: set interrupt pin type failed.\n");
        (void)mpu6050_deinit(&gs_handle);
       
        return 1;
    }
    
    /* disable motion */
    res = mpu6050_set_interrupt(&gs_handle, MPU6050_INTERRUPT_MOTION, MPU6050_BOOL_FALSE);
    if (res != 0)
    {
        mpu6050_interface_debug_print("mpu6050: set interrupt failed.\n");
        (void)mpu6050_deinit(&gs_handle);
       
        return 1;
    }
    
    /* enable fifo overflow */
    res = mpu6050_set_interrupt(&gs_handle, MPU6050_INTERRUPT_FIFO_OVERFLOW, MPU6050_BOOL_TRUE);
    if (res != 0)
    {
        mpu6050_interface_debug_print("mpu6050: set interrupt failed.\n");
        (void)mpu6050_deinit(&gs_handle);
       
        return 1;
    }
    
    /* disable dmp interrupt */
    res = mpu6050_set_interrupt(&gs_handle, MPU6050_INTERRUPT_DMP, MPU6050_BOOL_FALSE);
    if (res != 0)
    {
        mpu6050_interface_debug_print("mpu6050: set interrupt failed.\n");
        (void)mpu6050_deinit(&gs_handle);
       
        return 1;
    }
    
    /* disable i2c master */
    res = mpu6050_set_interrupt(&gs_handle, MPU6050_INTERRUPT_I2C_MAST, MPU6050_BOOL_FALSE);
    if (res != 0)
    {
        mpu6050_interface_debug_print("mpu6050: set interrupt failed.\n");
        (void)mpu6050_deinit(&gs_handle);
       
        return 1;
    }
    
    /* disable data ready */
    res = mpu6050_set_interrupt(&gs_handle, MPU6050_INTERRUPT_DATA_READY, MPU6050_BOOL_FALSE);
    if (res != 0)
    {
        mpu6050_interface_debug_print("mpu6050: set interrupt failed.\n");
        (void)mpu6050_deinit(&gs_handle);
       
        return 1;
    }
    
    /* enable latch */
    res = mpu6050_set_interrupt_latch(&gs_handle, MPU6050_BOOL_TRUE);
    if (res != 0)
    {
        mpu6050_interface_debug_print("mpu6050: set interrupt latch failed.\n");
        (void)mpu6050_deinit(&gs_handle);
       
        return 1;
    }
    
    /* enable interrupt read clear */
    res = mpu6050_set_interrupt_read_clear(&gs_handle, MPU6050_BOOL_TRUE);
    if (res != 0)
    {
        mpu6050_interface_debug_print("mpu6050: set interrupt read clear failed.\n");
        (void)mpu6050_deinit(&gs_handle);
       
        return 1;
    }
    
    /* disable sync input */
    res = mpu6050_set_extern_sync(&gs_handle, MPU6050_EXTERN_SYNC_INPUT_DISABLED);
    if (res != 0)
    {
        mpu6050_interface_debug_print("mpu6050: set extern sync failed.\n");
        (void)mpu6050_deinit(&gs_handle);
       
        return 1;
    }
    
    /* disable fsync interrupt */
    res = mpu6050_set_fsync_interrupt(&gs_handle, MPU6050_BOOL_FALSE);
    if (res != 0)
    {
        mpu6050_interface_debug_print("mpu6050: set fsync interrupt failed.\n");
        (void)mpu6050_deinit(&gs_handle);
       
        return 1;
    }
    
    /* fsync interrupt level low */
    res = mpu6050_set_fsync_interrupt_level(&gs_handle, MPU6050_PIN_LEVEL_LOW);
    if (res != 0)
    {
        mpu6050_interface_debug_print("mpu6050: set fsync interrupt level failed.\n");
        (void)mpu6050_deinit(&gs_handle);
       
        return 1;
    }
    
    /* disable iic master */
    res = mpu6050_set_iic_master(&gs_handle, MPU6050_BOOL_FALSE);
    if (res != 0)
    {
        mpu6050_interface_debug_print("mpu6050: set iic master failed.\n");
        (void)mpu6050_deinit(&gs_handle);
       
        return 1;
    }
    
    /* disable iic bypass */
    res = mpu6050_set_iic_bypass(&gs_handle, MPU6050_BOOL_FALSE);
    if (res != 0)
    {
        mpu6050_interface_debug_print("mpu6050: set iic bypass failed.\n");
        (void)mpu6050_deinit(&gs_handle);
       
        return 1;
    }
    
    /* force fifo reset */
    res = mpu6050_force_fifo_reset(&gs_handle);
    if (res != 0)
    {
        mpu6050_interface_debug_print("mpu6050: force fifo reset failed.\n");
        (void)mpu6050_deinit(&gs_handle);
       
        return 1;
    }
    
    /* delay 1000 ms */
    mpu6050_interface_delay_ms(1000);
    
    for (i = 0; i < times; i++)
    {
        /* read data */
        gs_len = 80;
        res = mpu6050_read(&gs_handle, gs_accel_raw, gs_accel_g, gs_gyro_raw, gs_gyro_dps, &gs_len);
        if (res != 0)
        {
            mpu6050_interface_debug_print("mpu6050: read failed.\n");
            (void)mpu6050_deinit(&gs_handle);
           
            return 1;
        }
        
        /* output data */
        mpu6050_interface_debug_print("mpu6050: fifo %d.\n", gs_len);
        mpu6050_interface_debug_print("mpu6050: acc x[0] is %0.2fg.\n", gs_accel_g[0][0]);
        mpu6050_interface_debug_print("mpu6050: acc y[0] is %0.2fg.\n", gs_accel_g[0][1]);
        mpu6050_interface_debug_print("mpu6050: acc z[0] is %0.2fg.\n", gs_accel_g[0][2]);
        mpu6050_interface_debug_print("mpu6050: gyro x[0] is %0.2fdps.\n", gs_gyro_dps[0][0]);
        mpu6050_interface_debug_print("mpu6050: gyro y[0] is %0.2fdps.\n", gs_gyro_dps[0][1]);
        mpu6050_interface_debug_print("mpu6050: gyro z[0] is %0.2fdps.\n", gs_gyro_dps[0][2]);
        
        /* delay 1000 ms */
        mpu6050_interface_delay_ms(1000);
        
    }
    
    /* finish fifo test */
    mpu6050_interface_debug_print("mpu6050: finish fifo test.\n");
    (void)mpu6050_deinit(&gs_handle);
    
    return 0;
}
