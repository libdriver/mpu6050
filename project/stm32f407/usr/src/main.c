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
 * @file      main.c
 * @brief     main source file
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

#include "driver_mpu6050_register_test.h"
#include "driver_mpu6050_read_test.h"
#include "driver_mpu6050_fifo_test.h"
#include "driver_mpu6050_dmp_read_test.h"
#include "driver_mpu6050_dmp_tap_orient_motion_test.h"
#include "driver_mpu6050_dmp_pedometer_test.h"
#include "driver_mpu6050_basic.h"
#include "driver_mpu6050_fifo.h"
#include "driver_mpu6050_dmp.h"
#include "shell.h"
#include "clock.h"
#include "delay.h"
#include "gpio.h"
#include "uart.h"
#include <stdlib.h>

/**
 * @brief global var definition
 */
uint8_t g_buf[256];                        /**< uart buffer */
uint16_t g_len;                            /**< uart buffer length */
uint8_t g_flag;                            /**< interrupt flag */
uint8_t (*g_gpio_irq)(void) = NULL;        /**< gpio irq */
static int16_t gs_accel_raw[128][3];       /**< accel raw buffer */
static float gs_accel_g[128][3];           /**< accel g buffer */
static int16_t gs_gyro_raw[128][3];        /**< gyro raw buffer */
static float gs_gyro_dps[128][3];          /**< gyro dps buffer */
static int32_t gs_quat[128][4];            /**< quat buffer */
static float gs_pitch[128];                /**< pitch buffer */
static float gs_roll[128];                 /**< roll buffer */
static float gs_yaw[128];                  /**< yaw buffer */
static int32_t gs_quat[128][4];            /**< quat buffer */
static float gs_pitch[128];                /**< pitch buffer */
static float gs_roll[128];                 /**< roll buffer */
static float gs_yaw[128];                  /**< yaw buffer */
static uint8_t gs_flag;                    /**< flag */

/**
 * @brief exti 0 irq
 * @note  none
 */
void EXTI0_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
}

/**
 * @brief     gpio exti callback
 * @param[in] pin is the gpio pin
 * @note      none
 */
void HAL_GPIO_EXTI_Callback(uint16_t pin)
{
    if (pin == GPIO_PIN_0)
    {
        if (g_gpio_irq)
        {
            g_gpio_irq();
        }
    }
}

/**
 * @brief     interface receive callback
 * @param[in] type is the irq type
 * @note      none
 */
static void a_receive_callback(uint8_t type)
{
    switch (type)
    {
        case MPU6050_INTERRUPT_MOTION :
        {
            gs_flag |= 1 << 0;
            mpu6050_interface_debug_print("mpu6050: irq motion.\n");
            
            break;
        }
        case MPU6050_INTERRUPT_FIFO_OVERFLOW :
        {
            mpu6050_interface_debug_print("mpu6050: irq fifo overflow.\n");
            
            break;
        }
        case MPU6050_INTERRUPT_I2C_MAST :
        {
            mpu6050_interface_debug_print("mpu6050: irq i2c master.\n");
            
            break;
        }
        case MPU6050_INTERRUPT_DMP :
        {
            mpu6050_interface_debug_print("mpu6050: irq dmp\n");
            
            break;
        }
        case MPU6050_INTERRUPT_DATA_READY :
        {
            mpu6050_interface_debug_print("mpu6050: irq data ready\n");
            
            break;
        }
        default :
        {
            mpu6050_interface_debug_print("mpu6050: irq unknown code.\n");
            
            break;
        }
    }
}

/**
 * @brief     interface dmp tap callback
 * @param[in] count is the tap count
 * @param[in] direction is the tap direction
 * @note      none
 */
static void a_dmp_tap_callback(uint8_t count, uint8_t direction)
{
    switch (direction)
    {
        case MPU6050_DMP_TAP_X_UP :
        {
            gs_flag |= 1 << 1;
            mpu6050_interface_debug_print("mpu6050: tap irq x up with %d.\n", count);
            
            break;
        }
        case MPU6050_DMP_TAP_X_DOWN :
        {
            gs_flag |= 1 << 1;
            mpu6050_interface_debug_print("mpu6050: tap irq x down with %d.\n", count);
            
            break;
        }
        case MPU6050_DMP_TAP_Y_UP :
        {
            gs_flag |= 1 << 1;
            mpu6050_interface_debug_print("mpu6050: tap irq y up with %d.\n", count);
            
            break;
        }
        case MPU6050_DMP_TAP_Y_DOWN :
        {
            gs_flag |= 1 << 1;
            mpu6050_interface_debug_print("mpu6050: tap irq y down with %d.\n", count);
            
            break;
        }
        case MPU6050_DMP_TAP_Z_UP :
        {
            gs_flag |= 1 << 1;
            mpu6050_interface_debug_print("mpu6050: tap irq z up with %d.\n", count);
            
            break;
        }
        case MPU6050_DMP_TAP_Z_DOWN :
        {
            gs_flag |= 1 << 1;
            mpu6050_interface_debug_print("mpu6050: tap irq z down with %d.\n", count);
            
            break;
        }
        default :
        {
            mpu6050_interface_debug_print("mpu6050: tap irq unknown code.\n");
            
            break;
        }
    }
}

/**
 * @brief     interface dmp orient callback
 * @param[in] orient is the dmp orient
 * @note      none
 */
static void a_dmp_orient_callback(uint8_t orientation)
{
    switch (orientation)
    {
        case MPU6050_DMP_ORIENT_PORTRAIT :
        {
            gs_flag |= 1 << 2;
            mpu6050_interface_debug_print("mpu6050: orient irq portrait.\n");
            
            break;
        }
        case MPU6050_DMP_ORIENT_LANDSCAPE :
        {
            gs_flag |= 1 << 2;
            mpu6050_interface_debug_print("mpu6050: orient irq landscape.\n");
            
            break;
        }
        case MPU6050_DMP_ORIENT_REVERSE_PORTRAIT :
        {
            gs_flag |= 1 << 2;
            mpu6050_interface_debug_print("mpu6050: orient irq reverse portrait.\n");
            
            break;
        }
        case MPU6050_DMP_ORIENT_REVERSE_LANDSCAPE :
        {
            gs_flag |= 1 << 2;
            mpu6050_interface_debug_print("mpu6050: orient irq reverse landscape.\n");
            
            break;
        }
        default :
        {
            mpu6050_interface_debug_print("mpu6050: orient irq unknown code.\n");
            
            break;
        }
    }
}

/**
 * @brief     mpu6050 full function
 * @param[in] argc is arg numbers
 * @param[in] **argv is the arg address
 * @return    status code
 *             - 0 success
 *             - 1 run failed
 *             - 5 param is invalid
 * @note      none
 */
uint8_t mpu6050(uint8_t argc, char **argv)
{
    if (argc == 1)
    {
        goto help;
    }
    else if (argc == 2)
    {
        if (strcmp("-i", argv[1]) == 0)
        {
            mpu6050_info_t info;
            
            /* print mpu6050 info */
            mpu6050_info(&info);
            mpu6050_interface_debug_print("mpu6050: chip is %s.\n", info.chip_name);
            mpu6050_interface_debug_print("mpu6050: manufacturer is %s.\n", info.manufacturer_name);
            mpu6050_interface_debug_print("mpu6050: interface is %s.\n", info.interface);
            mpu6050_interface_debug_print("mpu6050: driver version is %d.%d.\n", info.driver_version / 1000, (info.driver_version % 1000) / 100);
            mpu6050_interface_debug_print("mpu6050: min supply voltage is %0.1fV.\n", info.supply_voltage_min_v);
            mpu6050_interface_debug_print("mpu6050: max supply voltage is %0.1fV.\n", info.supply_voltage_max_v);
            mpu6050_interface_debug_print("mpu6050: max current is %0.2fmA.\n", info.max_current_ma);
            mpu6050_interface_debug_print("mpu6050: max temperature is %0.1fC.\n", info.temperature_max);
            mpu6050_interface_debug_print("mpu6050: min temperature is %0.1fC.\n", info.temperature_min);
            
            return 0;
        }
        else if (strcmp("-p", argv[1]) == 0)
        {
            /* print pin connection */
            mpu6050_interface_debug_print("mpu6050: SCL connected to GPIOB PIN8.\n");
            mpu6050_interface_debug_print("mpu6050: SDA connected to GPIOB PIN9.\n");
            mpu6050_interface_debug_print("mpu6050: INT connected to GPIOB PIN0.\n");
            
            return 0;
        }
        else if (strcmp("-h", argv[1]) == 0)
        {
            /* show mpu6050 help */
            
            help:
            mpu6050_interface_debug_print("mpu6050 -i\n\tshow mpu6050 chip and driver information.\n");
            mpu6050_interface_debug_print("mpu6050 -h\n\tshow mpu6050 help.\n");
            mpu6050_interface_debug_print("mpu6050 -p\n\tshow mpu6050 pin connections of the current board.\n");
            mpu6050_interface_debug_print("mpu6050 -t reg -a (0 | 1)\n\trun mpu6050 register test.\n");
            mpu6050_interface_debug_print("mpu6050 -t read <times> -a (0 | 1)\n\trun mpu6050 read test.times means the test times.\n");
            mpu6050_interface_debug_print("mpu6050 -t fifo <times> -a (0 | 1)\n\trun mpu6050 fifo test.times means the test times.\n");
            mpu6050_interface_debug_print("mpu6050 -t dmp <times> -a (0 | 1)\n\trun mpu6050 dmp test.times means the test times.\n");
            mpu6050_interface_debug_print("mpu6050 -t motion -a (0 | 1)\n\trun mpu6050 motion test.\n");
            mpu6050_interface_debug_print("mpu6050 -t pedometer <times> -a (0 | 1)\n\trun mpu6050 pedometer test.times means the test times.\n");
            mpu6050_interface_debug_print("mpu6050 -c read <times> -a (0 | 1)\n\trun mpu6050 read function.times means the read times.\n");
            mpu6050_interface_debug_print("mpu6050 -c fifo <times> -a (0 | 1)\n\trun mpu6050 fifo function. times means the read times.\n");
            mpu6050_interface_debug_print("mpu6050 -c dmp <times> -a (0 | 1)\n\trun mpu6050 dmp function.times means the read times.\n");
            mpu6050_interface_debug_print("mpu6050 -c motion -a (0 | 1)\n\trun mpu6050 motion function.\n");
            mpu6050_interface_debug_print("mpu6050 -c pedometer <times> -a (0 | 1)\n\trun mpu6050 pedometer function.times means the read times.\n");
            
            return 0;
        }
        else
        {
            return 5;
        }
    }
    else if (argc == 5)
    {
        /* run test */
        if (strcmp("-t", argv[1]) == 0)
        {
            /* reg test */
            if (strcmp("reg", argv[2]) == 0)
            {
                mpu6050_address_t addr;
                
                /* check iic address */
                if (strcmp("-a", argv[3]) != 0)
                {
                    return 5;
                }
                if (strcmp("0", argv[4]) == 0)
                {
                    addr = MPU6050_ADDRESS_AD0_LOW;
                }
                else if (strcmp("1", argv[4]) == 0)
                {
                    addr = MPU6050_ADDRESS_AD0_HIGH;
                }
                else
                {
                    return 5;
                }
                
                /* run reg test */
                if (mpu6050_register_test(addr) != 0)
                {
                    return 1;
                }
                else
                {
                    return 0;
                }
            }
            /* motion test */
            else if (strcmp("motion", argv[2]) == 0)
            {
                mpu6050_address_t addr;
                
                /* check iic address */
                if (strcmp("-a", argv[3]) != 0)
                {
                    return 5;
                }
                if (strcmp("0", argv[4]) == 0)
                {
                    addr = MPU6050_ADDRESS_AD0_LOW;
                }
                else if (strcmp("1", argv[4]) == 0)
                {
                    addr = MPU6050_ADDRESS_AD0_HIGH;
                }
                else
                {
                    return 5;
                }
                
                if (gpio_interrupt_init() != 0)
                {
                    return 1;
                }
                g_gpio_irq = mpu6050_dmp_tap_orient_motion_test_irq_handler;
                
                /* run reg test */
                if (mpu6050_dmp_tap_orient_motion_test(addr) != 0)
                {
                    g_gpio_irq = NULL;
                    (void)gpio_interrupt_deinit();
                    
                    return 1;
                }
                else
                {
                    g_gpio_irq = NULL;
                    (void)gpio_interrupt_deinit();
                    
                    return 0;
                }
            }
            /* param is invalid */
            else
            {
                return 5;
            }
        }
        /* run function */
        else if (strcmp("-c", argv[1]) == 0)
        {
            /* motion function */
            if (strcmp("motion", argv[2]) == 0)
            {
                mpu6050_address_t addr;
                
                /* check iic address */
                if (strcmp("-a", argv[3]) != 0)
                {
                    return 5;
                }
                if (strcmp("0", argv[4]) == 0)
                {
                    addr = MPU6050_ADDRESS_AD0_LOW;
                }
                else if (strcmp("1", argv[4]) == 0)
                {
                    addr = MPU6050_ADDRESS_AD0_HIGH;
                }
                else
                {
                    return 5;
                }
                
                if (gpio_interrupt_init() != 0)
                {
                    return 1;
                }
                g_gpio_irq = mpu6050_dmp_irq_handler;
                
                /* run dmp function */
                if (mpu6050_dmp_init(addr, a_receive_callback, 
                                     a_dmp_tap_callback, a_dmp_orient_callback) != 0)
                {
                    g_gpio_irq = NULL;
                    (void)gpio_interrupt_deinit();
                    
                    return 1;
                }
                else
                {
                    uint16_t i;
                    
                    gs_flag = 0;
                    mpu6050_interface_delay_ms(200);
                    for (i = 0; i < 1000; i++)
                    {
                        uint16_t l;
                        uint8_t res;
                        
                        /* read the data */
                        l = 128;
                        res = mpu6050_dmp_read_all(gs_accel_raw, gs_accel_g,
                                                   gs_gyro_raw, gs_gyro_dps,
                                                   gs_quat,
                                                   gs_pitch, gs_roll, gs_yaw,
                                                   &l
                                                  );
                        if (res != 0)
                        {
                            /* output data */
                            mpu6050_interface_debug_print("mpu6050: dmp read failed.\n");
                        }
                        mpu6050_interface_delay_ms(200);
                        
                        /* check the flag */
                        if ((gs_flag & 0x7) == 0x7)
                        {
                            break;
                        }
                    }
                    
                    /* finish dmp tap orient motion */
                    mpu6050_interface_debug_print("mpu6050: finish dmp tap orient motion.\n");
                    
                    (void)mpu6050_dmp_deinit();
                    g_gpio_irq = NULL;
                    (void)gpio_interrupt_deinit();
                    
                    return 0;
                }
            }
            /* param is invalid */
            else
            {
                return 5;
            }
        }
        /* param is invalid */
        else
        {
            return 5;
        }
    }
    else if (argc == 6)
    {
        /* run test */
        if (strcmp("-t", argv[1]) == 0)
        {
             /* read test */
            if (strcmp("read", argv[2]) == 0)
            {
                mpu6050_address_t addr;
                
                /* check iic address */
                if (strcmp("-a", argv[4]) != 0)
                {
                    return 5;
                }
                if (strcmp("0", argv[5]) == 0)
                {
                    addr = MPU6050_ADDRESS_AD0_LOW;
                }
                else if (strcmp("1", argv[5]) == 0)
                {
                    addr = MPU6050_ADDRESS_AD0_HIGH;
                }
                else
                {
                    return 5;
                }
                
                if (mpu6050_read_test(addr, atoi(argv[3])) != 0)
                {
                    return 1;
                }
                else
                {
                    return 0;
                }
            }
            /* fifo test */
            else if (strcmp("fifo", argv[2]) == 0)
            {
                mpu6050_address_t addr;
                
                /* check iic address */
                if (strcmp("-a", argv[4]) != 0)
                {
                    return 5;
                }
                if (strcmp("0", argv[5]) == 0)
                {
                    addr = MPU6050_ADDRESS_AD0_LOW;
                }
                else if (strcmp("1", argv[5]) == 0)
                {
                    addr = MPU6050_ADDRESS_AD0_HIGH;
                }
                else
                {
                    return 5;
                }
                
                if (gpio_interrupt_init() != 0)
                {
                    return 1;
                }
                g_gpio_irq = mpu6050_fifo_test_irq_handler;
                if (mpu6050_fifo_test(addr, atoi(argv[3])) != 0)
                {
                    g_gpio_irq = NULL;
                    (void)gpio_interrupt_deinit();
                    
                    return 1;
                }
                else
                {
                    g_gpio_irq = NULL;
                    (void)gpio_interrupt_deinit();
                    
                    return 0;
                }
            }
            /* dmp test */
            else if (strcmp("dmp", argv[2]) == 0)
            {
                mpu6050_address_t addr;
                
                /* check iic address */
                if (strcmp("-a", argv[4]) != 0)
                {
                    return 5;
                }
                if (strcmp("0", argv[5]) == 0)
                {
                    addr = MPU6050_ADDRESS_AD0_LOW;
                }
                else if (strcmp("1", argv[5]) == 0)
                {
                    addr = MPU6050_ADDRESS_AD0_HIGH;
                }
                else
                {
                    return 5;
                }
                
                if (gpio_interrupt_init() != 0)
                {
                    return 1;
                }
                g_gpio_irq = mpu6050_dmp_read_test_irq_handler;
                if (mpu6050_dmp_read_test(addr, atoi(argv[3])) != 0)
                {
                    g_gpio_irq = NULL;
                    (void)gpio_interrupt_deinit();
                    
                    return 1;
                }
                else
                {
                    g_gpio_irq = NULL;
                    (void)gpio_interrupt_deinit();
                    
                    return 0;
                }
            }
            /* dmp test */
            else if (strcmp("pedometer", argv[2]) == 0)
            {
                mpu6050_address_t addr;
                
                /* check iic address */
                if (strcmp("-a", argv[4]) != 0)
                {
                    return 5;
                }
                if (strcmp("0", argv[5]) == 0)
                {
                    addr = MPU6050_ADDRESS_AD0_LOW;
                }
                else if (strcmp("1", argv[5]) == 0)
                {
                    addr = MPU6050_ADDRESS_AD0_HIGH;
                }
                else
                {
                    return 5;
                }
                
                if (gpio_interrupt_init() != 0)
                {
                    return 1;
                }
                g_gpio_irq = mpu6050_dmp_pedometer_test_irq_handler;
                if (mpu6050_dmp_pedometer_test(addr, atoi(argv[3])) != 0)
                {
                    g_gpio_irq = NULL;
                    (void)gpio_interrupt_deinit();
                    
                    return 1;
                }
                else
                {
                    g_gpio_irq = NULL;
                    (void)gpio_interrupt_deinit();
                    
                    return 0;
                }
            }
            /* param is invalid */
            else
            {
                return 5;
            }
        }
        /* run function */
        else if (strcmp("-c", argv[1]) == 0)
        {
            /* read function */
            if (strcmp("read", argv[2]) == 0)
            {
                mpu6050_address_t addr;
                uint32_t times;
                
                /* check iic address */
                if (strcmp("-a", argv[4]) != 0)
                {
                    return 5;
                }
                if (strcmp("0", argv[5]) == 0)
                {
                    addr = MPU6050_ADDRESS_AD0_LOW;
                }
                else if (strcmp("1", argv[5]) == 0)
                {
                    addr = MPU6050_ADDRESS_AD0_HIGH;
                }
                else
                {
                    return 5;
                }
                
                times = atoi(argv[3]);
                if (mpu6050_basic_init(addr) != 0)
                {
                    return 1;
                }
                else
                {
                    uint32_t i;
                    float g[3];
                    float dps[3];
                    float degrees;
                    
                    for (i = 0; i < times; i++)
                    {
                        /* read */
                        if (mpu6050_basic_read(g, dps) != 0)
                        {
                            (void)mpu6050_basic_deinit();
                            
                            return 1;
                        }
                        
                        if (mpu6050_basic_read_temperature(&degrees) != 0)
                        {
                            (void)mpu6050_basic_deinit();
                            
                            return 1;
                        }
                        
                        /* output */
                        mpu6050_interface_debug_print("mpu6050: %d/%d.\n", i + 1, times);
                        mpu6050_interface_debug_print("mpu6050: acc x is %0.2fg.\n", g[0]);
                        mpu6050_interface_debug_print("mpu6050: acc y is %0.2fg.\n", g[1]);
                        mpu6050_interface_debug_print("mpu6050: acc z is %0.2fg.\n", g[2]);
                        mpu6050_interface_debug_print("mpu6050: gyro x is %0.2fdps.\n", dps[0]);
                        mpu6050_interface_debug_print("mpu6050: gyro y is %0.2fdps.\n", dps[1]);
                        mpu6050_interface_debug_print("mpu6050: gyro z is %0.2fdps.\n", dps[2]);
                        mpu6050_interface_debug_print("mpu6050: temperature %0.2fC.\n", degrees);
                        
                        /* delay 1000 ms */
                        mpu6050_interface_delay_ms(1000);
                    }
                    
                    (void)mpu6050_basic_deinit();
                    
                    return 0;
                }
            }
            /* fifo function */
            else if (strcmp("fifo", argv[2]) == 0)
            {
                mpu6050_address_t addr;
                uint32_t times;
                
                /* check iic address */
                if (strcmp("-a", argv[4]) != 0)
                {
                    return 5;
                }
                if (strcmp("0", argv[5]) == 0)
                {
                    addr = MPU6050_ADDRESS_AD0_LOW;
                }
                else if (strcmp("1", argv[5]) == 0)
                {
                    addr = MPU6050_ADDRESS_AD0_HIGH;
                }
                else
                {
                    return 5;
                }
                
                times = atoi(argv[3]);
                if (gpio_interrupt_init() != 0)
                {
                    return 1;
                }
                g_gpio_irq = mpu6050_fifo_irq_handler;
                if (mpu6050_fifo_init(addr) != 0)
                {
                    g_gpio_irq = NULL;
                    (void)gpio_interrupt_deinit();
                    
                    return 1;
                }
                else
                {
                    uint32_t i;
                    uint16_t len;
                    
                    /* delay 100 ms */
                    mpu6050_interface_delay_ms(100);
                    for (i = 0; i < times; i++)
                    {
                        len = 128;
                        
                        /* read */
                        if (mpu6050_fifo_read(gs_accel_raw, gs_accel_g,
                                              gs_gyro_raw, gs_gyro_dps, &len) != 0)
                        {
                            (void)mpu6050_fifo_deinit();
                             g_gpio_irq = NULL;
                            (void)gpio_interrupt_deinit();
                            
                            return 1;
                        }
                        
                        /* output */
                        mpu6050_interface_debug_print("mpu6050: %d/%d.\n", i + 1, times);
                        mpu6050_interface_debug_print("mpu6050: fifo %d.\n", len);
                        mpu6050_interface_debug_print("mpu6050: acc x[0] is %0.2fg.\n", gs_accel_g[0][0]);
                        mpu6050_interface_debug_print("mpu6050: acc y[0] is %0.2fg.\n", gs_accel_g[0][1]);
                        mpu6050_interface_debug_print("mpu6050: acc z[0] is %0.2fg.\n", gs_accel_g[0][2]);
                        mpu6050_interface_debug_print("mpu6050: gyro x[0] is %0.2fdps.\n", gs_gyro_dps[0][0]);
                        mpu6050_interface_debug_print("mpu6050: gyro y[0] is %0.2fdps.\n", gs_gyro_dps[0][1]);
                        mpu6050_interface_debug_print("mpu6050: gyro z[0] is %0.2fdps.\n", gs_gyro_dps[0][2]);
                        
                        /* delay 100 ms */
                        mpu6050_interface_delay_ms(100);
                    }
                    
                    (void)mpu6050_fifo_deinit();
                     g_gpio_irq = NULL;
                    (void)gpio_interrupt_deinit();
                    
                    return 0;
                }
            }
            /* dmp function */
            else if (strcmp("dmp", argv[2]) == 0)
            {
                mpu6050_address_t addr;
                uint32_t times;
                
                /* check iic address */
                if (strcmp("-a", argv[4]) != 0)
                {
                    return 5;
                }
                if (strcmp("0", argv[5]) == 0)
                {
                    addr = MPU6050_ADDRESS_AD0_LOW;
                }
                else if (strcmp("1", argv[5]) == 0)
                {
                    addr = MPU6050_ADDRESS_AD0_HIGH;
                }
                else
                {
                    return 5;
                }
                
                times = atoi(argv[3]);
                if (gpio_interrupt_init() != 0)
                {
                    return 1;
                }
                g_gpio_irq = mpu6050_dmp_irq_handler;
                if (mpu6050_dmp_init(addr, mpu6050_interface_receive_callback, NULL, NULL) != 0)
                {
                    g_gpio_irq = NULL;
                    (void)gpio_interrupt_deinit();
                    
                    return 1;
                }
                else
                {
                    uint32_t i;
                    uint16_t len;
                    
                    /* delay 500 ms */
                    mpu6050_interface_delay_ms(500);
                    for (i = 0; i < times; i++)
                    {
                        len = 128;
                        
                        /* read */
                        if (mpu6050_dmp_read_all(gs_accel_raw, gs_accel_g,
                                                 gs_gyro_raw, gs_gyro_dps, 
                                                 gs_quat,
                                                 gs_pitch, gs_roll, gs_yaw,
                                                 &len) != 0)
                        {
                            (void)mpu6050_dmp_deinit();
                             g_gpio_irq = NULL;
                            (void)gpio_interrupt_deinit();
                            
                            return 1;
                        }
                        
                        /* output */
                        mpu6050_interface_debug_print("mpu6050: %d/%d.\n", i + 1, times);
                        mpu6050_interface_debug_print("mpu6050: fifo %d.\n", len);
                        mpu6050_interface_debug_print("mpu6050: pitch[0] is %0.2fdps.\n", gs_pitch[0]);
                        mpu6050_interface_debug_print("mpu6050: roll[0] is %0.2fdps.\n", gs_roll[0]);
                        mpu6050_interface_debug_print("mpu6050: yaw[0] is %0.2fdps.\n", gs_yaw[0]);
                        mpu6050_interface_debug_print("mpu6050: acc x[0] is %0.2fg.\n", gs_accel_g[0][0]);
                        mpu6050_interface_debug_print("mpu6050: acc y[0] is %0.2fg.\n", gs_accel_g[0][1]);
                        mpu6050_interface_debug_print("mpu6050: acc z[0] is %0.2fg.\n", gs_accel_g[0][2]);
                        mpu6050_interface_debug_print("mpu6050: gyro x[0] is %0.2fdps.\n", gs_gyro_dps[0][0]);
                        mpu6050_interface_debug_print("mpu6050: gyro y[0] is %0.2fdps.\n", gs_gyro_dps[0][1]);
                        mpu6050_interface_debug_print("mpu6050: gyro z[0] is %0.2fdps.\n", gs_gyro_dps[0][2]);
                        
                        /* delay 500 ms */
                        mpu6050_interface_delay_ms(500);
                    }
                    
                    (void)mpu6050_dmp_deinit();
                     g_gpio_irq = NULL;
                    (void)gpio_interrupt_deinit();
                    
                    return 0;
                }
            }
            /* pedometer function */
            else if (strcmp("pedometer", argv[2]) == 0)
            {
                mpu6050_address_t addr;
                uint32_t times;
                
                /* check iic address */
                if (strcmp("-a", argv[4]) != 0)
                {
                    return 5;
                }
                if (strcmp("0", argv[5]) == 0)
                {
                    addr = MPU6050_ADDRESS_AD0_LOW;
                }
                else if (strcmp("1", argv[5]) == 0)
                {
                    addr = MPU6050_ADDRESS_AD0_HIGH;
                }
                else
                {
                    return 5;
                }
                
                times = atoi(argv[3]);
                if (gpio_interrupt_init() != 0)
                {
                    return 1;
                }
                g_gpio_irq = mpu6050_dmp_irq_handler;
                if (mpu6050_dmp_init(addr, mpu6050_interface_receive_callback, NULL, NULL) != 0)
                {
                    g_gpio_irq = NULL;
                    (void)gpio_interrupt_deinit();
                    
                    return 1;
                }
                else
                {
                    uint8_t res;
                    uint32_t i;
                    uint32_t cnt = 0;
                    uint32_t cnt_check = 0;
                    
                    i = 0;
                    while (times != 0)
                    {
                        uint16_t l;
                        
                        /* delay 200 ms */
                        mpu6050_interface_delay_ms(200);
                        
                        /* read the data */
                        l = 128;
                        res = mpu6050_dmp_read_all(gs_accel_raw, gs_accel_g,
                                                   gs_gyro_raw, gs_gyro_dps, 
                                                   gs_quat,
                                                   gs_pitch, gs_roll, gs_yaw,
                                                   &l);
                        if (res == 0)
                        {
                            i++;
                            if (i > 5)
                            {
                                i = 0;
                                
                                /* get the pedometer step count */
                                res = mpu6050_dmp_get_pedometer_counter(&cnt);
                                if (res != 0)
                                {
                                    mpu6050_interface_debug_print("mpu6050: dmp get pedometer step count failed.\n");
                                    (void)mpu6050_dmp_deinit();
                                     g_gpio_irq = NULL;
                                    (void)gpio_interrupt_deinit();
                                   
                                    return 1;
                                }
                                
                                /* check the cnt */
                                if (cnt != cnt_check)
                                {
                                    mpu6050_interface_debug_print("mpu6050: pedometer step count is %d.\n", cnt);
                                    cnt_check = cnt;
                                    times--;
                                }
                            }
                        }
                    }
                    
                    (void)mpu6050_dmp_deinit();
                     g_gpio_irq = NULL;
                    (void)gpio_interrupt_deinit();
                    
                    return 0;
                }
            }
            /* param is invalid */
            else
            {
                return 5;
            }
        }
        /* param is invalid */
        else
        {
            return 5;
        }
    } 
    /* param is invalid */
    else
    {
        return 5;
    }
}

/**
 * @brief main function
 * @note  none
 */
int main(void)
{
    uint8_t res;
    
    /* stm32f407 clock init and hal init */
    clock_init();
    
    /* delay init */
    delay_init();
    
    /* uart1 init */
    uart1_init(115200);
    
    /* shell init && register mpu6050 fuction */
    shell_init();
    shell_register("mpu6050", mpu6050);
    uart1_print("mpu6050: welcome to libdriver mpu6050.\n");
    
    while (1)
    {
        /* read uart */
        g_len = uart1_read(g_buf, 256);
        if (g_len)
        {
            /* run shell */
            res = shell_parse((char *)g_buf, g_len);
            if (res == 0)
            {
                /* run success */
            }
            else if (res == 1)
            {
                uart1_print("mpu6050: run failed.\n");
            }
            else if (res == 2)
            {
                uart1_print("mpu6050: unknow command.\n");
            }
            else if (res == 3)
            {
                uart1_print("mpu6050: length is too long.\n");
            }
            else if (res == 4)
            {
                uart1_print("mpu6050: pretreat failed.\n");
            }
            else if (res == 5)
            {
                uart1_print("mpu6050: param is invalid.\n");
            }
            else
            {
                uart1_print("mpu6050: unknow status code.\n");
            }
            uart1_flush();
        }
        delay_ms(100);
    }
}
