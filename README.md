[English](/README.md) | [ 简体中文](/README_zh-Hans.md) | [繁體中文](/README_zh-Hant.md) | [日本語](/README_ja.md) | [Deutsch](/README_de.md) | [한국어](/README_ko.md)

<div align=center>
<img src="/doc/image/logo.svg" width="400" height="150"/>
</div>

## LibDriver MPU6050

[![MISRA](https://img.shields.io/badge/misra-compliant-brightgreen.svg)](/misra/README.md) [![API](https://img.shields.io/badge/api-reference-blue.svg)](https://www.libdriver.com/docs/mpu6050/index.html) [![License](https://img.shields.io/badge/license-MIT-brightgreen.svg)](/LICENSE)

The MPU6050 is the world’s first integrated 6-axis MotionTracking device that combines a 3-axis gyroscope, 3-axis accelerometer, and a Digital Motion Processor™ (DMP) all in a small 4x4x0.9mm package. With its dedicated I2C sensor bus, it directly accepts inputs from an external 3-axis compass to provide a complete 9-axis MotionFusion™ output. The MPU6050 MotionTracking device, with its 6-axis integration, on-board MotionFusion™, and run-time calibration firmware, enables manufacturers to eliminate the costly and complex selection, qualification, and system level integration of discrete devices, guaranteeing optimal motion performance for consumers. The MPU6050 is also designed to interface with multiple non-inertial digital sensors, such as pressure sensors, on its auxiliary I2C port. The MPU6050 is footprint compatible with the MPU30X0 family.The MPU6050 features three 16-bit analog-to-digital converters (ADCs) for digitizing the gyroscope outputs and three 16-bit ADCs for digitizing the accelerometer outputs. For precision tracking of both fast and slow motions, the parts feature a user-programmable gyroscope full-scale range of ±250, ±500, ±1000, and ±2000°/sec (dps) and a user-programmable accelerometer full-scale range of ±2g, ±4g, ±8g, and ±16g.

LibDriver MPU6050 is a full-featured driver of MPU6050 launched by LibDriver. It provides acceleration reading, angular velocity reading, attitude angle reading, dmp reading, tap detection and other functions. LibDriver is MISRA compliant.

### Table of Contents

  - [Instruction](#Instruction)
  - [Install](#Install)
  - [Usage](#Usage)
    - [example basic](#example-basic)
    - [example fifo](#example-fifo)
    - [example dmp](#example-dmp)
  - [Document](#Document)
  - [Contributing](#Contributing)
  - [License](#License)
  - [Contact Us](#Contact-Us)

### Instruction

/src includes LibDriver MPU6050 source files.

/interface includes LibDriver MPU6050 IIC platform independent template.

/test includes LibDriver MPU6050 driver test code and this code can test the chip necessary function simply.

/example includes LibDriver MPU6050 sample code.

/doc includes LibDriver MPU6050 offline document.

/datasheet includes MPU6050 datasheet.

/project includes the common Linux and MCU development board sample code. All projects use the shell script to debug the driver and the detail instruction can be found in each project's README.md.

/misra includes the LibDriver MISRA code scanning results.

### Install

Reference /interface IIC platform independent template and finish your platform IIC driver.

Add the /src directory, the interface driver for your platform, and your own drivers to your project, if you want to use the default example drivers, add the /example directory to your project.

### Usage

You can refer to the examples in the /example directory to complete your own driver. If you want to use the default programming examples, here's how to use them.

#### example basic

```C
#include "driver_mpu6050_basic.h"

uint8_t res;
uint32_t i;
uint32_t times;
float g[3];
float dps[3];
float degrees;
mpu6050_address_t addr;

/* init */
addr = MPU6050_ADDRESS_AD0_LOW;
res = mpu6050_basic_init(addr);
if (res != 0)
{
    return 1;
}

...
    
/* read all */
times = 3;
for (i = 0; i < times; i++)
{
    /* read */
    if (mpu6050_basic_read(g, dps) != 0)
    {
        (void)mpu6050_basic_deinit();

        return 1;
    }

    ...
        
    if (mpu6050_basic_read_temperature(&degrees) != 0)
    {
        (void)mpu6050_basic_deinit();

        return 1;
    }

    ...
        
    /* output */
    mpu6050_interface_debug_print("mpu6050: %d/%d.\n", i + 1, times);
    mpu6050_interface_debug_print("mpu6050: acc x is %0.2fg.\n", g[0]);
    mpu6050_interface_debug_print("mpu6050: acc y is %0.2fg.\n", g[1]);
    mpu6050_interface_debug_print("mpu6050: acc z is %0.2fg.\n", g[2]);
    mpu6050_interface_debug_print("mpu6050: gyro x is %0.2fdps.\n", dps[0]);
    mpu6050_interface_debug_print("mpu6050: gyro y is %0.2fdps.\n", dps[1]);
    mpu6050_interface_debug_print("mpu6050: gyro z is %0.2fdps.\n", dps[2]);
    mpu6050_interface_debug_print("mpu6050: temperature %0.2fC.\n", degrees);

    ...
        
    /* delay 1000 ms */
    mpu6050_interface_delay_ms(1000);

    ...
}

...
    
/* deinit */
(void)mpu6050_basic_deinit();

return 0;
```

#### example fifo

```C
#include "driver_mpu6050_fifo.h"

uint32_t i;
uint32_t times;
uint16_t len;
uint8_t (*g_gpio_irq)(void) = NULL;
static int16_t gs_accel_raw[128][3];
static float gs_accel_g[128][3];
static int16_t gs_gyro_raw[128][3];
static float gs_gyro_dps[128][3];
mpu6050_address_t addr;

/* gpio init */
if (gpio_interrupt_init() != 0)
{
    return 1;
}
g_gpio_irq = mpu6050_fifo_irq_handler;

/* init */
addr = MPU6050_ADDRESS_AD0_LOW;
if (mpu6050_fifo_init(addr) != 0)
{
    g_gpio_irq = NULL;
    (void)gpio_interrupt_deinit();

    return 1;
}

/* delay 100 ms */
mpu6050_interface_delay_ms(100);

...

times = 3;
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
    
    ...
        
    /* output */
    mpu6050_interface_debug_print("mpu6050: %d/%d.\n", i + 1, times);
    mpu6050_interface_debug_print("mpu6050: fifo %d.\n", len);
    mpu6050_interface_debug_print("mpu6050: acc x[0] is %0.2fg.\n", gs_accel_g[0][0]);
    mpu6050_interface_debug_print("mpu6050: acc y[0] is %0.2fg.\n", gs_accel_g[0][1]);
    mpu6050_interface_debug_print("mpu6050: acc z[0] is %0.2fg.\n", gs_accel_g[0][2]);
    mpu6050_interface_debug_print("mpu6050: gyro x[0] is %0.2fdps.\n", gs_gyro_dps[0][0]);
    mpu6050_interface_debug_print("mpu6050: gyro y[0] is %0.2fdps.\n", gs_gyro_dps[0][1]);
    mpu6050_interface_debug_print("mpu6050: gyro z[0] is %0.2fdps.\n", gs_gyro_dps[0][2]);
    
    ...
        
    /* delay 100 ms */
    mpu6050_interface_delay_ms(100);
    
    ...
}

...
    
/* deinit */
(void)mpu6050_fifo_deinit();
g_gpio_irq = NULL;
(void)gpio_interrupt_deinit();

return 0;
```

#### example dmp

```C
#include "driver_mpu6050_dmp.h"

uint32_t i;
uint32_t times;
uint32_t cnt;
uint16_t len;
uint8_t (*g_gpio_irq)(void) = NULL;
static int16_t gs_accel_raw[128][3];
static float gs_accel_g[128][3];
static int16_t gs_gyro_raw[128][3];      
static float gs_gyro_dps[128][3];        
static int32_t gs_quat[128][4];          
static float gs_pitch[128];              
static float gs_roll[128];                
static float gs_yaw[128];                     
mpu6050_address_t addr;

static void a_receive_callback(uint8_t type)
{
    switch (type)
    {
        case MPU6050_INTERRUPT_MOTION :
        {
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

static void a_dmp_tap_callback(uint8_t count, uint8_t direction)
{
    switch (direction)
    {
        case MPU6050_DMP_TAP_X_UP :
        {
            mpu6050_interface_debug_print("mpu6050: tap irq x up with %d.\n", count);
            
            break;
        }
        case MPU6050_DMP_TAP_X_DOWN :
        {
            mpu6050_interface_debug_print("mpu6050: tap irq x down with %d.\n", count);
            
            break;
        }
        case MPU6050_DMP_TAP_Y_UP :
        {
            mpu6050_interface_debug_print("mpu6050: tap irq y up with %d.\n", count);
            
            break;
        }
        case MPU6050_DMP_TAP_Y_DOWN :
        {
            mpu6050_interface_debug_print("mpu6050: tap irq y down with %d.\n", count);
            
            break;
        }
        case MPU6050_DMP_TAP_Z_UP :
        {
            mpu6050_interface_debug_print("mpu6050: tap irq z up with %d.\n", count);
            
            break;
        }
        case MPU6050_DMP_TAP_Z_DOWN :
        {
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

static void a_dmp_orient_callback(uint8_t orientation)
{
    switch (orientation)
    {
        case MPU6050_DMP_ORIENT_PORTRAIT :
        {
            mpu6050_interface_debug_print("mpu6050: orient irq portrait.\n");
            
            break;
        }
        case MPU6050_DMP_ORIENT_LANDSCAPE :
        {
            mpu6050_interface_debug_print("mpu6050: orient irq landscape.\n");
            
            break;
        }
        case MPU6050_DMP_ORIENT_REVERSE_PORTRAIT :
        {
            mpu6050_interface_debug_print("mpu6050: orient irq reverse portrait.\n");
            
            break;
        }
        case MPU6050_DMP_ORIENT_REVERSE_LANDSCAPE :
        {
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

/* init */
if (gpio_interrupt_init() != 0)
{
    return 1;
}
g_gpio_irq = mpu6050_dmp_irq_handler;

/* run dmp function */
addr = MPU6050_ADDRESS_AD0_LOW;
if (mpu6050_dmp_init(addr, a_receive_callback, 
                     a_dmp_tap_callback, a_dmp_orient_callback) != 0)
{
    g_gpio_irq = NULL;
    (void)gpio_interrupt_deinit();

    return 1;
}

/* delay 500 ms */
mpu6050_interface_delay_ms(500);

...
    
times = 3;
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
    mpu6050_interface_debug_print("mpu6050: pitch[0] is %0.2fdeg.\n", gs_pitch[0]);
    mpu6050_interface_debug_print("mpu6050: roll[0] is %0.2fdeg.\n", gs_roll[0]);
    mpu6050_interface_debug_print("mpu6050: yaw[0] is %0.2fdeg.\n", gs_yaw[0]);
    mpu6050_interface_debug_print("mpu6050: acc x[0] is %0.2fg.\n", gs_accel_g[0][0]);
    mpu6050_interface_debug_print("mpu6050: acc y[0] is %0.2fg.\n", gs_accel_g[0][1]);
    mpu6050_interface_debug_print("mpu6050: acc z[0] is %0.2fg.\n", gs_accel_g[0][2]);
    mpu6050_interface_debug_print("mpu6050: gyro x[0] is %0.2fdps.\n", gs_gyro_dps[0][0]);
    mpu6050_interface_debug_print("mpu6050: gyro y[0] is %0.2fdps.\n", gs_gyro_dps[0][1]);
    mpu6050_interface_debug_print("mpu6050: gyro z[0] is %0.2fdps.\n", gs_gyro_dps[0][2]);

    /* delay 500 ms */
    mpu6050_interface_delay_ms(500);
    
    ....
        
    /* get the pedometer step count */
    res = mpu6050_dmp_get_pedometer_counter(&cnt);
    if (res != 0)
    {
        (void)mpu6050_dmp_deinit();
        g_gpio_irq = NULL;
        (void)gpio_interrupt_deinit();

        return 1;
    }
    
    ...
}

...

/* deinit */
(void)mpu6050_dmp_deinit();
g_gpio_irq = NULL;
(void)gpio_interrupt_deinit();

return 0;
```

### Document

Online documents: [https://www.libdriver.com/docs/mpu6050/index.html](https://www.libdriver.com/docs/mpu6050/index.html).

Offline documents: /doc/html/index.html.

### Contributing

Please refer to CONTRIBUTING.md.

### License

Copyright (c) 2015 - present LibDriver All rights reserved



The MIT License (MIT) 



Permission is hereby granted, free of charge, to any person obtaining a copy

of this software and associated documentation files (the "Software"), to deal

in the Software without restriction, including without limitation the rights

to use, copy, modify, merge, publish, distribute, sublicense, and/or sell

copies of the Software, and to permit persons to whom the Software is

furnished to do so, subject to the following conditions: 



The above copyright notice and this permission notice shall be included in all

copies or substantial portions of the Software. 



THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR

IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,

FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE

AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER

LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,

OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE

SOFTWARE. 

### Contact Us

Please send an e-mail to lishifenging@outlook.com.