### 1. chip

#### 1.1 chip info

chip name : STM32F407ZGT6.

extern oscillator : 8MHz.

uart pin: TX/RX PA9/PA10.

iic pin: SCL/SDA PB8/PB9.

gpio pin: INT PB0.

### 2. shell

#### 2.1 shell parameter

baud rate: 115200.

data bits : 8.

stop bits: 1.

parity: none.

flow control: none.

### 3. mpu6050

#### 3.1 command Instruction

​           mpu6050 is a basic command which can test all mpu6050 driver function:

​           -i         show mpu6050 chip and driver information.

​           -h        show mpu6050 help.

​           -p        show mpu6050 pin connections of the current board.

​           -t  (reg -a (0 | 1) | read <times> -a (0 | 1) | fifo <times> -a (0 | 1) | dmp <times> -a (0 | 1) | motion -a (0 | 1) | pedometer <times> -a (0 | 1))

​           -t reg -a (0 | 1)        run mpu6050 register test. 

​           -t read <times> -a (0 | 1)        run mpu6050 read test. times means the test times.

​           -t fifo <times> -a (0 | 1)        run mpu6050 fifo test. times means the test times.

​           -t dmp <times> -a (0 | 1)        run mpu6050 dmp test. times means the test times.

​           -t motion -a (0 | 1)        run mpu6050 motion test. 

​           -t pedometer <times> -a (0 | 1)        run mpu6050 pedometer test. times means the test times.

​           -c  (read <times> -a (0 | 1) | fifo <times> -a (0 | 1) | dmp <times> -a (0 | 1) | motion -a (0 | 1) | pedometer <times> -a (0 | 1))

​           -c read <times> -a (0 | 1)        run mpu6050 read function. times means the read times.

​           -c fifo <times> -a (0 | 1)        run mpu6050 fifo function. times means the read times.

​           -c dmp <times> -a (0 | 1)        run mpu6050 dmp function. times means the read times.

​           -c motion -a (0 | 1)        run mpu6050 motion function. 

​           -c pedometer <times> -a (0 | 1)        run mpu6050 pedometer function. times means the read times.

#### 3.2 command example

```shell
mpu6050 -i

mpu6050: chip is TDK MPU6050.
mpu6050: manufacturer is TDK.
mpu6050: interface is IIC.
mpu6050: driver version is 1.0.
mpu6050: min supply voltage is 2.4V.
mpu6050: max supply voltage is 3.5V.
mpu6050: max current is 3.90mA.
mpu6050: max temperature is 85.0C.
mpu6050: min temperature is -40.0C.
```

```shell
mpu6050 -p

mpu6050: SCL connected to GPIOB PIN8.
mpu6050: SDA connected to GPIOB PIN9.
mpu6050: INT connected to GPIOB PIN0.
```

```shell
mpu6050 -t reg -a 0

mpu6050: chip is TDK MPU6050.
mpu6050: manufacturer is TDK.
mpu6050: interface is IIC.
mpu6050: driver version is 1.0.
mpu6050: min supply voltage is 2.4V.
mpu6050: max supply voltage is 3.5V.
mpu6050: max current is 3.90mA.
mpu6050: max temperature is 85.0C.
mpu6050: min temperature is -40.0C.
mpu6050: start register test.
mpu6050: mpu6050_set_addr_pin/mpu6050_get_addr_pin test.
mpu6050: set addr pin low.
mpu6050: check addr pin ok.
mpu6050: set addr pin high.
mpu6050: check addr pin ok.
mpu6050: mpu6050_set_fifo/mpu6050_get_fifo test.
mpu6050: enable fifo.
mpu6050: check fifo ok.
mpu6050: disable fifo.
mpu6050: check fifo ok.
mpu6050: mpu6050_set_iic_master/mpu6050_get_iic_master test.
mpu6050: enable iic master.
mpu6050: check iic master ok.
mpu6050: disable iic master.
mpu6050: check iic master ok.
mpu6050: mpu6050_fifo_reset/mpu6050_get_fifo_reset test.
mpu6050: fifo reset.
mpu6050: check fifo reset ok.
mpu6050: mpu6050_iic_master_reset/mpu6050_get_iic_master_reset test.
mpu6050: iic master reset.
mpu6050: check iic master ok.
mpu6050: mpu6050_sensor_reset/mpu6050_get_sensor_reset test.
mpu6050: sensor reset.
mpu6050: check sensor reset ok.
mpu6050: mpu6050_device_reset/mpu6050_get_device_reset test.
mpu6050: device reset.
mpu6050: check device reset ok.
mpu6050: mpu6050_set_clock_source/mpu6050_get_clock_source test.
mpu6050: stop the clock.
mpu6050: check clock source ok.
mpu6050: set the clock source extern 19.2 MHz.
mpu6050: check clock source ok.
mpu6050: set the clock source extern 32.768 KHz.
mpu6050: check clock source ok.
mpu6050: set the clock source z axis gyroscope reference.
mpu6050: check clock source ok.
mpu6050: set the clock source y axis gyroscope reference.
mpu6050: check clock source ok.
mpu6050: set the clock source x axis gyroscope reference.
mpu6050: check clock source ok.
mpu6050: set the clock source internal 8MHz.
mpu6050: check clock source ok.
mpu6050: mpu6050_set_temperature_sensor/mpu6050_get_temperature_sensor test.
mpu6050: enable temperature sensor.
mpu6050: check temperature sensor ok.
mpu6050: disable temperature sensor.
mpu6050: check temperature sensor ok.
mpu6050: mpu6050_set_cycle_wake_up/mpu6050_get_cycle_wake_up test.
mpu6050: enable cycle wake up.
mpu6050: check cycle wake up ok.
mpu6050: disable cycle wake up.
mpu6050: check cycle wake up ok.
mpu6050: mpu6050_set_sleep/mpu6050_get_sleep test.
mpu6050: enable the sleep.
mpu6050: check sleep ok.
mpu6050: disable the sleep.
mpu6050: check sleep ok.
mpu6050: mpu6050_set_standby_mode/mpu6050_get_standby_mode test.
mpu6050: enable acc x standby mode.
mpu6050: check standby mode ok.
mpu6050: disable acc x standby mode.
mpu6050: check standby mode ok.
mpu6050: enable acc y standby mode.
mpu6050: check standby mode ok.
mpu6050: disable acc y standby mode.
mpu6050: check standby mode ok.
mpu6050: enable acc z standby mode.
mpu6050: check standby mode ok.
mpu6050: disable acc z standby mode.
mpu6050: check standby mode ok.
mpu6050: enable gyro x standby mode.
mpu6050: check standby mode ok.
mpu6050: disable gyro x standby mode.
mpu6050: check standby mode ok.
mpu6050: enable gyro y standby mode.
mpu6050: check standby mode ok.
mpu6050: disable gyro y standby mode.
mpu6050: check standby mode ok.
mpu6050: enable gyro z standby mode.
mpu6050: check standby mode ok.
mpu6050: disable gyro z standby mode.
mpu6050: check standby mode ok.
mpu6050: mpu6050_set_wake_up_frequency/mpu6050_get_wake_up_frequency test.
mpu6050: set wake up frequency 1.25Hz.
mpu6050: check wake up frequency ok.
mpu6050: set wake up frequency 5Hz.
mpu6050: check wake up frequency ok.
mpu6050: set wake up frequency 20Hz.
mpu6050: check wake up frequency ok.
mpu6050: set wake up frequency 40Hz.
mpu6050: check wake up frequency ok.
mpu6050: mpu6050_fifo_get/mpu6050_fifo_set test.
mpu6050: check fifo write ok.
mpu6050: check fifo read ok.
mpu6050: mpu6050_get_fifo_count test.
mpu6050: fifo count 0.
mpu6050: mpu6050_set_signal_path_reset test.
mpu6050: temp signal path reset.
mpu6050: check signal path reset ok.
mpu6050: accel signal path reset.
mpu6050: check signal path reset ok.
mpu6050: gyro signal path reset.
mpu6050: check signal path reset ok.
mpu6050: mpu6050_set_sample_rate_divider/mpu6050_get_sample_rate_divider test.
mpu6050: set sample rate divider 0x13.
mpu6050: check sample rate divider ok.
mpu6050: mpu6050_set_extern_sync/mpu6050_get_extern_sync test.
mpu6050: set extern sync input disabled.
mpu6050: check extern sync ok.
mpu6050: set extern sync temp out low.
mpu6050: check extern sync ok.
mpu6050: set extern sync gyro xout low.
mpu6050: check extern sync ok.
mpu6050: set extern sync gyro yout low.
mpu6050: check extern sync ok.
mpu6050: set extern sync gyro zout low.
mpu6050: check extern sync ok.
mpu6050: set extern sync accel xout low.
mpu6050: check extern sync ok.
mpu6050: set extern sync accel yout low.
mpu6050: check extern sync ok.
mpu6050: set extern sync accel zout low.
mpu6050: check extern sync ok.
mpu6050: mpu6050_set_low_pass_filter/mpu6050_get_low_pass_filter test.
mpu6050: set low pass filter 0.
mpu6050: check low pass filter ok.
mpu6050: set low pass filter 1.
mpu6050: check low pass filter ok.
mpu6050: set low pass filter 2.
mpu6050: check low pass filter ok.
mpu6050: set low pass filter 3.
mpu6050: check low pass filter ok.
mpu6050: set low pass filter 4.
mpu6050: check low pass filter ok.
mpu6050: set low pass filter 5.
mpu6050: check low pass filter ok.
mpu6050: set low pass filter 6.
mpu6050: check low pass filter ok.
mpu6050: mpu6050_set_gyroscope_test/mpu6050_get_gyroscope_test test.
mpu6050: enable axis x.
mpu6050: check gyroscope test ok.
mpu6050: disable axis x.
mpu6050: check gyroscope test ok.
mpu6050: enable axis y.
mpu6050: check gyroscope test ok.
mpu6050: disable axis y.
mpu6050: check gyroscope test ok.
mpu6050: enable axis z.
mpu6050: check gyroscope test ok.
mpu6050: disable axis z.
mpu6050: check gyroscope test ok.
mpu6050: mpu6050_set_gyroscope_range/mpu6050_get_gyroscope_range test.
mpu6050: set gyroscope range 250 dps.
mpu6050: check gyroscope range ok.
mpu6050: set gyroscope range 500 dps.
mpu6050: check gyroscope range ok.
mpu6050: set gyroscope range 1000 dps.
mpu6050: check gyroscope range ok.
mpu6050: set gyroscope range 2000 dps.
mpu6050: check gyroscope range ok.
mpu6050: mpu6050_set_accelerometer_test/mpu6050_get_accelerometer_test test.
mpu6050: enable accelerometer x.
mpu6050: check accelerometer test ok.
mpu6050: disable accelerometer x.
mpu6050: check accelerometer test ok.
mpu6050: enable accelerometer y.
mpu6050: check accelerometer test ok.
mpu6050: disable accelerometer y.
mpu6050: check accelerometer test ok.
mpu6050: enable accelerometer z.
mpu6050: check accelerometer test ok.
mpu6050: disable accelerometer z.
mpu6050: check accelerometer test ok.
mpu6050: mpu6050_set_accelerometer_range/mpu6050_get_accelerometer_range test.
mpu6050: set accelerometer range 2g.
mpu6050: check accelerometer range ok.
mpu6050: set accelerometer range 4g.
mpu6050: check accelerometer range ok.
mpu6050: set accelerometer range 8g.
mpu6050: check accelerometer range ok.
mpu6050: set accelerometer range 16g.
mpu6050: check accelerometer range ok.
mpu6050: mpu6050_set_fifo_enable/mpu6050_get_fifo_enable test.
mpu6050: set fifo temp enable.
mpu6050: check fifo enable ok.
mpu6050: set fifo temp disable.
mpu6050: check fifo enable ok.
mpu6050: set fifo gyroscope x enable.
mpu6050: check fifo enable ok.
mpu6050: set fifo gyroscope x disable.
mpu6050: check fifo enable ok.
mpu6050: set fifo gyroscope y enable.
mpu6050: check fifo enable ok.
mpu6050: set fifo gyroscope y disable.
mpu6050: check fifo enable ok.
mpu6050: set fifo gyroscope z enable.
mpu6050: check fifo enable ok.
mpu6050: set fifo gyroscope z disable.
mpu6050: check fifo enable ok.
mpu6050: set fifo accelerometer enable.
mpu6050: check fifo enable ok.
mpu6050: set fifo accelerometer disable.
mpu6050: check fifo enable ok.
mpu6050: mpu6050_set_interrupt_level/mpu6050_get_interrupt_level test.
mpu6050: set interrupt high level.
mpu6050: check interrupt level ok.
mpu6050: set interrupt low level.
mpu6050: check interrupt level ok.
mpu6050: mpu6050_set_interrupt_pin_type/mpu6050_get_interrupt_pin_type test.
mpu6050: set interrupt pin type push pull.
mpu6050: check interrupt pin type ok.
mpu6050: set interrupt pin type open drain.
mpu6050: check interrupt pin type ok.
mpu6050: mpu6050_set_interrupt_latch/mpu6050_get_interrupt_latch test.
mpu6050: enable interrupt latch.
mpu6050: check interrupt latch ok.
mpu6050: disable interrupt latch.
mpu6050: check interrupt latch ok.
mpu6050: mpu6050_set_interrupt_read_clear/mpu6050_get_interrupt_read_clear test.
mpu6050: enable interrupt read clear.
mpu6050: check interrupt read clear ok.
mpu6050: disable interrupt read clear.
mpu6050: check interrupt read clear ok.
mpu6050: mpu6050_set_fsync_interrupt_level/mpu6050_get_fsync_interrupt_level test.
mpu6050: set fsync interrupt level high.
mpu6050: check fsync interrupt level ok.
mpu6050: set fsync interrupt level low.
mpu6050: check fsync interrupt level ok.
mpu6050: mpu6050_set_fsync_interrupt/mpu6050_get_fsync_interrupt test.
mpu6050: enable fsync interrupt.
mpu6050: check fsync interrupt ok.
mpu6050: disable fsync interrupt.
mpu6050: check fsync interrupt ok.
mpu6050: mpu6050_set_iic_bypass/mpu6050_get_iic_bypass test.
mpu6050: enable iic bypass.
mpu6050: check iic bypass ok.
mpu6050: disable iic bypass.
mpu6050: check iic bypass ok.
mpu6050: mpu6050_set_interrupt/mpu6050_get_interrupt test.
mpu6050: enable motion interrupt.
mpu6050: check motion interrupt ok.
mpu6050: disable motion interrupt.
mpu6050: check motion interrupt ok.
mpu6050: enable fifo overflow interrupt.
mpu6050: check fifo overflow interrupt ok.
mpu6050: disable fifo overflow interrupt.
mpu6050: check fifo overflow interrupt ok.
mpu6050: enable dmp interrupt.
mpu6050: check dmp interrupt ok.
mpu6050: disable dmp interrupt.
mpu6050: check dmp interrupt ok.
mpu6050: enable i2c master interrupt.
mpu6050: check i2c master interrupt ok.
mpu6050: disable i2c master interrupt.
mpu6050: check i2c master interrupt ok.
mpu6050: enable data ready interrupt.
mpu6050: check data ready interrupt ok.
mpu6050: disable data ready interrupt.
mpu6050: check data ready interrupt ok.
mpu6050: mpu6050_get_interrupt_status test.
mpu6050: get interrupt status 0x01.
mpu6050: mpu6050_set_gyroscope_x_test/mpu6050_get_gyroscope_x_test test.
mpu6050: set gyroscope x test 0x04.
mpu6050: check gyroscope x test ok.
mpu6050: mpu6050_set_gyroscope_y_test/mpu6050_get_gyroscope_y_test test.
mpu6050: set gyroscope y test 0x0C.
mpu6050: check gyroscope y test ok.
mpu6050: mpu6050_set_gyroscope_z_test/mpu6050_get_gyroscope_z_test test.
mpu6050: set gyroscope z test 0x0B.
mpu6050: check gyroscope z test ok.
mpu6050: mpu6050_set_accelerometer_x_test/mpu6050_get_accelerometer_x_test test.
mpu6050: set accelerometer x test 0x1A.
mpu6050: check accelerometer x test ok.
mpu6050: mpu6050_set_accelerometer_y_test/mpu6050_get_accelerometer_y_test test.
mpu6050: set accelerometer y test 0x01.
mpu6050: check accelerometer y test ok.
mpu6050: mpu6050_set_accelerometer_z_test/mpu6050_get_accelerometer_z_test test.
mpu6050: set accelerometer z test 0x05.
mpu6050: check accelerometer z test ok.
mpu6050: mpu6050_set_motion_threshold/mpu6050_get_motion_threshold test.
mpu6050: set motion threshold 0xA1.
mpu6050: check motion threshold ok.
mpu6050: mpu6050_motion_threshold_convert_to_register/mpu6050_motion_threshold_convert_to_data test.
mpu6050: motion threshold convert to register 182.60.
mpu6050: check motion threshold 160.00.
mpu6050: mpu6050_set_motion_duration/mpu6050_get_motion_duration test.
mpu6050: set motion duration 0x64.
mpu6050: check motion duration ok.
mpu6050: mpu6050_motion_duration_convert_to_register/mpu6050_motion_duration_convert_to_data test.
mpu6050: set motion duration 0x39.
mpu6050: check motion duration 0x39.
mpu6050: mpu6050_set_force_accel_sample test.
mpu6050: enable force accel sample.
mpu6050: check force accel sample ok.
mpu6050: disable force accel sample.
mpu6050: check force accel sample ok.
mpu6050: mpu6050_set_iic_clock/mpu6050_get_iic_clock test.
mpu6050: set iic clock 348 kHz.
mpu6050: check iic clock ok.
mpu6050: set iic clock 333 kHz.
mpu6050: check iic clock ok.
mpu6050: set iic clock 320 kHz.
mpu6050: check iic clock ok.
mpu6050: set iic clock 308 kHz.
mpu6050: check iic clock ok.
mpu6050: set iic clock 296 kHz.
mpu6050: check iic clock ok.
mpu6050: set iic clock 286 kHz.
mpu6050: check iic clock ok.
mpu6050: set iic clock 276 kHz.
mpu6050: check iic clock ok.
mpu6050: set iic clock 267 kHz.
mpu6050: check iic clock ok.
mpu6050: set iic clock 258 kHz.
mpu6050: check iic clock ok.
mpu6050: set iic clock 500 kHz.
mpu6050: check iic clock ok.
mpu6050: set iic clock 471 kHz.
mpu6050: check iic clock ok.
mpu6050: set iic clock 444 kHz.
mpu6050: check iic clock ok.
mpu6050: set iic clock 421 kHz.
mpu6050: check iic clock ok.
mpu6050: set iic clock 400 kHz.
mpu6050: check iic clock ok.
mpu6050: set iic clock 381 kHz.
mpu6050: check iic clock ok.
mpu6050: set iic clock 364 kHz.
mpu6050: check iic clock ok.
mpu6050: mpu6050_set_iic_multi_master/mpu6050_get_iic_multi_master test.
mpu6050: enable iic multi master.
mpu6050: check iic multi master ok.
mpu6050: disable iic multi master.
mpu6050: check iic multi master ok.
mpu6050: mpu6050_set_iic_wait_for_external_sensor/mpu6050_get_iic_wait_for_external_sensor test.
mpu6050: enable iic wait for external sensor.
mpu6050: check iic wait for external sensor ok.
mpu6050: disable iic wait for external sensor.
mpu6050: check iic wait for external sensor ok.
mpu6050: mpu6050_set_iic_read_mode/mpu6050_get_iic_read_mode test.
mpu6050: set restart read mode.
mpu6050: check iic read mode ok.
mpu6050: set stop and start read mode.
mpu6050: check iic read mode ok.
mpu6050: mpu6050_set_iic_fifo_enable/mpu6050_get_iic_fifo_enable test.
mpu6050: enable iic fifo slave0.
mpu6050: check iic fifo enable ok.
mpu6050: disable iic fifo slave0.
mpu6050: check iic fifo enable ok.
mpu6050: enable iic fifo slave1.
mpu6050: check iic fifo enable ok.
mpu6050: disable iic fifo slave1.
mpu6050: check iic fifo enable ok.
mpu6050: enable iic fifo slave2.
mpu6050: check iic fifo enable ok.
mpu6050: disable iic fifo slave2.
mpu6050: check iic fifo enable ok.
mpu6050: enable iic fifo slave3.
mpu6050: check iic fifo enable ok.
mpu6050: disable iic fifo slave3.
mpu6050: check iic fifo enable ok.
mpu6050: mpu6050_set_iic_mode/mpu6050_get_iic_mode test.
mpu6050: set slave0 iic write mode.
mpu6050: check iic mode ok.
mpu6050: set slave0 iic read mode.
mpu6050: check iic mode ok.
mpu6050: set slave1 iic write mode.
mpu6050: check iic mode ok.
mpu6050: set slave1 iic read mode.
mpu6050: check iic mode ok.
mpu6050: set slave2 iic write mode.
mpu6050: check iic mode ok.
mpu6050: set slave2 iic read mode.
mpu6050: check iic mode ok.
mpu6050: set slave3 iic write mode.
mpu6050: check iic mode ok.
mpu6050: set slave3 iic read mode.
mpu6050: check iic mode ok.
mpu6050: set slave4 iic write mode.
mpu6050: check iic mode ok.
mpu6050: set slave4 iic read mode.
mpu6050: check iic mode ok.
mpu6050: mpu6050_set_iic_address/mpu6050_get_iic_address test.
mpu6050: set slave0 iic address 0x31.
mpu6050: check iic address ok.
mpu6050: set slave1 iic address 0x05.
mpu6050: check iic address ok.
mpu6050: set slave2 iic address 0x7A.
mpu6050: check iic address ok.
mpu6050: set slave3 iic address 0x46.
mpu6050: check iic address ok.
mpu6050: set slave4 iic address 0x44.
mpu6050: check iic address ok.
mpu6050: mpu6050_set_iic_register/mpu6050_get_iic_register test.
mpu6050: set slave0 iic register 0x39.
mpu6050: check iic register ok.
mpu6050: set slave1 iic register 0x68.
mpu6050: check iic register ok.
mpu6050: set slave2 iic register 0xE8.
mpu6050: check iic register ok.
mpu6050: set slave3 iic register 0xC9.
mpu6050: check iic register ok.
mpu6050: set slave4 iic register 0xAA.
mpu6050: check iic register ok.
mpu6050: mpu6050_set_iic_data_out/mpu6050_get_iic_data_out test.
mpu6050: set slave0 iic data out 0x5C.
mpu6050: check iic data out ok.
mpu6050: set slave1 iic data out 0x9F.
mpu6050: check iic data out ok.
mpu6050: set slave2 iic data out 0x30.
mpu6050: check iic data out ok.
mpu6050: set slave3 iic data out 0xC1.
mpu6050: check iic data out ok.
mpu6050: mpu6050_set_iic_enable/mpu6050_get_iic_enable test.
mpu6050: slave0 iic enable.
mpu6050: check iic enable ok.
mpu6050: slave0 iic disable.
mpu6050: check iic enable ok.
mpu6050: slave1 iic enable.
mpu6050: check iic enable ok.
mpu6050: slave1 iic disable.
mpu6050: check iic enable ok.
mpu6050: slave2 iic enable.
mpu6050: check iic enable ok.
mpu6050: slave2 iic disable.
mpu6050: check iic enable ok.
mpu6050: slave3 iic enable.
mpu6050: check iic enable ok.
mpu6050: slave3 iic disable.
mpu6050: check iic enable ok.
mpu6050: mpu6050_set_iic_byte_swap/mpu6050_get_iic_byte_swap test.
mpu6050: enable slave0 byte swap.
mpu6050: check iic byte swap ok.
mpu6050: disable slave0 byte swap.
mpu6050: check iic byte swap ok.
mpu6050: enable slave1 byte swap.
mpu6050: check iic byte swap ok.
mpu6050: disable slave1 byte swap.
mpu6050: check iic byte swap ok.
mpu6050: enable slave2 byte swap.
mpu6050: check iic byte swap ok.
mpu6050: disable slave2 byte swap.
mpu6050: check iic byte swap ok.
mpu6050: enable slave3 byte swap.
mpu6050: check iic byte swap ok.
mpu6050: disable slave3 byte swap.
mpu6050: check iic byte swap ok.
mpu6050: mpu6050_set_iic_transaction_mode/mpu6050_get_iic_transaction_mode test.
mpu6050: set slave0 data transaction mode.
mpu6050: check iic transaction mode ok.
mpu6050: set slave0 reg data transaction mode.
mpu6050: check iic transaction mode ok.
mpu6050: set slave1 data transaction mode.
mpu6050: check iic transaction mode ok.
mpu6050: set slave1 reg data transaction mode.
mpu6050: check iic transaction mode ok.
mpu6050: set slave2 data transaction mode.
mpu6050: check iic transaction mode ok.
mpu6050: set slave2 reg data transaction mode.
mpu6050: check iic transaction mode ok.
mpu6050: set slave3 data transaction mode.
mpu6050: check iic transaction mode ok.
mpu6050: set slave3 reg data transaction mode.
mpu6050: check iic transaction mode ok.
mpu6050: mpu6050_set_iic_group_order/mpu6050_get_iic_group_order test.
mpu6050: set slave0 group order even.
mpu6050: check iic group order ok.
mpu6050: set slave0 group order odd.
mpu6050: check iic group order ok.
mpu6050: set slave1 group order even.
mpu6050: check iic group order ok.
mpu6050: set slave1 group order odd.
mpu6050: check iic group order ok.
mpu6050: set slave2 group order even.
mpu6050: check iic group order ok.
mpu6050: set slave2 group order odd.
mpu6050: check iic group order ok.
mpu6050: set slave3 group order even.
mpu6050: check iic group order ok.
mpu6050: set slave3 group order odd.
mpu6050: check iic group order ok.
mpu6050: mpu6050_set_iic_transferred_len/mpu6050_get_iic_transferred_len test.
mpu6050: set slave0 iic transferred len 8.
mpu6050: check iic transferred len ok.
mpu6050: set slave1 iic transferred len 10.
mpu6050: check iic transferred len ok.
mpu6050: set slave2 iic transferred len 9.
mpu6050: check iic transferred len ok.
mpu6050: set slave3 iic transferred len 15.
mpu6050: check iic transferred len ok.
mpu6050: mpu6050_get_iic_status test.
mpu6050: iic status is 0x00.
mpu6050: mpu6050_set_iic_delay_enable/mpu6050_get_iic_delay_enable test.
mpu6050: enable delay shadow.
mpu6050: check iic delay enable ok.
mpu6050: disable delay shadow.
mpu6050: check iic delay enable ok.
mpu6050: enable delay slave4.
mpu6050: check iic delay enable ok.
mpu6050: disable delay slave4.
mpu6050: check iic delay enable ok.
mpu6050: enable delay slave3.
mpu6050: check iic delay enable ok.
mpu6050: disable delay slave3.
mpu6050: check iic delay enable ok.
mpu6050: enable delay slave2.
mpu6050: check iic delay enable ok.
mpu6050: disable delay slave2.
mpu6050: check iic delay enable ok.
mpu6050: enable delay slave1.
mpu6050: check iic delay enable ok.
mpu6050: disable delay slave1.
mpu6050: check iic delay enable ok.
mpu6050: enable delay slave0.
mpu6050: check iic delay enable ok.
mpu6050: disable delay slave0.
mpu6050: check iic delay enable ok.
mpu6050: mpu6050_set_iic4_enable/mpu6050_get_iic4_enable test.
mpu6050: enable iic4.
mpu6050: check iic4 enable ok.
mpu6050: disable iic4.
mpu6050: check iic4 enable ok.
mpu6050: mpu6050_set_iic4_interrupt/mpu6050_get_iic4_interrupt test.
mpu6050: enable iic4 interrupt.
mpu6050: check iic4 interrupt ok.
mpu6050: disable iic4 interrupt.
mpu6050: check iic4 interrupt ok.
mpu6050: mpu6050_set_iic4_transaction_mode/mpu6050_get_iic4_transaction_mode test.
mpu6050: set iic4 transaction mode data.
mpu6050: check iic4 transaction mode ok.
mpu6050: set iic4 transaction mode reg.
mpu6050: check iic4 transaction mode ok.
mpu6050: mpu6050_set_iic_delay/mpu6050_get_iic_delay test.
mpu6050: set iic delay 0x1E.
mpu6050: check iic delay ok.
mpu6050: mpu6050_set_iic4_data_out/mpu6050_get_iic4_data_out test.
mpu6050: set iic4 data out 0xDC.
mpu6050: check iic4 data out ok.
mpu6050: mpu6050_set_iic4_data_in/mpu6050_get_iic4_data_in test.
mpu6050: set iic4 data in 0x1B.
mpu6050: check iic4 data in ok.
mpu6050: finish register test.
```

```shell
mpu6050 -t read 3 -a 0 

mpu6050: chip is TDK MPU6050.
mpu6050: manufacturer is TDK.
mpu6050: interface is IIC.
mpu6050: driver version is 1.0.
mpu6050: min supply voltage is 2.4V.
mpu6050: max supply voltage is 3.5V.
mpu6050: max current is 3.90mA.
mpu6050: max temperature is 85.0C.
mpu6050: min temperature is -40.0C.
mpu6050: start read test.
mpu6050: set accelerometer range 2g.
mpu6050: acc x is -0.20g.
mpu6050: acc y is -0.18g.
mpu6050: acc z is 0.98g.
mpu6050: acc x is -0.20g.
mpu6050: acc y is -0.18g.
mpu6050: acc z is 0.98g.
mpu6050: acc x is -0.20g.
mpu6050: acc y is -0.18g.
mpu6050: acc z is 0.98g.
mpu6050: set accelerometer range 4g.
mpu6050: acc x is -0.20g.
mpu6050: acc y is -0.18g.
mpu6050: acc z is 0.97g.
mpu6050: acc x is -0.20g.
mpu6050: acc y is -0.18g.
mpu6050: acc z is 0.98g.
mpu6050: acc x is -0.20g.
mpu6050: acc y is -0.18g.
mpu6050: acc z is 0.98g.
mpu6050: set accelerometer range 8g.
mpu6050: acc x is -0.20g.
mpu6050: acc y is -0.18g.
mpu6050: acc z is 0.97g.
mpu6050: acc x is -0.20g.
mpu6050: acc y is -0.18g.
mpu6050: acc z is 0.98g.
mpu6050: acc x is -0.20g.
mpu6050: acc y is -0.18g.
mpu6050: acc z is 0.98g.
mpu6050: set accelerometer range 16g.
mpu6050: acc x is -0.20g.
mpu6050: acc y is -0.18g.
mpu6050: acc z is 0.97g.
mpu6050: acc x is -0.20g.
mpu6050: acc y is -0.18g.
mpu6050: acc z is 0.98g.
mpu6050: acc x is -0.20g.
mpu6050: acc y is -0.18g.
mpu6050: acc z is 0.98g.
mpu6050: set gyroscope range 250dps.
mpu6050: gyro x is -0.11dps.
mpu6050: gyro y is 1.02dps.
mpu6050: gyro z is 0.19dps.
mpu6050: gyro x is -0.02dps.
mpu6050: gyro y is 1.02dps.
mpu6050: gyro z is 0.22dps.
mpu6050: gyro x is -0.02dps.
mpu6050: gyro y is 0.99dps.
mpu6050: gyro z is 0.22dps.
mpu6050: set gyroscope range 500dps.
mpu6050: gyro x is -0.05dps.
mpu6050: gyro y is 0.96dps.
mpu6050: gyro z is 0.20dps.
mpu6050: gyro x is 0.06dps.
mpu6050: gyro y is 0.92dps.
mpu6050: gyro z is 0.17dps.
mpu6050: gyro x is -0.06dps.
mpu6050: gyro y is 0.93dps.
mpu6050: gyro z is 0.11dps.
mpu6050: set gyroscope range 1000dps.
mpu6050: gyro x is -0.09dps.
mpu6050: gyro y is 1.10dps.
mpu6050: gyro z is 0.21dps.
mpu6050: gyro x is 0.00dps.
mpu6050: gyro y is 0.91dps.
mpu6050: gyro z is 0.03dps.
mpu6050: gyro x is -0.03dps.
mpu6050: gyro y is 1.01dps.
mpu6050: gyro z is 0.15dps.
mpu6050: set gyroscope range 2000dps.
mpu6050: gyro x is -0.12dps.
mpu6050: gyro y is 0.98dps.
mpu6050: gyro z is 0.18dps.
mpu6050: gyro x is -0.06dps.
mpu6050: gyro y is 0.91dps.
mpu6050: gyro z is 0.06dps.
mpu6050: gyro x is -0.06dps.
mpu6050: gyro y is 0.98dps.
mpu6050: gyro z is 0.12dps.
mpu6050: read temperature.
mpu6050: temperature 26.85C.
mpu6050: temperature 26.87C.
mpu6050: temperature 26.86C.
mpu6050: finish read test.
```

```shell
mpu6050 -t fifo 3 -a 0

mpu6050: chip is TDK MPU6050.
mpu6050: manufacturer is TDK.
mpu6050: interface is IIC.
mpu6050: driver version is 1.0.
mpu6050: min supply voltage is 2.4V.
mpu6050: max supply voltage is 3.5V.
mpu6050: max current is 3.90mA.
mpu6050: max temperature is 85.0C.
mpu6050: min temperature is -40.0C.
mpu6050: start fifo test.
mpu6050: fifo 48.
mpu6050: acc x[0] is -0.20g.
mpu6050: acc y[0] is -0.18g.
mpu6050: acc z[0] is 0.97g.
mpu6050: gyro x[0] is -10.49dps.
mpu6050: gyro y[0] is -0.61dps.
mpu6050: gyro z[0] is 29.02dps.
mpu6050: fifo 50.
mpu6050: acc x[0] is -0.20g.
mpu6050: acc y[0] is -0.18g.
mpu6050: acc z[0] is 0.97g.
mpu6050: gyro x[0] is 0.00dps.
mpu6050: gyro y[0] is 0.91dps.
mpu6050: gyro z[0] is 0.12dps.
mpu6050: fifo 50.
mpu6050: acc x[0] is -0.20g.
mpu6050: acc y[0] is -0.18g.
mpu6050: acc z[0] is 0.97g.
mpu6050: gyro x[0] is -0.06dps.
mpu6050: gyro y[0] is 0.98dps.
mpu6050: gyro z[0] is 0.18dps.
mpu6050: finish fifo test.
```

```shell
mpu6050 -t dmp 3 -a 0

mpu6050: chip is TDK MPU6050.
mpu6050: manufacturer is TDK.
mpu6050: interface is IIC.
mpu6050: driver version is 1.0.
mpu6050: min supply voltage is 2.4V.
mpu6050: max supply voltage is 3.5V.
mpu6050: max current is 3.90mA.
mpu6050: max temperature is 85.0C.
mpu6050: min temperature is -40.0C.
mpu6050: start dmp read test.
mpu6050: load dmp firmware.
mpu6050: load dmp firmware successful .
mpu6050: mpu6050_dmp_set_pedometer_walk_time/mpu6050_dmp_get_pedometer_walk_time test.
mpu6050: dmp set pedometer walk time 200 ms.
mpu6050: check pedometer walk time ok.
mpu6050: mpu6050_dmp_set_pedometer_step_count/mpu6050_dmp_get_pedometer_step_count test.
mpu6050: dmp set pedometer step count 768.
mpu6050: check pedometer step count ok.
mpu6050: mpu6050_dmp_set_shake_reject_timeout/mpu6050_dmp_get_shake_reject_timeout test.
mpu6050: dmp set shake reject timeout 10 ms.
mpu6050: check shake reject timeout ok.
mpu6050: mpu6050_dmp_set_shake_reject_time/mpu6050_dmp_get_shake_reject_time test.
mpu6050: dmp set shake reject time 40 ms.
mpu6050: check shake reject time ok.
mpu6050: mpu6050_dmp_set_shake_reject_thresh/mpu6050_dmp_get_shake_reject_thresh test.
mpu6050: set shake reject thresh 200 dps.
mpu6050: check shake reject thresh ok.
mpu6050: mpu6050_dmp_set_tap_time_multi/mpu6050_dmp_get_tap_time_multi test.
mpu6050: dmp set tap time multi 500 ms.
mpu6050: check tap time multi ok.
mpu6050: mpu6050_dmp_set_tap_time/mpu6050_dmp_get_tap_time test.
mpu6050: dmp set tap time 100 ms.
mpu6050: check tap time ok.
mpu6050: mpu6050_dmp_set_min_tap_count/mpu6050_dmp_get_min_tap_count test.
mpu6050: dmp set min tap count 1.
mpu6050: check min tap count ok.
mpu6050: mpu6050_dmp_set_tap_axes/mpu6050_dmp_get_tap_axes test.
mpu6050: disable tap axes x.
mpu6050: check tap axes ok.
mpu6050: enable tap axes x.
mpu6050: check tap axes ok.
mpu6050: disable tap axes y.
mpu6050: check tap axes ok.
mpu6050: enable tap axes y.
mpu6050: check tap axes ok.
mpu6050: disable tap axes z.
mpu6050: check tap axes ok.
mpu6050: enable tap axes z.
mpu6050: check tap axes ok.
mpu6050: mpu6050_dmp_set_tap_thresh/mpu6050_dmp_get_tap_thresh test.
mpu6050: dmp set tap thresh x 250 mg/ms.
mpu6050: check tap thresh ok.
mpu6050: dmp set tap thresh y 250 mg/ms.
mpu6050: check tap thresh ok.
mpu6050: dmp set tap thresh z 250 mg/ms.
mpu6050: check tap thresh ok.
mpu6050: mpu6050_dmp_set_fifo_rate/mpu6050_dmp_get_fifo_rate test.
mpu6050: dmp set fifo rate 200Hz.
mpu6050: check fifo rate ok.
mpu6050: mpu6050_dmp_set_gyro_calibrate test.
mpu6050: enable gyro calibrate.
mpu6050: disable gyro calibrate.
mpu6050: mpu6050_dmp_set_3x_quaternion test.
mpu6050: enable 3x quaternion.
mpu6050: disable 3x quaternion.
mpu6050: mpu6050_dmp_set_6x_quaternion test.
mpu6050: enable 6x quaternion.
mpu6050: disable 6x quaternion.
mpu6050: mpu6050_dmp_set_interrupt_mode test.
mpu6050: dmp set gesture interrupt mode.
mpu6050: dmp set gesture continuous mode.
mpu6050: mpu6050_dmp_set_orientation test.
mpu6050: set the dmp orientation.
mpu6050: mpu6050_dmp_set_feature test.
mpu6050: enable feature 6x quat.
mpu6050: enable feature tap.
mpu6050: enable feature pedometer.
mpu6050: enable feature orient.
mpu6050: enable feature send raw accel.
mpu6050: enable feature send cal gyro.
mpu6050: enable feature gyro cal.
mpu6050: fifo 5.
mpu6050: pitch[0] is 0.56dps.
mpu6050: roll[0] is 0.93dps.
mpu6050: yaw[0] is 0.01dps.
mpu6050: acc x[0] is -0.20g.
mpu6050: acc y[0] is -0.18g.
mpu6050: acc z[0] is 0.98g.
mpu6050: gyro x[0] is -0.06dps.
mpu6050: gyro y[0] is 0.79dps.
mpu6050: gyro z[0] is 0.06dps.
mpu6050: fifo 25.
mpu6050: pitch[0] is 0.78dps.
mpu6050: roll[0] is 0.72dps.
mpu6050: yaw[0] is 0.01dps.
mpu6050: acc x[0] is -0.20g.
mpu6050: acc y[0] is -0.18g.
mpu6050: acc z[0] is 0.97g.
mpu6050: gyro x[0] is -0.06dps.
mpu6050: gyro y[0] is -0.12dps.
mpu6050: gyro z[0] is -0.12dps.
mpu6050: fifo 30.
mpu6050: pitch[0] is 0.70dps.
mpu6050: roll[0] is 0.65dps.
mpu6050: yaw[0] is 0.01dps.
mpu6050: acc x[0] is -0.20g.
mpu6050: acc y[0] is -0.18g.
mpu6050: acc z[0] is 0.98g.
mpu6050: gyro x[0] is -0.06dps.
mpu6050: gyro y[0] is -0.12dps.
mpu6050: gyro z[0] is -0.06dps.
mpu6050: finish dmp read test.
```

```shell
mpu6050 -t motion -a 0

mpu6050: chip is TDK MPU6050.
mpu6050: manufacturer is TDK.
mpu6050: interface is IIC.
mpu6050: driver version is 1.0.
mpu6050: min supply voltage is 2.4V.
mpu6050: max supply voltage is 3.5V.
mpu6050: max current is 3.90mA.
mpu6050: max temperature is 85.0C.
mpu6050: min temperature is -40.0C.
mpu6050: start dmp tap orient motion test.
mpu6050: load dmp firmware.
mpu6050: load dmp firmware successful .
mpu6050: mpu6050_dmp_set_pedometer_walk_time/mpu6050_dmp_get_pedometer_walk_time test.
mpu6050: dmp set pedometer walk time 200 ms.
mpu6050: check pedometer walk time ok.
mpu6050: mpu6050_dmp_set_pedometer_step_count/mpu6050_dmp_get_pedometer_step_count test.
mpu6050: dmp set pedometer step count 768.
mpu6050: check pedometer step count ok.
mpu6050: mpu6050_dmp_set_shake_reject_timeout/mpu6050_dmp_get_shake_reject_timeout test.
mpu6050: dmp set shake reject timeout 10 ms.
mpu6050: check shake reject timeout ok.
mpu6050: mpu6050_dmp_set_shake_reject_time/mpu6050_dmp_get_shake_reject_time test.
mpu6050: dmp set shake reject time 40 ms.
mpu6050: check shake reject time ok.
mpu6050: mpu6050_dmp_set_shake_reject_thresh/mpu6050_dmp_get_shake_reject_thresh test.
mpu6050: set shake reject thresh 200 dps.
mpu6050: check shake reject thresh ok.
mpu6050: mpu6050_dmp_set_tap_time_multi/mpu6050_dmp_get_tap_time_multi test.
mpu6050: dmp set tap time multi 200 ms.
mpu6050: check tap time multi ok.
mpu6050: mpu6050_dmp_set_tap_time/mpu6050_dmp_get_tap_time test.
mpu6050: dmp set tap time 100 ms.
mpu6050: check tap time ok.
mpu6050: mpu6050_dmp_set_min_tap_count/mpu6050_dmp_get_min_tap_count test.
mpu6050: dmp set min tap count 1.
mpu6050: check min tap count ok.
mpu6050: mpu6050_dmp_set_tap_axes/mpu6050_dmp_get_tap_axes test.
mpu6050: disable tap axes x.
mpu6050: check tap axes ok.
mpu6050: enable tap axes x.
mpu6050: check tap axes ok.
mpu6050: disable tap axes y.
mpu6050: check tap axes ok.
mpu6050: enable tap axes y.
mpu6050: check tap axes ok.
mpu6050: disable tap axes z.
mpu6050: check tap axes ok.
mpu6050: enable tap axes z.
mpu6050: check tap axes ok.
mpu6050: mpu6050_dmp_set_tap_thresh/mpu6050_dmp_get_tap_thresh test.
mpu6050: dmp set tap thresh x 250 mg/ms.
mpu6050: check tap thresh ok.
mpu6050: dmp set tap thresh y 250 mg/ms.
mpu6050: check tap thresh ok.
mpu6050: dmp set tap thresh z 250 mg/ms.
mpu6050: check tap thresh ok.
mpu6050: mpu6050_dmp_set_fifo_rate/mpu6050_dmp_get_fifo_rate test.
mpu6050: dmp set fifo rate 50Hz.
mpu6050: check fifo rate ok.
mpu6050: mpu6050_dmp_set_gyro_calibrate test.
mpu6050: enable gyro calibrate.
mpu6050: disable gyro calibrate.
mpu6050: mpu6050_dmp_set_3x_quaternion test.
mpu6050: enable 3x quaternion.
mpu6050: disable 3x quaternion.
mpu6050: mpu6050_dmp_set_6x_quaternion test.
mpu6050: enable 6x quaternion.
mpu6050: disable 6x quaternion.
mpu6050: mpu6050_dmp_set_interrupt_mode test.
mpu6050: dmp set gesture interrupt mode.
mpu6050: dmp set gesture continuous mode.
mpu6050: mpu6050_dmp_set_orientation test.
mpu6050: set the dmp orientation.
mpu6050: mpu6050_dmp_set_feature test.
mpu6050: enable feature 6x quat.
mpu6050: enable feature tap.
mpu6050: enable feature pedometer.
mpu6050: enable feature orient.
mpu6050: enable feature send raw accel.
mpu6050: enable feature send cal gyro.
mpu6050: enable feature gyro cal.
mpu6050: orient irq reverse landscape.
mpu6050: irq motion.
mpu6050: irq dmp
mpu6050: irq data ready
mpu6050: tap irq x up with 2.
mpu6050: finish dmp tap orient motion test.
```

```shell
mpu6050 -t pedometer 3 -a 0

mpu6050: chip is TDK MPU6050.
mpu6050: manufacturer is TDK.
mpu6050: interface is IIC.
mpu6050: driver version is 1.0.
mpu6050: min supply voltage is 2.4V.
mpu6050: max supply voltage is 3.5V.
mpu6050: max current is 3.90mA.
mpu6050: max temperature is 85.0C.
mpu6050: min temperature is -40.0C.
mpu6050: start dmp pedometer test.
mpu6050: load dmp firmware.
mpu6050: load dmp firmware successful .
mpu6050: mpu6050_dmp_set_pedometer_walk_time/mpu6050_dmp_get_pedometer_walk_time test.
mpu6050: dmp set pedometer walk time 200 ms.
mpu6050: check pedometer walk time ok.
mpu6050: mpu6050_dmp_set_pedometer_step_count/mpu6050_dmp_get_pedometer_step_count test.
mpu6050: dmp set pedometer step count 667.
mpu6050: check pedometer step count ok.
mpu6050: dmp set gesture continuous mode.
mpu6050: mpu6050_dmp_set_fifo_rate/mpu6050_dmp_get_fifo_rate test.
mpu6050: dmp set fifo rate 50Hz.
mpu6050: check fifo rate ok.
mpu6050: mpu6050_dmp_set_feature test.
mpu6050: enable feature pedometer.
mpu6050: pedometer step count is 775.
mpu6050: pedometer step count is 776.
mpu6050: pedometer step count is 777.
mpu6050: finish dmp pedometer test.
```

```shell
mpu6050 -c read 3 -a 0

mpu6050: 1/3.
mpu6050: acc x is -0.01g.
mpu6050: acc y is -0.04g.
mpu6050: acc z is 0.12g.
mpu6050: gyro x is 154.51dps.
mpu6050: gyro y is 25.61dps.
mpu6050: gyro z is 527.38dps.
mpu6050: temperature 26.51C.
mpu6050: 2/3.
mpu6050: acc x is -0.09g.
mpu6050: acc y is -0.33g.
mpu6050: acc z is 0.95g.
mpu6050: gyro x is 0.00dps.
mpu6050: gyro y is 0.98dps.
mpu6050: gyro z is 0.18dps.
mpu6050: temperature 26.56C.
mpu6050: 3/3.
mpu6050: acc x is -0.09g.
mpu6050: acc y is -0.33g.
mpu6050: acc z is 0.95g.
mpu6050: gyro x is 0.00dps.
mpu6050: gyro y is 0.98dps.
mpu6050: gyro z is 0.18dps.
mpu6050: temperature 26.57C.
```

```shell
mpu6050 -c fifo 3 -a 0

mpu6050: 1/3.
mpu6050: fifo 18.
mpu6050: acc x[0] is -0.08g.
mpu6050: acc y[0] is -0.29g.
mpu6050: acc z[0] is 0.84g.
mpu6050: gyro x[0] is -20.00dps.
mpu6050: gyro y[0] is -23.17dps.
mpu6050: gyro z[0] is 24.57dps.
mpu6050: 2/3.
mpu6050: fifo 23.
mpu6050: acc x[0] is -0.09g.
mpu6050: acc y[0] is -0.33g.
mpu6050: acc z[0] is 0.95g.
mpu6050: gyro x[0] is 0.00dps.
mpu6050: gyro y[0] is 0.91dps.
mpu6050: gyro z[0] is 0.12dps.
mpu6050: 3/3.
mpu6050: fifo 23.
mpu6050: acc x[0] is -0.09g.
mpu6050: acc y[0] is -0.33g.
mpu6050: acc z[0] is 0.95g.
mpu6050: gyro x[0] is -0.06dps.
mpu6050: gyro y[0] is 0.85dps.
mpu6050: gyro z[0] is 0.06dps.
```

```shell
mpu6050 -c dmp 3 -a 0

mpu6050: 1/3.
mpu6050: fifo 6.
mpu6050: pitch[0] is 0.11dps.
mpu6050: roll[0] is 0.79dps.
mpu6050: yaw[0] is 0.00dps.
mpu6050: acc x[0] is -0.09g.
mpu6050: acc y[0] is -0.34g.
mpu6050: acc z[0] is 0.95g.
mpu6050: gyro x[0] is -0.06dps.
mpu6050: gyro y[0] is -0.18dps.
mpu6050: gyro z[0] is 0.00dps.
mpu6050: 2/3.
mpu6050: fifo 7.
mpu6050: pitch[0] is 0.09dps.
mpu6050: roll[0] is 0.72dps.
mpu6050: yaw[0] is 0.00dps.
mpu6050: acc x[0] is -0.09g.
mpu6050: acc y[0] is -0.34g.
mpu6050: acc z[0] is 0.95g.
mpu6050: gyro x[0] is -0.06dps.
mpu6050: gyro y[0] is -0.24dps.
mpu6050: gyro z[0] is 0.00dps.
mpu6050: 3/3.
mpu6050: fifo 6.
mpu6050: pitch[0] is 0.07dps.
mpu6050: roll[0] is 0.64dps.
mpu6050: yaw[0] is -0.00dps.
mpu6050: acc x[0] is -0.09g.
mpu6050: acc y[0] is -0.33g.
mpu6050: acc z[0] is 0.95g.
mpu6050: gyro x[0] is -0.06dps.
mpu6050: gyro y[0] is -0.12dps.
mpu6050: gyro z[0] is -0.06dps.
```

```shell
mpu6050 -c motion -a 0

mpu6050: irq motion.
mpu6050: irq dmp
mpu6050: irq data ready
mpu6050: orient irq reverse portrait.
mpu6050: tap irq x up with 6.
mpu6050: finish dmp tap orient motion.
```

```shell
mpu6050 -c pedometer 3 -a 0

mpu6050: pedometer step count is 7.
mpu6050: pedometer step count is 8.
mpu6050: pedometer step count is 9.
```

```shell
mpu6050 -h

mpu6050 -i
	show mpu6050 chip and driver information.
mpu6050 -h
	show mpu6050 help.
mpu6050 -p
	show mpu6050 pin connections of the current board.
mpu6050 -t reg -a (0 | 1)
	run mpu6050 register test.
mpu6050 -t read <times> -a (0 | 1)
	run mpu6050 read test.times means the test times.
mpu6050 -t fifo <times> -a (0 | 1)
	run mpu6050 fifo test.times means the test times.
mpu6050 -t dmp <times> -a (0 | 1)
	run mpu6050 dmp test.times means the test times.
mpu6050 -t motion -a (0 | 1)
	run mpu6050 motion test.
mpu6050 -t pedometer <times> -a (0 | 1)
	run mpu6050 pedometer test.times means the test times.
mpu6050 -c read <times> -a (0 | 1)
	run mpu6050 read function.times means the read times.
mpu6050 -c fifo <times> -a (0 | 1)
	run mpu6050 fifo function. times means the read times.
mpu6050 -c dmp <times> -a (0 | 1)
	run mpu6050 dmp function.times means the read times.
mpu6050 -c motion -a (0 | 1)
	run mpu6050 motion function.
mpu6050 -c pedometer <times> -a (0 | 1)
	run mpu6050 pedometer function.times means the read times.
```

