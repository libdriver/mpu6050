[English](/README.md) | [ 简体中文](/README_zh-Hans.md) | [繁體中文](/README_zh-Hant.md) | [日本語](/README_ja.md) | [Deutsch](/README_de.md) | [한국어](/README_ko.md)

<div align=center>
<img src="/doc/image/logo.svg" width="400" height="150"/>
</div>

## LibDriver MPU6050

[![MISRA](https://img.shields.io/badge/misra-compliant-brightgreen.svg)](/misra/README.md) [![API](https://img.shields.io/badge/api-reference-blue.svg)](https://www.libdriver.com/docs/mpu6050/index.html) [![License](https://img.shields.io/badge/license-MIT-brightgreen.svg)](/LICENSE)

MPU6050은 3축 자이로스코프, 3축 가속도계 및 DMP(디지털 모션 프로세서™)를 모두 작은 4x4x0.9mm 패키지에 결합한 세계 최초의 통합 6축 MotionTracking 장치입니다. 전용 I2C 센서 버스를 통해 외부 3축 나침반의 입력을 직접 받아 완전한 9축 MotionFusion™ 출력을 제공합니다. 6축 통합, 온보드 MotionFusion™ 및 런타임 보정 펌웨어가 포함된 MPU6050 MotionTracking 장치를 통해 제조업체는 비용이 많이 들고 복잡한 개별 장치의 선택, 검증 및 시스템 수준 통합을 제거할 수 있으므로 최적의 모션 성능을 보장할 수 있습니다. 소비자. MPU6050은 또한 보조 I2C 포트에서 압력 센서와 같은 여러 비관성 디지털 센서와 인터페이스하도록 설계되었습니다. MPU6050은 MPU30X0 제품군과 풋프린트 호환이 가능합니다. MPU6050에는 자이로스코프 출력을 디지털화하기 위한 3개의 16비트 ADC(아날로그-디지털 변환기)와 가속도계 출력을 디지털화하기 위한 3개의 16비트 ADC가 있습니다. 빠르고 느린 동작 모두의 정밀한 추적을 위해 부품은 사용자 프로그래밍 가능한 자이로스코프 전체 범위 ±250, ±500, ±1000 및 ±2000°/초(dps)와 사용자 프로그래밍 가능한 가속도계 전체 범위를 특징으로 합니다. ±2g, ±4g, ±8g 및 ±16g의 범위.

LibDriver MPU6050은 LibDriver에서 출시한 MPU6050의 전체 기능 드라이버입니다. 가속도 판독, 각속도 판독, 자세각 판독, dmp 판독, 탭 감지 및 기타 기능을 제공합니다. LibDriver는 MISRA를 준수합니다.

### 콘텐츠

  - [설명](#설명)
  - [설치](#설치)
  - [사용](#사용)
    - [example basic](#example-basic)
    - [example fifo](#example-fifo)
    - [example dmp](#example-dmp)
  - [문서](#문서)
  - [기고](#기고)
  - [저작권](#저작권)
  - [문의하기](#문의하기)

### 설명

/src 디렉토리에는 LibDriver MPU6050의 소스 파일이 포함되어 있습니다.

/interface 디렉토리에는 LibDriver MPU6050용 플랫폼 독립적인 IIC버스 템플릿이 포함되어 있습니다.

/test 디렉토리에는 LibDriver MPU6050드라이버 테스트 프로그램이 포함되어 있어 칩의 필요한 기능을 간단히 테스트할 수 있습니다.

/example 디렉토리에는 LibDriver MPU6050프로그래밍 예제가 포함되어 있습니다.

/doc 디렉토리에는 LibDriver MPU6050오프라인 문서가 포함되어 있습니다.

/datasheet 디렉토리에는 MPU6050데이터시트가 있습니다.

/project 디렉토리에는 일반적으로 사용되는 Linux 및 마이크로컨트롤러 개발 보드의 프로젝트 샘플이 포함되어 있습니다. 모든 프로젝트는 디버깅 방법으로 셸 스크립트를 사용하며, 자세한 내용은 각 프로젝트의 README.md를 참조하십시오.

/misra 에는 LibDriver misra 코드 검색 결과가 포함됩니다.

### 설치

/interface 디렉토리에서 플랫폼 독립적인IIC버스 템플릿을 참조하여 지정된 플랫폼에 대한 IIC버스 드라이버를 완성하십시오.

/src 디렉터리, 플랫폼용 인터페이스 드라이버 및 자체 드라이버를 프로젝트에 추가합니다. 기본 예제 드라이버를 사용하려면 /example 디렉터리를 프로젝트에 추가합니다.

### 사용

/example 디렉터리의 예제를 참조하여 자신만의 드라이버를 완성할 수 있습니다. 기본 프로그래밍 예제를 사용하려는 경우 사용 방법은 다음과 같습니다.

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

```c
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

### 문서

온라인 문서: [https://www.libdriver.com/docs/mpu6050/index.html](https://www.libdriver.com/docs/mpu6050/index.html).

오프라인 문서: /doc/html/index.html.

### 기고

CONTRIBUTING.md 를 참조하십시오.

### 저작권

저작권 (c) 2015 - 지금 LibDriver 판권 소유

MIT 라이선스(MIT)

이 소프트웨어 및 관련 문서 파일("소프트웨어")의 사본을 얻은 모든 사람은 이에 따라 무제한 사용, 복제, 수정, 통합, 출판, 배포, 2차 라이선스를 포함하여 소프트웨어를 처분할 수 있는 권리가 부여됩니다. 소프트웨어의 사본에 대한 라이선스 및/또는 판매, 그리고 소프트웨어가 위와 같이 배포된 사람의 권리에 대한 2차 라이선스는 다음 조건에 따릅니다.

위의 저작권 표시 및 이 허가 표시는 이 소프트웨어의 모든 사본 또는 내용에 포함됩니다.

이 소프트웨어는 상품성, 특정 목적에의 적합성 및 비침해에 대한 보증을 포함하되 이에 국한되지 않는 어떠한 종류의 명시적 또는 묵시적 보증 없이 "있는 그대로" 제공됩니다. 어떤 경우에도 저자 또는 저작권 소유자는 계약, 불법 행위 또는 기타 방식에 관계없이 소프트웨어 및 기타 소프트웨어 사용으로 인해 발생하거나 이와 관련하여 발생하는 청구, 손해 또는 기타 책임에 대해 책임을 지지 않습니다.

### 문의하기

연락주세요lishifenging@outlook.com.