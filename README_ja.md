[English](/README.md) | [ 简体中文](/README_zh-Hans.md) | [繁體中文](/README_zh-Hant.md) | [日本語](/README_ja.md) | [Deutsch](/README_de.md) | [한국어](/README_ko.md)

<div align=center>
<img src="/doc/image/logo.svg" width="400" height="150"/>
</div>

## LibDriver MPU6050

[![MISRA](https://img.shields.io/badge/misra-compliant-brightgreen.svg)](/misra/README.md) [![API](https://img.shields.io/badge/api-reference-blue.svg)](https://www.libdriver.com/docs/mpu6050/index.html) [![License](https://img.shields.io/badge/license-MIT-brightgreen.svg)](/LICENSE)

MPU6050は、3軸ジャイロスコープ、3軸加速度計、およびDigital MotionProcessor™（DMP）をすべて小さな4x4x0.9mmパッケージに組み合わせた、世界初の統合6軸MotionTrackingデバイスです。専用のI2Cセンサーバスにより、外部の3軸コンパスからの入力を直接受け入れて、完全な9軸MotionFusion™出力を提供します。 MPU6050 MotionTrackingデバイスは、6軸統合、オンボードMotionFusion™、およびランタイムキャリブレーションファームウェアを備えているため、メーカーは、ディスクリートデバイスのコストのかかる複雑な選択、認定、およびシステムレベルの統合を排除し、最適なモーションパフォーマンスを保証します。消費者。 MPU6050は、補助I2Cポートで、圧力センサーなどの複数の非慣性デジタルセンサーとインターフェイスするようにも設計されています。 MPU6050は、MPU30X0ファミリとフットプリント互換性があります。MPU6050は、ジャイロスコープ出力をデジタル化するための3つの16ビットアナログ-デジタルコンバータ（ADC）と、加速度計出力をデジタル化するための3つの16ビットADCを備えています。速い動きと遅い動きの両方を正確に追跡するために、パーツは、±250、±500、±1000、および±2000°/秒（dps）のユーザープログラム可能なジャイロスコープのフルスケール範囲と、ユーザーがプログラム可能な加速度計のフルスケールを備えています。 ±2g、±4g、±8g、および±16gの範囲。

LibDriver MPU6050は、LibDriverによって起動されたmpu6050の全機能ドライバーです。 加速度の読み取り、角速度の読み取り、姿勢角の読み取り、dmpの読み取り、タップ検出などの機能を提供します。 LibDriverはMISRAに準拠しています。

### 目次

  - [説明](#説明)
  - [インストール](#インストール)
  - [使用](#使用)
    - [example basic](#example-basic)
    - [example fifo](#example-fifo)
    - [example dmp](#example-dmp)
  - [ドキュメント](#ドキュメント)
  - [貢献](#貢献)
  - [著作権](#著作権)
  - [連絡して](#連絡して)

### 説明

/ srcディレクトリには、LibDriver MPU6050のソースファイルが含まれています。

/ interfaceディレクトリには、LibDriver MPU6050用のプラットフォームに依存しないIICバステンプレートが含まれています。

/ testディレクトリには、チップの必要な機能を簡単にテストできるLibDriver MPU6050ドライバーテストプログラムが含まれています。

/ exampleディレクトリには、LibDriver MPU6050プログラミング例が含まれています。

/ docディレクトリには、LibDriver MPU6050オフラインドキュメントが含まれています。

/ datasheetディレクトリには、MPU6050データシートが含まれています。

/ projectディレクトリには、一般的に使用されるLinuxおよびマイクロコントローラー開発ボードのプロジェクトサンプルが含まれています。 すべてのプロジェクトは、デバッグ方法としてシェルスクリプトを使用しています。詳細については、各プロジェクトのREADME.mdを参照してください。

### インストール

/ interfaceディレクトリにあるプラットフォームに依存しないIICバステンプレートを参照して、指定したプラットフォームのIICバスドライバを完成させます。

/ srcディレクトリ、/ interfaceディレクトリ、および/exampleディレクトリをプロジェクトに追加します。

### 使用

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

### ドキュメント

オンラインドキュメント: [https://www.libdriver.com/docs/mpu6050/index.html](https://www.libdriver.com/docs/mpu6050/index.html)。

オフラインドキュメント: /doc/html/index.html。

### 貢献

CONTRIBUTING.mdを参照してください。

### 著作権

著作権（c）2015-今 LibDriver 全著作権所有

MITライセンス（MIT）

このソフトウェアおよび関連するドキュメントファイル（「ソフトウェア」）のコピーを取得した人は、無制限の使用、複製、変更、組み込み、公開、配布、サブライセンスを含む、ソフトウェアを処分する権利を制限なく付与されます。ソフトウェアのライセンスおよび/またはコピーの販売、および上記のようにソフトウェアが配布された人の権利のサブライセンスは、次の条件に従うものとします。

上記の著作権表示およびこの許可通知は、このソフトウェアのすべてのコピーまたは実体に含まれるものとします。

このソフトウェアは「現状有姿」で提供され、商品性、特定目的への適合性、および非侵害の保証を含むがこれらに限定されない、明示または黙示を問わず、いかなる種類の保証もありません。 いかなる場合も、作者または著作権所有者は、契約、不法行為、またはその他の方法で、本ソフトウェアおよび本ソフトウェアの使用またはその他の廃棄に起因または関連して、請求、損害、またはその他の責任を負わないものとします。

### 連絡して

お問い合わせくださいlishifenging@outlook.com。