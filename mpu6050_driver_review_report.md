# MPU6050 驱动"像素级"审查报告

**审查对象**：LibDriver MPU6050 驱动 v1.0.0（2022-06-30）
- `src/driver_mpu6050.c`（10675 行）
- `src/driver_mpu6050.h`（2965 行）
- `src/driver_mpu6050_code.h`（289 行，DMP 固件镜像）

**核对基准**：
- `datasheet/mpu6050_register_map.pdf`（RM-MPU-6000A-00 rev **4.2**，2013-08-19）
- `datasheet/mpu6050_product_specification.pdf`（PS-MPU-6000A-00 rev **3.4**，2013-08-19）
- 辅助基准：InvenSense eMPL 5.1 官方源码（inv_mpu.c / inv_mpu_dmp_motion_driver.c）、寄存器手册 rev 4.0（rev 4.2 删除的运动检测寄存器在 4.0 中有记载）

**审查方法**：全部源码逐行通读；两份 PDF 全文提取后逐章节比对寄存器地址、位域、复位值、公式、时序；DMP 部分与 eMPL 5.1 官方实现逐字节比对；存疑项上网查证权威出处。

---

## 一、确认的错误（BUG，共 2 处）

### BUG-1：运动检测阈值换算系数错误（偏差 16 倍）⭐ 最严重

**位置**：`driver_mpu6050.c` 行 7447 / 7463 附近
```c
*reg = (uint8_t)(mg / 2.0f);        /* convert real data to register data */
*mg = (float)reg * 2.0f;
```
驱动按 **1 LSB = 2 mg** 换算 MOT_THR（0x1F）。

**手册依据**：
- 用户提供的寄存器手册 **rev 4.2 已整体删除寄存器 0x1D~0x22 的文档**（汇总表从 0x1C 直接跳到 0x23，正文章节从 Register 28 直接跳到 Register 35）——这正是不容易查到的原因。
- 寄存器手册 **rev 4.0** §4.6 Register 31 MOT_THR："The mg per LSB increment for MOT_THR can be found in the Electrical Specifications table of the MPU-6000/MPU-6050 Product Specification document."
- 产品规格书 §6.2 电气表末行：**"INTELLIGENCE FUNCTION INCREMENT …… 32 mg/LSB"**
- InvenSense 官方 LPA（低功耗运动检测）应用流程图原文："In MOT_THR (0x1F) set the value to from 1-255 LSBs. **Where 1LSB = 32 mg**"

**结论**：正确换算应为 `mg / 32.0f`。现驱动把阈值寄存器值放大了 16 倍——用户调用 `mpu6050_motion_threshold_convert_to_register(handle, 64.0f, &reg)` 期望 64 mg 阈值，实际写入 32（=1024 mg），运动中断灵敏度严重失真。

---

### BUG-2：`mpu6050_set_force_accel_sample` 功能在 MPU6050 上不存在（疑似从 MPU6500 误移植）

**位置**：`driver_mpu6050.c` 行 7614~7653
```c
if (enable == MPU6050_BOOL_TRUE)
    prev |= 0x7;          /* set the accel sample */   ← 写 ACCEL_CONFIG(0x1C) bits[2:0]
```

**手册依据**：
- rev 4.2 Register 28 ACCEL_CONFIG：bit7~bit3 为 XA_ST/YA_ST/ZA_ST/AFS_SEL[1:0]，**bits[2:0] 记为 "-"（未定义）**；
- rev 4.0 同寄存器 bits[2:0] 为 **ACCEL_HPF[2:0]（数字高通滤波器）**，其中 7 = **Hold 模式**；
- MPU6050 全部寄存器中不存在 "force accel sample" 功能（该概念来自 MPU6500/9250 系列）。

**后果**：调用该接口实际效果是把加速度计高通滤波器设为 Hold（0x7）或 Reset（0x0），与函数名宣称的功能完全不符；且副作用会改变运动检测/自由落体检测的高通滤波状态。
**建议**：MPU6050 驱动应删除该函数，或改名为 `set_accel_high_pass_filter` 并按 ACCEL_HPF 语义重新定义枚举。

---

### BUG-3：`mpu6050_dmp_read` 解析 FIFO 包时 accel/gyro 顺序颠倒（用户复核发现，已证实；代码保留原状）⭐

**位置**：`driver_mpu6050.c` 行 3332~3433（现行代码，gyro 块在 accel 块之前）
**现行代码**：quat(16) → **gyro(6)** → **accel(6)** → gesture(4)
**正确顺序**：quat(16) → **accel(6)** → **gyro(6)** → gesture(4)

**依据链**（四条独立证据互相印证）：
1. **FIFO 包布局由固件执行 CFG_15 键序列决定**——本地 `dmp_set_feature`（行 2283~2311）写 CFG_15 时 **accel 键 (0xC0,0xC8,0xC2) 在 gyro 键 (0xC4,0xCC,0xC6) 之前**（tmp[1..3] accel、tmp[4..6] gyro），因此固件实际输出的 FIFO 包是 accel 在 gyro 之前；
2. eMD 6.12 官方 `dmp_read_fifo()` 解析顺序为 quat → **accel** → gyro → gesture，且其 `dmp_enable_feature` 的 CFG_15 写入序列与本地逐字节一致（固件相同 → 布局相同）；
3. 上游 libdriver/mpu6050 仓库自 2022 年第 2 笔提交起即 accel 在前（最新 main 分支 accel 块行 3326、gyro 块行 3377）；
4. InvenSense 官方库（eMPL/MotionDriver 系列）从未有过 gyro 在前的写法。

**影响**（与发现者描述完全一致）：
- 当 feature mask **同时**含 `SEND_RAW_ACCEL` 和 `SEND_ANY_GYRO` 时（LibDriver 官方 DMP 例程 `driver_mpu6050_dmp_read_test.c` 行 1106 默认同时开启 `SEND_RAW_ACCEL | SEND_CAL_GYRO | GYRO_CAL`），**accel_raw/accel_g 与 gyro_raw/gyro_dps 数据完全互换**；
- quat/pitch/roll/yaw 不受影响（包首 16 字节，解析位置正确）；
- gesture（tap/orient）不受影响（包尾 4 字节，两种顺序下偏移均为 28）；
- 只开 accel、只开 gyro 或都不开时不出错（单块总在偏移 16 处）。

**修复状态**：⛔ **未修复，代码保留原状**（用户要求不改动驱动源码；审查过程中曾一度修改，已逐字节退回原样）。如需自行修复：将 accel 解析块（行 3383~3433）整块移动到 gyro 解析块（行 3332~3382）之前即可——纯块移动、无其他改动，改完与上游 main 分支及 eMD 6.12 对齐。

> ⚠️ 勘误：本报告初版曾将"DMP 包解析顺序 quat+gyro+accel+gesture"列为"与 eMPL 一致、确认正确"——**该结论是错的**，审查时仅比对了包总长与手势解码而漏比了 accel/gyro 的先后顺序。此条由用户独立复核发现，特此更正。

---

## 二、健壮性隐患（2 处）

### 隐患-1：`mpu6050_dmp_set_fifo_rate` 整数除零

**位置**：行 2718~2725
```c
if (rate > MPU6050_DMP_SAMPLE_RATE)   /* 只拦了 >200，没拦 ==0 */
{ ... return 5; }
d = MPU6050_DMP_SAMPLE_RATE / rate - 1;   /* rate==0 → 除零（UB/硬件异常） */
```
**说明**：eMPL 5.1 官方 `dmp_set_fifo_rate` 同样只有 `rate > 200` 一条检查——此为**官方代码的"遗传"缺陷**，非驱动作者新引入。但作为独立发布、带参数校验风格的驱动，应补 `rate == 0` 检查。

### 隐患-2：`mpu6050_init` 复位后即返回，芯片处于睡眠 + 内部 RC 时钟状态

**位置**：行 3796~3829
```c
prev = 1 << 7;                      /* DEVICE_RESET */
... /* 轮询 100×10ms 直到复位位自清 */
return 0;                           /* 未清 SLEEP、未设 CLKSEL */
```
**手册依据**（寄存器手册 §4.28 PWR_MGMT_1）：
> "Upon power up, the MPU-60X0 clock source defaults to the internal oscillator. However, **it is highly recommended** that the device be configured to use one of the gyroscopes (or an external clock source) as the clock reference **for improved stability**."
复位后 PWR_MGMT_1 = 0x40（SLEEP=1、CLKSEL=0）。驱动把"唤醒 + 选时钟源"留给用户调用，文档若未强调此点，直接 init 后读数据会得到全 0。

**结论**：不算寄存器错误，但 init 返回成功却不可用，属于易踩坑的使用性缺陷；复位等待超时 1000 ms 的余量设置合理（产品规格书寄存器读写下电启动时间上限 100 ms）。

---

## 三、注释 / 元数据问题（2 处）

1. **行 730/734 注释错误**：`data[0] = 0x10 | 0xE0; /* enable 2g and test */` —— 0x10 = AFS_SEL=2，实为 **±8g**（恰好符合产品规格书"加速度自检必须在 ±8g 量程下进行"的要求，代码正确，注释误导）。后文偏移换算 `32768/8` 也证明代码按 8g 自洽。
2. **`mpu6050_info` 元数据**：芯片名/厂商写 "TDK MPU6050"/"TDK"——手册为 InvenSense（2017 年被 TDK 收购，写成 TDK 不算事实错误但不与手册一致）。其余元数据全部正确：供电 2.375~3.46 V ✓、最大电流 3.9 mA ✓（=产品规格书"陀螺+加速度+DMP 典型 3.9mA"）、温度范围 -40~+85 ℃ ✓。

---

## 四、初审疑似、复核后排除的项（2 处）—— 驱动是对的

1. **`mpu6050_set_iic4_transaction_mode` 写 SLV4_CTRL bit5**：手册 Register 52 bit5 = **I2C_SLV4_REG_DIS**（0=先写寄存器地址，1=纯数据收发）。驱动枚举 `MODE_REG=0 / MODE_DATA=1` 与此**语义完全一致**。✔
2. **自检流程 FIFO_EN=0x78 按 12 字节包读取**：0x78 = 0111_1000 = XG|YG|ZG|ACCEL（**bit7 TEMP_FIFO_EN=0，不含温度**），FIFO 按"寄存器号升序"写入 → 每包 = ACCEL(6)+GYRO(6) = 12 字节，驱动解析顺序（前 6 字节 accel、后 6 字节 gyro）与手册完全吻合，也与 eMPL 官方实现逐行一致。✔

---

## 五、逐项核对确认正确的部分（摘要）

| 类别 | 核对结果 |
|---|---|
| 寄存器地址宏（~40 个） | 全部与寄存器手册汇总表一致（0x19/0x1A/0x1B/0x1C/0x23/0x24/0x25~0x35/0x37/0x38/0x3A/0x3B~0x48/0x49~0x60/0x63~0x66/0x67~0x6F/0x70/0x72~0x74/0x75） |
| WHO_AM_I=0x68 判断 | ✔（手册：AD0 接地时 0x68） |
| 位域拆装 | PWR_MGMT_1/2、SIGNAL_PATH_RESET、USER_CTRL、INT_PIN_CFG、INT_ENABLE、INT_STATUS、GYRO_CONFIG、SLV0~4 全部与手册逐位一致 |
| 温度公式 | `TEMP_OUT/340 + 36.53` ✔；产品规格书 "340 LSB/℃、35℃ 偏移 -521 LSB" 与其**数学等价**（-521/340+35=36.53），两份手册不矛盾，驱动的写法正确 |
| 采样率 | 分频公式 GyroOut/(1+SMPLRT_DIV)，DLPF=0/7 时 8kHz、其余 1kHz ✔；驱动提供原始分频值接口，语义正确 |
| 自检流程 | 陀螺 ±250dps+0xE0 ✔、加速度 ±8g+0xE0 ✔（符合手册"自检量程要求"）；FT 公式（陀螺 3275/(32768/250)·1.046^(code-1)、加速度 0.34·1.034^(code-1)）、SELF_TEST 位域拆包（XA_TEST[4:2]‖SELF_TEST_A[5:4]）、Z 轴 ±65536 重力修正——与 eMPL 官方完全一致 |
| 灵敏度换算 | 陀螺 131/65.5/32.8/16.4、加速度 16384/8192/4096/2048 ✔（产品规格书 §6.1/6.2 原值） |
| DLPF 表 | 260/188/98/42/20/10/5 Hz ✔ |
| I2C 主机 | I2C_MST_CLK 16 档（348~364 kHz）枚举与手册表格逐档一致 ✔；SLV0~3 CTRL（EN/BYTE_SW/REG_DIS/GRP/LEN）✔；EXT_SENS_DATA 24 字节上限 ✔ |
| FIFO | 1024 字节容量钳位 ✔；FIFO_COUNT 16 位 ✔；溢出处理 ✔ |
| 中断 | 中断类型位（MOTION=bit6/FIFO_OFLOW=bit4/I2C_MST=bit3/DMP=bit1/DATA_RDY=bit0）✔ |
| DMP | 固件 3062 字节 ✔；PRGM_START=0x0400 ✔；全部 DMP 内存地址（D_0_22=534、CFG_6=2753、CFG_8=2718、CFG_LP_QUAT=2712、CFG_15=2727、CFG_27=2742、TAP_THX/Y/Z=468/472/476、D_PEDSTD_STEPCTR=864、D_PEDSTD_TIMECTR=964）与 eMPL 5.1 逐一相等 ✔；GYRO_SF=46850825 ✔；FIFO 包格式 quat16+**accel6**+gyro6+gesture4 与手势解码（dir=tap>>3、count=(tap%8)+1）✔（注：固件包布局确为 accel 在前，但驱动 `dmp_read` 解析顺序为 gyro 在前——即 BUG-3，代码保留原状）；write_mem/read_mem 的 BANK_SEL 双字节协议 ✔；dmp_set_fifo_rate 的 regs_end 12 字节序列逐字节一致 ✔ |
| 直读模式 | 14 字节突发读（accel6+temp2+gyro6，跳过 temp）✔（寄存器连续布局 0x3B~0x48） |

---

## 六、总结

- **寄存器层（对数据手册）**：驱动整体质量高，地址、位域、公式、DMP 内存映射与两份手册及 eMPL 官方实现高度一致。
- **需要修的**：① `motion_threshold` 换算改 `/32.0f`（严重，影响运动中断功能）；② 删除或重命名 `set_force_accel_sample`（语义错误）。
- **建议修的**：③ `dmp_set_fifo_rate` 补 `rate==0` 检查；④ 修正行 730/734 的 "2g" 注释（实为 8g）；⑤ `mpu6050_info` 厂商名建议改为 "InvenSense (TDK)"；⑥ `mpu6050_init` 文档中注明"返回后芯片仍在 SLEEP、时钟为内部 RC，需先调用 set_sleep(DISABLE)+set_clock_source(PLL)"。
- **附带说明**：用户手中寄存器手册 rev 4.2 删掉了 0x1D~0x22（运动/自由落体检测寄存器）的文档，BUG-1 的正确标度需查 rev 4.0 + 产品规格书电气表（32 mg/LSB）或 InvenSense 官方应用笔记。

*报告生成：WorkBuddy 像素级审查 · 基于两份 PDF 全文提取比对 + eMPL 5.1 官方源码逐字节核对*

---

## 七、附：上游考证与修复状态（2026-10-24 复核更新）

### 7.1 上游 libdriver/mpu6050 仓库考证（GitHub main 分支）

对上游全部相关提交逐一取证后的关键事实：

| 时间 | 提交 | 内容 |
|---|---|---|
| 2022-06-30 | feat: first upload | 1.0.0 首发 |
| 2022（第 2 笔提交） | style: change copyright | **此时 dmp_read 已是 accel 在前**——本地这份"gyro 在前"的写法应是官网早期发布版/早期状态的遗留 |
| 2023-02-26 | fix: fix range convert bugs | 修复 ±4g 灵敏度常数 **8092→8192** 两处（`dmp_set_accel_bias`、`gyro_accel_raw_offset_convert`）；本地已含此修复 |
| 2025/2026 | fix i2c 400khz delay / doc errors | 时序与注释类修改 |

另：上游曾把 MOT_THR 换算改为 `/32.0f`（与本地现行一致）；上游 `mpu6050_iic4_transaction_mode_t` 枚举值已重定义为 `DATA=0x00 / REG=0x01`（与本地 `REG=0x00 / DATA=0x01` 相反），故其函数内部有 `(!enable)` 取反——两边各自内部自洽、都符合手册 REG_DIS 位定义，仅 API 值不同，**不算错误**；上游最新自检加速度配置改为 `0x18`（±16g，注释仍写 "2g"），与手册"自检应在 ±8g 下进行"的要求不符，**本地 ±8g（0x10）才是手册正确的**。

### 7.2 本地修复状态

| 问题 | 状态 |
|---|---|
| BUG-1 MOT_THR 换算（mg/2 → mg/32） | ✅ 本地已修复（现行 `/32.0f`） |
| BUG-2 set_force_accel_sample 语义错误 | ⚠️ 未处理（MPU6050 上该功能不存在，建议删除或改名） |
| **BUG-3 dmp_read accel/gyro 互换** | ⛔ **已证实存在，代码保留原状**（用户要求不修改驱动；修复方法见 BUG-3 节，曾修改后已逐字节退回） |
| 自检注释 "2g" | ✅ 已修正为 "8g" |
| get_st_biases pack_cnt==0 防护 | ✅ 本地已含（上游反而没有） |
| dmp_set_fifo_rate rate==0 除零 | ⚠️ 未处理（eMPL 官方同款缺陷） |
| 8092→8192 | ✅ 本地已含 |
| mpu6050_init 后 SLEEP/时钟源 | ⚠️ 使用注意：init 返回后需自行调用 set_sleep(DISABLE) + set_clock_source(PLL) |

### 7.3 BUG-3 验证方法备忘

复现条件：`dmp_set_feature(mask)` 中 mask 同时含 `SEND_RAW_ACCEL` 与 `SEND_ANY_GYRO`（含 `SEND_CAL_GYRO`/`GYRO_CAL` 组合）。受影响场景下静止放置时典型症状：`gyro_dps` 显示约 ±1000 dps 量级（实为 1g≈16384 LSB ÷ 16.4 的加速度读数），`accel_g` 显示接近 0.0x g（实为陀螺零偏）；四元数/欧拉角正常。
