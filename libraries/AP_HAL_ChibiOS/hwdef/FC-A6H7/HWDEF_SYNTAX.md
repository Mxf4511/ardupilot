# FC-A6H7 `hwdef.dat` Syntax Rules

本文档整理 `hwdef.dat` 常用语法，面向 `ChibiOS` 板级定义。
解析实现来源：`libraries/AP_HAL_ChibiOS/hwdef/scripts/chibios_hwdef.py`。

## 1) 基本规则

- 一行一个定义。
- `#` 为注释起始，行内注释有效。
- 使用空白分词（解析脚本基于 `shlex.split`）。
- 常见结构分三类：
  - 通用配置行：`<KEY> <VALUE...>`
  - 引脚定义行：`<PIN> <LABEL> <TYPE> [EXTRA...]`
  - 设备定义行：如 `SPIDEV`、`IMU`、`BARO` 等。

---

## 2) 顶层关键字语法

### `include`

```text
include <path>
```

- 包含其它 `hwdef.dat` 片段。
- 相对路径以当前文件目录为基准解析。

示例：

```text
include ../common/hwdef-common.dat
```

### `define`

```text
define <NAME> <VALUE...>
```

- 定义宏，最终用于生成 `hwdef.h`/编译配置。
- 若值是纯整数，会被解析器额外记录为整型 define。

示例：

```text
define HAL_STORAGE_SIZE 32768
define AP_CUSTOM_FIRMWARE_STRING "FC_A6"
```

### `undef`

```text
undef <NAME...>
```

- 取消之前定义（可一次多个）。
- 可用于移除配置项、设备项、部分 pin 项等。

示例：

```text
undef IMU
undef HAL_BATT_MONITOR_DEFAULT
```

### `env`

```text
env <NAME> <VALUE...>
```

- 设置构建环境变量。

示例：

```text
env OPTIMIZE -Os
```

### 通用配置行（非专用关键字）

```text
<KEY> <VALUE...>
```

示例：

```text
MCU STM32H7xx STM32H743xx
APJ_BOARD_ID 1013
OSCILLATOR_HZ 8000000
SERIAL_ORDER OTG1 UART7 USART1 USART2 USART3 UART8 USART6 OTG2
I2C_ORDER I2C2 I2C1 I2C4
STORAGE_FLASH_PAGE 14
DMA_PRIORITY S*
DMA_NOSHARE SPI1* SPI4*
```

---

## 3) 引脚定义语法

```text
<PIN> <LABEL> <TYPE> [EXTRA...]
```

- `<PIN>`: `PA0..PK15` 形式。
- `<LABEL>`: 逻辑名（如 `SPI1_SCK`、`IMU1_CS`、`LED0`）。
- `<TYPE>`: 外设类型（必须匹配允许模式）。
- `[EXTRA...]`: 可选属性，支持多个。

示例：

```text
PA5  SPI1_SCK      SPI1
PC15 IMU1_CS       CS
PE3  LED0          OUTPUT LOW GPIO(90)
PC7  TIM3_CH2      TIM3 RCININT PULLDOWN LOW
PC0  BATT_VOLTAGE_SENS ADC1 SCALE(1)
PB1  TIM8_CH3N     TIM8 PWM(2) GPIO(51)
PC7  USART6_RX     USART6 NODMA ALT(1)
```

### 允许的 `<TYPE>` 模式（解析器校验）

```text
INPUT
OUTPUT
TIM<NUM>
USART<NUM>
UART<NUM>
ADC<NUM>
SPI<NUM>
OTG<NUM>
SWD
CAN<NUM?>
I2C<NUM>
CS
SDMMC<NUM>
SDIO
QUADSPI<NUM>
OCTOSPI<NUM>
ETH<NUM>
RCC
```

### 常见 `EXTRA` 字段

- 电平/上下拉：`LOW` `HIGH` `PULLUP` `PULLDOWN` `FLOATING`
- 输出电气：`PUSHPULL` `OPENDRAIN`
- 速度：`SPEED_VERYLOW` `SPEED_LOW` `SPEED_MEDIUM` `SPEED_HIGH`
- GPIO 映射：`GPIO(<n>)`（见下）
- 复用槽：`ALT(<n>)`
- PWM 编号：`PWM(<n>)`
- ADC 比例：`SCALE(<v>)`
- DMA 禁用：`NODMA`
- 其它板级标签：`BIDIR` `ALARM` `RCININT` 等（按驱动/生成逻辑使用）

### `GPIO(n)` 引脚号

- **含义**：`n` 为逻辑 GPIO 编号，与同行左侧 `<PIN>`（如 `PE3`）一起被脚本生成到 `HAL_GPIO_PINS` / `HAL_GPIO_LINE_GPIOn`，HAL 用该编号做 `pinMode`/`write` 等。
- **范围**：**0～255**（HAL 与 `gpio_entry.pin_num` 均为 `uint8_t`）。同一板内不可重复。
- **惯例**：LED 常用 90/91/92；PWM 输出常用 50 起；PINIO/静默等常用 70～83；板载小功能常用 0～9。

---

## 4) SPI / WSPI 设备定义

### `SPIDEV`

```text
SPIDEV <name> <SPIx> <DEVIDn> <CS_label> <MODE0|MODE1|MODE2|MODE3> <low_speed> <high_speed>
```

- `speed` 支持 `*MHZ` 或 `*KHZ` 后缀。
- `CS_label` 必须对应已定义且类型为 `CS` 或带 `CS` extra 的 pin label。

示例：

```text
SPIDEV icm42688 SPI1 DEVID1 IMU1_CS MODE3 2*MHZ 16*MHZ
SPIDEV osd      SPI2 DEVID4 MAX7456_CS MODE0 10*MHZ 10*MHZ
```

### `QSPIDEV` / `OSPIDEV`

```text
QSPIDEV <name> <QUADSPIx|OCTOSPIx> <MODE1|MODE3> <speed> <size_pow2> <ncs_clk_delay>
OSPIDEV <name> <QUADSPIx|OCTOSPIx> <MODE1|MODE3> <speed> <size_pow2> <ncs_clk_delay>
```

- `speed` 同样要求 `*MHZ` 或 `*KHZ`。

---

## 5) 传感器探测定义

### `IMU`

```text
IMU <Driver> <bus/device args...> [BOARD_MATCH(...)] [AUX:<n>] [INSTANCE:<n>]
```

常见参数：

- `SPI:<spidev_name>`
- `I2C:<bus|ALL|ALL_INTERNAL|ALL_EXTERNAL>:<addr>`
- 方向参数如：`ROTATION_NONE`、`ROTATION_YAW_180` 等

示例：

```text
IMU Invensensev3 SPI:icm42688 ROTATION_NONE
IMU BMI270      SPI:bmi270_1 ROTATION_YAW_90
```

### `COMPASS`

```text
COMPASS <Driver[:probeFn]> <args...>
```

### `BARO`

```text
BARO <Driver[:probeFn]> <args...>
```

示例：

```text
BARO MS56XX I2C:1:0x77
BARO DPS310 I2C:1:0x76
```

### `AIRSPEED`

```text
AIRSPEED <Driver> <args...>
```

---

## 6) ROMFS 相关

### `ROMFS`

```text
ROMFS <virtual_name> <source_path>
```

### `ROMFS_WILDCARD`

```text
ROMFS_WILDCARD <glob>
```

### `ROMFS_DIRECTORY`

```text
ROMFS_DIRECTORY <dir>
```

示例：

```text
ROMFS_WILDCARD libraries/AP_OSD/fonts/font*.bin
```

---

## 7) 常见注意事项

- 同一总线上可定义多个 `SPIDEV`，驱动通过探测决定实际芯片。
- `SPIDEV` 的 `DEVIDn` 需要是数字格式（`DEVID1`、`DEVID4`）。
- `MODE` 必须是合法 SPI 模式，否则会在生成阶段报错。
- pin `TYPE` 与 `LABEL` 之间有一致性检查（如 `USART1` 对应 `USART1_RX/TX/...`）。
- `I2C:` 设备描述里地址支持 `0x` 十六进制写法。

---

## 8) 你当前板子可直接参考的片段

```text
SPIDEV icm42688  SPI1 DEVID1 IMU1_CS MODE3 2*MHZ 16*MHZ
SPIDEV mpu6000   SPI1 DEVID1 IMU1_CS MODE3 1*MHZ 4*MHZ
SPIDEV bmi270_1  SPI1 DEVID1 IMU1_CS MODE3 1*MHZ 8*MHZ

IMU Invensensev3 SPI:icm42688 ROTATION_NONE
IMU BMI270       SPI:bmi270_1 ROTATION_YAW_90

BARO MS56XX I2C:1:0x77
BARO DPS310 I2C:1:0x76
```

