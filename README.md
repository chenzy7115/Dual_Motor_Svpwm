# ESP32 双电机有感 FOC / SVPWM

本工程将 DengFOC V4 课程第十一课 `Dual_Motor_SVPWM` 示例迁移到 PlatformIO，目标开发板为 **WEMOS LOLIN32 Lite（ESP32）**，使用 **Arduino 框架**。

工程实现两路电机的：

- AS5600 磁编码器读取、跨零圈数累计和角速度计算；
- 两相电流采样、ADC 零偏校准和 Clarke/Park 变换；
- 电流闭环、速度闭环、位置闭环及位置-速度-电流三级闭环接口；
- 七段式 SVPWM 和六路 30 kHz PWM 输出；
- 串口力矩目标输入及电流、角度、速度数据输出。

> **安全提示**：上电前必须核对驱动器母线电压、电流采样参数、使能极性和电机相序。编码器对齐时程序会分别向两个电机施加约 3 V 电压并持续约 1 秒，请固定电机、移除螺旋桨并允许转子安全转动。首次运行建议使用限流电源，并在小电流目标下验证。

## 1. 硬件与引脚

默认参数来自源工程，假定目标板接线与源 DengFOC V4 示例一致。所有引脚均为 ESP32 GPIO 编号，不是开发板丝印编号。

| 功能 | M0 | M1 |
| --- | --- | --- |
| PWM A | GPIO 32 | GPIO 26 |
| PWM B | GPIO 33 | GPIO 27 |
| PWM C | GPIO 25 | GPIO 14 |
| LEDC 通道 | 0、1、2 | 3、4、5 |
| 电流采样 A | GPIO 39 | GPIO 35 |
| 电流采样 B | GPIO 36 | GPIO 34 |
| 编码器 I²C | I²C0 | I²C1 |
| I²C SDA | GPIO 19 | GPIO 23 |
| I²C SCL | GPIO 18 | GPIO 5 |

共用及固定参数：

| 参数 | 默认值 |
| --- | --- |
| 驱动器使能 | GPIO 12，高电平有效 |
| 母线电压 | 12.6 V |
| PWM 频率 | 30 kHz |
| PWM 分辨率 | 8 位，占空比计数 0–255 |
| AS5600 地址 | `0x36` |
| I²C 频率 | 400 kHz |
| ADC | 3.3 V 参考、12 位、最大计数 4095 |
| 采样电阻 | 0.01 Ω |
| 电流检测运放增益 | 50 |
| 电机磁极对数 | M0/M1 均为 7 |
| 电机/编码器方向 | M0/M1 均为 1 |
| 串口波特率 | 115200 |

两路电流检测均只使用 A/B 相，C 相保持未连接状态。

## 2. 环境准备

1. 安装 VS Code 和 PlatformIO IDE 扩展。
2. 使用 PlatformIO 打开本工程根目录，即包含 `platformio.ini` 的目录。
3. 工程固定使用 `espressif32@8.0.0`，首次构建时 PlatformIO 会自动安装对应平台、Arduino-ESP32 框架和 Xtensa 工具链。
4. 用 USB 连接 LOLIN32 Lite，并确认电脑识别到对应 COM 口。

## 3. 构建、烧录和串口监视

在 PlatformIO 终端中，当前目录应为本工程根目录。

### 构建

```powershell
pio run -e lolin32_lite
```

成功后生成：

```text
.pio/build/lolin32_lite/firmware.bin
.pio/build/lolin32_lite/firmware.elf
```

### 烧录

```powershell
pio run -e lolin32_lite -t upload
```

如果自动检测不到端口，可指定实际 COM 口：

```powershell
pio run -e lolin32_lite -t upload --upload-port COM5
```

### 打开串口

```powershell
pio device monitor -e lolin32_lite -b 115200
```

退出监视器通常按 `Ctrl+C`。

## 4. 首次上电流程

`setup()` 按以下顺序执行：

1. 初始化 115200 串口；
2. 使能两路驱动器；
3. 设置母线电压，初始化六路 LEDC PWM；
4. 初始化两路独立 I²C 总线并读取 AS5600；
5. 分别校准两路 ADC 零偏，各采样 1000 次；
6. 依次对齐 M0、M1 编码器；
7. 进入 `loop()`，每个循环更新两路编码器和电流，再执行电流闭环。

串口初始化信息包括：

```text
完成PWM初始化设置
编码器加载完毕
M0 0电角度：...
M1 0电角度：...
```

由于编码器稳定等待、ADC 校准和机械对齐，上电到进入主循环需要数秒。

## 5. 串口控制

默认启用双电机电流力矩模式。发送一个浮点数并以换行结尾，例如：

```text
0.5\n
```

表示将 M0、M1 的目标 q 轴电流都设置为 `0.5 A`。

- 发送 `0\n`：目标电流为零；
- 发送正值或负值：改变目标电流方向；
- 源例程只解析一个目标值，因此两个电机当前共享同一目标，不支持独立双目标；
- 建议先使用 `0.1` 等小目标验证电流方向和电机转向，再逐步增加。

串口周期输出格式为：

```text
M0电流(A),M1电流(A),M0累计角度(rad),M0速度(rad/s),目标电流(A)
```

例如：

```text
0.498700,0.497900,1.234500,0.056700,0.500000
```

默认每 101 个控制循环输出一次。速度单位为 **rad/s**，不是 rpm。

## 6. 修改硬件和控制参数

主要参数集中在：

```text
lib/DengFOC/src/config.h
```

常见修改项：

| 常量 | 用途 |
| --- | --- |
| `kEnablePin` / `kEnableActiveLevel` | 驱动器使能引脚和有效电平 |
| `kPowerSupplyVoltage` | 实际母线电压，影响占空比换算和速度环限幅 |
| `kM0PwmPins` / `kM1PwmPins` | 两路电机 PWM 引脚 |
| `kM0PwmChannels` / `kM1PwmChannels` | 两路 LEDC 通道 |
| `kPwmFrequencyHz` / `kPwmResolutionBits` | PWM 频率和分辨率 |
| `kM0I2cSdaPin` / `kM0I2cSclPin` | M0 编码器 I²C 引脚 |
| `kM1I2cSdaPin` / `kM1I2cSclPin` | M1 编码器 I²C 引脚 |
| `kM0CurrentPins` / `kM1CurrentPins` | A/B 相电流 ADC 引脚 |
| `kShuntResistanceOhm` | 采样电阻 |
| `kCurrentSenseAmplifierGain` | 电流检测运放增益 |
| `kM0CurrentGainSign` / `kM1CurrentGainSign` | 电流方向校正，反向时改为 `-1.0f` |
| `kM0PolePairs` / `kM1PolePairs` | 电机磁极对数 |
| `kM0Direction` / `kM1Direction` | 电机和编码器方向 |
| `kCurrentPidP/I/D/Ramp` | 主程序实际使用的电流 PID 参数 |
| `kSerialBaudRate` | 串口波特率 |

修改硬件参数后需要重新构建并烧录。电机方向、极对数或电流方向改变后，应重新执行上电对齐。

## 7. 切换控制模式

`src/main.cpp` 保留了源例程的多种控制示例，但默认只启用最后的电流力矩模式。

要启用其他模式：

1. 注释掉当前的 `DFOC_M0_setTorque()` 和 `DFOC_M1_setTorque()` 调用；
2. 取消所需模式的 PID 设置和目标接口注释；
3. 根据需要调整目标量纲和 PID 参数；
4. 重新构建、烧录，并先使用小目标验证。

可用接口：

| 模式 | M0 接口 | M1 接口 | 目标单位 |
| --- | --- | --- | --- |
| 电流力矩 | `DFOC_M0_setTorque` | `DFOC_M1_setTorque` | A |
| 速度闭环 | `DFOC_M0_setVelocity` | `DFOC_M1_setVelocity` | rad/s |
| 位置闭环 | `DFOC_M0_set_Force_Angle` | `DFOC_M1_set_Force_Angle` | rad |
| 位置-速度-电流 | `DFOC_M0_set_Velocity_Angle` | `DFOC_M1_set_Velocity_Angle` | rad |

底层 `M0_setTorque(Uq, angle_el)` / `M1_setTorque(Uq, angle_el)` 接收的是 **q 轴电压（V）和电角度（rad）**，与上层单参数 `DFOC_Mx_setTorque(Target)` 的电流目标接口不同。

## 8. 工程结构

```text
.
├── platformio.ini                 # lolin32_lite / Arduino 构建配置
├── src/
│   └── main.cpp                   # 入口、初始化流程和默认电流力矩控制
├── lib/
│   └── DengFOC/
│       └── src/
│           ├── config.h           # 集中硬件、控制环和串口参数
│           ├── DengFOC.cpp/.h     # 双电机 FOC、SVPWM 和公共接口
│           ├── AS5600.cpp/.h      # 编码器驱动
│           ├── InlineCurrent.cpp/.h # 电流采样和零偏校准
│           ├── pid.cpp/.h         # PID 控制器
│           └── lowpass_filter.cpp/.h # 一阶低通滤波
└── README.md
```

PlatformIO 会自动发现 `lib/DengFOC`，并将 `lib/DengFOC/src` 加入头文件搜索路径。工程不需要额外第三方 PlatformIO 库，只使用 Arduino-ESP32 自带的 `Wire`、LEDC、ADC 和串口功能。

## 9. 移植结果说明

### 源工程检查

源目录为扁平 Arduino 工程，共 11 个文件，没有 `platformio.ini`、`library.json` 或额外依赖目录。实现双电机 SVPWM 所需的文件为：

- `DengFOC_Lib_Lesson11_Dual_Motor_SVPWM.ino`
- `DengFOC.cpp/.h`
- `AS5600.cpp/.h`
- `InlineCurrent.cpp/.h`
- `pid.cpp/.h`
- `lowpass_filter.cpp/.h`

### 迁移方式

- 将 10 个算法和驱动文件完整迁入 `lib/DengFOC/src`，保留原文件名、公开接口和 GPL 声明。
- 将 `.ino` 入口适配为 `src/main.cpp`。
- 新增 `config.h`，把原先分散在源码中的引脚、PWM、I²C、ADC、电机及控制参数集中管理。
- `platformio.ini` 固定为 `espressif32@8.0.0`，保留 `lolin32_lite` 和 Arduino 框架，并设置 115200 串口监视速率。

### ESP32 / Arduino 适配

当前环境使用 Arduino-ESP32 3.3.12，旧式 `ledcSetup()` 和 `ledcAttachPin()` 已不再提供，因此：

- 使用 `ledcAttachChannel(pin, freq, resolution, channel)` 绑定六个 PWM 通道；
- 使用按 GPIO 寻址的 `ledcWrite(pin, duty)` 输出占空比；
- 保持源工程的 30 kHz、8 位、0–255 占空比和通道映射不变。

同时修正了源工程的初始化问题：

- 写使能引脚前先配置为 `OUTPUT`，保证入口“先使能、后校准”的顺序有效；
- 明确初始化未使用的 C 相 ADC 引脚，避免读取未初始化内存；
- 两路速度环都按实际母线电压的一半初始化输出限幅。

### 保留的算法和功能

- SVPWM 六扇区判断、有效矢量时间 `T1/T2`、零矢量时间 `T0` 和七段占空比分配；
- 两相电流 Clarke/Park 变换及 q 轴电流计算；
- AS5600 12 位角度读取、跨零圈数累计和速度计算；
- 梯形积分 PID、积分限幅、输出限幅和变化率限制；
- 电流、速度、位置及位置-速度-电流控制接口；
- 单串口目标同时控制两路电机；
- 串口输出字段、顺序和格式。

### 构建验证

已在当前 Windows 环境完成 `lolin32_lite` 构建：

```powershell
pio run -e lolin32_lite
```

构建成功，五个算法 `.cpp` 均参与编译。最近一次构建资源占用：

- Flash：329,827 字节，约 25.2%；
- RAM：24,544 字节，约 7.5%；
- 固件输出：`.pio/build/lolin32_lite/firmware.bin`。

## 10. 假设与未验证事项

以下内容仍需要结合实际硬件确认：

- 外接线是否与源工程一致，特别是驱动器使能极性和电机相序；
- AS5600 是否正常连接、I²C 上拉是否合适以及实际地址是否为 `0x36`；
- 电流采样方向、0.01 Ω 采样电阻和 50 倍运放增益是否准确；
- 两路电机是否均为 7 对极，编码器安装方向是否与配置一致；
- 实际母线电压是否为 12.6 V，30 kHz PWM 和驱动器死区是否匹配；
- ADC 参考电压、线性度及零偏校准是否满足控制要求；
- 双电机带载性能、电流环和速度环稳定性；
- 实际控制周期是否满足电机和驱动器要求。

当前验证仅覆盖软件构建，不包括上传、编码器通信、电流方向、机械对齐或双电机带载运行。

## 11. 许可说明

迁移代码保留了源工程的 GPL 版权和许可声明。修改或分发时请遵守相应许可要求。
