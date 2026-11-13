# ESP32-S3 四轴无人机

本项目是一个基于 ESP32-S3 的四轴无人机工程，包含飞行控制固件、Android 遥控端、WiFi 图传和无人机停靠基站硬件设计。

飞控负责姿态采集、PID 控制、电机混控、电池监测、遥控指令接收与图传数据发送。Android APP 负责摇杆控制、解锁、PID 参数设置、图传显示和视频录制。

仓库地址：

https://github.com/Miyakomio/ESP32WiFilinkLinux

## 主要功能

- ESP32-S3 Arduino/PlatformIO 固件
- MPU6050 姿态采集与陀螺仪校准
- 串级 PID 姿态控制
- X 型四轴电机混控
- Android APP 实时遥控
- WiFi SoftAP 通信
- UDP 遥控、状态和遥测数据
- OV3660 摄像头图传
- APP 图传显示、字节流录制和视频解码
- MAX17048 电池电压、电量与低电量报警
- APP 端五组 PID 参数与独立量程设置
- 电机输出及 WiFi 接收状态串口诊断
- 无人机停靠基站 PCB、Gerber、BOM 和原理图

## 项目结构

```text
ESP32WiFilinkLinux/
├─ src/                         固件入口
├─ lib/
│  ├─ Hardware/                 电机、电量计、NFC 等硬件驱动
│  ├─ SoftApp/                  IMU、PID、WiFi、UDP 等应用模块
│  └─ Task/                     FreeRTOS 任务和任务调度
├─ app/ESP-Drone-Android/       Android 遥控端源码
├─ hardware/无人机停靠基站/      停靠基站 PCB 与生产文件
├─ docs/                        芯片数据手册
├─ platformio.ini               PlatformIO 固件配置
└─ README.md
```

## 硬件配置

| 模块 | 型号或接口 | 默认连接 |
| --- | --- | --- |
| 主控 | ESP32-S3，16 MB Flash，8 MB PSRAM | 4D Systems ESP32-S3 R8N16 |
| 姿态传感器 | MPU6050 | SDA GPIO10，SCL GPIO11 |
| 电量计 | MAX17048 | SDA GPIO38，SCL GPIO39 |
| 电机 1 | 20 kHz PWM | GPIO3 |
| 电机 2 | 20 kHz PWM | GPIO4 |
| 电机 3 | 20 kHz PWM | GPIO42 |
| 电机 4 | 20 kHz PWM | GPIO45 |
| 低电量指示灯 | LED | GPIO5 |
| 摄像头 | OV3660 | 由 CameraTask 配置 |

电机 PWM 分辨率为 10 位，软件速度范围为 0 至 1000。飞控当前固定使用 X 型机架混控。

硬件引脚以源码中的实际配置为准：

- `lib/Hardware/MOTOR/motor.h`
- `lib/Task/TaskRun.cpp`
- `lib/Task/TaskGroup/CameraTask.cpp`

## 摇杆映射

Android APP 当前使用固定摇杆映射：

| 摇杆输入 | 控制量 | 方向 |
| --- | --- | --- |
| 左摇杆上下 | Throttle | 向上增大，向下减小 |
| 左摇杆左右 | Yaw | 向左增大，向右减小 |
| 右摇杆上下 | Roll | 向上减小，向下增大 |
| 右摇杆左右 | Pitch | 向左增大，向右减小 |

APP 直接发送最终的 Roll、Pitch、Yaw 和 Throttle 数值，不再在数据包内部执行 X-Mode 45 度旋转或额外轴取反。飞控端继续使用 X 型电机混控。

## 通信方式

固件启动 SoftAP，Android 手机连接该热点后通过 UDP 与飞控通信。

默认网络参数：

| 参数 | 默认值 |
| --- | --- |
| SSID | `FLYInternet` |
| 密码 | `12345678` |
| 飞控 IP | `192.168.26.28` |
| 飞控 UDP 接收端口 | `2390` |
| APP 本地 UDP 端口 | `2399` |

默认 WiFi 名称和密码位于：

```text
lib/SoftApp/Network/WiFi/WiFiService.h
```

实际使用时应修改默认名称和密码。

遥控数据包括：

- Roll 目标角度
- Pitch 目标角度
- Yaw 目标角速度
- 油门
- 解锁状态
- 悬停状态
- PID 参数
- 电池电压、电量和相对高度遥测

APP 与固件通过 CRC 校验 UDP 数据包，避免错误数据直接进入电机控制。

## 姿态与电机控制

控制流程如下：

```text
遥控目标姿态
    -> 角度环 PID
    -> 目标角速度
    -> 角速度环 PID
    -> Roll/Pitch/Yaw 修正量
    -> X 型四轴混控
    -> 四路 PWM 电机输出
```

当前包含五组可调 PID：

1. Roll 角度环
2. Pitch 角度环
3. Yaw 角速度环
4. Roll 角速度环
5. Pitch 角速度环

每组 PID 可在 Android APP 中设置 Kp、Ki、Kd，并可分别设置参数滑块的最小值和最大值。

PID 只在飞控上锁时允许修改。飞控解锁后会拒绝 PID 更新。APP 发送的 PID 参数仅保存在内存中，飞控重启后恢复固件默认值。

固件默认值参考 Tiny-Drone，并换算到本项目 0 至 1000 的电机输出范围。相关实现位于：

```text
lib/SoftApp/Control/MotorControl.cpp
```

## IMU 水平校准

Android APP 的设置图标下方提供 IMU 校准按钮。

校准步骤：

1. 将飞机放在水平台面，安装状态与实际飞行姿态一致。
2. 点击设置图标下方的校准按钮。
3. APP 会先锁定电机，再发送校准请求。
4. 保持飞机完全静止约 4 秒。
5. 串口显示 `MPU calibration complete` 后完成校准。

校准时会同时执行：

- 重新计算陀螺仪三轴零偏
- 将当前水平姿态记录为 Roll/Pitch 的 0 度
- 清零姿态融合状态和 PID 积分

校准期间飞控拒绝解锁。校准完成后需要重新按下解锁按钮。水平零位和陀螺仪零偏当前只保存在内存中，飞控重启后需要重新校准。

## 编译和烧录固件

### 环境要求

- Visual Studio Code
- PlatformIO 插件，或 PlatformIO CLI
- USB 数据线
- ESP32-S3 开发板

### 编译

```powershell
pio run
```

### 烧录

```powershell
pio run -t upload --upload-port COM13
```

`platformio.ini` 中当前写有 `upload_port = COM13`。如果电脑识别的串口不是 COM13，需要改成实际端口。

### 查看串口日志

```powershell
pio device monitor --port COM13 --baud 115200
```

正常启动后可以观察以下日志：

```text
MPU6050 初始化成功
MPU: Roll=... Pitch=... Yaw=...
Battery: ... mV, ...%, STATUS=...
WiFi ready: SSID=FLYInternet IP=...
RX R=... P=... Y=... T=...
MOTOR out=...,...,...,...
```

### 电机输出诊断

飞控解锁后，每 500 ms 输出一次四路电机命令：

```text
MOTOR out=M1,M2,M3,M4 control=on
```

`control=on` 表示 PID 和电机控制链路有效，`control=off` 表示遥控、IMU 或解锁状态不满足控制条件。

### 电机顺序测试

`src/main.cpp` 中默认启用了单电机顺序测试。飞控启动后会依次运行 M1、M2、M3、M4，每路以 `kMinSpeed` 运行 2 秒，各路之间停止 1 秒，测试完成后进入正常任务。

测试前必须拆掉全部桨叶。测试完成后将 `kEnableMotorSequenceTest` 改为 `false`，再编译正式飞行固件。

## 编译 Android APP

Android 工程位置：

```text
app/ESP-Drone-Android
```

当前 APP 版本：

```text
0.7.45-imu-calibration
```

Windows 命令行编译：

```powershell
cd app\ESP-Drone-Android
.\gradlew.bat assembleDebug
```

生成的 APK 通常位于：

```text
app\ESP-Drone-Android\build\outputs\apk\debug\
```

构建前需要在 `local.properties` 中设置本机 Android SDK 路径。构建产物不会提交到仓库。

## 飞行前检查

首次试飞前至少完成以下检查：

1. 电池充满，空载电压应接近 4.15 V。
2. 确认四支桨叶方向正确，且桨叶没有裂纹或变形。
3. 确认四个电机都能独立启动和改变转速。
4. 确认电机顺序、旋转方向和 X 型混控一致。
5. 确认 IMU 水平静置时 Roll、Pitch 接近零，陀螺仪零偏正常。
6. 在 APP 中开启 X-Mode，油门上限建议先设为 100%。
7. 先拆桨进行电机与姿态方向测试，确认无误后再安装桨叶。
8. 解锁前必须保持油门在安全范围。

串口姿态与电机输出是排查故障的主要依据：

- 四路输出接近 0：检查解锁、遥控数据、IMU 有效性和命令超时。
- 四路输出接近但无法升空：检查电池压降、桨叶方向、电机推力和整机重量。
- 某一路输出为 0 或明显偏低：检查混控、对应电机、驱动通道和接线。
- 四路输出差异很大：检查 PID、IMU 方向、电机方向和机架配置。

## 无人机停靠基站

停靠基站设计资料位于：

```text
hardware/无人机停靠基站/
```

包含：

- 嘉立创工程文件
- 原理图和 PCB PDF
- PCB 预览图
- Gerber 压缩包
- BOM
- 贴片坐标文件
- 历史备份工程

## 注意事项

- 调试电机和 PID 时必须拆除桨叶。
- 正式试飞时远离人员和易损物品。
- 电池低电量或大电流压降明显时不要继续起飞。
- 默认 WiFi 密码仅用于调试，公开环境使用前必须修改。
- 固件中的串口诊断会影响实时性，完成排查后可酌情降低日志频率。
- 本工程仍在持续调试，首次飞行前应完整检查机械、电气和软件状态。

## 相关数据手册

- `docs/MAX17048-MAX17049.pdf`
- `docs/MAX17040-MAX17041.pdf`
