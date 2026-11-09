# Embedded Framework

基于 CMake 和 ARM GNU Toolchain 的嵌入式工程，支持 `h723_mc02` 和 `f407_cboard`。

## 目录与职责

| 目录 | 职责 |
| --- | --- |
| `boards` | CubeMX 工程、板级资源映射和启动入口；手写接入代码放在 wrapper 中 |
| `bsp` | GPIO、SPI、CAN、UART、PWM 等外设操作 |
| `libs` | 数学、控制、消息、CRC 等基础设施 |
| `devices` | 由应用驱动的 LED、电机、遥控器和裁判 UI，不创建后台线程 |
| `modules/imu` | BMI088/DMIMU 采集服务、姿态输出、ESKF 和 BMI088 温控 |
| `modules/rfid` | 默认启动的 RFID data 读取、蜂鸣提示和遥控输入限制 |
| `modules/referee` | 裁判接收线程、协议解析、类型缓存和可选类型回调 |
| `configs` | 用户 JSON 配置及生成脚本、生成头文件 |
| `applications/big_head` | H723 云台应用：VT03/DR16 输入、USB 视觉、BMI088、Pitch/双摩擦轮及下位机通信 |
| `applications/test` | 可选的 H723 单 M3508 SMC 台架测试应用 |

根工程链接 `modules`，通过它引入设备、BSP 和基础库。固件在链接时接入 ThreadX 初始化入口，默认创建 RFID 读取线程；BSP 保持原有实现；
其他服务仍由应用显式启动。RFID 的遥控限制在设备解码层执行，应用无需检查权限标志。

H723 默认启用 [`big_head` 应用](applications/big_head/README.md)，由其 `app_start()` 显式启动 BMI088
并创建控制和 USB 发送线程。拨弹盘与 yaw 电机由下位机执行，本板通过 CAN 发送控制命令。
`applications/big_head_config.hpp` 中的 `big_head_enabled` 可关闭应用；F407 不构建该应用。
配置 `-DUSER_TEST=ON` 时由 [SMC 测试应用](applications/test/README.md) 替代 big_head 入口。

## 配置与接入

板载 BMI088、LED 和固定 GPIO 接线随选板切换；AHRS 设备和应用串口选择集中在
[`configs/user_config.json`](configs/user_config.json)，生成结果为 `configs/include/user_config.hpp`。
当前 H723 沿用旧应用接线：VT03 使用 USART1（921600 baud）、DR16 使用 UART5，本地电机使用
FDCAN1，下位机使用 FDCAN2；两路应用 CAN 在 `big_head_config.hpp` 中选择。UI 经下位机发送，
关闭本板 UI/裁判串口及 PS2 输入。
F407 的 UI 仍以 USART1 为示例。配置头不初始化设备，调用方式见
[devices 文档](devices/README.md#应用资源选择)。

配置入口为 `configs/scripts/generate_config.cmake`，输入保持一个 JSON，输出保持一个头文件。
入口组织公共校验、通信参数、IMU 参数和板级资源选择，C++ 输出布局由独立模板描述：

| `configs/scripts/` 文件 | 职责 |
| --- | --- |
| `values.cmake` | 数值、布尔值和三维向量校验 |
| `communication.cmake` | UI、PS2、遥控与裁判接收参数 |
| `imu.cmake` | DMIMU、AHRS、ESKF 和温控参数 |
| `boards.cmake` | 板级资源、启用选择和资源冲突检查 |
| `user_config.hpp.in` | 生成头文件模板 |

添加配置项时，在对应参数文件中读取并校验，在模板中声明 C++ 字段。
CMake 配置阶段自动生成头文件，构建时会检测 JSON 修改并重新生成。只更新编辑器配置头、
不编译固件时运行 `cmake -P configs/scripts/generate_config.cmake`。
生成头文件随仓库提交，初次打开无需构建；不要直接编辑它。
资源名称引用各板 `config.hpp` 已有定义，`none` 表示未连接。串口协议格式由生成脚本固定，
不属于接线选择。

| JSON 字段 | 用途 |
| --- | --- |
| `boards.<板型>` | AHRS 后端、串口/CAN/PS2 资源、遥控器启用选择和板级温控参数 |
| `ahrs` | 采样周期、线程栈、恢复策略、传感器零偏及 ESKF 参数 |
| `dmimu` | 设备 ID、主动/请求模式、组帧及离线超时 |
| `bmi088_heater` | 温控周期、温度超时、稳定判定、线程优先级和栈大小 |
| `referee` | 接收线程优先级、周期、离线超时和 `stack_size`（默认 2048 字节） |
| `ui`、`ps2` | 发送超时、PS2 时序及死区 |
| `remoter` | 供应用组织遥控处理的调度参数，不会创建遥控线程 |
| `application.imu_offset` | 应用参考点到 IMU 的位移，设备服务不自动执行位置补偿 |

大头应用的启用选择、两路 CAN、控制、USB、VT03、PID/前馈和超时参数放在
[applications/big_head_config.hpp](applications/big_head_config.hpp)，不进入共享配置生成链。

`boards.<板型>.ahrs_device` 选择 `bmi088`、`dmimu` 或 `none`，`ahrs` 保存调度参数。
通过 `imu::start_bmi088()` 或 `imu::start_dmimu()` 启动，在 `imu::attitude::output`
订阅姿态；服务约束见 [模块接入文档](modules/README.md)，应用的轴向适配与资源所有权见
[big_head 文档](applications/big_head/README.md)。

裁判系统通过 `referee::init(user_config::referee)` 启动。接收线程完成组帧、校验和解析，
应用用 `referee::read(sample<T>&)` 读取已解析的数据；需要逐包处理时，在 `referee::callbacks`
中注册对应类型，回调直接接收 `sample<T>`，无需再次解包。缓存读取和回调示例见
[模块接入文档](modules/README.md#裁判接收)。

遥控输入选择、控制映射和失联策略由应用决定，使用方式见 [设备文档](devices/README.md)。
共享总线须按各接口约定协调所有权和互斥；裁判回调运行在接收线程中，不能阻塞。

## 开发环境

安装以下工具，并将其可执行文件所在目录加入本机 `PATH`：

- CMake 3.22 或更新版本。
- Ninja。
- ARM GNU Toolchain（`arm-none-eabi-gcc`、`arm-none-eabi-g++`）。

仓库不保存个人工具链的绝对路径。更改 `PATH` 后，重新启动编辑器使其生效。
不要复制其他电脑上的 `build/`；CMake 缓存包含本机路径，应在本机重新配置。
需要个人配置时使用已被 Git 忽略的 `CMakeUserPresets.json`。

## VS Code

1. 打开仓库根目录，安装工作区推荐的 **CMake Tools** 和 **C/C++** 扩展。
2. 执行 `C/C++: Select a Configuration`，选择 `h723_mc02` 或 `f407_cboard`。
3. 直接编辑源码，使用补全、跳转和诊断；不需要 Configure 或 Build，也不需要 `build/` 目录。

IntelliSense 使用仓库内 `.vscode/c_cpp_properties.json` 的独立配置，从 `PATH` 查找
`arm-none-eabi-g++`，查询系统头文件与内置宏。所有工程路径都相对于工作区，
不依赖本机绝对路径、CMake 配置提供者或编译数据库。
两块板卡的 wrapper、HAL、CMSIS 和 ThreadX 端口分别配置，避免同名头文件混用。
`libs/`、`bsp/`、`devices/`、`modules/` 下新增头文件可直接参与解析，无需先加入构建。
共享代码一次按所选板卡的 Debug 配置分析。修改板卡宏或第三方包含目录时，
需要同步更新此文件；编辑器可见不代表该文件已接入实际构建。

需要构建固件时，再通过 `CMake: Select Configure Preset` 选择同名板卡并执行构建。
CMake preset 与 IntelliSense 配置独立选择，切板时需保持两者一致。
工作区已关闭打开工程时自动运行 CMake Configure。

本仓库默认使用微软 C/C++ 扩展提供语言服务和基于 `.clang-format` 的格式化。
如果本机安装了 clangd，请在此工作区禁用 clangd 扩展，避免两套语言服务同时运行。

如果已有窗口仍沿用旧配置，重新选择上述 C/C++ 配置并执行 `Developer: Reload Window`。
若仍有异常红线，执行 `C/C++: Log Diagnostics`，检查实际采用的 ARM 编译器、宏和包含目录。
真正的语法错误仍会正常显示；不要通过关闭诊断隐藏错误。

## 命令行构建

在仓库根目录运行：

```sh
cmake --preset h723_mc02
cmake --build --preset h723_mc02
```

构建 F407 时将上述两个命令中的 preset 名称改为 `f407_cboard`。
构建产物位于 `build/<preset名称>/`，编译数据库由 CMake 自动生成，供其他工具按需使用。

RFID 默认读取线程及底层遥控限制见 [RFID 服务](modules/README.md#rfid)。
