# big_head 应用

从旧 `big_head_pid` 迁移的 H723 云台应用：本板驱动 Pitch 和左右摩擦轮，通过下位机
控制 yaw、拨弹盘及底盘；USB 与视觉通信，BMI088 提供姿态，DR16/VT03 提供操作输入。
当前不包含旧工程的在线系统辨识功能。F407 默认不编译此应用。

`app.cpp` 负责启动、资源和线程，`control.cpp` 负责控制与射击状态机，`protocol.hpp`
定义应用通信协议；`remote_stream.hpp` 为设备解码接口提供串口字节流组帧。
应用提供强 `app_start()`，由 BSP 的 ThreadX 初始化入口调用，不另建 `main()`。

## 接线与协议

| 用途 | 默认资源 | 说明 |
| --- | --- | --- |
| VT03 | USART1，PA9 TX / PA10 RX | **921600、8N1**，沿用旧工程实际接线；不是 UART7 |
| DR16 | UART5，PC12 TX / PD2 RX | 100000、8E1；仅接收 |
| 左摩擦轮 | FDCAN1，反馈 `0x201` | 无减速箱 M3508，目标 `+600 rad/s` |
| 右摩擦轮 | FDCAN1，反馈 `0x202` | 无减速箱 M3508，目标 `−600 rad/s` |
| Pitch | FDCAN1，反馈 `0x205` | GM6020，电流命令模式，限幅 `±15000` 协议单位 |
| 下位机 | FDCAN2 | 接收 `0xC1`；发送 UI `0xB1`、控制 `0xB2` |
| BMI088 | SPI2，PC0 / PC3 片选 | 由既有 BMI088 服务独占 |
| 视觉 | USB CDC ACM | 接收 28 字节，发送 44 字节 |

两路 CAN 均由应用以经典 CAN、标准 ID 初始化，当前板级标称速率为 1 Mbit/s。
电机 manager 发送 `0x200` 和 `0x1FF` 分组命令。`0xB2` 保留旧布局，携带
`yaw_cur`、`tri_spd` 及底盘控制字段；本板不注册拨弹电机。
裁判热量、弹速和队伍颜色来自下位机 `0xC1`，UI 经下位机转发，因此本应用不启动
裁判串口服务和本地 UI 串口，H723 的 `referee_uart`、`ui_uart` 配置为 `none`。

DR16 使用新仓库文档定义的 8E1 接口：HAL 的 9 位字长包含校验位。旧工程 CubeMX
填写的是 9N2，本版未照搬该设置；实际 DR16 接收仍需上板验证。
VT03 波特率通过应用配置覆盖通用 `vt03_line`，不修改设备层默认串口格式。
设备文档中的 UART7 / 115200 是模板接线示例，此应用以本表和用户 JSON 为准。
当前 USART BSP 的 `configure()` 会使初始化状态失效，应用按 `init → configure → init`
顺序配置线路后启动 RX DMA。这里 `init` 使用默认 block 发送模式：UART5 没有 TX DMA，
该模式仍支持 RX DMA。

摩擦轮使用应用内的 `friction_motor`，基于现有 DJI 电机实现设置减速比 1；通用
`m3508` 的减速比和接口保持原样。摩擦轮速度 PID 默认 `{100, 0, 0, 15000, 3}`。

USB 使用旧帧头、CRC16、小端 IEEE 754 单精度布局。接收角度为 rad、角速度为 rad/s、
角加速度为 rad/s²；发送四元数为 WXYZ，yaw 为连续角度，yaw/pitch 使用 degree，
角速度仍为 rad/s，弹速为 m/s。接收支持分片、粘包、CRC 错帧重同步和非有限数拒绝。
发送仅在 IMU、下位机状态新鲜时进行；PC 端同样应按字节流组帧，检查 CRC 和超时。
帧布局以 [protocol.hpp](src/protocol.hpp) 为准，不依赖编译器位域布局。

## 配置与姿态

所有用户参数集中在 [configs/user_config.json](../../configs/user_config.json)，由同一
生成入口输出 `configs/include/user_config.hpp`，不要手动修改生成头。

| 配置组 | 用途与单位 |
| --- | --- |
| `boards.h723_mc02` | `big_head_enabled`、遥控 UART、`motor_can`、`lower_can`、BMI088 和温控选择 |
| `application.big_head` | 控制/USB 周期、线程优先级和栈、VT03 波特率、超时、ID、PID、限位、发射参数 |
| `remoter.offline_timeout_ms` | 两种遥控输入的有效期，默认 100 ms；本应用不使用独立遥控线程 |
| `ahrs`、`bmi088_heater` | 既有姿态和温控服务配置，应用显式启动 |

应用控制周期固定为 1 ms，旧 PID 的 I/D 系数已经包含采样周期，生成器拒绝其他周期。
PID 数组顺序是 `{kp, ki, kd, max_out, max_iout}`；前馈数组为
`{预测时间秒, 惯量, 阻尼, 命令/力矩比例}`，Pitch 比例对应速度 PID 输出单位，
之后再乘 `pitch_current_scale`；一维 Kalman 数组为 `{Q, R}`。
内部角度均为 rad，角速度为 rad/s；电流限制是协议命令值，不是安培。
`trigger_*` 为下位机拨弹速度的有符号整数协议值，沿用旧默认 `4/8/16/−8`。
`friction_gear_ratio` 默认 1，速度和掉速阈值按该输出轴定义，修改减速比时须同步整定。

BMI088 服务输出原始传感器轴姿态，应用保留旧安装映射：`pitch = attitude.roll`、
`roll = -attitude.pitch`、yaw/total_yaw 不变，Pitch 角速度取 gyro.x，yaw 取 gyro.z。
USB 四元数由映射后的欧拉角重建。服务继续独占 SPI2、加热 TIM3 CH4 及其
TIM6 / DMA2 Stream6 断点保护资源，详见 [modules](../../modules/README.md)。

## 操作与控制行为

输入优先级为在线 DR16 优先，其次 VT03；全部离线时停机。开机、遥控失联恢复或切换
输入源后，需要先到左开关低档再切到中/高档，或在非低档按一次 `Ctrl+X` 解锁。
左开关低档、VT03 暂停键或 `Ctrl+Z` 撤销解锁；IMU 未运行、反馈超时时保持零输出。

| 输入 | 行为 |
| --- | --- |
| 左开关中 / 高 | 普通 / 自旋；`Shift` 也请求自旋 |
| 右摇杆、鼠标 | 手动 yaw / Pitch；静止时保持角度，Pitch 默认限位 `[-0.6, 0.4] rad` |
| 左摇杆、WASD | 底盘移动请求；键盘输入有斜坡 |
| DR16 右开关低 / 中 / 高 | Closed / Warm / Fire |
| VT03 Fn1 / Fn2 | 关闭射击 / 在 Closed → Warm → Fire 间循环，按下沿生效 |
| Warm + 鼠标左键 | 手动拨弹；同时按 Ctrl 为反转 |
| X | 切换低/高弹频；`Ctrl+X` 解锁不切弹频 |
| F / Ctrl+F | 切换打符；Ctrl+F 还切换仅云台模式 |
| Q / E | 腿长档位减/加，范围 0～2；自旋时归零 |
| R / C / G / V | 转身 180° / 跳跃 / 飞坡 / 楼梯请求 |
| B | 请求下位机 UI 重置 |

`auto_aim_default`、`auto_shoot_default`、`gimbal_only_default` 默认均为 `true`，
保留旧工程实际强制开启的行为。若将前两项设为 `false`，鼠标右键或 VT03 扳机启用
自瞄，自瞄期间 Ctrl 启用自动射击。仅云台模式的 Ctrl+F 切换现在实际生效。

自动瞄准保留双环、预测、速度/加速度前馈和 yaw 的 ±90°目标接收范围；视觉失效后
转手动并重置鼠标滤波历史。Pitch 目标夹在机械限位内，越界目标不允许视觉自动发射。
视觉 `mode == 2` 且角度误差合格时可发射；**Fire 档是旧工程的试射请求，可不依赖
视觉目标**。默认自动射击通过摩擦轮掉速结束一次拨弹；持续请求可启动下一次。
单次拨弹默认最多 1000 ms，超时后必须撤销请求再触发。关闭自动射击时，Fire 档使用
`trigger_continuous`。遥测弹数仅统计自动拨弹期间检测到的掉速，不等同于裁判弹数。

拨弹均要求摩擦轮在线、转向正确且达到目标速度的 90%，下位机热量上限非零，当前
热量低于上限的 70%。任一摩擦轮或下位机离线时，两轮和拨弹均停止。视觉丢失会取消
视觉触发的发射，仍保留明确的 Warm 手动/Fire 试射操作。

## 线程、生命周期与恢复

`app_start()` 注册电机和 CAN 回调，启动两路 CAN，注册 USB 接收，显式启动 BMI088、
订阅姿态，最后创建控制与 USB TX 线程。初始化失败进入板级 `Error_Handler()`。
UART 在控制线程启动接收；没有静态构造自动启动服务，也不修改 CubeMX 生成入口。

| 执行上下文 | 所有权与约束 |
| --- | --- |
| CAN IRQ | 只把目标帧复制到固定最新值槽，记录时间/序号；不操作电机或 PID |
| UART IRQ | 只复制 DMA 接收字节到固定缓冲；溢出丢弃未解析数据并重同步 |
| 控制线程，默认优先级 4 / 8192 字节栈 | 1 kHz，唯一拥有遥控解析、控制状态、PID、电机对象及 CAN 发送/恢复 |
| BSP USB RX 线程，优先级 10 / 2048 字节栈 | 调用应用回调完成视觉解析，发布最新值；回调不阻塞 |
| USB TX 线程，默认优先级 11 / 2048 字节栈 | 唯一 USB 写者，默认间隔 2 ms，单次写超时 5 ms |
| BMI088/温控服务线程 | 服务持有传感器和加热资源，通过既有 `msg` 发布姿态 |

共享临界区只覆盖快照复制；解码、外设调用和控制计算在临界区外。所有 DMA 缓冲、
回调目标、线程栈和订阅具有固件生命周期。本版没有运行时卸载或停止入口，正常任务
持续运行；停用应用应关闭配置并重新构建。

默认有效期为 IMU 20 ms、视觉/下位机/电机 100 ms。数据过期时先归零相应命令；
CAN 首次故障立即在控制线程恢复，失败后按 100 ms 间隔重试，清除旧反馈，恢复周期
先输出零，再等待新反馈。电机总线只有在 `0x200`、`0x1FF` 两组命令都成功提交后才
刷新发送看门狗，持续 busy 或发送错误同样触发恢复。UART 错误或离线后由控制线程停止并重建接收，丢弃残帧。
总线断线时软件无法保证零命令到达对端，下位机和电调仍需自己的命令超时保护。

IMU/Pitch 临时失效会清空 PID 和射击状态，但保留解锁状态：反馈恢复后可自动恢复
云台控制；DR16 物理射击档也会重新生效，VT03 需重新选择射击模式。这与遥控失联/
切源后必须重新解锁的策略不同。

调试器可查看 `big_head::diagnostic` 的周期超限、下位机发送失败、CAN 重启、USB
发送失败/短写和 Pitch 饱和计数。首次上电应先确认解锁和失联停机，再逐个验证电机方向、安装轴和参数。

## 构建与验证

在仓库根目录构建固件：

```sh
cmake --preset h723_mc02
cmake --build --preset h723_mc02
```

主机测试使用独立 CMake 工程和主机 `g++`，不要使用 ARM 交叉编译器；Windows MinGW
所需的 GNU 位域选项已由测试工程设置：

```sh
cmake -S applications/big_head/tests -B build/big_head_tests -G Ninja -DCMAKE_CXX_COMPILER=g++
cmake --build build/big_head_tests
ctest --test-dir build/big_head_tests --output-on-failure
cmake -P configs/tests/validate.cmake
```

主机测试覆盖协议黄金字节、分片/粘包/CRC/非有限数、遥控组帧与映射、电机速度单位和
分组命令，以及解锁、切源、故障归零、限位、自动射击和手动接管回归。配置测试覆盖
两块板的默认选择及资源冲突、重复 ID、非法周期和控制参数。

本次已完成 H723 清理后全量编译链接、F407 编译链接、三个主机测试以及配置的两个
正常/十二个非法用例验证。USB/遥控实机收发、BMI088 安装轴、CAN 对端协议、电机
方向与负载下的 PID、温控和断线恢复仍需台架验证；主机测试不能验证这些硬件行为。
