# M3508 SMC 测试应用

该应用使用 `src/test_smc.cpp` 中的本地 CAN 配置（当前为 FDCAN1），控制一台反馈 ID
为 `0x201` 的 M3508。
控制线程以 1 kHz 消费电机反馈，使用速度模式 SMC 生成受限的 C620 原始电流命令。
默认目标是输出轴 `1 rad/s`，电流限制为 `±1500`。

首次反馈连续在线 500 ms 后才使能输出；反馈丢失时立即进入 relax 并等待新反馈，CAN
状态异常时会尝试恢复 CAN，不使用发送进度超时看门狗。调试器可查看
`smc_motor_test::diagnostic` 的速度、滑模面、输出、电流、在线状态和恢复计数。

仓库目前没有 M3508 的 Nm→协议电流标定，因此第一版把 `inertia` 当作等效电流尺度，输出
并非物理转矩。当前参数只是保守的台架起点；首次上电应架空电机、确认 `0x201` 和转向，
并保持急停可用。调参集中在 `src/test_smc.cpp` 顶部的 `cfg` 与 `controller_config()`。

使用独立构建目录，避免改变默认 big_head 构建缓存：

```powershell
cmake -S . -B build/h723_smc_test -G Ninja -DBOARD=h723_mc02 -DUSER_TEST=ON
cmake --build build/h723_smc_test
```

`USER_TEST=ON` 时不会编译 big_head 的 `app_start()`；关闭该选项后恢复默认应用。
测试入口自身只启动电机控制；当前公共配置仍会由框架启动 USB 和 RFID 线程。若要求固件中
彻底只保留电机，需要另外关闭这两个公共服务，不能仅靠应用入口实现。

命令帧 `0x200` 同时覆盖 `0x201`～`0x204`，未注册的三个槽会发送零。台架上不要让这些
电调同时接受另一控制器的同组命令。
