# 固件故障4角度阈值验证记录（2026-10-08）

> 当时的实际软件验证；本次文档整理未重跑。

## 6. 验证及烧录后检查

实际完成：

- tools/test_robot.ps1：13组主机回归通过，包含三种减速比边界、配置暂存/失败/生效、禁用但有减速比、未配置、大计数饱和/回绕、自检和协议；Flash测试包含129个写入中断场景及20000个异常ADU。
- tools/test_robot_port_selftest.py：提取实际端口中断/驱动/主循环函数进行模拟，通过三种阈值、未配置不报警、捕获错误仍生效和自检往返保护。
- test_robot_host.py：10项通过；test_robot_docs.py及test_robot_auto_frames.py通过。
- CMake configure/build debug：成功。
- tools/build_eide.ps1：EIDE等效构建成功，53个对象。两套ELF text=23080、data=20、bss=10788。
- 构建仍有链接器RWX LOAD段警告；EIDE另有厂商USB代码epcs变量未使用警告。本次未修改这些位置。

Flash、真实串口/CAN、引脚电压、真实活动角度：not actually run。

固件输出：[CMake HEX](../../build/debug/GD32F303VET6_MotorControl.hex)、[EIDE HEX](../../build-eide/Debug/GD32F303VET6_MotorControl.hex)。下载前读取并记录UID、配置和位置；确认下载工具是否擦除0x0807F000开始的4KB持久化区。擦除后应恢复配置并在明确姿态设零，不能把旧角度直接当作可信零点恢复。

烧录后先只读核对UID及已生效减速比，再在可控条件下验证516的86/87、290的48/49、82的13/14脉冲边界。gear=0时只验证不报故障4，并确认捕获错误等其他保护仍正常。清故障后核对unexpected_fg归零。上述硬件步骤尚未执行。
