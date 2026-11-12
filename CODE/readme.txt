NEUQ 智能车自研代码目录（CODE）
================================

建议新增代码仍放在本目录下，不要再创建子文件夹；添加 .c 文件后，
在 Keil 工程的 CODE 分组中右键 Add Existing Files 即可参与编译。

当前模块划分（每个外设尽量独立成一个 .c/.h）：

  Car_Config.h     整车配置：串口、引脚、控制周期、保护阈值、PID 默认参数
  Car_Utils.c/.h   公共工具：浮点绝对值/限幅、大端 16/32 位读取
  Car_PID.c/.h     速度环 PID：位置式（默认）+ 增量式（CAR_PID_USE_INCREMENTAL=1 启用）
  Car_Core.c/.h    状态机、蓝牙命令协议、测速换算、斜坡与保护
  Car_Vision.c/.h  香橙派视觉协议解析（观测 + 数独子块）
  Car_Motor.c/.h   左右电机方向 GPIO 与 PWM 输出（含换向保护）
  Car_Encoder.c/.h 左右编码器采样与 16 位回绕处理
  Car_Link.c/.h    蓝牙/视觉串口 FIFO、错误恢复与视觉快照接口
  Car_Hardware.c/.h 板级初始化、5ms 控制拍编排、VOFA+ 遥测打包
  My_Vofa.c/.h     VOFA+ JustFloat 遥测发送

依赖关系（由下到上）：
  Car_Utils / Car_PID / Car_Vision
        ↑
  Car_Core
        ↑
  Car_Motor / Car_Encoder / Car_Link
        ↑
  Car_Hardware → User/NEUQ_Main.c → main.c

公共头文件：Library/NEUQ_lib/NEUQ_lib_include.h 已包含 Car_Utils.h。
