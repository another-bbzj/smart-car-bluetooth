当前目录只包含蓝牙双电机差速车需要的业务代码。

Car_Config.h       硬件引脚、控制周期、限幅、默认 PID 与停车保护阈值
Car_Core.c/.h      蓝牙协议、双轮速度控制、状态与故障保护
Car_Hardware.c/.h  电机 PWM、方向、编码器、串口和定时中断接入
Car_PID.c/.h       速度环位置式 PID；保留原算法并统一命名
My_Vofa.c/.h       16 通道 JustFloat 回传缓冲与发送，当前仍在使用
Car_Vision.c/.h    香橙派视觉观测/元素/数独子块接收，含校验、会话和过期检查

入口：User/NEUQ_Main.c -> Car_Main()
操作与接线：Docs/Bluetooth_Car.md
图像接口：Docs/Vision_Interface.md；尚未实现图像算法或自主状态机
