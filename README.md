# NEUQ STM32F407 库



*东北大学秦皇岛分校智能车实验室 · NEUQ Smartcar*

当前运行 **双电机差速 + 前万向轮的蓝牙手动调试版**，含双速度闭环、方向键驾驶、停车调参及断连停车。

接线、编译烧录、蓝牙配对和首次校准见 [蓝牙小车使用说明](Docs/Bluetooth_Car.md)。启动电脑驾驶工具：

```powershell
python -m pip install -r Tools/requirements-car.txt
python Tools/car_host.py
```

硬件参数集中在 `CODE/Car_Config.h`，工程仅保留当前双轮蓝牙程序。

已接入独立的香橙派视觉观测串口：USART3，PB10/PB11，115200。支持道路误差、赛道元素和四份数独子块；协议及Python发送器见 [图像通信接口](Docs/Vision_Interface.md)。当前未实现图像识别、自主巡线或任务状态机。

速度控制已加入实际采样时间、测量微分滤波、抗积分饱和、可标定前馈和双轮共享进度的加速度/加加速度受限参考，详见[速度控制升级](Docs/Speed_Control_Upgrade.md)。尚未进行实车高速过弯验证。

## 项目贡献者

- [邹俊（another-bbzj）](https://github.com/another-bbzj)

依赖库原有版权与许可声明保留在对应源码中。
