# NEUQ STM32F407 开源培训库

东北大学秦皇岛分校智能车实验室（**NEUQ Smartcar**）面向新生培训与招新校赛整理的 STM32F407 工程：在 ST 官方 **HAL** 之上做二次封装，**API 风格贴近比赛环境中常用的逐飞开源库**，便于入门后无缝衔接队内开发习惯。

## 定位与目的

- **培训与校赛**：覆盖校赛赛题中常见外设与用法，适合实验室招新、校内智能车相关实践。
- **熟悉逐飞风格**：通过相近的模块划分与调用方式，降低入队后的学习成本。
- **硬件性价比**：STM32F407 在性能与价格上与比赛用车规级 MCU 相比更易采购，更适合大量板子用于教学与练习；本库侧重**教学与校赛**，与正式国赛硬件选型无强制对应关系。

当前库仍在迭代中，实现质量与文档会持续完善；**欢迎爱好者交流讨论、提交 Issue / PR 贡献代码与示例**。

## 硬件与工具链

| 项目 | 说明 |
|------|------|
| 芯片 / 封装 | STM32F407（工程目标为 **STM32F407VETx**，512K Flash） |
| 固件库 | STM32F4xx HAL（随工程附带） |
| 推荐 IDE | **Keil MDK-ARM**（源码注释中示例环境为 MDK-ARM v5.38） |
| 工程入口 | `MDK-ARM/NEUQ_STM32F407VET6_Opensource_Library.uvprojx` |

用户程序从 `User/NEUQ_Main.c` 中的 `User_Main()` 开始编写；总头文件为 `Library/NEUQ_lib/NEUQ_lib_include.h`，可按需包含各模块头文件。

## 仓库结构（概要）

```
├── MDK-ARM/                 # Keil 工程与链接脚本等
├── Library/
│   ├── HAL_lib/             # HAL、CMSIS、启动文件与 Cube 相关代码
│   └── NEUQ_lib/            # NEUQ 二次封装
│       ├── NEUQ_lib_Driver/ # 底层驱动封装（GPIO、UART、ADC、定时器、编码器、I2C/SPI、Flash 等）
│       ├── NEUQ_lib_Device/ # 外设器件驱动（如 LCD、摄像头等）
│       ├── NEUQ_lib_APP/    # 应用层算法（如 PID、IMU 解算等）
│       ├── NEUQ_lib_Debug/  # 调试输出等
│       └── NEUQ_lib_include.h
├── User/                    # 用户代码入口、中断分发等
├── CODE/                    # 自研整车代码（控制核心、协议、PID、电机、编码器、串口链路、遥测）
└── Docs/                    # 可选参考文档（如数据手册 PDF）
```

具体模块以仓库内实际文件为准；各文件顶部的**修改记录表**中注明了作者与版本备注。

## 版权与许可证

本库为 **NEUQ Library** 的一部分，声明采用 **GNU GPL v3.0**（及后续版本任选）。使用、再发布或修改时须遵守 GPL 条款，并**保留原有版权声明**；详见各源文件头部说明及 [GNU GPL](https://www.gnu.org/licenses/)。

## 致谢与贡献者

主要代码贡献来自实验室队员 **张瑞、张献政、沈泉、袁一丁** 等；更细的作者与日期见**各 `.c` / `.h` 文件开头**的修改记录。

## 培训资料

- **NEUQ 21 届智能车实验室招新 · 控制方向培训第一讲**（Bilibili）：  
  [https://www.bilibili.com/video/BV1Qdp4zuESv/](https://www.bilibili.com/video/BV1Qdp4zuESv/)

---

*东北大学秦皇岛分校智能车实验室 · NEUQ Smartcar*
