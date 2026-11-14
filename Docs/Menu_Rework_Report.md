# 现场调试菜单改造报告

> 分支：`menu`
> 日期：2026-11-14
> 范围：把原“竞赛资料目录”整体替换为现场调试菜单，并接入 LCD200 显示、运行时可调参数和内部 Flash 持久化。

---

## 1. 改动概述

| 文件 | 说明 |
|---|---|
| `CODE/Car_menu.h/.c` | 重写为链表式调试菜单；LCD200 10 行渲染；顶部 2 行固定；选中项到倒数第 3 行开始滚动；参数直连控制量；动作节点长按执行 |
| `CODE/Car_params.h/.c` | 新增内部 Flash 参数持久化：扇区 11、`0x08060000`、magic/version/size/CRC16、回读校验 |
| `CODE/Car_Core.h/.c` | 增加运行时 `rpm_limit/pwm_limit/accel/jerk/cpr`；新增停车态 `SetPid/SetLimits/SetCpr/Stop` |
| `CODE/Car_Hardware.h/.c` | 新增调试快照、受控设置接口、Flash 擦写前后 PIT 暂停/时基重建 |
| `CODE/Car_adc.h/.c` | 低电压警告/严重阈值改为运行时可设 |
| `CODE/key.h` | 默认关闭双击，单击立即响应 |
| `MDK-ARM/*.uvprojx` | 加入 `Car_params.c/h` |
| `Docs/NEUQ_Linked_List_Index.md` | 手册改为调试菜单版，加入新的 ASCII/Mermaid 树状图 |

---

## 2. 菜单结构

```text
MAIN
├── STATUS（只读）
│   ├── STATE / FAULT / MODE
│   ├── L/R SPEED、L/R REF、L/R OUT
│   └── VISION / BATTERY
├── TUNE（仅 STOP 可改）
│   ├── PID LEFT / PID RIGHT（Kp/Ki/Kd）
│   ├── LIMITS（RPM/PWM/ACCEL/JERK）
│   ├── ENCODER（CPR）
│   └── BAT THRESH（WARN/CUT）
├── ACTIONS（长按 OK）
│   ├── STOP/CLEAR
│   ├── SAVE FLASH
│   ├── LOAD FLASH
│   └── DEFAULTS
└── INFO
    ├── FW VERSION / NODE COUNT
    └── LINK ERR / FLASH
```

共 42 个节点，只有 `MAIN` 一个根，最大深度 3。

---

## 3. 显示与按键

- LCD200：320×240 横屏，`ASCII_Font24`（12×24），10 行 × 24 px。
- 行 0：目录路径，固定。
- 行 1：状态/故障/模式/电压/提示，固定。
- 行 2~9：8 个列表项；选中项到屏幕第 7 行（倒数第 3 行）后开始滚动。
- 按键：PE10/PE8/PE7/PE9 = UP/DOWN/OK/BACK，低电平有效、内部上拉。
- 动作节点必须长按 OK；长按 OK/BACK 回 MAIN。

---

## 4. 参数与持久化

- 仅 STOP 状态可进入编辑和应用；RUN/FAULT 下显示 `STOP FIRST`。
- 应用顺序：限幅 → 左右 PID → CPR → 电池阈值；失败回滚。
- 安全阈值（通信超时、堵转时间、方向异常时间、周期范围）不在菜单中修改。
- 持久化介质：内部 Flash 扇区 11（`0x08060000~0x0807FFFF`），不使用 W25Q，避免 PA5/PA6/PA7 冲突。
- 写流程：解锁 → 擦除 → 按字编程 → 回读校验 → 上锁。
- 擦写前后由 `Car_Debug_FlashBegin/End` 暂停 TIM6、重建 `last_control_us`，避免误判周期故障。

---

## 5. 验证

| 级别 | 结果 |
|---|---|
| L0 静态 | 菜单表 42 节点、1 根、深度 3；建链自检通过 |
| L1 Keil | **0 Error / 0 Warning**，日志 `MDK-ARM/build/keil_menu4.log`；Code 32088 B |
| L2 主机 | `python -m unittest discover -s Tests -p "test_*.py" -v`：**24 项通过** |
| L3 上电 | **未上电验证** |

C 主机测试未运行：当前 Git Bash 环境没有 `clang`。

---

## 6. 未验证 / TBD

1. LCD200 实际显示方向、亮度、坐标偏移未上电确认。
2. 按键 PE10/PE8/PE7/PE9 与核心板实际连接未实测。
3. 内部 Flash 扇区 11 的擦写与掉电恢复未实测。
4. 为让 LCD 库的 `printf` 不死锁，新增 `NEUQ_Debug_init()`，占用 UART0 / PA9 / PA10；需确认是否保留。
5. `KEY_ENABLE_DOUBLE_CLICK` 默认由 1 改为 0，属行为变化。
6. 电池 ADC 通道 `PA0`、200k/20k 分压仍为 TBD。

---

## 7. 风险

- Flash 擦除会阻塞 CPU；已暂停 PIT 并在结束后重建时基，但实车时序未验证。
- 参数直连控制量，虽只在 STOP 应用，仍属安全相关改动，建议先架空验证。
- LCD 首次绘制和滚动刷新耗时未实测，可能影响按键响应。
