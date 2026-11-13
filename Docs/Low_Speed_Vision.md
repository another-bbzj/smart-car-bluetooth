# 低速视觉联调

本阶段把原视觉输出接入 F407 轮速闭环，适用于有人监督的普通道路低速试车。视觉丢线停车，不实施赛题的断路穿越、障碍任务、数独或比赛自动发车。软件测试通过不代表实车已标定。

## 原视觉代码保持不变

`Vision/` 是提供的 `vision.rar` 中源文件、配置及两份标定表的原样快照；不含附件预编译程序、日志或回放输出。原压缩包、原 `main.cpp`、算法、配置和 `build.sh` 均未修改。新增的 `Tools/vision_adapter.cpp` 单独构建，直接调用原 `img_handle`。原程序没有观测串口输出，因此联调需要运行这个独立入口。

适配尽量由下位机完成：TYPE=3 收到原视觉“右转正”的曲率后转换为内部“左转正”，再计算左右轮转速。原 TYPE=1 左正协议仍兼容。

## 未测量尺寸的参数接口

不预填轮胎直径或驱动轮中心距，不将测试尺寸作为实车默认值。测量后每次启动传入，参数只存 RAM。

| 参数 | CLI 选项 | 接受范围 |
|---|---|---|
| 轮胎直径（不是半径） | `--wheel-diameter-mm` | 20..200 mm，整数，必填 |
| 左右驱动轮中心距 | `--track-mm` | 50..500 mm，整数，必填 |
| 车体前进速度 | `--speed-mm-s` | 20..150 mm/s，默认80 |
| 试车时限 | `--seconds` | 大于0且不超过60秒，默认10 |

固件接口：蓝牙命令 `CMD=6 AUTO_ARM`，载荷长6字节，依次为上述直径、轮距、速度三个大端 uint16；沿用 `C3 3C CMD LEN PAYLOAD CRC8`。可由其他工具调用，也可复用 `Tools/car_auto.py` 的 `auto_arm_packet()`。命令只在 STOP、两轮实测绝对转速小于5 rpm、视觉新鲜有效时接受。

下位机按 `v_left/right = v * (1 ∓ curvature * track / 2)` 换算 rpm。两轮共同缩放到最高60 rpm，保留目标曲率；拒绝 `abs(curvature)>8/m` 或 `abs(curvature*track/2)>0.8` 的急弯请求。实际路径效果还受轮径误差、打滑和原视觉标定影响。

## 编译和连接

F407 使用原 Keil 工程构建、烧录。视觉串口为 USART3，PB11 RX 接香橙派 TX，GND 共地，3.3V TTL，115200 8N1。PB10 TX 目前不发送 ACK。蓝牙仍是 UART4 PC10/PC11、9600。

香橙派需要 Linux、C++17、OpenCV 开发库及 pkg-config。在仓库根目录编译：

```bash
bash Tools/build_vision_adapter.sh
```

先让 F407 保持 STOP，再启动独立入口；把设备名替换为实际串口，相机编号按现场确定：

```bash
cd Vision
../build/vision_adapter --camera 0 --serial /dev/实际串口 --calibrated
```

`--calibrated` 表示操作者已确认实际相机和现有标定相符，不执行自动标定。原标定表固定640×480，必须先核对实车画面和尺度。启动时发送新会话 HELLO；如果当时车辆还在 RUN，HELLO 会被拒绝，须停车后重启入口。需要新标定时另行确认修改范围，当前保持原表不变。

## 用尺寸接口启动试车

先完成轮向、编码器和相机校准；首次架空检查左右方向，随后在空旷普通赛道低速试车。关闭占用同一蓝牙串口的网页/桌面调试台。保持视觉入口运行，在 Windows 仓库根目录执行：

```powershell
python -m pip install -r Tools/requirements-car.txt
$wheelDiameterMm = [int](Read-Host '实测轮胎直径 mm')
$driveTrackMm = [int](Read-Host '实测左右驱动轮中心距 mm')
python Tools/car_auto.py --port COM实际编号 --wheel-diameter-mm $wheelDiameterMm --track-mm $driveTrackMm --speed-mm-s 80 --seconds 10 --confirm-ready
```

工具先重复 STOP 并等待停车回传，再发送一次 AUTO_ARM，收到成功回传后才发送心跳。默认10秒结束，Ctrl+C 提前结束；异常或退出均尝试发送三次 STOP。串口断开时无法保证 STOP 送达，固件500ms蓝牙看门狗仍会清零输出。一次试车失败不会自动重试启动。

## 停车条件和当前限制

- 视觉无道路、曲率无效、处理年龄加接收后年龄达到100ms、会话改变：故障7并清零输出。恢复画面后保持故障，须人工重新启动。
- 蓝牙500ms心跳、堵转、反馈方向、周期异常及控制器数值保护继续生效；手动模式可独立运行。自主运行拒绝 WHEELS 命令，STOP 在任意模式有效。
- 保留的堵转检测要求参考转速至少40 rpm，并同时满足输出和反馈门限；更低转速不在该检测范围内，不能依赖它判断低速卡住。首次实车仍需检查低速起步和跟随效果。
- 当前16通道遥测格式不变，未新增模式通道；启动工具用 STOP、命令帧数、结果和 RUN 回传确认本次启动。要求独占蓝牙控制。
- 原视觉仅提供有效标志，线上 confidence 的0/1000是有效门禁编码，并非统计置信度；当前没有误识别概率估计。
- 入口计入读相机等待和处理耗时，串口积压时丢帧。相机驱动可能忽略单帧缓冲设置；线上年龄尚不包含完整曝光、队列及传输延迟。实车必须检查帧率和最坏延迟。
- 未连接真实相机、香橙派或车轮验证；不能据此认定能完成整场比赛。原算法的环岛/十字逻辑也尚未实车验收。

## 自动验证

Windows（Python、clang/clang++、Node、Keil）：

```powershell
./Tools/check_car.ps1
```

Linux/WSL（OpenCV 开发库、C/C++ 编译器）：

```bash
bash Tools/check_vision_adapter.sh
```

前者覆盖控制、PID、协议、电脑启停、网页回归并完整重建固件。后者运行原视觉19项测试、C++编码和真实伪终端串口测试，再生成15帧“道路—丢线—道路”视频，通过独立入口生成二进制帧并交给生产版 F407 解析器和 Core，验证丢线停车与恢复后不自动重启。产物都放在忽略的 `build/` 中；这属于主机仿真，不是硬件在环。

2026-11-13 验证记录：24项 Python、17项网页测试、C 控制/PID/视觉测试、C++编码与伪终端测试、原视觉19项测试及15帧端到端回放均通过；Keil 全量重建0错误0警告。导入的20份原始源文件/配置/标定表与附件解压副本逐字节一致。输入 `vision.rar` 的 SHA256 为 `4a1bb5015e4451adbcb1bb86af41bbe48093c14ac81afe598c10ad3700a26c21`。
