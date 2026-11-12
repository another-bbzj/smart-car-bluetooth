# 校赛图像处理原型

这是从原考核代码复制出的独立工作目录，不修改原考核程序和参考固件。当前第一步聚焦校赛白色 PVC / 蓝色背景的前端适配，并提供离线回放与调试输出。

## 当前实现

- 原考核灰度边线模式保留，可用 `--mode gray` 对照。
- 新增 HSV 白色候选掩膜与蓝色排除掩膜；小型封闭的非蓝色缺口可填补，蓝色缺口始终保留。
- 只在画面近端搜索赛道起点，避免越过蓝色缺口直接捕获远处白色赛道。
- 图像丢失赛道时会清空边线和预瞄/Pure Pursuit 有效标志，避免复用上一帧的结果。
- 支持单张图片、视频文件、摄像头回放；可保存每帧 CSV 和抽样的输入、mask、蓝色排除图、边线叠加图、鸟瞰图。
- 默认不启动 TCP 服务器；需要远程查看时显式传 `--tcp`。
- 不包含电机控制输出。

## 编译与测试（WSL Ubuntu）

```bash
cd /mnt/c/smartcar/校赛/vision
bash build.sh vision
bash build.sh test_frontend
./test_frontend
```

若 Ubuntu 缺少 OpenCV 开发包，需要先安装 `libopencv-dev`。

## 离线回放

输入图像需为标定尺寸 `640x480`，并按 BGR 读取。

```bash
./vision --image /path/to/frame.png --output /path/to/new-output-dir
./vision --video /path/to/recording.avi --output /path/to/new-output-dir --save-every 10
./vision --camera 0 --show --tcp
```

输出目录必须不存在，程序不会覆盖现有数据。`frames.csv` 对所有处理帧留记录；PNG 按 `--save-every` 抽样。`--show` 打开本机窗口，`q` 或 ESC 退出、空格暂停视频。默认 HSV 阈值在 `track_frontend.yml`，或用 `--config` 指定另一份配置。运行灰度对照时添加 `--mode gray`。

示例合成数据：

```bash
./vision --video test_fixtures/sequence.avi --output /tmp/vision-replay --save-every 4
```

## 验证情况

- `build.sh vision` 与 `build.sh test_frontend` 已在 WSL Ubuntu / OpenCV 4.6 编译通过。
- 合成前端用例 19 项通过，覆盖纯白/纯蓝拒绝、亮度变化、非蓝暗背景、彩色小遮挡、蓝色缺口保护、断路附近远赛道拒绝、图像边界截断、空图、配置合法性、二值边线搜索及丢线输出清空。
- 合成视频 15 帧离线回放通过：5 帧赛道、5 帧蓝色缺失区、5 帧赛道；CSV 中缺失区输出 `seed_found=0`、边线计数为零、`preview_found=0`、`pp_valid=0`，恢复后重新有效。
- 合成图像不能证明实车识别率，也不能证明闭环车辆安全或速度。

## 重要限制与下一步

- HSV 参数只是初始值，必须用比赛实拍数据校准，包括曝光、阴影、白色 PVC、蓝底、蓝锥桶、黄色/红色障碍和数独纸面。
- 当前 160 cm 断路只表现为“无种子/丢线”。尚未实现独立断路状态机、物理距离/时间估计、编码器/IMU 直行和接回确认；在该模块完成并与控制端集成前，不要用于实车无人值守通过断路。
- 当前没有路障分类/绕行、AprilTag、数独识别与求解或盲盒处理。
- 当前 Pure Pursuit 参数及鸟瞰标定沿用考核工程，不等于新相机安装位置或校赛 PVC 场景已标定。
- 仓库中 `TCP_BIND_IP` 默认绑定所有网卡；仅在可信局域网运行 `--tcp`。
