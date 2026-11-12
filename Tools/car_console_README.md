# 蓝牙差速车网页调试台

用桌面版 Chrome / Edge，通过 **Web Serial + 经典蓝牙 SPP** 直接连接小车：实时驾驶、转速曲线、停车调 PID、导出 CSV。不需要 VOFA+、不需要 `pyserial`、不需要安装任何 Python 包。

**在线版：** <https://another-bbzj.github.io/smart-car-bluetooth/>（只需浏览器，不需要下载或 Python）

## 文件

| 文件 | 用途 |
|---|---|
| `car_console.html` | 页面本体，CSS/JS 全部内嵌，无外部依赖 |
| `car_console.bat` | Windows 启动器：起本地服务并自动打开 Chrome/Edge |
| `car_console_server.py` | 本地 HTTP 服务（仅 Python 标准库），bat 会调用它 |

三个文件必须放在同一目录，文件名不要改。

## 快速开始（Windows）

1. 在 Windows 设置里配对蓝牙模块 `JDY-31-SPP`，PIN `1234`
2. 双击 `car_console.bat`（需要 **Python 3.8+**）
3. 页面点「选择串口」，选蓝牙配对生成的**出站 COM 口**
4. 点「连接」，等状态显示「停止」
5. **先把车轮架空**，勾选「已确认车轮架空或场地安全」，点「使能」
6. 按住 **W / A / S / D** 或方向键驾驶；**空格**急停

浏览器要求：**桌面版 Chrome / Edge 117+**（蓝牙 SPP 支持从 Chrome 117 开始）。Firefox、Safari、手机浏览器不支持 Web Serial。

## 不用 Python 的两种方式

- 直接用 Chrome/Edge 双击打开 `car_console.html`（`file://` 下也是安全上下文，实测可用）
- 把 `car_console.html` 放到任意 **HTTPS 静态托管**（如 GitHub Pages），打开链接即可；每个用户第一次都要授权访问自己的串口

## 安全行为

- 连接后自动发送 STOP
- 只有收到「停止」回传并使能后才接受方向键；运行中拒绝写 PID
- 方向键松开 → 双轮 0 rpm；窗口失焦 / 切后台 / 页面关闭 → 自动 STOP
- 回传中断 350 ms → 自动 STOP 并取消使能
- STOP 连发 3 次；车端另有 500 ms 通信超时，双保险
- 故障后必须先 STOP 清故障，再重新 ARM

上述保护依赖固件，**首次使用必须架空车轮**，先 40–60 rpm 核对左右轮方向和编码器符号。

## 协议

对应本仓库 `CODE/Car_Core.h`：

- 命令：`C3 3C CMD LEN DATA CRC8`（CRC-8 多项式 0x07）
- 命令：1 ARM、2 STOP、3 WHEELS（左右大端 int16 rpm）、4 PID、5 心跳
- 回传：16 × float32 小端 + 帧尾 `00 00 80 7F`，10 Hz

固件默认 **9600 8N1**；页面可切 115200，但蓝牙模块和 `CAR_UART_BAUD` 必须一起改。

## 测试

```powershell
node --test Tests/test_car_console.cjs
python -m unittest discover -s Tests -p "test_car_console_server.py" -v
```

测试使用模拟串口，覆盖协议、状态机、超时、断开重连、键盘安全和页面服务；**不能替代实车链路、制动距离和方向验证**。
