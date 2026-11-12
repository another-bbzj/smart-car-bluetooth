#!/usr/bin/env python3
"""Bluetooth SPP differential-drive console. Run: python Tools/car_host.py

Requires pyserial for serial communication; tkinter is included with standard
Windows Python. Firmware default: UART4 9600 baud.
"""
import argparse
import csv
from decimal import Decimal, InvalidOperation
import math
from pathlib import Path
import struct
import time

TAIL = bytes.fromhex('00 00 80 7f')
CHANNELS = ['state', 'fault', 'left_ref', 'left_rpm', 'right_ref', 'right_rpm',
            'left_pwm', 'right_pwm', 'result', 'frames',
            'left_kp', 'left_ki', 'left_kd', 'right_kp', 'right_ki', 'right_kd']
FAULTS = {0: '正常', 1: '通信超时', 2: '堵转/编码器无反馈', 3: '反馈方向异常', 4: '串口错误/溢出',
          5: '控制周期异常', 6: '控制器参数/数值异常'}


def crc8(data):
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ (7 if crc & 128 else 0)) & 255
    return crc


def packet(cmd, data=b''):
    body = bytes([cmd, len(data)]) + data
    return b'\xc3\x3c' + body + bytes([crc8(body)])


def wheels(left, right):
    if any(not isinstance(x, int) or abs(x) > 180 for x in (left, right)):
        raise ValueError('轮速必须是 -180~180 的整数 rpm')
    return packet(3, struct.pack('>hh', left, right))


def pid_packet(wheel, kp, ki, kd):
    if wheel not in (0, 1):
        raise ValueError('wheel 必须是 0 或 1')
    try:
        values = [Decimal(str(x)) * 100 for x in (kp, ki, kd)]
        if any(not x.is_finite() or x < 0 or x > 65535 or x != x.to_integral_value() for x in values):
            raise ValueError()
        return packet(4, bytes([wheel]) + struct.pack('>3H', *(int(x) for x in values)))
    except (InvalidOperation, ValueError):
        raise ValueError('PID 必须在 0~655.35 之间，最多两位小数') from None


def mix(forward, turn_left, rpm):
    """Positive turn means left; preserve curvature when either wheel saturates."""
    turn = turn_left * (0.6 if forward else 1)
    left, right = forward - turn, forward + turn
    scale = max(1, abs(left), abs(right))
    return round(left * rpm / scale), round(right * rpm / scale)


class TelemetryReader:
    def __init__(self):
        self.buffer = b''

    def feed(self, data):
        self.buffer += data
        frames = []
        while True:
            index = self.buffer.find(TAIL)
            if index < 0:
                self.buffer = self.buffer[-67:]
                return frames
            chunk, self.buffer = self.buffer[:index], self.buffer[index + 4:]
            if len(chunk) >= 64:
                values = struct.unpack('<16f', chunk[-64:])
                if all(math.isfinite(x) for x in values) and values[0] in (0, 1, 2):
                    frames.append(values)


class Console:
    def __init__(self, root, args):
        import tkinter as tk
        from tkinter import ttk
        from serial.tools import list_ports
        self.tk, self.ttk, self.root = tk, ttk, root
        self.port = None
        self.armed = False
        self.pending_arm = False
        self.stop_retries = 0
        self.stop_retry_at = 0
        self.keys = set()
        self.reader = TelemetryReader()
        self.last_rx = 0
        self.next_tx = 0
        self.sample = None
        self.log_file = None
        root.title('NEUQ 双电机蓝牙调试 · 前万向轮底盘')
        root.geometry('780x610')
        root.minsize(720, 560)
        panel = ttk.Frame(root, padding=18)
        panel.pack(fill='both', expand=True)
        ttk.Label(panel, text='双电机蓝牙驾驶', font=('Microsoft YaHei UI', 20, 'bold')).pack(anchor='w')
        ttk.Label(panel, text='先架空校准方向，再以 40–80 rpm 落地调试。WASD / 方向键，空格急停。').pack(anchor='w', pady=(4, 14))
        connect = ttk.Frame(panel)
        connect.pack(fill='x')
        self.port_name = tk.StringVar(value=args.port or '')
        self.port_box = ttk.Combobox(connect, textvariable=self.port_name, width=14)
        self.port_box.pack(side='left')
        def refresh():
            ports = [p.device for p in list_ports.comports()]
            self.port_box['values'] = ports
            if not self.port_name.get() and ports:
                self.port_name.set(ports[0])
        ttk.Button(connect, text='刷新串口', command=refresh).pack(side='left', padx=5)
        self.baud = tk.StringVar(value='9600')
        ttk.Combobox(connect, textvariable=self.baud, values=['9600', '115200'], width=9, state='readonly').pack(side='left')
        ttk.Button(connect, text='连接', command=self.connect).pack(side='left', padx=5)
        ttk.Button(connect, text='断开', command=self.disconnect).pack(side='left')
        refresh()
        actions = ttk.Frame(panel)
        actions.pack(fill='x', pady=14)
        ttk.Button(actions, text='使能（从零开始）', command=self.arm).pack(side='left')
        tk.Button(actions, text='停止 / 清故障 [空格]', bg='#b42318', fg='white', command=self.stop).pack(side='left', padx=12)
        self.rpm = tk.IntVar(value=60)
        ttk.Label(actions, text='速度 rpm').pack(side='left')
        ttk.Spinbox(actions, from_=20, to=180, increment=10, textvariable=self.rpm, width=6).pack(side='left', padx=6)
        drive = ttk.Frame(panel)
        drive.pack(pady=4)
        for text, key, row, col in [('前进 W / ↑', 'w', 0, 1), ('左转 A / ←', 'a', 1, 0),
                                     ('后退 S / ↓', 's', 1, 1), ('右转 D / →', 'd', 1, 2)]:
            button = ttk.Button(drive, text=text, width=18, takefocus=False)
            button.grid(row=row, column=col, padx=4, pady=4)
            button.bind('<ButtonPress-1>', lambda e, k=key: self.set_key(k, True))
            button.bind('<ButtonRelease-1>', lambda e, k=key: self.set_key(k, False))
        self.status = tk.StringVar(value='未连接。选择 Windows 蓝牙配对生成的出站 COM 口。')
        ttk.Label(panel, textvariable=self.status, wraplength=700).pack(anchor='w', pady=12)
        self.values = tk.StringVar(value='等待回传…')
        ttk.Label(panel, textvariable=self.values, font=('Consolas', 12)).pack(anchor='w')
        tuning = ttk.LabelFrame(panel, text='停车后分别调整左右轮 PID（仅 RAM，上电恢复默认）', padding=10)
        tuning.pack(fill='x', pady=14)
        self.pid_entries = []
        for wheel, label in enumerate(('左轮', '右轮')):
            ttk.Label(tuning, text=label).grid(row=wheel, column=0, padx=6)
            entries = []
            for col, (name, default) in enumerate(zip(('Kp', 'Ki', 'Kd'), ('2', '0.12', '0'))):
                ttk.Label(tuning, text=name).grid(row=wheel, column=col * 2 + 1)
                entry = ttk.Entry(tuning, width=8)
                entry.insert(0, default)
                entry.grid(row=wheel, column=col * 2 + 2, padx=6, pady=4)
                entries.append(entry)
            self.pid_entries.append(entries)
            ttk.Button(tuning, text='写入', command=lambda w=wheel: self.set_pid(w)).grid(row=wheel, column=7, padx=8)
        self.actual_pid = tk.StringVar(value='实际 PID：等待回传')
        ttk.Label(panel, textvariable=self.actual_pid).pack(anchor='w')
        ttk.Label(panel, text='松开方向键：两轮目标归零并制动。失去窗口焦点：停止并取消使能。', foreground='#555555').pack(anchor='w', pady=8)
        if args.log:
            self.log_file = open(Path(args.log), 'w', newline='', encoding='utf-8')
            self.csv = csv.writer(self.log_file)
            self.csv.writerow(['time_s'] + CHANNELS)
        root.bind('<KeyPress>', self.key_press)
        root.bind('<KeyRelease>', self.key_release)
        root.bind('<FocusOut>', lambda e: root.after_idle(self.focus_check))
        root.protocol('WM_DELETE_WINDOW', self.close)
        root.after(50, self.poll)

    def send(self, data):
        if not self.port:
            return False
        try:
            if self.port.write(data) != len(data):
                raise OSError('串口写入不完整')
            return True
        except Exception as exc:
            self.disconnect(send_stop=False)
            self.status.set(f'通信失败：{exc}；小车将在通信超时后停车')
            return False

    def connect(self):
        import serial
        self.disconnect()
        try:
            self.port = serial.Serial(self.port_name.get(), int(self.baud.get()), timeout=0, write_timeout=0.1)
            self.port.reset_input_buffer()
            self.reader = TelemetryReader()
            self.sample = None
            self.last_rx = time.monotonic()
            self.stop()
            self.status.set('已连接，等待停止状态回传，再点击使能。')
        except Exception as exc:
            self.disconnect(send_stop=False)
            self.status.set(f'连接失败：{exc}')

    def disconnect(self, send_stop=True):
        self.armed = self.pending_arm = False
        self.stop_retries = 0
        self.keys.clear()
        if self.port:
            port, self.port = self.port, None
            try:
                if send_stop:
                    port.write(packet(2))
            except Exception:
                pass
            finally:
                port.close()
        self.sample = None
        self.status.set('已断开')

    def arm(self):
        if self.stop_retries or not self.port or self.sample is None or self.sample[0] != 0 or time.monotonic() - self.last_rx > 0.35:
            self.status.set('请先连接并收到停止状态；故障时先按停止。')
            return
        self.keys.clear()
        if self.send(packet(1)):
            self.pending_arm = True

    def stop(self):
        self.armed = self.pending_arm = False
        self.keys.clear()
        self.send(packet(2))
        # A STOP can land inside a truncated payload. Wait beyond the firmware's
        # 100ms inter-byte timeout, then send complete STOP frames again.
        self.stop_retries = 2 if self.port else 0
        self.stop_retry_at = time.monotonic() + 0.15

    def set_pid(self, wheel):
        if self.sample is None or self.sample[0] != 0 or self.armed or self.pending_arm:
            self.status.set('先按停止，收到停止回传后才能写入 PID。')
            return
        try:
            data = pid_packet(wheel, *(entry.get() for entry in self.pid_entries[wheel]))
        except ValueError as exc:
            self.status.set(str(exc))
            return
        self.send(data)

    def set_key(self, key, pressed):
        if pressed and not self.armed:
            return
        if pressed:
            self.keys.add(key)
        else:
            self.keys.discard(key)
        self.drive()

    @staticmethod
    def key_name(event):
        return {'Up': 'w', 'Down': 's', 'Left': 'a', 'Right': 'd'}.get(event.keysym, event.keysym.lower())

    def key_press(self, event):
        if event.keysym == 'space':
            self.stop()
            return 'break'
        if event.widget.winfo_class() in ('TEntry', 'TSpinbox', 'TCombobox'):
            return
        key = self.key_name(event)
        if key in 'wasd' and len(key) == 1:
            self.set_key(key, True)
            return 'break'

    def key_release(self, event):
        key = self.key_name(event)
        if key in self.keys:
            self.set_key(key, False)

    def focus_check(self):
        if self.root.focus_get() is None:
            self.stop()

    def drive(self):
        if not self.armed:
            return
        try:
            rpm = self.rpm.get()
            if not 20 <= rpm <= 180:
                raise ValueError()
            forward = int('w' in self.keys) - int('s' in self.keys)
            left = int('a' in self.keys) - int('d' in self.keys)
            self.send(wheels(*mix(forward, left, rpm)))
        except (ValueError, self.tk.TclError):
            self.stop()
            self.status.set('速度必须为 20~180 的整数 rpm')

    def poll(self):
        now = time.monotonic()
        if self.port:
            try:
                if self.stop_retries and now >= self.stop_retry_at:
                    self.stop_retries -= 1
                    self.stop_retry_at = now + 0.15
                    if not self.send(packet(2)):
                        self.root.after(50, self.poll)
                        return
                data = self.port.read(min(self.port.in_waiting, 4096))
                for sample in self.reader.feed(data):
                    self.sample, self.last_rx = sample, now
                    state, fault = int(sample[0]), int(sample[1])
                    if state == 1 and self.pending_arm:
                        self.armed, self.pending_arm = True, False
                    elif state != 1:
                        self.armed = False
                        self.keys.clear()
                        if state == 2:
                            self.pending_arm = False
                    self.status.set(f'状态：{["停止", "运行", "故障"][state]}　{FAULTS.get(fault, "未知故障")}　指令结果：{int(sample[8])}　有效帧：{int(sample[9])}')
                    self.values.set(f'左轮 目标 {sample[2]:6.1f} / 实测 {sample[3]:6.1f} rpm  PWM {sample[6]:6.1f}%\n右轮 目标 {sample[4]:6.1f} / 实测 {sample[5]:6.1f} rpm  PWM {sample[7]:6.1f}%')
                    self.actual_pid.set('实际 PID：左 %.2f / %.2f / %.2f　右 %.2f / %.2f / %.2f' % tuple(sample[10:16]))
                    if self.log_file:
                        self.csv.writerow([time.time()] + list(sample))
                if now - self.last_rx > 0.35 and (self.armed or self.pending_arm):
                    self.stop()
                    self.status.set('回传中断，已发送停止；恢复后需重新使能。')
                if self.armed and now >= self.next_tx:
                    # Repeat wheel commands themselves act as the 100ms heartbeat.
                    self.drive()
                    self.next_tx = now + 0.1
            except Exception as exc:
                self.disconnect(send_stop=False)
                self.status.set(f'接收失败：{exc}')
        self.root.after(50, self.poll)

    def close(self):
        self.disconnect()
        if self.log_file:
            self.log_file.close()
        self.root.destroy()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', help='Windows 出站蓝牙串口，例如 COM8')
    parser.add_argument('--log', help='保存回传 CSV 的路径')
    parser.add_argument('--hex', nargs='+', help='离线生成 HEX: stop / arm / heartbeat / wheels L R / pid wheel kp ki kd')
    args = parser.parse_args()
    if args.hex:
        words = args.hex
        if words[0] in ('stop', 'arm', 'heartbeat') and len(words) == 1:
            data = packet({'stop': 2, 'arm': 1, 'heartbeat': 5}[words[0]])
        elif words[0] == 'wheels' and len(words) == 3:
            data = wheels(int(words[1]), int(words[2]))
        elif words[0] == 'pid' and len(words) == 5:
            data = pid_packet(int(words[1]), *words[2:])
        else:
            parser.error('无效的 --hex 指令')
        print(data.hex(' ').upper())
        return
    try:
        import serial  # noqa: F401
    except ImportError:
        parser.exit(1, '请先安装依赖：python -m pip install -r Tools/requirements-car.txt\n')
    import tkinter as tk
    root = tk.Tk()
    Console(root, args)
    root.mainloop()


if __name__ == '__main__':
    main()
