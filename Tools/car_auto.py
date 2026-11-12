#!/usr/bin/env python3
"""Supervised low-speed vision trial over the existing Bluetooth SPP link.

Geometry is deliberately required at runtime; nothing is written to flash.
Vision arrives on the separate Orange Pi UART. This tool never sends wheel targets.
"""
import argparse
import math
import struct
import time

from car_host import FAULTS, TelemetryReader, packet


def auto_arm_packet(diameter_mm, track_mm, speed_mm_s):
    values = (diameter_mm, track_mm, speed_mm_s)
    limits = ((20, 200), (50, 500), (20, 150))
    if any(type(value) is not int or not lo <= value <= hi
           for value, (lo, hi) in zip(values, limits)):
        raise ValueError('尺寸必须是实测整数：直径 20..200 mm、轮距 50..500 mm；速度 20..150 mm/s')
    return packet(6, struct.pack('>HHH', *values))


def drive(port, diameter_mm, track_mm, speed_mm_s, duration=10,
          now=time.monotonic, sleep=time.sleep, report=print):
    arm = auto_arm_packet(diameter_mm, track_mm, speed_mm_s)
    if not math.isfinite(duration) or not 0 < duration <= 60:
        raise ValueError('试车时长必须在 0..60 秒之间')
    reader = TelemetryReader()
    sample = None
    last_rx = float('-inf')

    def send(data):
        if port.write(data) != len(data):
            raise OSError('串口写入不完整')

    def receive():
        nonlocal sample, last_rx
        for values in reader.feed(port.read(4096)):
            if (values[1] not in FAULTS or values[8] not in (0, 1, 2, 3)
                    or values[9] < 0 or values[9] != int(values[9])):
                continue
            sample, last_rx = values, now()

    try:
        port.reset_input_buffer()
        # Repeated STOP frames recover a previously truncated command. ARM is never retried.
        deadline, next_stop, stops = now() + 2.0, now(), 0
        stopped_since = None
        while now() < deadline:
            if stops < 3 and now() >= next_stop:
                send(packet(2)); stops += 1; next_stop = now() + .15
            receive()
            stopped = (sample is not None and now() - last_rx < .35
                       and sample[0] == 0 and sample[1] == 0
                       and abs(sample[3]) < 5 and abs(sample[5]) < 5)
            if not stopped: stopped_since = None
            elif stopped_since is None: stopped_since = now()
            if stops == 3 and stopped_since is not None and now() - stopped_since >= .3:
                break
            sleep(.01)
        else:
            raise RuntimeError('未确认车轮停止，拒绝自主启动')
        expected_frames = sample[9] + 1
        send(arm)
        deadline = now() + 1.0
        while now() < deadline:
            receive()
            if sample is not None and sample[9] >= expected_frames:
                if sample[0] == 1 and sample[1] == 0 and sample[8] == 0:
                    break
                raise RuntimeError('自主启动被拒绝：检查新鲜视觉、尺寸和固件版本')
            sleep(.01)
        else:
            raise RuntimeError('未收到自主启动确认')
        report(f'低速自主模式已确认：{speed_mm_s} mm/s，最长 {duration:g} 秒；Ctrl+C 停止。')
        end, next_heartbeat = now() + duration, now()
        while now() < end:
            receive()
            if now() - last_rx >= .35:
                raise RuntimeError('遥测中断，正在停止')
            if sample[0] != 1 or sample[1] != 0:
                raise RuntimeError('车辆退出运行：' + FAULTS.get(sample[1], '未知故障'))
            if now() >= next_heartbeat:
                send(packet(5)); next_heartbeat = now() + .1
            sleep(.01)
    finally:
        # Never send ARM during recovery; failure to deliver STOP is backed by the 500 ms watchdog.
        for attempt in range(3):
            try: send(packet(2))
            except (OSError, RuntimeError): pass
            if attempt < 2: sleep(.15)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True, help='Windows 蓝牙出站 COM 口')
    parser.add_argument('--wheel-diameter-mm', required=True, type=int)
    parser.add_argument('--track-mm', required=True, type=int, help='左右驱动轮中心距，不是前后轴距')
    parser.add_argument('--speed-mm-s', type=int, default=80)
    parser.add_argument('--seconds', type=float, default=10)
    parser.add_argument('--confirm-ready', action='store_true', help='确认已测量尺寸并校准轮向、编码器和相机')
    args = parser.parse_args()
    if not args.confirm_ready:
        parser.error('先完成尺寸/轮向/编码器/相机校准，再显式传 --confirm-ready')
    try:
        auto_arm_packet(args.wheel_diameter_mm, args.track_mm, args.speed_mm_s)
        if not math.isfinite(args.seconds) or not 0 < args.seconds <= 60:
            raise ValueError('试车时长必须在 0..60 秒之间')
        import serial
        with serial.Serial(args.port, 9600, timeout=.02, write_timeout=.1) as port:
            drive(port, args.wheel_diameter_mm, args.track_mm, args.speed_mm_s, args.seconds)
        print('本次试车结束，停止命令已发送。')
        return 0
    except KeyboardInterrupt:
        print('已中止试车，停止命令已发送。')
        return 130
    except Exception as error:
        print(f'试车未完成：{error}')
        return 1


if __name__ == '__main__': raise SystemExit(main())
