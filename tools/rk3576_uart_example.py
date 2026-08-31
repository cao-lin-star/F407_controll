#!/usr/bin/env python3
"""RK3576 侧 UART 协议参考实现。

运行时仅额外需要 pyserial：python3 -m pip install pyserial
示例：python3 tools/rk3576_uart_example.py --port /dev/ttyS4 --linear 0.05
"""

from __future__ import annotations

import argparse
import math
import struct
import time
from dataclasses import dataclass
from typing import Iterable

SOF = b"\xAA\x55"
VERSION = 0x01
MAX_PAYLOAD = 64
MSG_CMD_VEL = 0x01
MSG_ODOM = 0x02
MSG_HEARTBEAT = 0x03
MSG_STOP = 0x04


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def build_frame(msg_type: int, seq: int, payload: bytes = b"") -> bytes:
    if len(payload) > MAX_PAYLOAD:
        raise ValueError("payload too long")
    body = struct.pack("<BBHH", VERSION, msg_type, seq & 0xFFFF, len(payload)) + payload
    return SOF + body + struct.pack("<H", crc16_ccitt_false(body))


def build_cmd_vel(seq: int, linear_mps: float, angular_rps: float) -> bytes:
    if not math.isfinite(linear_mps) or not math.isfinite(angular_rps):
        raise ValueError("CMD_VEL must contain finite values")
    return build_frame(MSG_CMD_VEL, seq, struct.pack("<ff", linear_mps, angular_rps))


def build_stop(seq: int) -> bytes:
    return build_frame(MSG_STOP, seq)


@dataclass(frozen=True)
class Frame:
    msg_type: int
    seq: int
    payload: bytes


class StreamParser:
    """支持拆包、粘包、噪声、坏长度、CRC 错误后的重新同步。"""

    def __init__(self) -> None:
        self.buffer = bytearray()
        self.valid_frames = 0
        self.crc_errors = 0
        self.length_errors = 0
        self.version_errors = 0

    def feed(self, data: bytes) -> Iterable[Frame]:
        self.buffer.extend(data)
        while True:
            sof_at = self.buffer.find(SOF)
            if sof_at < 0:
                self.buffer[:] = self.buffer[-1:] if self.buffer.endswith(b"\xAA") else b""
                return
            if sof_at:
                del self.buffer[:sof_at]
            if len(self.buffer) < 8:
                return

            version, msg_type, seq, payload_len = struct.unpack_from("<BBHH", self.buffer, 2)
            if version != VERSION:
                self.version_errors += 1
                del self.buffer[0]
                continue
            if payload_len > MAX_PAYLOAD:
                self.length_errors += 1
                del self.buffer[0]
                continue
            total = 2 + 6 + payload_len + 2
            if len(self.buffer) < total:
                return

            body = bytes(self.buffer[2 : 8 + payload_len])
            received_crc = struct.unpack_from("<H", self.buffer, 8 + payload_len)[0]
            if received_crc != crc16_ccitt_false(body):
                self.crc_errors += 1
                del self.buffer[0]
                continue

            payload = bytes(self.buffer[8 : 8 + payload_len])
            del self.buffer[:total]
            self.valid_frames += 1
            yield Frame(msg_type, seq, payload)


def decode_frame(frame: Frame) -> dict[str, float | int]:
    if frame.msg_type == MSG_ODOM and len(frame.payload) == 28:
        x_m, y_m, yaw_rad, linear_mps, angular_rps, left_ticks, right_ticks = struct.unpack(
            "<fffffii", frame.payload
        )
        return {
            "x_m": x_m,
            "y_m": y_m,
            "yaw_rad": yaw_rad,
            "linear_mps": linear_mps,
            "angular_rps": angular_rps,
            "left_ticks": left_ticks,
            "right_ticks": right_ticks,
        }
    if frame.msg_type == MSG_HEARTBEAT and len(frame.payload) == 6:
        uptime_ms, fault_flags = struct.unpack("<IH", frame.payload)
        return {"uptime_ms": uptime_ms, "fault_flags": fault_flags}
    return {"msg_type": frame.msg_type, "payload_len": len(frame.payload)}


def main() -> int:
    parser = argparse.ArgumentParser(description="RK3576 chassis UART smoke test")
    parser.add_argument("--port", required=True, help="例如 /dev/ttyS4 或 /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--linear", type=float, default=0.0, help="m/s")
    parser.add_argument("--angular", type=float, default=0.0, help="rad/s")
    parser.add_argument("--duration", type=float, default=2.0, help="发送秒数")
    args = parser.parse_args()

    try:
        import serial  # type: ignore
    except ImportError as exc:
        raise SystemExit("缺少 pyserial，请执行：python3 -m pip install pyserial") from exc

    stream_parser = StreamParser()
    seq = 0
    deadline = time.monotonic() + args.duration
    with serial.Serial(args.port, args.baud, timeout=0.01) as uart:
        try:
            while time.monotonic() < deadline:
                uart.write(build_cmd_vel(seq, args.linear, args.angular))
                seq = (seq + 1) & 0xFFFF
                until = time.monotonic() + 0.05  # 20 Hz，显著快于 500 ms 看门狗
                while time.monotonic() < until:
                    for frame in stream_parser.feed(uart.read(256)):
                        print(frame.seq, decode_frame(frame))
        finally:
            # 无论 Ctrl+C 或异常都尽力发送 STOP；底盘另有 500 ms 硬超时兜底。
            uart.write(build_stop(seq))
            uart.flush()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
