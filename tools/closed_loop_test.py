#!/usr/bin/env python3
"""Safe STM32F407 two-wheel closed-loop test and CSV telemetry monitor."""

from __future__ import annotations

import argparse
import struct
import time

from rk3576_uart_example import StreamParser, build_cmd_vel, build_stop

MSG_DEBUG_STATUS = 0x7F
FAULT_CMD_TIMEOUT = 1 << 0

DEBUG_COLUMNS = (
    "t_ms",
    "target_left_mm_s",
    "target_right_mm_s",
    "measured_left_mm_s",
    "measured_right_mm_s",
    "output_left_percent",
    "output_right_percent",
    "left_ticks",
    "right_ticks",
    "odom_linear_mps",
    "odom_angular_rps",
    "tof_left_m",
    "tof_right_m",
    "ultrasonic_m",
    "fault_flags",
    "valid_flags",
    "obstacle_flags",
    "sensor_fault_flags",
)


def decode_debug(payload: bytes) -> dict[str, float | int]:
    if len(payload) != 64:
        raise ValueError(f"DEBUG_STATUS length is {len(payload)}, expected 64")
    values = struct.unpack("<IffffffiifffffHHHH", payload)
    return dict(zip(DEBUG_COLUMNS, values))


def print_csv(data: dict[str, float | int]) -> None:
    fields: list[str] = []
    for key in DEBUG_COLUMNS:
        value = data[key]
        if key.endswith("flags"):
            fields.append(f"0x{int(value):04X}")
        elif isinstance(value, float):
            fields.append(f"{value:.4f}")
        else:
            fields.append(str(value))
    print(",".join(fields), flush=True)


def main() -> int:
    parser = argparse.ArgumentParser(description="F407 chassis closed-loop test")
    parser.add_argument("--port", required=True, help="Windows COM port, for example COM8")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--linear", type=float, default=0.0, help="chassis command in m/s")
    parser.add_argument("--angular", type=float, default=0.0, help="chassis command in rad/s")
    parser.add_argument("--duration", type=float, default=2.0, help="command duration in seconds")
    parser.add_argument("--rate", type=float, default=20.0, help="CMD_VEL rate in Hz")
    parser.add_argument(
        "--watchdog-test",
        action="store_true",
        help="then stay silent for 1.2 s and verify the 500 ms hard stop",
    )
    args = parser.parse_args()
    if args.duration <= 0.0 or args.rate <= 0.0:
        parser.error("duration and rate must be positive")

    try:
        import serial  # type: ignore
    except ImportError as exc:
        raise SystemExit("Missing pyserial. Run: python -m pip install pyserial") from exc

    stream_parser = StreamParser()
    sequence = 0
    period_s = 1.0 / args.rate
    watchdog_passed = False
    print(",".join(DEBUG_COLUMNS))

    with serial.Serial(args.port, args.baud, timeout=0.01) as uart:
        uart.reset_input_buffer()
        try:
            deadline = time.monotonic() + args.duration
            while time.monotonic() < deadline:
                uart.write(build_cmd_vel(sequence, args.linear, args.angular))
                sequence = (sequence + 1) & 0xFFFF
                until = time.monotonic() + period_s
                while time.monotonic() < until:
                    for frame in stream_parser.feed(uart.read(256)):
                        if frame.msg_type == MSG_DEBUG_STATUS:
                            print_csv(decode_debug(frame.payload))

            if args.watchdog_test:
                silent_deadline = time.monotonic() + 1.2
                while time.monotonic() < silent_deadline:
                    for frame in stream_parser.feed(uart.read(256)):
                        if frame.msg_type != MSG_DEBUG_STATUS:
                            continue
                        data = decode_debug(frame.payload)
                        print_csv(data)
                        outputs_zero = (
                            abs(float(data["output_left_percent"])) < 0.01
                            and abs(float(data["output_right_percent"])) < 0.01
                        )
                        watchdog_passed |= bool(int(data["fault_flags"]) & FAULT_CMD_TIMEOUT) and outputs_zero
        finally:
            uart.write(build_stop(sequence))
            uart.flush()

    print(
        f"frames={stream_parser.valid_frames} crc_errors={stream_parser.crc_errors} "
        f"length_errors={stream_parser.length_errors} version_errors={stream_parser.version_errors}"
    )
    if args.watchdog_test:
        print("PASS 500 ms command watchdog" if watchdog_passed else "FAIL 500 ms command watchdog")
        return 0 if watchdog_passed else 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
