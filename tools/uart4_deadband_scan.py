#!/usr/bin/env python3
"""Measure wheel breakaway duty through the UART4 ASCII debug interface.

The scan temporarily uses P-only control.  With the wheel stopped, the initial
output percentage is Kp * target_mm_s, so each requested duty can be tested
without adding an unsafe open-loop firmware command.
"""

from __future__ import annotations

import argparse
import dataclasses
import time
from typing import TextIO


@dataclasses.dataclass
class Trial:
    side: str
    direction: int
    duty_percent: float
    delta_ticks: int
    peak_speed_mm_s: int
    peak_pwm_x100: int
    fault: int
    moved: bool


def parse_tel(line: str) -> dict[str, int] | None:
    if not line.startswith("TEL,"):
        return None
    result: dict[str, int] = {}
    for item in line[4:].split(","):
        if "=" not in item:
            continue
        key, value = item.split("=", 1)
        try:
            result[key] = int(value, 0)
        except ValueError:
            return None
    return result


class UartConsole:
    def __init__(self, uart: object, transcript: TextIO) -> None:
        self.uart = uart
        self.transcript = transcript
        self.latest_tel: dict[str, int] | None = None

    def send(self, command: str) -> None:
        self.uart.write((command + "\r\n").encode("ascii"))

    def read_for(self, duration_s: float, show: bool = False) -> list[dict[str, int]]:
        deadline = time.monotonic() + duration_s
        telemetry: list[dict[str, int]] = []
        while time.monotonic() < deadline:
            raw = self.uart.readline()
            if not raw:
                continue
            line = raw.rstrip(b"\r\n").decode("ascii", errors="replace")
            self.transcript.write(line + "\n")
            self.transcript.flush()
            if show:
                print(line, flush=True)
            tel = parse_tel(line)
            if tel is not None:
                self.latest_tel = tel
                telemetry.append(tel)
        return telemetry

    def status(self) -> dict[str, int]:
        self.send("status")
        telemetry = self.read_for(0.25)
        if telemetry:
            return telemetry[-1]
        if self.latest_tel is None:
            raise RuntimeError("UART4 returned no TEL line")
        return self.latest_tel

    def stop_and_settle(self, settle_s: float) -> dict[str, int]:
        self.send("stop")
        self.read_for(settle_s)
        return self.status()


def run_trial(
    console: UartConsole,
    side: str,
    direction: int,
    duty_percent: float,
    target_mps: float,
    duration_s: float,
    settle_s: float,
    rate_hz: float,
    tick_threshold: int,
) -> Trial:
    target_mm_s = target_mps * 1000.0
    kp = duty_percent / target_mm_s
    console.send(f"pid {side} {kp:.6f} 0 0")
    console.read_for(0.15)
    start = console.stop_and_settle(settle_s)
    start_ticks = start[f"enc_{'l' if side == 'left' else 'r'}"]

    left = direction * target_mps if side == "left" else 0.0
    right = direction * target_mps if side == "right" else 0.0
    command = f"wheel {left:.6f} {right:.6f}"
    period_s = 1.0 / rate_hz
    deadline = time.monotonic() + duration_s
    next_send = time.monotonic()
    telemetry: list[dict[str, int]] = []
    while time.monotonic() < deadline:
        now = time.monotonic()
        if now >= next_send:
            console.send(command)
            next_send += period_s
        telemetry.extend(console.read_for(min(0.02, max(0.0, deadline - now))))

    # Capture the encoder while the command is still valid, then stop.
    console.send("status")
    telemetry.extend(console.read_for(0.12))
    end = telemetry[-1] if telemetry else console.status()
    console.stop_and_settle(settle_s)

    end_ticks = end[f"enc_{'l' if side == 'left' else 'r'}"]
    delta_ticks = end_ticks - start_ticks
    speed_key = f"spd_{'l' if side == 'left' else 'r'}_mm_s"
    pwm_key = f"pwm_{'l' if side == 'left' else 'r'}_x100"
    peak_speed = max((abs(t.get(speed_key, 0)) for t in telemetry), default=0)
    peak_pwm = max((abs(t.get(pwm_key, 0)) for t in telemetry), default=0)
    fault = end.get("fault", 0)
    moved = delta_ticks * direction >= tick_threshold
    if delta_ticks * direction < -tick_threshold:
        raise RuntimeError(
            f"{side} encoder direction reversed: direction={direction}, delta={delta_ticks}"
        )
    return Trial(
        side=side,
        direction=direction,
        duty_percent=duty_percent,
        delta_ticks=delta_ticks,
        peak_speed_mm_s=peak_speed,
        peak_pwm_x100=peak_pwm,
        fault=fault,
        moved=moved,
    )


def duty_values(start: float, stop: float, step: float) -> list[float]:
    values: list[float] = []
    value = start
    while value <= stop + 1e-6:
        values.append(round(value, 3))
        value += step
    return values


def main() -> int:
    parser = argparse.ArgumentParser(description="UART4 wheel breakaway duty scanner")
    parser.add_argument("--port", required=True)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--target", type=float, default=0.02, help="wheel target m/s")
    parser.add_argument("--duration", type=float, default=0.45)
    parser.add_argument("--settle", type=float, default=0.55)
    parser.add_argument("--rate", type=float, default=20.0)
    parser.add_argument("--coarse-start", type=float, default=6.0)
    parser.add_argument("--coarse-stop", type=float, default=24.0)
    parser.add_argument("--coarse-step", type=float, default=2.0)
    parser.add_argument("--fine-step", type=float, default=0.5)
    parser.add_argument("--tick-threshold", type=int, default=4)
    parser.add_argument("--transcript", default="deadband_scan_uart4.log")
    parser.add_argument(
        "--i-understand-motors-will-move",
        action="store_true",
        help="required safety acknowledgement",
    )
    args = parser.parse_args()
    if not args.i_understand_motors_will_move:
        parser.error("pass --i-understand-motors-will-move after securing the chassis")
    if args.target <= 0.0 or args.target * 1000.0 >= 50.0:
        parser.error("target must be positive and below the 50 mm/s stall threshold")

    try:
        import serial  # type: ignore
    except ImportError as exc:
        raise SystemExit("Missing pyserial") from exc

    results: list[Trial] = []
    thresholds: dict[tuple[str, int], float] = {}
    with open(args.transcript, "w", encoding="utf-8", newline="\n") as transcript:
        with serial.Serial(args.port, args.baud, timeout=0.02) as uart:
            uart.reset_input_buffer()
            console = UartConsole(uart, transcript)
            console.send("stream on")
            console.send("pid both 0 0 0")
            console.stop_and_settle(args.settle)
            try:
                print(
                    "side,direction,duty_percent,delta_ticks,peak_speed_mm_s,"
                    "peak_pwm_percent,fault,moved",
                    flush=True,
                )
                for side in ("left", "right"):
                    for direction in (1, -1):
                        coarse_hit: float | None = None
                        for duty in duty_values(
                            args.coarse_start, args.coarse_stop, args.coarse_step
                        ):
                            trial = run_trial(
                                console,
                                side,
                                direction,
                                duty,
                                args.target,
                                args.duration,
                                args.settle,
                                args.rate,
                                args.tick_threshold,
                            )
                            results.append(trial)
                            print(
                                f"{side},{direction:+d},{duty:.1f},{trial.delta_ticks},"
                                f"{trial.peak_speed_mm_s},{trial.peak_pwm_x100 / 100.0:.2f},"
                                f"0x{trial.fault:04X},{int(trial.moved)}",
                                flush=True,
                            )
                            if trial.moved:
                                coarse_hit = duty
                                break
                        if coarse_hit is None:
                            raise RuntimeError(
                                f"{side} direction {direction:+d} did not move by "
                                f"{args.coarse_stop:.1f}%"
                            )

                        fine_start = max(args.coarse_start, coarse_hit - args.coarse_step)
                        fine_hit = coarse_hit
                        for duty in duty_values(fine_start, coarse_hit, args.fine_step):
                            trial = run_trial(
                                console,
                                side,
                                direction,
                                duty,
                                args.target,
                                args.duration,
                                args.settle,
                                args.rate,
                                args.tick_threshold,
                            )
                            results.append(trial)
                            print(
                                f"{side},{direction:+d},{duty:.1f},{trial.delta_ticks},"
                                f"{trial.peak_speed_mm_s},{trial.peak_pwm_x100 / 100.0:.2f},"
                                f"0x{trial.fault:04X},{int(trial.moved)}",
                                flush=True,
                            )
                            if trial.moved:
                                fine_hit = duty
                                break
                        thresholds[(side, direction)] = fine_hit
            finally:
                console.send("stop")
                console.send("pid both 0.05 0.001 0")
                console.read_for(0.4)

    print("SUMMARY")
    for side in ("left", "right"):
        for direction in (1, -1):
            label = "forward" if direction > 0 else "reverse"
            print(f"{side}_{label}_breakaway_percent={thresholds[(side, direction)]:.1f}")
    recommended = max(thresholds.values()) + 2.0
    print(f"recommended_minimum_percent={recommended:.1f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
