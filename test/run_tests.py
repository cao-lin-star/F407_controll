#!/usr/bin/env python3
"""无第三方依赖的主机仿真测试；任何装有 Python 3 的电脑均可运行。"""

from __future__ import annotations

import math
import pathlib
import re
import struct
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from rk3576_uart_example import (  # noqa: E402
    MSG_CMD_VEL,
    MSG_HEARTBEAT,
    MSG_ODOM,
    MSG_STOP,
    StreamParser,
    build_cmd_vel,
    build_frame,
    build_stop,
    crc16_ccitt_false,
    decode_frame,
)


def test_crc() -> None:
    assert crc16_ccitt_false(b"123456789") == 0x29B1


def test_fragment_sticky_and_noise() -> None:
    cmd = build_cmd_vel(0x1234, 0.25, -0.5)
    stop = build_stop(0x1235)
    parser = StreamParser()
    frames = []
    frames.extend(parser.feed(b"noise\xAA"))
    frames.extend(parser.feed(b"\x7E" + cmd[:3]))
    frames.extend(parser.feed(cmd[3:9]))
    frames.extend(parser.feed(cmd[9:] + stop))  # 剩余拆包 + 粘包
    assert [frame.msg_type for frame in frames] == [MSG_CMD_VEL, MSG_STOP]
    assert frames[0].seq == 0x1234
    assert struct.unpack("<ff", frames[0].payload) == (0.25, -0.5)


def test_crc_length_version_resync() -> None:
    good = build_stop(8)
    bad_crc = good[:-1] + bytes([good[-1] ^ 0x55])
    bad_length = b"\xAA\x55\x01\x01\x00\x00\xFF\x7F"
    bad_version = b"\xAA\x55\x02\x04\x00\x00\x00\x00"
    parser = StreamParser()
    frames = list(parser.feed(bad_crc + bad_length + bad_version + good))
    assert len(frames) == 1 and frames[0].msg_type == MSG_STOP
    assert parser.crc_errors == 1
    assert parser.length_errors == 1
    assert parser.version_errors == 1


def test_payload_layouts() -> None:
    odom_payload = struct.pack("<fffffii", 1.0, -2.0, 0.5, 0.1, -0.2, 123, -456)
    heartbeat_payload = struct.pack("<IH", 42_000, 0x0123)
    parser = StreamParser()
    frames = list(
        parser.feed(
            build_frame(MSG_ODOM, 1, odom_payload)
            + build_frame(MSG_HEARTBEAT, 2, heartbeat_payload)
        )
    )
    odom = decode_frame(frames[0])
    heartbeat = decode_frame(frames[1])
    assert odom["left_ticks"] == 123 and odom["right_ticks"] == -456
    assert heartbeat == {"uptime_ms": 42_000, "fault_flags": 0x0123}


def test_kinematics_and_old_speed_equivalence() -> None:
    track = 0.330
    linear, angular = 1.0, 1.0
    left = linear - angular * track / 2.0
    right = linear + angular * track / 2.0
    assert math.isclose(left, 0.835)
    assert math.isclose(right, 1.165)
    assert math.isclose((left + right) / 2.0, linear)
    assert math.isclose((right - left) / track, angular)

    # 旧 Kinematics.cpp：ticks * 105.805 / dt_ms，单位是 mm/s。
    ticks, dt_ms = 10, 10
    old_formula_mm_s = ticks * 105.805 / dt_ms
    migrated_si_mm_s = ticks * 0.000105805 * 1000.0 / (dt_ms / 1000.0)
    assert math.isclose(old_formula_mm_s, migrated_si_mm_s, rel_tol=1e-12)


def test_pid_reference_and_anti_windup() -> None:
    kp, ki, integral_limit = 0.625, 0.125, 2500.0
    target, measurement = 100.0, 0.0
    integral = 0.0
    previous_error = 0.0
    outputs = []
    for _ in range(100):
        error = target - measurement
        candidate = max(-integral_limit, min(integral_limit, integral + error))
        raw = kp * error + ki * candidate
        if not ((raw > 100.0 and error > 0.0) or (raw < -100.0 and error < 0.0)):
            integral = candidate
        outputs.append(max(-100.0, min(100.0, kp * error + ki * integral)))
        previous_error = error
    assert outputs[0] == 75.0
    assert max(outputs) == 100.0
    assert integral < integral_limit
    assert previous_error == 100.0


def _read_float_macro(name: str) -> float:
    config = (ROOT / "include" / "board_config.h").read_text(encoding="utf-8")
    match = re.search(
        rf"^\s*#define\s+{re.escape(name)}\s+([0-9]+(?:\.[0-9]+)?)f\b",
        config,
        re.MULTILINE,
    )
    assert match is not None, f"missing float macro: {name}"
    return float(match.group(1))


def _read_uint_macro(name: str) -> int:
    config = (ROOT / "include" / "board_config.h").read_text(encoding="utf-8")
    match = re.search(
        rf"^\s*#define\s+{re.escape(name)}\s+([0-9]+)U\b",
        config,
        re.MULTILINE,
    )
    assert match is not None, f"missing unsigned macro: {name}"
    return int(match.group(1))


def test_speed_limit_configuration() -> None:
    ps2_limit = _read_float_macro("PS2_MANUAL_MAX_LINEAR_MPS")
    assert math.isclose(ps2_limit, 0.3, abs_tol=1e-9)

    names = (
        "MAX_LINEAR_MPS",
        "MAX_WHEEL_MPS",
    )
    values = [_read_float_macro(name) for name in names]
    assert all(math.isclose(value, 1.0, abs_tol=1e-9) for value in values)


def test_obstacle_interlock_configuration() -> None:
    assert _read_uint_macro("FRONT_OBSTACLE_SAFETY_ENABLE") == 1
    assert _read_uint_macro("IR_OBSTACLE_SAFETY_ENABLE") == 0
    assert _read_uint_macro("CLIFF_SAFETY_ENABLE") == 1
    assert math.isclose(_read_float_macro("CLIFF_LEFT_GROUND_BASELINE_M"), 0.155)
    assert math.isclose(_read_float_macro("CLIFF_RIGHT_GROUND_BASELINE_M"), 0.160)

    source = (
        ROOT / "F407" / "App" / "Function" / "Src" / "sensor_hub.c"
    ).read_text(encoding="utf-8")
    ultrasonic_mask = re.search(
        r"#if FRONT_OBSTACLE_SAFETY_ENABLE(?P<body>.*?)#endif",
        source,
        re.DOTALL,
    )
    ir_mask = re.search(
        r"#if IR_OBSTACLE_SAFETY_ENABLE(?P<body>.*?)#endif",
        source,
        re.DOTALL,
    )
    assert ultrasonic_mask is not None
    assert ir_mask is not None
    assert "OBSTACLE_ULTRASONIC" in ultrasonic_mask.group("body")
    assert "OBSTACLE_IR_LEFT" not in ultrasonic_mask.group("body")
    assert "OBSTACLE_IR_RIGHT" not in ultrasonic_mask.group("body")
    assert "OBSTACLE_IR_LEFT" in ir_mask.group("body")
    assert "OBSTACLE_IR_RIGHT" in ir_mask.group("body")
    assert "OBSTACLE_ULTRASONIC" not in ir_mask.group("body")


def test_loaded_pid_tof_and_directional_safety_contract() -> None:
    for name in ("LEFT_PID_KP", "RIGHT_PID_KP"):
        assert math.isclose(_read_float_macro(name), 0.04)
    for name in ("LEFT_PID_KI", "RIGHT_PID_KI"):
        assert math.isclose(_read_float_macro(name), 0.002)
    for name in ("LEFT_PID_KD", "RIGHT_PID_KD"):
        assert math.isclose(_read_float_macro(name), 0.0)
    assert _read_uint_macro("TOFSENSE_RX_RING_SIZE") == 1024

    sensor_source = (
        ROOT / "F407" / "App" / "Function" / "Src" / "sensor_hub.c"
    ).read_text(encoding="utf-8")
    callback = re.search(
        r"void sensor_hub_on_uart_rx_complete.*?\n}\n",
        sensor_source,
        re.DOTALL,
    )
    process = re.search(
        r"void sensor_hub_process.*?\n}\n",
        sensor_source,
        re.DOTALL,
    )
    assert callback is not None
    assert process is not None
    assert "tofsense_feed" not in callback.group(0)
    assert "tofsense_rx_push" in callback.group(0)
    assert "tofsense_process_rx" in process.group(0)
    accept_frame = re.search(
        r"static void tofsense_accept_frame.*?\n}\n",
        sensor_source,
        re.DOTALL,
    )
    assert accept_frame is not None
    insufficient_zones = re.search(
        r"if \(valid_count < minimum_valid\).*?\n    }",
        accept_frame.group(0),
        re.DOTALL,
    )
    assert insufficient_zones is not None
    assert "snapshot.valid_flags &= " not in insufficient_zones.group(0)
    assert "SENSOR_DATA_TIMEOUT_MS" in process.group(0)
    assert "const uint32_t tof_now_ms = HAL_GetTick();" in process.group(0)
    assert "tof_now_ms - tof_left.last_update_ms" in process.group(0)
    assert "tof_now_ms - tof_right.last_update_ms" in process.group(0)


    app_source = (
        ROOT / "F407" / "App" / "Business" / "Src" / "chassis_app.c"
    ).read_text(encoding="utf-8")
    assert "forward_motion_requested" in app_source
    assert (
        "forward_motion_requested && sensor_hub_obstacle_stop_required()"
        in app_source
    )


def test_ultrasonic_out_of_range_and_isr_contract() -> None:
    source = (
        ROOT / "F407" / "App" / "Function" / "Src" / "sensor_hub.c"
    ).read_text(encoding="utf-8")
    callback = re.search(
        r"void sensor_hub_on_tim_ic_capture.*?\n}\n",
        source,
        re.DOTALL,
    )
    process = re.search(
        r"static void ultrasonic_process_measurement.*?\n}\n",
        source,
        re.DOTALL,
    )
    start = re.search(
        r"static void ultrasonic_start_measurement.*?\n}\n",
        source,
        re.DOTALL,
    )
    publish_clear = re.search(
        r"static void ultrasonic_publish_clear.*?\n}\n",
        source,
        re.DOTALL,
    )

    assert callback is not None
    assert process is not None
    assert start is not None
    assert publish_clear is not None

    # The capture ISR may only record timer edges/pulse width.
    for forbidden in ("float ", "HAL_GetTick", "snapshot."):
        assert forbidden not in callback.group(0)
    assert "ultrasonic_captured_pulse_us" in callback.group(0)
    assert "ultrasonic_pulse_ready = 1U;" in callback.group(0)

    # No echo and over-range pulses are valid clear readings at max range.
    assert "ULTRASONIC_MAX_DISTANCE_M" in publish_clear.group(0)
    assert "RANGE_VALID_ULTRASONIC" in publish_clear.group(0)
    assert "~SENSOR_FAULT_ULTRASONIC" in publish_clear.group(0)
    assert process.group(0).count("ultrasonic_publish_clear(now_ms);") >= 2

    # A rising edge that stays high remains a genuine hardware/signal fault.
    assert "if (saw_rising_edge != 0U)" in process.group(0)
    assert "ultrasonic_publish_fault();" in process.group(0)

    # TRIG is bounded to microseconds in task context, not the 2 ms task period.
    assert "< 12U" in start.group(0)
    assert "HAL_GPIO_WritePin" in start.group(0)

def test_recoverable_fault_contract() -> None:
    assert _read_uint_macro("ENCODER_RECOVERY_STABLE_MS") == 500
    assert _read_uint_macro("ENCODER_RECOVERY_MAX_DELTA_PER_PERIOD") == 2
    assert _read_uint_macro("TRANSIENT_FAULT_CLEAR_MS") == 1000

    control_header = (ROOT / "include" / "control.h").read_text(encoding="utf-8")
    control_source = (ROOT / "src" / "control.c").read_text(encoding="utf-8")
    app_source = (
        ROOT / "F407" / "App" / "Business" / "Src" / "chassis_app.c"
    ).read_text(encoding="utf-8")
    debug_source = (
        ROOT / "F407" / "App" / "Function" / "Src" / "debug_service.c"
    ).read_text(encoding="utf-8")

    for source in (control_header, control_source):
        assert "encoder_fault_latched" not in source
    assert "motion_recovery_required" in control_header
    assert control_source.count("if (!control->recovery_neutral_seen)") >= 3
    assert "control_request_fault_recovery" in control_header
    assert "ENCODER_RECOVERY_STABLE_MS" in control_source
    assert "control_raise_stop_fault(control, FAULT_LEFT_ENCODER)" in control_source
    assert "control_raise_stop_fault(control, FAULT_RIGHT_ENCODER)" in control_source
    assert "control_clear_fault(&chassis, FAULT_ESTOP)" in app_source
    assert "control_clear_fault(&chassis, FAULT_OBSTACLE)" in app_source
    assert "update_transient_fault_recovery(now_ms)" in app_source
    assert "control_request_fault_recovery(&chassis)" in app_source
    assert "control_request_fault_recovery(chassis)" in debug_source


def test_control_source_priority_and_ramp_contract() -> None:
    assert math.isclose(_read_float_macro("WHEEL_ACCEL_LIMIT_MPS2"), 0.30)
    assert math.isclose(_read_float_macro("WHEEL_DECEL_LIMIT_MPS2"), 0.50)

    config = (ROOT / "include" / "board_config.h").read_text(encoding="utf-8")
    control_header = (ROOT / "include" / "control.h").read_text(encoding="utf-8")
    control_source = (ROOT / "src" / "control.c").read_text(encoding="utf-8")
    ps2_source = (
        ROOT / "F407" / "App" / "Function" / "Src" / "ps2_remote.c"
    ).read_text(encoding="utf-8")
    app_source = (
        ROOT / "F407" / "App" / "Business" / "Src" / "chassis_app.c"
    ).read_text(encoding="utf-8")
    debug_source = (
        ROOT / "F407" / "App" / "Function" / "Src" / "debug_service.c"
    ).read_text(encoding="utf-8")

    assert "CONTROL_SOURCE_RK = 1" in control_header
    assert "CONTROL_SOURCE_DEBUG = 2" in control_header
    assert "CONTROL_SOURCE_PS2 = 3" in control_header
    assert "return source >= control->active_source;" in control_source
    assert "slew_towards" in control_source
    assert "requested_left_mm_s" in control_source
    assert "PS2_REMOTE_ACTION_RELEASE" in ps2_source
    assert "status.manual_mode = 0U;" in ps2_source
    assert "status.connected != 0U && status.manual_mode != 0U" in ps2_source
    assert "CONTROL_SOURCE_RK" in app_source
    assert "CONTROL_SOURCE_PS2" in app_source
    assert "control_release_source(&chassis, CONTROL_SOURCE_PS2)" in app_source
    assert "CONTROL_SOURCE_DEBUG" in debug_source
    assert "ps2_remote_manual_active()" not in debug_source
    assert re.search(r"#define\s+MOTOR_STOP_BRAKE\s+0\b", config)
def main() -> int:
    tests = [
        test_crc,
        test_fragment_sticky_and_noise,
        test_crc_length_version_resync,
        test_payload_layouts,
        test_kinematics_and_old_speed_equivalence,
        test_pid_reference_and_anti_windup,
        test_speed_limit_configuration,
        test_obstacle_interlock_configuration,
        test_recoverable_fault_contract,
        test_loaded_pid_tof_and_directional_safety_contract,
        test_ultrasonic_out_of_range_and_isr_contract,
        test_control_source_priority_and_ramp_contract,
    ]
    for test in tests:
        test()
        print(f"PASS {test.__name__}")
    print(f"All {len(tests)} Python host tests passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
