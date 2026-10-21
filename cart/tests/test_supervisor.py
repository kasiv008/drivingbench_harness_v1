"""Supervisor logic without ROS: authority, mux, handover, speed loop, emergencies."""

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "ros"))

from cart_supervisor import SpeedLoop, Supervisor  # noqa: E402

AXES_IDLE = [0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0]


def joy(sup, now, deadman=True, lt=1.0, rt=1.0, stick=0.0):
    axes = list(AXES_IDLE)
    axes[5], axes[4], axes[2] = lt, rt, stick
    buttons = [0] * 12
    buttons[2] = int(deadman)
    sup.on_joy(axes, buttons, now)


def reports(sup, now, speed=0.0, steer=0.0, gear=3, enabled=True):
    sup.on_report("speed", now, speed)
    sup.on_report("steer", now, steer, True, False)
    sup.on_report("accel", now, 0.0, True, False)
    sup.on_report("brake", now, 0.0, True, False)
    sup.on_report("shift", now, gear, True, False)
    sup.on_report("enabled", now, enabled)


def joy_cmd(sup, now, throttle=0.0, brake=0.0, steer=0.0):
    sup.on_joy_cmd("throttle", throttle, now)
    sup.on_joy_cmd("brake", brake, now)
    sup.on_joy_cmd("steer_rad", steer, now)


def ready(params=None, now=10.0):
    sup = Supervisor(dict({"ff_speeds": [0.5, 3.0], "ff_throttle": [0.2, 0.4]}, **(params or {})))
    joy(sup, now)
    reports(sup, now)
    joy_cmd(sup, now)
    return sup


def test_authorized_when_everything_ready():
    sup = ready()
    assert sup.authorization(10.0) == (True, "authorized")
    assert sup.bridge_state(10.0)["authorized"]


def test_untouched_triggers_block_authority():
    sup = Supervisor()
    sup.on_joy([0.0] * 8, [0, 0, 1] + [0] * 9, 1.0)  # triggers read 0 before first touch
    assert sup.authorization(1.0) == (False, "triggers_untouched")


def test_each_override_revokes_authority():
    for kwargs, reason in [
        ({"deadman": False}, "deadman_released"),
        ({"lt": 0.5}, "operator_brake"),
        ({"rt": 0.5}, "operator_throttle"),
        ({"stick": 0.5}, "operator_steer"),
    ]:
        sup = ready()
        joy(sup, 10.0, **kwargs)
        assert sup.authorization(10.0) == (False, reason), kwargs


def test_stale_inputs_revoke_authority():
    sup = ready()
    assert sup.authorization(10.3) == (False, "joy_stale")
    sup = ready()
    joy(sup, 10.4)
    assert sup.authorization(10.4) == (False, "reports_stale")
    sup = ready()
    reports(sup, 10.0, enabled=False)
    assert sup.authorization(10.0) == (False, "pacmod_disabled")


def test_llm_target_drives_and_steers_left_positive():
    sup = ready()
    sup.on_target({"active": True, "speed_mps": 1.0, "angle_deg": 90.0}, 10.0)
    out = None
    for i in range(20):
        now = 10.0 + 0.03 * i
        joy(sup, now)
        reports(sup, now)
        sup.on_target({"active": True, "speed_mps": 1.0, "angle_deg": 90.0}, now)
        out = sup.tick(now)
    assert out["source"] == "llm" and out["brake"] == 0.0 and 0.0 < out["throttle"] <= 0.5
    assert math.isclose(out["steer_rad"], math.pi / 2)


def test_inactive_target_holds_brake():
    sup = ready()
    sup.on_target({"active": False, "speed_mps": 0.0, "angle_deg": 0.0}, 10.0)
    out = sup.tick(10.0)
    assert out["source"] == "llm" and out["throttle"] == 0.0 and out["brake"] == 0.8


def test_stale_bridge_holds_brake():
    sup = ready()
    sup.on_target({"active": True, "speed_mps": 1.0, "angle_deg": 0.0}, 9.0)
    out = sup.tick(10.0)
    assert out["reason"] == "bridge_stale" and out["brake"] == 0.8


def test_release_deadman_brakes_until_operator_acts():
    sup = ready()
    sup.on_target({"active": True, "speed_mps": 1.0, "angle_deg": 30.0}, 10.0)
    sup.tick(10.0)
    joy(sup, 10.03, deadman=False)
    joy_cmd(sup, 10.03)
    out = sup.tick(10.03)
    assert out["source"] == "handover" and out["brake"] == 0.8
    assert math.isclose(out["steer_rad"], math.radians(30.0))  # steering held, not snapped
    joy_cmd(sup, 10.06, brake=0.7)
    out = sup.tick(10.06)
    assert out["source"] == "joy" and out["brake"] == 0.7


def test_joystick_forwarded_when_never_authorized():
    sup = Supervisor()
    joy(sup, 1.0, deadman=False)
    joy_cmd(sup, 1.0)
    assert sup.tick(1.0)["source"] == "joy"


def test_dead_joystick_node_holds_brake():
    sup = Supervisor()
    joy(sup, 1.0, deadman=False)
    out = sup.tick(1.0)
    assert out["source"] == "hold" and out["brake"] == 0.8


def test_overspeed_brakes_hard_in_llm_mode():
    sup = ready()
    reports(sup, 10.0, speed=3.5)
    sup.on_target({"active": True, "speed_mps": 1.0, "angle_deg": 0.0}, 10.0)
    out = sup.tick(10.0)
    assert out["source"] == "emergency" and out["brake"] == 1.1 and out["throttle"] == 0.0


def test_gear_and_sample_fields():
    sup = ready()
    state = sup.bridge_state(10.0)
    assert state["gear_drive"] and state["sensors_valid"] and state["standstill"]
    reports(sup, 10.0, gear=1)
    assert not sup.bridge_state(10.0)["gear_drive"]


def test_speed_loop_never_mixes_pedals():
    p = dict(Supervisor().p, ff_speeds=[0.5, 3.0], ff_throttle=[0.2, 0.4])
    loop = SpeedLoop(p)
    for v in [0.0, 0.5, 1.0, 1.2, 1.5, 2.0]:
        throttle, brake = loop.step(1.0, v, 0.03)
        assert throttle == 0.0 or brake == 0.0
        assert 0.0 <= throttle <= p["throttle_cap"]
        assert brake == 0.0 or p["brake_min"] <= brake <= p["brake_cap"]
    assert loop.step(1.0, 2.0, 0.03)[1] >= p["brake_min"]


def test_speed_loop_slews_throttle_up():
    p = dict(Supervisor().p, ff_speeds=[0.5, 3.0], ff_throttle=[0.4, 0.4])
    loop = SpeedLoop(p)
    first, _ = loop.step(1.0, 0.0, 0.03)
    assert first <= p["throttle_slew"] * 0.03 + 1e-9


def test_calib_mode_ramps_to_fixed_throttle():
    sup = ready({"mode": "calib", "calib_throttle": 0.3})
    out = None
    for i in range(40):
        now = 10.0 + 0.03 * i
        joy(sup, now)
        reports(sup, now)
        out = sup.tick(now)
    assert out["source"] == "calib" and math.isclose(out["throttle"], 0.3)
