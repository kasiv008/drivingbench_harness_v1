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
    # Speed is low-pass filtered (speed_tau 0.25 s): a sustained overspeed trips within 0.6 s.
    now = 10.0
    while now < 10.6:
        now += 0.03
        joy(sup, now)
        joy_cmd(sup, now)
        reports(sup, now, speed=3.5)
        sup.on_target({"active": True, "speed_mps": 1.0, "angle_deg": 0.0}, now)
    out = sup.tick(now)
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


# Grade trim. A crude cart: throttle above the sustain level accelerates, plus a slope term.
TRIM = {"ff_speeds": [0.5, 1.5], "ff_throttle": [0.325, 0.45], "throttle_start": 0.30,
        "throttle_cap": 0.7, "kp": 0.12, "ki": 0.03, "i_max": 1.0, "breakaway_throttle": 0.36,
        "kp_over": 0.3, "trim_gain": 0.15}


def drive(params, slope_accel, v_ref=0.5, seconds=12.0, loop=None):
    p = dict(Supervisor(params).p)
    loop = loop or SpeedLoop(p)
    v, dt, speeds = 0.0, 0.03, []
    for _ in range(int(seconds / dt)):
        throttle, brake = loop.step(v_ref, v, dt)
        assert not (throttle > 0.0 and brake > 0.0)
        push = 4.6 * (throttle - 0.325) if throttle >= 0.30 else -0.3
        if v <= 0.0 and throttle < 0.35 + max(0.0, -slope_accel) / 4.6:
            push = 0.0  # static friction (and gravity uphill) holds it
        v = max(0.0, v + (push + slope_accel - 3.0 * brake) * dt)
        speeds.append(v)
    return loop, speeds


def test_trim_is_off_by_default():
    loop, _ = drive({k: v for k, v in TRIM.items() if k not in ("trim_gain", "kp_over")}, 0.3)
    assert loop.trim == 0.0


def test_trim_learns_less_throttle_downhill():
    old = dict(TRIM, trim_gain=0.0, kp_over=0.0)
    _, before = drive(old, 0.6)  # steep enough to roll with no throttle at all
    loop, after = drive(dict(TRIM, brake_band_downhill=0.15), 0.6)
    assert loop.trim == -loop.p["trim_max"]
    assert max(before[-100:]) > 0.75  # old loop sits at the edge of the 0.3 brake band
    assert max(after[-100:]) < 0.7


def test_trim_learns_more_throttle_uphill_and_gets_moving():
    old = dict(TRIM, trim_gain=0.0, kp_over=0.0)
    _, before = drive(old, -0.3)
    loop, after = drive(TRIM, -0.3)
    assert before[-1] == 0.0  # the flat-ground breakaway throttle never moves it
    assert 0.0 < loop.trim <= loop.p["trim_max"]
    assert after[-1] > 0.3


def test_trim_stays_bounded_and_survives_a_stop():
    loop, _ = drive(TRIM, -2.0)  # far steeper than the trim can handle
    assert loop.trim == loop.p["trim_max"]
    loop.reset()
    assert loop.trim == loop.p["trim_max"]
    loop.step(0.5, 1.5, 0.03)  # clearly too fast: uphill bias dropped at once
    assert loop.trim == 0.0


def test_trim_does_not_wind_up_during_a_normal_launch():
    loop = SpeedLoop(dict(Supervisor(TRIM).p))
    for _ in range(int(1.4 / 0.03)):  # the launch delay seen on the cart
        loop.step(0.5, 0.0, 0.03)
    assert loop.trim == 0.0


def test_log_row_has_every_field_and_reflects_the_tick():
    sup = ready({"trim_gain": 0.15})
    now = 10.0
    sup.on_target({"active": True, "speed_mps": 0.6, "angle_deg": 90.0, "epoch": 3}, now)
    out = sup.tick(now)
    row = sup.log_row(now, out, wall_time=123.0)
    assert tuple(row) == Supervisor.LOG_FIELDS
    assert row["source"] == "llm" and row["reason"] == "executing"
    assert row["target_active"] is True and row["target_speed_mps"] == 0.6
    assert row["target_epoch"] == 3 and row["target_age_s"] == 0.0
    assert row["throttle_cmd"] == out["throttle"] and row["brake_cmd"] == 0.0
    assert abs(row["steer_cmd_deg"] - 90.0) < 1e-9
    assert row["speed_mps"] == 0.0 and row["deadman"] is True and row["dt"] is None
    assert sup.log_row(now + 0.03, sup.tick(now + 0.03))["dt"] > 0

    idle = Supervisor()
    row = idle.log_row(0.0, idle.tick(0.0))  # nothing received yet: still loggable
    assert row["target_active"] is None and row["speed_mps"] is None
    assert row["steer_cmd_deg"] is None or isinstance(row["steer_cmd_deg"], float)
