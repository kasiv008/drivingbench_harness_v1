#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Cart supervisor: joystick/LLM command mux, speed loop, and the state link to the harness.

Runs in the ROS 2 (system) Python. The Supervisor class has no ROS imports, so it is unit
tested without ROS.

Data flow
    /joy, PACMod reports  ->  Supervisor  ->  /pacmod/as_rx/{accel,brake,steer}_cmd
    /joy_cmd/* (joystick_demo in its own namespace)  ->  forwarded when the LLM is not authorized
    UDP state  ->  cart_bridge.py (harness Controller)  ->  UDP target

The LLM only drives while the operator holds the deadman button, both triggers are released,
the steering stick is centered, PACMod is enabled and all required reports are fresh.
Releasing the deadman, or touching a trigger or the stick, ends LLM authority immediately and
holds the brake until the operator acts on the joystick.
"""
import csv
import json
import math
import os
import socket
import threading
import time

INF = float("inf")


def clip(x, lo, hi):
    return max(lo, min(hi, x))


def interp(x, xs, ys):
    """Piecewise linear, clamped at the ends."""
    if not xs:
        return 0.0
    if x <= xs[0]:
        return ys[0]
    for i in range(1, len(xs)):
        if x <= xs[i]:
            a = (x - xs[i - 1]) / max(xs[i] - xs[i - 1], 1e-9)
            return ys[i - 1] + a * (ys[i] - ys[i - 1])
    return ys[-1]


def get_field(msg, path):
    for part in path.split("."):
        msg = getattr(msg, part)
    return msg


DEFAULTS = {
    # Mode: "llm" drives from the harness, "calib" applies a fixed throttle while the deadman is
    # held (for measuring the feedforward table). Steering is held in calib mode.
    "mode": "llm",
    "calib_throttle": 0.0,
    # Optional kick: apply calib_kick_throttle until speed reaches calib_kick_speed, then drop to
    # calib_throttle. Measures the sustain throttle below breakaway. 0 disables the kick.
    "calib_kick_throttle": 0.0,
    "calib_kick_speed": 0.3,
    # Controller mapping (same indices as joystick_demo)
    "deadman_button": 2,
    "throttle_axis": 4,
    "brake_axis": 5,
    "steer_axis": 2,
    "trigger_deadband": 0.05,
    "stick_override": 0.2,
    # Freshness limits [s]
    "joy_timeout": 0.25,
    "joy_cmd_timeout": 0.25,
    "report_timeout": 0.3,
    "target_timeout": 0.2,
    # Brake (this cart engages from 0.5, full at 1.1)
    "brake_min": 0.5,
    "brake_cap": 1.1,
    "hold_brake": 0.8,
    "stop_brake": 0.8,
    "kb": 0.5,
    # Throttle and speed loop
    "throttle_cap": 0.5,
    "throttle_slew": 0.5,
    "kp": 0.15,
    "ki": 0.05,
    "i_max": 2.0,
    "brake_band": 0.3,
    "i_band": 0.2,
    "throttle_start": 0.0,
    "breakaway_throttle": 0.0,
    "breakaway_speed": 0.15,
    "ff_speeds": [0.5, 3.0],
    "ff_throttle": [0.0, 0.0],
    "max_speed_mps": 3.0,
    "standstill_speed": 0.05,
    "speed_tau": 0.25,         # speed low-pass time constant [s]
    # Grade trim: a slow throttle bias learned from sustained speed error, so slopes (and tyre
    # scrub at full lock) do not leave the cart well above or below target. 0 gain = off.
    "kp_over": 0.0,            # proportional gain while above target. 0 = use kp
    "trim_gain": 0.0,          # throttle units per second per m/s of error
    "trim_max": 0.08,          # bias limit [throttle units], about +-0.37 m/s^2 on this cart
    "trim_band": 0.1,          # ignore errors smaller than this [m/s]
    "trim_accel": 0.1,         # learn only while not already converging faster than this [m/s^2]
    "trim_accel_tau": 0.5,     # low-pass for the acceleration estimate [s]
    "trim_stall_s": 2.0,       # below breakaway_speed this long with a target: add throttle
    "brake_band_downhill": 0.0,  # tighter brake band once the trim is at its lower limit. 0 = off
    # Per-tick CSV log (about 33 Hz, ~25 MB per hour). Empty log_dir = off.
    "log_dir": "~/drivingbench-cart/logs",
    # Steering. Harness angles are left-positive steering-wheel degrees.
    "steer_sign": 1.0,
    "max_steer_rad": 5.5,
    "steer_vel_llm": 1.5,
    # Gear report value that means forward/drive
    "gear_forward_value": 3,
    # Empty: derive PACMod enabled from the steer, accel and brake reports
    "use_enabled_topic": True,
    # Trust each report's own enabled flag for steering/longitudinal activity
    "use_system_enabled": True,
}


class Trigger(object):
    """Axis is 1 when released and -1 at full press. Linux reports 0 until first touched."""

    def __init__(self, deadband):
        self.deadband = deadband
        self.valid = False
        self.value = 0.0

    def update(self, axis):
        if axis != 0.0:
            self.valid = True
        self.value = clip(0.5 - 0.5 * axis, 0.0, 1.0) if self.valid else 0.0

    @property
    def pressed(self):
        return self.valid and self.value > self.deadband


class SpeedLoop(object):
    """Speed target to pedals. Throttle and brake are never both nonzero."""

    def __init__(self, p):
        self.p = p
        self.trim = 0.0  # survives reset(): the slope is still there after a stop
        self.demand = 0.0  # throttle the loop wanted before the slew limit (for the log)
        self.reset()

    def reset(self):
        self.integ = 0.0
        self.throttle = 0.0
        self.v_prev = None
        self.accel = 0.0
        self.stall_t = 0.0

    def _learn_trim(self, v_ref, v, err, dt):
        """Slow bias for what the feedforward table cannot know: slope, load, tyre scrub."""
        p = self.p
        if self.v_prev is not None and dt > 0.0:
            alpha = dt / (p["trim_accel_tau"] + dt)
            self.accel += alpha * ((v - self.v_prev) / dt - self.accel)
        self.v_prev = v
        if v < p["breakaway_speed"]:
            # Not rolling. The launch delay looks the same as a stall, so wait before adding.
            self.stall_t += dt
            if self.stall_t > p["trim_stall_s"]:
                self.trim = max(self.trim, 0.0) + p["trim_gain"] * v_ref * dt
        else:
            self.stall_t = 0.0
            slow = err > p["trim_band"] and self.accel < p["trim_accel"]
            fast = err < -p["trim_band"] and self.accel > -p["trim_accel"]
            if slow or fast:
                self.trim += p["trim_gain"] * err * dt
        self.trim = clip(self.trim, -p["trim_max"], p["trim_max"])

    def step(self, v_ref, v, dt):
        p = self.p
        err = v_ref - v
        band = p["brake_band"]
        if p["brake_band_downhill"] > 0.0 and self.trim <= -p["trim_max"]:
            band = min(band, p["brake_band_downhill"])  # throttle alone cannot hold this slope
        if err < -band:
            # Clearly too fast: brake proportionally inside the effective band.
            self.reset()
            self.trim = min(self.trim, 0.0)  # any uphill bias is wrong here
            brake = clip(p["brake_min"] + p["kb"] * (-err - band),
                         p["brake_min"], p["brake_cap"])
            return 0.0, brake
        # Integrate only near the target. Integrating the large start-up error winds up and
        # overshoots on this cart, whose throttle sets acceleration rather than speed.
        if abs(err) < p["i_band"]:
            self.integ = clip(self.integ + err * dt, -p["i_max"], p["i_max"])
        ff = interp(v_ref, p["ff_speeds"], p["ff_throttle"])
        kp = p["kp_over"] if err < 0.0 and p["kp_over"] > 0.0 else p["kp"]
        if p["trim_gain"] > 0.0 and v_ref > 0.0:
            self._learn_trim(v_ref, v, err, dt)
        else:
            self.trim = 0.0
        target = ff + kp * err + p["ki"] * self.integ + self.trim
        # Static friction: below breakaway_speed the cart needs more than the sustain throttle.
        if p["breakaway_throttle"] > 0.0 and v < p["breakaway_speed"] and v_ref > 0.0:
            target = max(target, p["breakaway_throttle"] + max(self.trim, 0.0))
        target = clip(target, 0.0, p["throttle_cap"])
        self.demand = target
        # Throttle below throttle_start does nothing on this cart: skip that range instantly.
        if self.throttle < p["throttle_start"] < target:
            self.throttle = p["throttle_start"]
        # Rate-limit increases only. Decreases apply immediately.
        self.throttle = min(target, self.throttle + p["throttle_slew"] * dt)
        return self.throttle, 0.0


class Report(object):
    def __init__(self):
        self.t = -INF
        self.value = None
        self.enabled = False
        self.override = False
        self.has_flags = False  # older pacmod_msgs reports carry no enabled/override fields

    def fresh(self, now, timeout):
        return now - self.t < timeout


class Supervisor(object):
    REPORTS = ("speed", "steer", "accel", "brake", "shift", "enabled")

    def __init__(self, params=None):
        p = dict(DEFAULTS)
        p.update(params or {})
        self.p = p
        self.lt = Trigger(p["trigger_deadband"])
        self.rt = Trigger(p["trigger_deadband"])
        self.joy_t = -INF
        self.deadman = False
        self.stick = 0.0
        self.reports = dict((name, Report()) for name in self.REPORTS)
        self.target = None
        self.target_t = -INF
        self.joy_cmd = {"throttle": 0.0, "brake": 0.0, "steer_rad": 0.0}
        self.joy_cmd_t = -INF
        self.loop = SpeedLoop(p)
        self.handover = False
        self.last_steer_rad = None
        self.last_tick = None
        self.last_log_t = None
        self.calib_throttle = 0.0
        self.calib_kicked = False
        self.v_filt = None
        self.v_raw = None

    # ---- inputs -----------------------------------------------------------------------------
    def on_joy(self, axes, buttons, now):
        p = self.p
        needed_axes = max(p["throttle_axis"], p["brake_axis"], p["steer_axis"])
        if len(axes) <= needed_axes or len(buttons) <= p["deadman_button"]:
            return  # Wrong layout: leave joy_t stale so authority lapses.
        self.joy_t = now
        self.lt.update(axes[p["brake_axis"]])
        self.rt.update(axes[p["throttle_axis"]])
        self.stick = axes[p["steer_axis"]]
        self.deadman = bool(buttons[p["deadman_button"]])

    def on_report(self, name, now, value=None, enabled=None, override=None):
        r = self.reports[name]
        if name == "speed" and value is not None:
            # Low-pass the speed report. The raw value can alternate by +-0.5 m/s between
            # samples at ~1 m/s, which is physically impossible and made the loop chatter.
            value = float(value)
            if self.v_filt is None or not r.fresh(now, self.p["report_timeout"]):
                self.v_filt = value
            else:
                alpha = 1.0 - math.exp(-max(now - r.t, 0.0) / self.p["speed_tau"])
                self.v_filt += alpha * (value - self.v_filt)
            self.v_raw = value
        r.t = now
        r.value = value
        if enabled is not None:
            r.has_flags = True
            r.enabled = bool(enabled)
        if override is not None:
            r.override = bool(override)

    def on_target(self, target, now):
        self.target = target
        self.target_t = now

    def on_joy_cmd(self, kind, value, now):
        self.joy_cmd[kind] = value
        self.joy_cmd_t = now

    # ---- derived state ----------------------------------------------------------------------
    def _fresh(self, name, now):
        return self.reports[name].fresh(now, self.p["report_timeout"])

    def speed(self, now):
        """Filtered speed: used for control, overspeed and the harness."""
        r = self.reports["speed"]
        if not self._fresh("speed", now) or r.value is None:
            return None
        return self.v_filt

    def speed_raw(self, now):
        if not self._fresh("speed", now) or self.v_raw is None:
            return None
        return self.v_raw

    def steer_measured_rad(self, now):
        r = self.reports["steer"]
        if not self._fresh("steer", now) or r.value is None:
            return None
        return float(r.value)

    def reports_fresh(self, now):
        names = ["speed", "steer", "accel", "brake", "shift"]
        if self.p["use_enabled_topic"]:
            names.append("enabled")
        return all(self._fresh(n, now) for n in names)

    def pacmod_enabled(self, now):
        if self.p["use_enabled_topic"]:
            r = self.reports["enabled"]
            return self._fresh("enabled", now) and bool(r.value)
        return all(self._fresh(n, now) and self.reports[n].enabled
                   for n in ("steer", "accel", "brake"))

    def system_enabled(self, name, now):
        """Per-system enabled flag, or PACMod's global enable when the report has none or the
        flags are unreliable (this cart: steer_rpt.enabled stays false while PACMod drives)."""
        r = self.reports[name]
        if r.has_flags and self.p["use_system_enabled"]:
            return r.enabled
        return self.pacmod_enabled(now)

    def gear_forward(self, now):
        r = self.reports["shift"]
        return (self._fresh("shift", now) and r.value is not None
                and int(r.value) == int(self.p["gear_forward_value"]))

    def operator_override(self):
        return {
            "brake": self.lt.pressed or self.reports["brake"].override,
            "gas": self.rt.pressed or self.reports["accel"].override,
            "steer": abs(self.stick) > self.p["stick_override"] or self.reports["steer"].override,
        }

    def authorization(self, now):
        """(authorized, reason). Order matters: the first failing check is reported."""
        p = self.p
        if now - self.joy_t >= p["joy_timeout"]:
            return False, "joy_stale"
        if not (self.lt.valid and self.rt.valid):
            return False, "triggers_untouched"
        if not self.deadman:
            return False, "deadman_released"
        ov = self.operator_override()
        if ov["brake"]:
            return False, "operator_brake"
        if ov["gas"]:
            return False, "operator_throttle"
        if ov["steer"]:
            return False, "operator_steer"
        if not self.reports_fresh(now):
            return False, "reports_stale"
        if not self.pacmod_enabled(now):
            return False, "pacmod_disabled"
        return True, "authorized"

    def bridge_state(self, now):
        """Everything cart_bridge.py needs to build a harness Sample."""
        auth, reason = self.authorization(now)
        v = self.speed(now)
        steer = self.steer_measured_rad(now)
        ov = self.operator_override()
        fresh = self.reports_fresh(now)
        return {
            "t": time.time(),
            "speed_mps": v if v is not None else 0.0,
            "steering_deg": (math.degrees(steer * self.p["steer_sign"])
                             if steer is not None else 0.0),
            "gear_drive": self.gear_forward(now),
            "brake_pressed": ov["brake"],
            "gas_pressed": ov["gas"],
            "steering_pressed": ov["steer"],
            "authorized": auth,
            "auth_reason": reason,
            "lateral_active": auth and self.system_enabled("steer", now),
            "longitudinal_active": (auth and self.system_enabled("accel", now)
                                    and self.system_enabled("brake", now)),
            "sensors_valid": fresh and v is not None and steer is not None,
            "car_state_valid": v is not None,
            "standstill": v is not None and abs(v) < self.p["standstill_speed"],
        }

    # ---- output -----------------------------------------------------------------------------
    LOG_FIELDS = (
        "wall_time", "t", "dt", "source", "reason", "auth_reason",
        "target_active", "target_speed_mps", "target_angle_deg", "target_epoch", "target_age_s",
        "speed_mps", "speed_raw_mps", "accel_est_mps2",
        "throttle_cmd", "brake_cmd", "steer_cmd_deg", "steer_vel_cmd",
        "throttle_rpt", "brake_rpt", "steer_rpt_deg", "gear_rpt",
        "loop_demand", "loop_integ", "loop_trim", "loop_stall_s",
        "deadman", "lt", "rt", "stick",
        "override_brake", "override_gas", "override_steer", "pacmod_enabled", "handover",
    )

    def log_row(self, now, out, wall_time=None):
        """One flat record of what the supervisor saw, decided and commanded this tick."""
        p = self.p
        target = self.target or {}
        ov = self.operator_override()
        steer = self.steer_measured_rad(now)
        steer_cmd = out.get("steer_rad")

        def report(name):
            value = self.reports[name].value
            return value if self._fresh(name, now) else None

        row = {
            "wall_time": time.time() if wall_time is None else wall_time,
            "t": now,
            "dt": None if self.last_log_t is None else now - self.last_log_t,
            "source": out["source"],
            "reason": out["reason"],
            "auth_reason": self.authorization(now)[1],
            "target_active": bool(target.get("active")) if target else None,
            "target_speed_mps": target.get("speed_mps"),
            "target_angle_deg": target.get("angle_deg"),
            "target_epoch": target.get("epoch"),
            "target_age_s": now - self.target_t if self.target is not None else None,
            "speed_mps": self.speed(now),
            "speed_raw_mps": self.speed_raw(now),
            "accel_est_mps2": self.loop.accel,
            "throttle_cmd": out["throttle"],
            "brake_cmd": out["brake"],
            "steer_cmd_deg": (None if steer_cmd is None
                              else math.degrees(steer_cmd * p["steer_sign"])),
            "steer_vel_cmd": out.get("steer_vel"),
            "throttle_rpt": report("accel"),
            "brake_rpt": report("brake"),
            "steer_rpt_deg": None if steer is None else math.degrees(steer * p["steer_sign"]),
            "gear_rpt": report("shift"),
            "loop_demand": self.loop.demand,
            "loop_integ": self.loop.integ,
            "loop_trim": self.loop.trim,
            "loop_stall_s": self.loop.stall_t,
            "deadman": self.deadman,
            "lt": self.lt.value,
            "rt": self.rt.value,
            "stick": self.stick,
            "override_brake": ov["brake"],
            "override_gas": ov["gas"],
            "override_steer": ov["steer"],
            "pacmod_enabled": self.pacmod_enabled(now),
            "handover": self.handover,
        }
        self.last_log_t = now
        return row

    def _hold_steer(self, now):
        if self.last_steer_rad is not None:
            return self.last_steer_rad
        measured = self.steer_measured_rad(now)
        return measured if measured is not None else 0.0

    def _out(self, source, reason, throttle, brake, steer_rad, steer_vel):
        p = self.p
        if brake > 0.0:
            throttle = 0.0
        if steer_rad is not None:
            steer_rad = clip(steer_rad, -p["max_steer_rad"], p["max_steer_rad"])
            self.last_steer_rad = steer_rad
        return {"source": source, "reason": reason, "throttle": throttle, "brake": brake,
                "steer_rad": steer_rad, "steer_vel": steer_vel}

    def tick(self, now):
        p = self.p
        dt = 0.03 if self.last_tick is None else clip(now - self.last_tick, 0.0, 0.2)
        self.last_tick = now
        auth, reason = self.authorization(now)
        v = self.speed(now)

        if (auth or self.handover) and v is not None and v > p["max_speed_mps"]:
            self.loop.reset()
            return self._out("emergency", "overspeed", 0.0, p["brake_cap"],
                             self._hold_steer(now), p["steer_vel_llm"])

        if auth:
            self.handover = True
            if p["mode"] == "calib":
                kick = p["calib_kick_throttle"]
                if kick > 0.0 and not self.calib_kicked:
                    if v is not None and v >= p["calib_kick_speed"]:
                        self.calib_kicked = True  # latched: later slowdown is the measurement
                    else:
                        self.calib_throttle = min(kick,
                                                  self.calib_throttle + p["throttle_slew"] * dt)
                        return self._out("calib", "calib_kick", self.calib_throttle, 0.0,
                                         self._hold_steer(now), p["steer_vel_llm"])
                if kick > 0.0:
                    self.calib_throttle = p["calib_throttle"]  # step down immediately
                else:
                    self.calib_throttle = min(p["calib_throttle"],
                                              self.calib_throttle + p["throttle_slew"] * dt)
                return self._out("calib", "calib", self.calib_throttle, 0.0,
                                 self._hold_steer(now), p["steer_vel_llm"])
            target = self.target if now - self.target_t < p["target_timeout"] else None
            if target is not None and target.get("active"):
                throttle, brake = self.loop.step(float(target["speed_mps"]),
                                                 v if v is not None else 0.0, dt)
                steer = math.radians(float(target["angle_deg"])) * p["steer_sign"]
                return self._out("llm", "executing", throttle, brake, steer, p["steer_vel_llm"])
            self.loop.reset()
            moving = v is not None and v > p["standstill_speed"]
            brake = p["stop_brake"] if moving else p["hold_brake"]
            if target is not None:
                steer = math.radians(float(target["angle_deg"])) * p["steer_sign"]
                why = "stopping" if moving else "held"
            else:
                steer = self._hold_steer(now)
                why = "bridge_stale"
            return self._out("llm", why, 0.0, brake, steer, p["steer_vel_llm"])

        # Not authorized: the joystick owns the cart.
        self.loop.reset()
        self.calib_throttle = 0.0
        self.calib_kicked = False
        if now - self.joy_cmd_t >= p["joy_cmd_timeout"]:
            return self._out("hold", "joystick_node_stale", 0.0, p["hold_brake"],
                             self._hold_steer(now), p["steer_vel_llm"])
        jc = self.joy_cmd
        acting = (jc["throttle"] > 0.0 or jc["brake"] > 0.0
                  or abs(self.stick) > p["stick_override"])
        if self.handover and not acting:
            return self._out("handover", reason, 0.0, p["hold_brake"],
                             self._hold_steer(now), p["steer_vel_llm"])
        self.handover = False
        self.last_steer_rad = jc["steer_rad"]
        return {"source": "joy", "reason": reason, "throttle": jc["throttle"],
                "brake": jc["brake"], "steer_rad": None, "steer_vel": None}


# ---- ROS 2 wiring -------------------------------------------------------------------------------
def message_class(type_name):
    """'pkg/Type' or 'pkg/msg/Type' to the message class, or None."""
    from rosidl_runtime_py.utilities import get_message

    parts = type_name.split("/")
    if len(parts) == 2:
        parts.insert(1, "msg")
    try:
        return get_message("/".join(parts))
    except (AttributeError, ModuleNotFoundError, ValueError):
        return None


def main():
    import rclpy
    from pacmod_msgs.msg import PacmodCmd, PositionWithSpeed
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from sensor_msgs.msg import Joy
    from std_msgs.msg import Bool, String

    rclpy.init()
    node = Node("cart_supervisor", automatically_declare_parameters_from_overrides=True)
    log = node.get_logger()

    def param(name, default):
        """Parameter from the yaml/launch, else the default, coerced to the default's type."""
        if not node.has_parameter(name):
            return default
        value = node.get_parameter(name).value
        if isinstance(default, float) and isinstance(value, int) and not isinstance(value, bool):
            return float(value)
        if isinstance(default, list):
            return [float(v) for v in value]
        return value

    params = dict((k, param(k, v)) for k, v in DEFAULTS.items())
    sup = Supervisor(params)
    lock = threading.Lock()
    clock = time.monotonic

    # Report topics. Each: (topic, type, value field, has enabled/override fields)
    spec = {
        "speed": (param("speed_topic", "/pacmod/parsed_tx/vehicle_speed_rpt"),
                  param("speed_type", "pacmod_msgs/VehicleSpeedRpt"),
                  param("speed_field", "vehicle_speed"), False),
        "steer": (param("steer_rpt_topic", "/pacmod/parsed_tx/steer_rpt"),
                  param("steer_rpt_type", "pacmod_msgs/SystemRptFloat"), "output", True),
        "accel": (param("accel_rpt_topic", "/pacmod/parsed_tx/accel_rpt"),
                  param("accel_rpt_type", "pacmod_msgs/SystemRptFloat"), "output", True),
        "brake": (param("brake_rpt_topic", "/pacmod/parsed_tx/brake_rpt"),
                  param("brake_rpt_type", "pacmod_msgs/SystemRptFloat"), "output", True),
        "shift": (param("shift_rpt_topic", "/pacmod/parsed_tx/shift_rpt"),
                  param("shift_rpt_type", "pacmod_msgs/SystemRptInt"), "output", True),
    }
    if params["use_enabled_topic"]:
        spec["enabled"] = (param("enabled_topic", "/pacmod/as_tx/enable"),
                           "std_msgs/Bool", "data", False)

    def make_cb(name, field, has_flags):
        if has_flags and not params["use_enabled_topic"]:
            log.warning("cart_supervisor: use_enabled_topic is false; this only works if the "
                        "PACMod reports carry an 'enabled' field", once=True)

        def cb(msg):
            value = get_field(msg, field)
            enabled = getattr(msg, "enabled", None) if has_flags else None
            override = getattr(msg, "override_active", None) if has_flags else None
            if has_flags and enabled is None:
                log.info("cart_supervisor: PACMod reports have no 'enabled' field; using %s for "
                         "system state" % params.get("enabled_topic", ""), once=True)
            with lock:
                sup.on_report(name, clock(), value, enabled, override)
        return cb

    for name, (topic, type_name, field, flags) in spec.items():
        cls = message_class(type_name)
        if cls is None:
            log.fatal("cart_supervisor: unknown message type %s for %s" % (type_name, name))
            node.destroy_node()
            rclpy.shutdown()
            return
        node.create_subscription(cls, topic, make_cb(name, field, flags), 1)

    def on_joy(msg):
        with lock:
            sup.on_joy(list(msg.axes), list(msg.buttons), clock())

    node.create_subscription(Joy, "/joy", on_joy, 1)

    # Joystick node outputs, namespaced into joy_ns by the launch file
    joy_ns = param("joy_cmd_ns", "/joy_cmd").rstrip("/")
    out_ns = param("out_ns", "/pacmod/as_rx").rstrip("/")
    pub = {
        "accel": node.create_publisher(PacmodCmd, out_ns + "/accel_cmd", 1),
        "brake": node.create_publisher(PacmodCmd, out_ns + "/brake_cmd", 1),
        "steer": node.create_publisher(PositionWithSpeed, out_ns + "/steer_cmd", 1),
        "shift": node.create_publisher(PacmodCmd, out_ns + "/shift_cmd", 1),
        "turn": node.create_publisher(PacmodCmd, out_ns + "/turn_cmd", 1),
        "enable": node.create_publisher(Bool, out_ns + "/enable", 1),
    }
    status_pub = node.create_publisher(String, "~/status", 1)
    last_joy_msgs = {}

    def cache(kind, extract):
        def cb(msg):
            with lock:
                last_joy_msgs[kind] = msg
                sup.on_joy_cmd(extract[0], extract[1](msg), clock())
        return cb

    node.create_subscription(PacmodCmd, joy_ns + "/accel_cmd",
                             cache("accel", ("throttle", lambda m: m.f64_cmd)), 1)
    node.create_subscription(PacmodCmd, joy_ns + "/brake_cmd",
                             cache("brake", ("brake", lambda m: m.f64_cmd)), 1)
    node.create_subscription(PositionWithSpeed, joy_ns + "/steer_cmd",
                             cache("steer", ("steer_rad", lambda m: m.angular_position)), 1)
    # Gear, turn signal and enable always come from the operator, forwarded as they arrive.
    node.create_subscription(PacmodCmd, joy_ns + "/shift_cmd", pub["shift"].publish, 1)
    node.create_subscription(PacmodCmd, joy_ns + "/turn_cmd", pub["turn"].publish, 1)
    node.create_subscription(Bool, joy_ns + "/enable", pub["enable"].publish, 1)

    # UDP link to cart_bridge.py (harness side)
    bridge_addr = ("127.0.0.1", int(param("bridge_port", 47801)))
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind(("127.0.0.1", int(param("target_port", 47802))))
    rx.setblocking(False)
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    ignore = bool(param("ignore", False))
    status_every = [0]

    # Per-tick CSV log. A logging failure must never touch control: it disables the log.
    tick_log = {"file": None, "writer": None, "path": None}
    if params["log_dir"]:
        try:
            log_dir = os.path.expanduser(str(params["log_dir"]))
            os.makedirs(log_dir, exist_ok=True)
            path = os.path.join(log_dir, time.strftime("supervisor_%Y%m%d_%H%M%S.csv"))
            tick_log["file"] = open(path, "w", newline="")
            tick_log["writer"] = csv.DictWriter(tick_log["file"], fieldnames=Supervisor.LOG_FIELDS)
            tick_log["writer"].writeheader()
            tick_log["path"] = path
        except OSError as error:
            tick_log["file"] = tick_log["writer"] = None
            log.warning("cart_supervisor: tick log disabled (%s)" % error)

    def write_tick(now, out):
        if tick_log["writer"] is None:
            return
        try:
            with lock:
                row = sup.log_row(now, out)
            tick_log["writer"].writerow(
                dict((k, "" if v is None else (round(v, 5) if isinstance(v, float) else v))
                     for k, v in row.items()))
            if status_every[0] % 33 == 0:
                tick_log["file"].flush()
        except Exception as error:  # noqa: BLE001 - never let logging stop the control loop
            tick_log["writer"] = None
            log.warning("cart_supervisor: tick log stopped (%s)" % error)

    def pedal(value):
        return PacmodCmd(f64_cmd=float(value), ui16_cmd=0, enable=True, ignore=ignore,
                         clear=False)

    def on_timer():
        now = clock()
        while True:
            try:
                data = rx.recv(65536)
            except OSError:
                break
            try:
                target = json.loads(data.decode("utf-8"))
                if isinstance(target, dict) and "active" in target:
                    with lock:
                        sup.on_target(target, now)
            except ValueError:
                pass
        with lock:
            out = sup.tick(now)
            state = sup.bridge_state(now)
            joy_msgs = dict(last_joy_msgs)
        try:
            tx.sendto(json.dumps(state).encode("utf-8"), bridge_addr)
        except OSError:
            pass
        if out["source"] == "joy":
            for kind in ("accel", "brake", "steer"):
                if kind in joy_msgs:
                    pub[kind].publish(joy_msgs[kind])
        else:
            pub["accel"].publish(pedal(out["throttle"]))
            pub["brake"].publish(pedal(out["brake"]))
            pub["steer"].publish(PositionWithSpeed(angular_position=float(out["steer_rad"]),
                                                   angular_velocity_limit=float(out["steer_vel"])))
        status_every[0] += 1
        write_tick(now, out)
        if status_every[0] % 6 == 0:  # ~5 Hz
            with lock:
                raw = sup.speed_raw(now)
            info = dict(out, speed_mps=state["speed_mps"], speed_raw=raw,
                        auth_reason=state["auth_reason"], steering_deg=state["steering_deg"])
            status_pub.publish(String(data=json.dumps(info)))
            if params["mode"] == "calib" and out["source"] == "calib":
                log.info("%s throttle %.3f speed %.3f m/s"
                         % (out["reason"], out["throttle"], state["speed_mps"]),
                         throttle_duration_sec=0.5)

    def check_topics():
        check_timer.cancel()
        for name, (topic, _, _, _) in spec.items():
            if node.count_publishers(topic) == 0:
                log.warning("cart_supervisor: no publisher on %s (%s). LLM authority will stay "
                            "off until it reports." % (topic, name))

    node.create_timer(0.03, on_timer)
    check_timer = node.create_timer(3.0, check_topics)
    log.info("cart_supervisor: mode=%s deadman_button=%d throttle_cap=%.2f"
             % (params["mode"], params["deadman_button"], params["throttle_cap"]))
    if tick_log["path"]:
        log.info("cart_supervisor: tick log %s" % tick_log["path"])
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if tick_log["file"] is not None:
            tick_log["file"].close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
