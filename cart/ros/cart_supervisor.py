#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Cart supervisor: joystick/LLM command mux, speed loop, and the state link to the harness.

Runs in the ROS (system) Python. Compatible with Python 2.7 (Melodic) and 3 (Noetic).

Data flow
    /joy, PACMod reports  ->  Supervisor  ->  /pacmod/as_rx/{accel,brake,steer}_cmd
    /joy_cmd/* (joystick_demo in its own namespace)  ->  forwarded when the LLM is not authorized
    UDP state  ->  cart_bridge.py (harness Controller)  ->  UDP target

The LLM only drives while the operator holds the deadman button, both triggers are released,
the steering stick is centered, PACMod is enabled and all required reports are fresh.
Releasing the deadman, or touching a trigger or the stick, ends LLM authority immediately and
holds the brake until the operator acts on the joystick.
"""
from __future__ import division, print_function

import json
import math
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
    "ff_speeds": [0.5, 3.0],
    "ff_throttle": [0.0, 0.0],
    "max_speed_mps": 3.0,
    "standstill_speed": 0.05,
    # Steering. Harness angles are left-positive steering-wheel degrees.
    "steer_sign": 1.0,
    "max_steer_rad": 5.5,
    "steer_vel_llm": 1.5,
    # Gear report value that means forward/drive
    "gear_forward_value": 3,
    # Empty: derive PACMod enabled from the steer, accel and brake reports
    "use_enabled_topic": True,
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
        self.reset()

    def reset(self):
        self.integ = 0.0
        self.throttle = 0.0

    def step(self, v_ref, v, dt):
        p = self.p
        err = v_ref - v
        if err < -p["brake_band"]:
            # Clearly too fast: brake proportionally inside the effective band.
            self.reset()
            brake = clip(p["brake_min"] + p["kb"] * (-err - p["brake_band"]),
                         p["brake_min"], p["brake_cap"])
            return 0.0, brake
        self.integ = clip(self.integ + err * dt, -p["i_max"], p["i_max"])
        ff = interp(v_ref, p["ff_speeds"], p["ff_throttle"])
        target = clip(ff + p["kp"] * err + p["ki"] * self.integ, 0.0, p["throttle_cap"])
        # Rate-limit increases only. Decreases apply immediately.
        self.throttle = min(target, self.throttle + p["throttle_slew"] * dt)
        return self.throttle, 0.0


class Report(object):
    def __init__(self):
        self.t = -INF
        self.value = None
        self.enabled = False
        self.override = False

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
        self.calib_throttle = 0.0

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
        r.t = now
        r.value = value
        if enabled is not None:
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
        r = self.reports["speed"]
        if not self._fresh("speed", now) or r.value is None:
            return None
        return float(r.value)

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
            "lateral_active": auth and self.reports["steer"].enabled,
            "longitudinal_active": (auth and self.reports["accel"].enabled
                                    and self.reports["brake"].enabled),
            "sensors_valid": fresh and v is not None and steer is not None,
            "car_state_valid": v is not None,
            "standstill": v is not None and abs(v) < self.p["standstill_speed"],
        }

    # ---- output -----------------------------------------------------------------------------
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


# ---- ROS wiring ---------------------------------------------------------------------------------
def main():
    import roslib.message
    import rospy
    from pacmod_msgs.msg import PacmodCmd, PositionWithSpeed
    from sensor_msgs.msg import Joy
    from std_msgs.msg import Bool, String

    rospy.init_node("cart_supervisor")
    params = dict((k, rospy.get_param("~" + k, v)) for k, v in DEFAULTS.items())
    sup = Supervisor(params)
    lock = threading.Lock()
    clock = time.monotonic if hasattr(time, "monotonic") else time.time

    # Report topics. Each: (topic, type, value field, has enabled/override fields)
    spec = {
        "speed": (rospy.get_param("~speed_topic", "/pacmod/parsed_tx/vehicle_speed_rpt"),
                  rospy.get_param("~speed_type", "pacmod_msgs/VehicleSpeedRpt"),
                  rospy.get_param("~speed_field", "vehicle_speed"), False),
        "steer": (rospy.get_param("~steer_rpt_topic", "/pacmod/parsed_tx/steer_rpt"),
                  rospy.get_param("~steer_rpt_type", "pacmod_msgs/SystemRptFloat"),
                  "output", True),
        "accel": (rospy.get_param("~accel_rpt_topic", "/pacmod/parsed_tx/accel_rpt"),
                  rospy.get_param("~accel_rpt_type", "pacmod_msgs/SystemRptFloat"),
                  "output", True),
        "brake": (rospy.get_param("~brake_rpt_topic", "/pacmod/parsed_tx/brake_rpt"),
                  rospy.get_param("~brake_rpt_type", "pacmod_msgs/SystemRptFloat"),
                  "output", True),
        "shift": (rospy.get_param("~shift_rpt_topic", "/pacmod/parsed_tx/shift_rpt"),
                  rospy.get_param("~shift_rpt_type", "pacmod_msgs/SystemRptInt"),
                  "output", True),
    }
    if params["use_enabled_topic"]:
        spec["enabled"] = (rospy.get_param("~enabled_topic", "/pacmod/as_tx/enable"),
                           "std_msgs/Bool", "data", False)

    def make_cb(name, field, has_flags):
        def cb(msg):
            value = get_field(msg, field)
            enabled = getattr(msg, "enabled", None) if has_flags else None
            override = getattr(msg, "override_active", None) if has_flags else None
            with lock:
                sup.on_report(name, clock(), value, enabled, override)
        return cb

    subs = []
    for name, (topic, type_name, field, flags) in spec.items():
        cls = roslib.message.get_message_class(type_name)
        if cls is None:
            rospy.logfatal("cart_supervisor: unknown message type %s for %s", type_name, name)
            return
        subs.append(rospy.Subscriber(topic, cls, make_cb(name, field, flags), queue_size=1))

    def on_joy(msg):
        with lock:
            sup.on_joy(list(msg.axes), list(msg.buttons), clock())

    subs.append(rospy.Subscriber("/joy", Joy, on_joy, queue_size=1))

    # Joystick node outputs, remapped into joy_ns by the launch file
    joy_ns = rospy.get_param("~joy_cmd_ns", "/joy_cmd").rstrip("/")
    out_ns = rospy.get_param("~out_ns", "/pacmod/as_rx").rstrip("/")
    pub = {
        "accel": rospy.Publisher(out_ns + "/accel_cmd", PacmodCmd, queue_size=1),
        "brake": rospy.Publisher(out_ns + "/brake_cmd", PacmodCmd, queue_size=1),
        "steer": rospy.Publisher(out_ns + "/steer_cmd", PositionWithSpeed, queue_size=1),
        "shift": rospy.Publisher(out_ns + "/shift_cmd", PacmodCmd, queue_size=1),
        "turn": rospy.Publisher(out_ns + "/turn_cmd", PacmodCmd, queue_size=1),
        "enable": rospy.Publisher(out_ns + "/enable", Bool, queue_size=1),
    }
    status_pub = rospy.Publisher("~status", String, queue_size=1)
    last_joy_msgs = {}

    def cache(kind, extract):
        def cb(msg):
            with lock:
                last_joy_msgs[kind] = msg
                sup.on_joy_cmd(extract[0], extract[1](msg), clock())
        return cb

    subs.append(rospy.Subscriber(joy_ns + "/accel_cmd", PacmodCmd,
                                 cache("accel", ("throttle", lambda m: m.f64_cmd)), queue_size=1))
    subs.append(rospy.Subscriber(joy_ns + "/brake_cmd", PacmodCmd,
                                 cache("brake", ("brake", lambda m: m.f64_cmd)), queue_size=1))
    subs.append(rospy.Subscriber(joy_ns + "/steer_cmd", PositionWithSpeed,
                                 cache("steer", ("steer_rad", lambda m: m.angular_position)),
                                 queue_size=1))
    # Gear, turn signal and enable always come from the operator, forwarded as they arrive.
    subs.append(rospy.Subscriber(joy_ns + "/shift_cmd", PacmodCmd, pub["shift"].publish,
                                 queue_size=1))
    subs.append(rospy.Subscriber(joy_ns + "/turn_cmd", PacmodCmd, pub["turn"].publish,
                                 queue_size=1))
    subs.append(rospy.Subscriber(joy_ns + "/enable", Bool, pub["enable"].publish, queue_size=1))

    # UDP link to cart_bridge.py (harness side)
    bridge_addr = ("127.0.0.1", int(rospy.get_param("~bridge_port", 47801)))
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind(("127.0.0.1", int(rospy.get_param("~target_port", 47802))))
    rx.setblocking(False)
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    ignore = bool(rospy.get_param("~ignore", False))
    status_every = [0]

    def pedal(value):
        return PacmodCmd(f64_cmd=float(value), ui16_cmd=0, enable=True, ignore=ignore,
                         clear=False)

    def on_timer(_event):
        now = clock()
        while True:
            try:
                data = rx.recv(65536)
            except (socket.error, OSError):
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
        except (socket.error, OSError):
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
        if status_every[0] % 6 == 0:  # ~5 Hz
            info = dict(out, speed_mps=state["speed_mps"], auth_reason=state["auth_reason"],
                        steering_deg=state["steering_deg"])
            status_pub.publish(String(data=json.dumps(info)))
            if params["mode"] == "calib" and out["source"] == "calib":
                rospy.loginfo_throttle(0.5, "calib throttle %.3f speed %.3f m/s"
                                       % (out["throttle"], state["speed_mps"]))

    def check_topics(_event):
        published = set(t for t, _ in rospy.get_published_topics())
        for name, (topic, _, _, _) in spec.items():
            if topic not in published:
                rospy.logwarn("cart_supervisor: no publisher on %s (%s). LLM authority will "
                              "stay off until it reports.", topic, name)

    rospy.Timer(rospy.Duration(0.03), on_timer)
    rospy.Timer(rospy.Duration(3.0), check_topics, oneshot=True)
    rospy.loginfo("cart_supervisor: mode=%s deadman_button=%d throttle_cap=%.2f",
                  params["mode"], params["deadman_button"], params["throttle_cap"])
    rospy.spin()


if __name__ == "__main__":
    main()
