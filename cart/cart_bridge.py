"""Cart command owner: the harness Controller fed by the ROS cart_supervisor over UDP.

Replaces device/native.py (the openpilot controlsd hook). Runs in the harness Python (3.12).

    cart_supervisor (ROS) --UDP state 33 Hz--> this loop (50 Hz) --UDP target--> cart_supervisor
    producer (device/service.py) <--Unix socket RPC--> this loop

The Controller keeps the harness rules unchanged: one timed command, expiry, cancel on any
operator intervention or disengagement, nothing resumes without a fresh command.
"""

import argparse
import json
import logging
import socket
import time
from pathlib import Path

from drivingbench.controller.core import Controller, Sample
from drivingbench.device.ipc import NativeServer

STATE_TIMEOUT_S = 0.2
PERIOD_S = 0.02

# Reasons the model sees, renamed where openpilot wording would mislead it.
RENAME = {
    "native_disengaged": "supervisor_not_engaged",
    "waiting_for_res": "starting",
}

log = logging.getLogger("cart_bridge")


class CartController(Controller):
    allow_steer_in_place = True  # tick() sends an inactive target: brake held, wheels turn

    def motion_block(self, now, settings=None):
        reason = super().motion_block(now, settings)
        return RENAME.get(reason, reason)

    def status(self, now):
        result = super().status(now)
        result["reason"] = RENAME.get(result.get("reason"), result.get("reason"))
        if self.command is not None and self.command["speed_mps"] == 0:
            result["reason"] = "steering_in_place"
        return result


def to_sample(state, fresh):
    """Build a harness Sample. A stale link keeps last values but invalidates them."""
    if state is None:
        return Sample()
    return Sample(
        speed_mps=float(state["speed_mps"]),
        steering_deg=float(state["steering_deg"]),
        gear="drive" if state["gear_drive"] else "not_drive",
        brake_pressed=bool(state["brake_pressed"]),
        gas_pressed=bool(state["gas_pressed"]),
        steering_pressed=bool(state["steering_pressed"]),
        enabled=fresh and bool(state["authorized"]),
        lateral_active=fresh and bool(state["lateral_active"]),
        longitudinal_active=fresh and bool(state["longitudinal_active"]),
        sensors_valid=fresh and bool(state["sensors_valid"]),
        car_state_valid=fresh and bool(state["car_state_valid"]),
        standstill=bool(state["standstill"]),
    )


class Bridge:
    def __init__(self, socket_path, state_port, target_port, clock=time.monotonic):
        self.clock = clock
        self.server = NativeServer(socket_path)
        self.controller = CartController()
        self.rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rx.bind(("127.0.0.1", state_port))
        self.rx.setblocking(False)
        self.tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.target_addr = ("127.0.0.1", target_port)
        self.state = None
        self.state_at = float("-inf")
        self.failure = None
        self.last_auth_reason = None

    def _drain(self, now):
        while True:
            try:
                data = self.rx.recv(65536)
            except BlockingIOError:
                return
            try:
                state = json.loads(data)
                to_sample(state, True)  # validates every field
            except (ValueError, KeyError, TypeError):
                continue
            self.state, self.state_at = state, now

    def _reply(self, message):
        return self.controller.handle(message, self.clock())

    def _failed_reply(self, message):
        c = self.controller
        if message.get("operation") in ("status", "heartbeat"):
            return dict(c.status(self.clock()), native_error=self.failure)
        if message.get("operation") == "stop":
            result = c.handle(message, self.clock())
            c.reason = "cart_bridge_failed"
            return result
        return c.reply(status="rejected", reason="cart_bridge_failed")

    def _send(self, target):
        try:
            self.tx.sendto(json.dumps(target).encode(), self.target_addr)
        except OSError:
            pass

    def tick(self):
        now = self.clock()
        self._drain(now)
        fresh = self.state is not None and now - self.state_at < STATE_TIMEOUT_S
        c = self.controller
        if self.failure:
            c.sample = to_sample(self.state, fresh)
            self.server.poll(self._failed_reply)
            self._send({"active": False, "speed_mps": 0.0,
                        "angle_deg": c.sample.steering_deg, "epoch": c.command_epoch})
            return
        try:
            c.refresh(to_sample(self.state, fresh), now)
            self.server.poll(self._reply)
            selection = c.step(self.clock())
            command = c.command
            # A zero-speed command is steer-in-place: an inactive target makes the supervisor
            # hold the brake while it still turns the wheels to angle_deg.
            moving = command is not None and command["speed_mps"] > 0
            self._send({
                "active": moving and not selection.should_stop,
                "speed_mps": command["speed_mps"] if command else 0.0,
                "angle_deg": selection.angle_deg,
                "epoch": c.command_epoch,
            })
        except Exception as error:  # Latch: stop is still served, motion is refused.
            self.failure = f"{type(error).__name__}: {error}"
            log.exception("cart_bridge failed; motion disabled until restart")
            c.cancel("cart_bridge_failed")
            self._send({"active": False, "speed_mps": 0.0, "angle_deg": 0.0,
                        "epoch": c.command_epoch})
        reason = self.state.get("auth_reason") if fresh and self.state else "cart_state_stale"
        if reason != self.last_auth_reason:
            log.info("supervisor: %s", reason)
            self.last_auth_reason = reason

    def run(self):
        log.info("cart_bridge running, boot_id %s", self.controller.boot_id)
        next_tick = self.clock()
        try:
            while True:
                self.tick()
                next_tick += PERIOD_S
                delay = next_tick - self.clock()
                if delay > 0:
                    time.sleep(delay)
                else:
                    next_tick = self.clock()
        finally:
            self.server.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--native-socket", type=Path,
                        default=Path.home() / "drivingbench-cart/native.sock")
    parser.add_argument("--state-port", type=int, default=47801)
    parser.add_argument("--target-port", type=int, default=47802)
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(message)s")
    Bridge(args.native_socket, args.state_port, args.target_port).run()


if __name__ == "__main__":
    main()
