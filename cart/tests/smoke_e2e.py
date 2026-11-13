"""End-to-end smoke test without ROS or a camera.

Starts cart_bridge, producer (synthetic camera) and gateway, plays the ROS supervisor over UDP,
then drives the harness through the gateway HTTP API and through the real MCP server.
Run: uv run python cart/tests/smoke_e2e.py
"""

import asyncio
import json
import os
import socket
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path

import httpx

ROOT = Path(__file__).resolve().parents[2]
CART = ROOT / "cart"
PORTS = dict(state=47901, target=47902, producer=8976, gateway=8966)


class FakeSupervisor:
    """Sends supervisor state at 33 Hz and records the latest target."""

    def __init__(self):
        self.state = dict(speed_mps=0.0, steering_deg=0.0, gear_drive=True, brake_pressed=False,
                          gas_pressed=False, steering_pressed=False, authorized=True,
                          auth_reason="authorized", lateral_active=True,
                          longitudinal_active=True, sensors_valid=True, car_state_valid=True,
                          standstill=True)
        self.target = None
        self.running = True
        self.rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rx.bind(("127.0.0.1", PORTS["target"]))
        self.rx.settimeout(0.01)
        self.tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        threading.Thread(target=self.run, daemon=True).start()

    def run(self):
        while self.running:
            self.tx.sendto(json.dumps(dict(self.state, t=time.time())).encode(),
                           ("127.0.0.1", PORTS["state"]))
            end = time.monotonic() + 0.03
            while time.monotonic() < end:
                try:
                    self.target = json.loads(self.rx.recv(65536))
                except (socket.timeout, BlockingIOError):
                    pass


def wait_for(predicate, timeout=5.0, what=""):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        value = predicate()
        if value:
            return value
        time.sleep(0.05)
    raise AssertionError(f"timed out waiting for {what}")


def check(condition, message):
    print(("PASS " if condition else "FAIL ") + message)
    if not condition:
        raise SystemExit(1)


async def mcp_session(gateway_url):
    from mcp.client.stdio import stdio_client

    from mcp import ClientSession, StdioServerParameters

    params = StdioServerParameters(
        command=sys.executable,
        args=[str(CART / "mcp_server.py"), "--gateway-url", gateway_url, "--client", "smoke"],
        env=dict(os.environ),
    )
    async with stdio_client(params) as (read, write):
        async with ClientSession(read, write) as session:
            init = await session.initialize()
            tools = sorted(t.name for t in (await session.list_tools()).tools)
            observed = await session.call_tool("observe", {})
            moved = await session.call_tool("set_motion", {
                "direction": "right", "steering_percent": 50, "speed_mps": 1.0,
                "duration_s": 5, "reason": "smoke test"})
            stopped = await session.call_tool("stop_now", {"reason": "smoke test"})
            return init.instructions, tools, observed, moved, stopped


def main():
    tmp = Path(tempfile.mkdtemp(prefix="cart-smoke-"))
    sock = tmp / "native.sock"
    env = dict(os.environ, PYTHONUNBUFFERED="1")
    py = sys.executable
    fake = FakeSupervisor()
    procs = [
        subprocess.Popen([py, str(CART / "cart_bridge.py"), "--native-socket", str(sock),
                          "--state-port", str(PORTS["state"]),
                          "--target-port", str(PORTS["target"])], env=env),
    ]
    time.sleep(1.0)
    procs.append(subprocess.Popen([py, str(CART / "producer.py"), "--state-dir", str(tmp / "state"),
                                   "--native-socket", str(sock), "--port", str(PORTS["producer"]),
                                   "--test-cameras", "--width", "1920", "--height", "1080"],
                                  env=env))
    procs.append(subprocess.Popen([py, str(CART / "gateway.py"), "--producer-url",
                                   f"http://127.0.0.1:{PORTS['producer']}",
                                   "--port", str(PORTS["gateway"])], env=env))
    api = httpx.Client(base_url=f"http://127.0.0.1:{PORTS['gateway']}", timeout=10)
    try:
        def status():
            try:
                return api.get("/api/status").json()
            except (httpx.HTTPError, ValueError):
                return None

        wait_for(lambda: (s := status()) and s.get("state") in ("held", "stopping"),
                 10, "native status through gateway")
        check(True, "gateway -> producer -> bridge status: " + status()["state"])

        obs = wait_for(lambda: len((o := api.get("/api/observe").json()).get("images", [])) == 4
                       and o, 10, "four camera frames")
        names = [img["camera"] for img in obs["images"]]
        check(names == ["front", "left", "right", "rear"], f"observe returns four cameras {names}")
        check(all(len(img["data"]) > 1000 and img["description"] for img in obs["images"]),
              "each camera has a JPEG and a description")
        check(obs["state"] == "held", "observe reports held at standstill")

        r = api.post("/api/motion", json={"direction": "left", "steering_percent": 50,
                                         "speed_mps": 1.0, "duration_s": 5}).json()
        check(r.get("status") == "accepted", f"motion accepted: {r}")
        t = wait_for(lambda: fake.target if fake.target and fake.target["active"] else None, 2,
                     "active target")
        scale = json.loads((CART / "cart_settings.json").read_text())["max_steering_angle_deg"]
        check(t["speed_mps"] == 1.0 and t["angle_deg"] == scale / 2,
              f"target speed 1.0, angle +{scale / 2:g} deg (left, 50% of {scale:g}): {t}")

        r = api.post("/api/motion", json={"direction": "left", "steering_percent": 10,
                                         "speed_mps": 3.0, "duration_s": 5}).json()
        check(r.get("error") == "speed_limit_exceeded", f"speed ceiling 1.5 enforced: {r}")

        r = api.post("/api/stop", json={"reason": "test"}).json()
        check(r.get("status") == "stopping", f"stop accepted: {r}")
        wait_for(lambda: fake.target and not fake.target["active"], 2, "inactive after stop")
        check(True, "target inactive after stop")

        api.post("/api/motion", json={"direction": "straight", "steering_percent": 0,
                                     "speed_mps": 1.0, "duration_s": 5})
        wait_for(lambda: fake.target and fake.target["active"], 2, "active again")
        fake.state.update(authorized=False, auth_reason="deadman_released",
                          lateral_active=False, longitudinal_active=False)
        wait_for(lambda: fake.target and not fake.target["active"], 2, "cancel on deadman")
        # The bridge cancels first; the producer's status follows on its next poll.
        s = wait_for(lambda: (s := status())["reason"] != "starting" and s, 1, "status update")
        check(s["reason"] == "supervisor_not_engaged", f"deadman release cancels: {s['reason']}")
        fake.state.update(authorized=True, auth_reason="authorized", lateral_active=True,
                          longitudinal_active=True)
        time.sleep(0.3)
        check(not fake.target["active"], "nothing resumes after re-engaging")

        fake.state.update(brake_pressed=True)
        time.sleep(0.2)
        r = api.post("/api/motion", json={"direction": "straight", "steering_percent": 0,
                                         "speed_mps": 1.0, "duration_s": 5}).json()
        check(r.get("error") == "operator_brake", f"operator brake rejects motion: {r}")
        fake.state.update(brake_pressed=False)
        time.sleep(0.2)  # let the bridge see the release before the next command

        api.post("/api/motion", json={"direction": "straight", "steering_percent": 0,
                                     "speed_mps": 1.0, "duration_s": 5})
        wait_for(lambda: fake.target and fake.target["active"], 2, "active for stale test")
        fake.running = False
        wait_for(lambda: (s := status()) and s.get("command") is None, 2, "cancel on stale state")
        check(True, "stale supervisor link cancels the command")
        fake.running = True
        threading.Thread(target=fake.run, daemon=True).start()
        time.sleep(0.5)

        api.post("/api/motion", json={"direction": "straight", "steering_percent": 0,
                                     "speed_mps": 1.0, "duration_s": 5})
        wait_for(lambda: fake.target and fake.target["active"], 2, "active for expiry test")
        wait_for(lambda: fake.target and not fake.target["active"], 7, "expiry")
        check(True, "command expires after duration_s")

        instructions, tools, observed, moved, stopped = asyncio.run(
            mcp_session(f"http://127.0.0.1:{PORTS['gateway']}"))
        check(tools == ["observe", "set_motion", "steer_in_place", "stop_now"], f"MCP tools: {tools}")
        check("golf cart" in instructions, "MCP serves cart instructions")
        kinds = [c.type for c in observed.content]
        check(kinds == ["text"] + ["text", "image"] * 4, f"MCP observe: summary + 4 labelled images {kinds}")
        labels = [block.text.split(",")[0] for block in observed.content[1::2]]
        check(labels == [f"Image {i} of 4: {name} camera" for i, name in
                         enumerate(("FRONT", "LEFT", "RIGHT", "REAR"), 1)], f"MCP labels: {labels}")
        check("rear:" in instructions and "four wide-angle cameras" in instructions.lower(),
              "MCP instructions describe the camera rig")
        check('"accepted"' in moved.content[0].text, f"MCP set_motion: {moved.content[0].text}")
        check('"stopping"' in stopped.content[0].text, f"MCP stop_now: {stopped.content[0].text}")
        print("ALL PASSED")
    finally:
        fake.running = False
        for p in reversed(procs):
            p.terminate()
        for p in procs:
            p.wait(timeout=5)


if __name__ == "__main__":
    main()
