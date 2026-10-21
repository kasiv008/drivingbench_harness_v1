# DrivingBench on the golf cart

This folder ports the DrivingBench harness from comma/openpilot to a PACMod golf cart with a
USB camera. The MCP tools, controller rules, producer, gateway, UI and traces are reused
unchanged. Only the bottom layer is new.

```
MCP client (Claude Code, ...)
   │ observe · set_motion · stop_now        cart/mcp_server.py   (mcp/tools.py, cart prompt)
gateway + UI + traces, :8766                cart/gateway.py      (gateway/app.py)
   │ HTTP
producer, :8876                             cart/producer.py     (device/service.py + USB camera)
   │ Unix socket RPC
cart_bridge, 50 Hz                          cart/cart_bridge.py  (controller/core.py)
   │ UDP 47801/47802, localhost
cart_supervisor (ROS), 33 Hz                cart/ros/cart_supervisor.py
   │ mux: joystick or LLM, speed loop, deadman
/pacmod/as_rx/*  →  PACMod  →  cart
```

The harness side needs Python 3.12 (uv). The supervisor runs in your ROS Python (2.7 or 3).
They only talk over localhost UDP.

## Safety model

The LLM drives only while all of these hold. Otherwise the joystick owns the cart.

- The deadman button is held (`deadman_button`, default 2).
- Both triggers are released and the steering stick is centered.
- PACMod is enabled, all reports are fresh, and the gear report reads forward.
- The bridge target is fresh (< 0.2 s).

Releasing the deadman or touching a trigger or the stick ends LLM authority on the next
tick. The harness command is cancelled and never resumes on its own. The supervisor holds
`hold_brake` and the current steering angle until you act on the joystick. Speed above
`max_speed_mps` brakes at `brake_cap`. If the joystick node dies, the supervisor holds the
brake.

These are software checks. Keep a person in the seat with the physical brake and an e-stop.

## Install

### ROS side (cart PC)

```bash
PKG=~/compile_ws/src/joystick_demo        # your joystick_demo package
cp cart/ros/cart_supervisor.py $PKG/scripts/ && chmod +x $PKG/scripts/cart_supervisor.py
mkdir -p $PKG/config && cp cart/ros/cart_supervisor.yaml $PKG/config/
cp cart/ros/llm_drive.launch $PKG/launch/
cd ~/compile_ws && catkin_make && source devel/setup.bash
```

On Melodic (Python 2), change the first line of `cart_supervisor.py` to `#!/usr/bin/env python`.

`llm_drive.launch` starts joy_node, your unchanged joystick_demo in the `/joy_cmd` namespace,
and the supervisor. **Use it instead of joystick_demo.launch.** Nothing else may publish to
`/pacmod/as_rx/*`.

### Harness side (cart PC)

```bash
cd drivingbench_harness_v1          # repo root, with this cart/ folder inside
uv sync --locked
uv pip install opencv-python-headless numpy
uv run python cart/tests/smoke_e2e.py        # should end with ALL PASSED
uv run pytest -q cart/tests                  # supervisor logic
```

## Configure

1. **Topics.** Check `rostopic list | grep pacmod` and `rostopic type <topic>`. Set the report
   topics and types in `cart_supervisor.yaml`. If the cart has no speed report, point
   `speed_topic` at another source (for Odometry use `speed_type: nav_msgs/Odometry` and
   `speed_field: twist.twist.linear.x`). The supervisor warns about missing publishers 3 s
   after start.
2. **Gear.** Shift to forward with the joystick, run `rostopic echo` on the shift report, and
   set `gear_forward_value` to its `output`.
3. **Deadman.** Pick a free button with `jstest /dev/input/js0`. Set `deadman_button`.
4. **Steering scale.** Find the steering-wheel angle you want at 100% (the usable lock).
   Set `max_steering_angle_deg` in `cart_settings.json` (or in the UI). Set `max_steer_rad`
   in the yaml at or above that value in radians. The settings file only seeds the first
   start. After that the UI owns settings (`~/drivingbench-cart/state/settings.json`).
5. **Camera.** `v4l2-ctl --list-formats-ext` shows supported modes. MJPG at 1920x1080 is the
   default. The model sees a 1280 px wide JPEG at up to 5 Hz.

## Bring-up ladder

Pass each stage before the next. Wheels lifted for 1 to 3.

1. **Supervisor only.** `roslaunch joystick_demo llm_drive.launch` with the harness stopped.
   Drive with the joystick as before. Hold the deadman: `rostopic echo
   /cart_supervisor/status` shows `source: llm, reason: bridge_stale` and the brake holds.
   Release: brake holds (`handover`) until you touch a trigger.
2. **Feedforward.** Set `mode: calib` and `calib_throttle` to 0.2, 0.3, ... For each, hold the
   deadman and read the steady speed from the log. Fill `ff_speeds` and `ff_throttle`. Repeat
   on the ground at low values. Set `mode: llm` afterwards.
3. **Harness by hand.** `cart/run_harness.sh` (camera arg optional). Open
   http://127.0.0.1:8766 and check the camera. Hold the deadman and send:
   ```bash
   curl -s localhost:8766/api/motion -H 'content-type: application/json' \
     -d '{"direction":"straight","steering_percent":0,"speed_mps":0.8,"duration_s":5}'
   curl -s localhost:8766/api/stop -H 'content-type: application/json' -d '{}'
   ```
   Confirm expiry and stop brake, the deadman cancels, and left/right match `direction`.
4. **On the ground, by hand.** Same commands at 0.5 to 1.0 m/s. Tune `kp`, `ki`, `stop_brake`.
5. **Model.** Connect the MCP client and let it drive a cone course at `speed_limit_mps` ≤ 1.5.

Record everything: `rosbag record /pacmod /joy /joy_cmd /cart_supervisor/status`. The harness
also writes its own traces under `~/drivingbench-cart/state/recordings`.

## Connect a model

On the cart PC, or on a laptop with `ssh -L 8766:127.0.0.1:8766 <cart-pc>`:

```bash
claude mcp add drivingbench_sandbox -- \
  uv run --directory /path/to/drivingbench_harness_v1 python cart/mcp_server.py --client claude
```

Then paste the objective from the UI into the chat. Any MCP client with image input and tool
calls works the same way.

## Files

| File | Role |
|---|---|
| `ros/cart_supervisor.py` | Mux, deadman, speed loop, emergency brake, UDP state link (ROS) |
| `ros/cart_supervisor.yaml` | All supervisor parameters (CALIBRATE marks) |
| `ros/llm_drive.launch` | joy_node + joystick_demo in `/joy_cmd` + supervisor |
| `cart_bridge.py` | Harness Controller and native socket, replaces `device/native.py` |
| `usb_camera.py` | USB camera with the producer's camera interface; `test` = synthetic |
| `producer.py` | Starts `device/service.py` with the USB camera |
| `gateway.py` | Starts the gateway against the local producer |
| `mcp_server.py` | Harness MCP server with cart instructions |
| `cart_settings.json` | First-start settings (steering scale, speed ceiling, objective) |
| `run_harness.sh` | Starts bridge, producer and gateway |
| `tests/` | Supervisor unit tests and an end-to-end smoke test |
