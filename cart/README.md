# DrivingBench on the golf cart

This folder runs the DrivingBench harness on a PACMod golf cart with four USB cameras (front,
left, right, rear). The MCP tools, controller rules, producer, gateway, UI and traces come from
the harness core; the camera layer (`device/cameras.py`) and the model instructions
(`shared/contracts.py`) are the cart's.

```
MCP client (Claude Code, ...)
   │ observe · set_motion · stop_now        cart/mcp_server.py   (mcp/tools.py, cart prompt)
gateway + UI + traces, :8766                cart/gateway.py      (gateway/app.py)
   │ HTTP
producer, :8876                             cart/producer.py     (device/service.py + 4 USB cameras)
   │ Unix socket RPC
cart_bridge, 50 Hz                          cart/cart_bridge.py  (controller/core.py)
   │ UDP 47801/47802, localhost
cart_supervisor (ROS 2), 33 Hz               cart/ros/cart_supervisor.py
   │ mux: joystick or LLM, speed loop, deadman
/pacmod/as_rx/*  →  PACMod  →  cart
```

The harness side needs Python 3.12 (uv). The supervisor runs in the ROS 2 Jazzy system Python.
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

### ROS side (cart PC, ROS 2 Jazzy)

`~/compile_ws` is self-contained: `golfcart_joystick_demo` plus `src/drivers/pacmod2` (ROS 2
port, SocketCAN) and `pacmod_msgs`. `cart/ros/` holds copies of the supervisor, parameters and
launch file; keep them identical.

```bash
source /opt/ros/jazzy/setup.bash
cd ~/compile_ws && colcon build --symlink-install && source install/setup.bash
```

`llm_drive.launch.xml` starts PACMod (SocketCAN on `can0`), joy_node, the unchanged joystick_demo
in the `/joy_cmd` namespace, and the supervisor. **Use it instead of demo.launch.xml.** Nothing
else may publish to `/pacmod/as_rx/*`: if PACMod already runs (for example from
`cart_hardware sensors.launch.py`), pass `launch_pacmod:=false`.

### Harness side (cart PC)

```bash
cd drivingbench_harness_v1          # repo root, with this cart/ folder inside
uv sync --locked
uv pip install opencv-python-headless numpy
uv run python cart/tests/smoke_e2e.py        # four synthetic cameras; should end with ALL PASSED
PYTHONPATH= uv run pytest -q cart/tests      # supervisor logic (keep ROS 2 off the path)
```

## Configure

1. **Topics.** Check `ros2 topic list | grep pacmod` and `ros2 topic type <topic>`. Set the report
   topics and types in `cart_supervisor.yaml`. If the cart has no speed report, point
   `speed_topic` at another source (for Odometry use `speed_type: nav_msgs/Odometry` and
   `speed_field: twist.twist.linear.x`). The supervisor warns about missing publishers 3 s
   after start.
2. **Gear.** Shift to forward with the joystick, run `ros2 topic echo` on the shift report, and
   set `gear_forward_value` to its `output`.
3. **Controller.** Button and axis indices live in `controller.yaml`, shared by joystick_demo and
   the supervisor. They depend on the controller's mode: check with `ros2 topic echo /joy`.
   Pick a free button for `deadman_button`. joystick_demo rejects joy messages with fewer axes
   or buttons than the mapping uses ("Check controller mode") and holds the brake.
4. **Steering scale.** Find the steering-wheel angle you want at 100% (the usable lock).
   Set `max_steering_angle_deg` in `cart_settings.json` (or in the UI). Set `max_steer_rad`
   in the yaml at or above that value in radians. The settings file only seeds the first
   start. After that the UI owns settings (`~/drivingbench-cart/state/settings.json`).
5. **Cameras.** The harness opens all four cameras itself: `/dev/cam_front`, `/dev/cam_left`,
   `/dev/cam_right`, `/dev/cam_rear` (udev names from `cart_ws` `99-cart-cameras.rules`). A
   camera can have only one reader, so start `cart_ws` `sensors.launch.py` with
   `launch_cameras:=false` while the harness runs. MJPG at 1920x1080 is captured; each
   camera is sent to the model as a 1280 px wide JPEG at up to 5 Hz.

   Names, mounts and the text the model reads for each camera are in `CAMERAS` in
   `shared/contracts.py`. The left/right aim (about 50 degrees off straight ahead) comes from
   `cart_ws` `static_tfs.yaml`; correct the description there if the mounts change. The rear
   image is not mirrored, and the description tells the model so.

   `observe` returns the cameras chosen under Setup → Model cameras (`camera_mode`: `all`,
   `front_and_sides`, `front_only`) in the order front, left, right, rear. The MCP server puts
   a text label (`Image 2 of 4: LEFT camera, 0.12 s old. <description>`) before each image. A
   camera without frames is listed in `cameras_unavailable` and the rest are still returned.

   **USB bandwidth.** These cameras reserve the full USB 2.0 isochronous bandwidth for MJPG at
   any resolution or frame rate, so **one USB controller carries only two cameras**. The PC has
   two controllers: the chipset (`02:00.0`, bus 1) and the CPU (`2d:00.3`, bus 3). Spread the
   cameras two and two, check with `lsusb -t`, and update `ID_PATH` in the udev rule after
   moving a cable. If a camera does not fit, the kernel logs `Not enough bandwidth for
   altsetting` and status shows `no_frames (check USB bandwidth ...)` for that camera. Cameras
   open in the order front, left, right, rear, so front always gets bandwidth first.

   Overrides: `cart/run_harness.sh --camera rear=off` (or `rear=/dev/videoN`), or
   `--test-cameras` for synthetic images.

## Bring-up ladder

Pass each stage before the next. Wheels lifted for 1 to 3.

1. **Supervisor only.** `ros2 launch golfcart_joystick_demo llm_drive.launch.xml` with the
   harness stopped. Drive with the joystick as before. Hold the deadman: `ros2 topic echo
   /cart_supervisor/status` shows `source: llm, reason: bridge_stale` and the brake holds.
   Release: brake holds (`handover`) until you touch a trigger.
2. **Feedforward.** Set `mode: calib` and `calib_throttle` to 0.2, 0.3, ... For each, hold the
   deadman and read the steady speed from the log. Fill `ff_speeds` and `ff_throttle`. Repeat
   on the ground at low values. Set `mode: llm` afterwards.
3. **Harness by hand.** `cart/run_harness.sh`. Open http://127.0.0.1:8766 and check that the
   Drive tab shows all four cameras, each under its right name. Hold the deadman and send:
   ```bash
   curl -s localhost:8766/api/motion -H 'content-type: application/json' \
     -d '{"direction":"straight","steering_percent":0,"speed_mps":0.8,"duration_s":5}'
   curl -s localhost:8766/api/stop -H 'content-type: application/json' -d '{}'
   ```
   Confirm expiry and stop brake, the deadman cancels, and left/right match `direction`.
4. **On the ground, by hand.** Same commands at 0.5 to 1.0 m/s. Tune `kp`, `ki`, `stop_brake`.
5. **Model.** Connect the MCP client and let it drive a cone course at `speed_limit_mps` ≤ 1.5.

Record everything: `ros2 bag record -e '/pacmod/.*|/joy.*|/cart_supervisor/status'`. The harness
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
| `ros/cart_supervisor.py` | Mux, deadman, speed loop, emergency brake, UDP state link (ROS 2) |
| `ros/cart_supervisor.yaml` | All supervisor parameters (CALIBRATE marks) |
| `ros/controller.yaml` | Joystick button and axis indices (joystick_demo and supervisor) |
| `ros/llm_drive.launch.xml` | PACMod + joy_node + joystick_demo in `/joy_cmd` + supervisor |
| `cart_bridge.py` | Harness Controller and native socket, replaces `device/native.py` |
| `producer.py` | Starts `device/service.py` with the four USB cameras (`../device/cameras.py`) |
| `gateway.py` | Starts the gateway against the local producer |
| `mcp_server.py` | Harness MCP server; instructions and camera descriptions in `../shared/contracts.py` |
| `cart_settings.json` | First-start settings (steering scale, speed ceiling, objective) |
| `run_harness.sh` | Starts bridge, producer and gateway |
| `tests/` | Supervisor unit tests and an end-to-end smoke test |
