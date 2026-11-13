#!/usr/bin/env bash
# Start the harness side on the cart PC: bridge, producer, gateway. Ctrl+C stops all three.
# ROS side (separately): ros2 launch golfcart_joystick_demo llm_drive.launch.xml
# Usage: cart/run_harness.sh [producer args]
#   Cameras: front, left, right, rear from /dev/cam_<name> (udev, cart_ws 99-cart-cameras.rules).
#   cart/run_harness.sh --test-cameras            synthetic images, no hardware
#   cart/run_harness.sh --camera rear=off         run without one camera
set -euo pipefail
cd "$(dirname "$0")/.."
# The harness has its own Python environment: keep ROS 2 packages off its path.
unset PYTHONPATH

uv run python cart/cart_bridge.py &
BRIDGE=$!
sleep 1
uv run python cart/producer.py "$@" &
PRODUCER=$!
sleep 2
uv run python cart/gateway.py &
GATEWAY=$!

trap 'kill $GATEWAY $PRODUCER $BRIDGE 2>/dev/null; wait' INT TERM EXIT
echo "UI: http://127.0.0.1:8766  (from a laptop: ssh -L 8766:127.0.0.1:8766 <cart-pc>)"
wait -n
echo "a harness process exited; stopping the rest"
