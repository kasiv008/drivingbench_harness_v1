"""The harness MCP server (observe, set_motion, stop_now) with cart-specific instructions.

Same tools and schema as mcp/tools.py. Only the server instructions change: the cart steers at
standstill, and authority comes from the operator's deadman, not openpilot engagement.
"""

from drivingbench.shared import contracts

contracts.INSTRUCTIONS = (
    "Use only the drivingbench_sandbox MCP. You are driving a golf cart. Observe the road, then "
    "use set_motion(direction, steering_percent, speed_mps, duration_s, reason). "
    f"Speed is {contracts.SPEED_RANGE_MPS[0]}–{contracts.SPEED_RANGE_MPS[1]} m/s "
    f"(typically {contracts.TYPICAL_SPEED_MPS}); speeds above the operator's shared ceiling are "
    f"rejected. Duration is {contracts.DURATION_RANGE_S[0]:g}–{contracts.DURATION_RANGE_S[1]:g} s; "
    f"aim for about {contracts.TYPICAL_DURATION_S:g} s and adjust to the latency you measure "
    f"from the timestamps. {contracts.DURATION_GUIDANCE}"
    "Direction is left, right or straight. Percent is 0–100 of the "
    "operator's shared wheel-angle setting; zero means straight. State in reason, in about "
    "thirty words, what the command is for; it is recorded for the operator and never affects "
    "the cart. Commands replace globally and start on acceptance. Motion continues while you "
    "think; expiry starts braking. Observe measured steering_percent (same scale, positive "
    "left) and movement; the cart can steer while stopped. Use stop_now then observe to confirm "
    "standstill. A supervising operator must actively authorize motion; if the reason is "
    "supervisor_not_engaged, operator_brake or similar, motion is not possible until the "
    "operator re-engages, and you must send a fresh command afterwards. A rejected command "
    "leaves the previous one unchanged; after an uncertain result, observe rather than blindly "
    "retrying. timestamp is UTC response time; image_age_s describes the image's age. "
    "What this cart does, measured: it takes about 1.5 s to start rolling and 1 to 2 s to "
    "coast down, so a 3 s command is mostly launch and braking; send the next command while "
    "still rolling to avoid a stop and relaunch. Measured speed can run well above the "
    "command, especially downhill, so command 0.5 m/s unless told otherwise and check "
    "speed_mps. On a clear straight path use 8 to 10 s commands and observe during them; "
    "within about 8 m of a turn, obstacle or parking spot use 3 to 5 s. Steering is slow and "
    "nonlinear in effect: 100% is near full lock (roughly a 3 m turning radius) and takes "
    "about 3 s to wind on from centre; below about 20% the cart goes almost straight. Use 30 "
    "to 50% for a lane correction and 70 to 100% for a real turn, and start turning early. "
    "For a tight turn from rest, call steer_in_place first: it turns the wheels with the "
    "brake held, then set_motion with the same steering."
)

from drivingbench.mcp import tools  # noqa: E402  (must import after the override)

tools.STEER_IN_PLACE = True  # cart_bridge treats a zero-speed command as steer-only

if __name__ == "__main__":
    tools.main()
