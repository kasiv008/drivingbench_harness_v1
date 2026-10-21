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
    "from the timestamps. Direction is left, right or straight. Percent is 0–100 of the "
    "operator's shared wheel-angle setting; zero means straight. State in reason, in about "
    "thirty words, what the command is for; it is recorded for the operator and never affects "
    "the cart. Commands replace globally and start on acceptance. Motion continues while you "
    "think; expiry starts braking. Observe measured steering_percent (same scale, positive "
    "left) and movement; the cart can steer while stopped. Use stop_now then observe to confirm "
    "standstill. A supervising operator must actively authorize motion; if the reason is "
    "supervisor_not_engaged, operator_brake or similar, motion is not possible until the "
    "operator re-engages, and you must send a fresh command afterwards. A rejected command "
    "leaves the previous one unchanged; after an uncertain result, observe rather than blindly "
    "retrying. timestamp is UTC response time; image_age_s describes the image's age."
)

from drivingbench.mcp import tools  # noqa: E402  (must import after the override)

if __name__ == "__main__":
    tools.main()
