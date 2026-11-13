"""The harness MCP server for the golf cart (observe, set_motion, stop_now, steer_in_place).

Instructions, including the four-camera rig, live in shared/contracts.py.
"""

from drivingbench.mcp import tools

tools.STEER_IN_PLACE = True  # cart_bridge treats a zero-speed command as steer-only

if __name__ == "__main__":
    tools.main()
