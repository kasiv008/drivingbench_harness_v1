"""The complete v0.1 command and shared-settings contract. Angles are left-positive."""

from typing import Literal

from pydantic import BaseModel, ConfigDict, Field, field_validator

PROTOCOL = 2
SERVER_NAME = "drivingbench_sandbox"
NATIVE_WATCHDOG_S = 2.0
REASON_MAX_CHARS = 200

# Hard command limits, enforced in the MCP schema and again in the controller.
# Below ~0.5 m/s the Corolla's launch never rolls; a command shorter than an
# observe round trip would brake before the model could see anything.
SPEED_RANGE_MPS = (0.5, 3.5)
DURATION_RANGE_S = (3.0, 60.0)
TYPICAL_SPEED_MPS = 1.0
TYPICAL_DURATION_S = 10.0
# The model picks its own duration inside the range; this tells it how.
DURATION_GUIDANCE = (
    "Scale duration to your confidence: use the shortest durations when close to an obstacle, "
    "near the stopping point or unsure of the scene, and longer ones only when the path ahead "
    "is clearly open. "
)
# Fixed native emergency stop, well above any commandable speed plus launch overshoot.
# Never the editable ceiling, so lowering the ceiling cannot make overshoot an emergency.
EMERGENCY_SPEED_MPS = 6.0


class Model(BaseModel):
    model_config = ConfigDict(extra="forbid", allow_inf_nan=False)


class Motion(Model):
    direction: Literal["left", "right", "straight"]
    steering_percent: float = Field(ge=0, le=100)
    # 0 means steer in place: the owner holds the brake and only turns the wheels.
    speed_mps: float = Field(ge=0, le=SPEED_RANGE_MPS[1])
    duration_s: float = Field(ge=DURATION_RANGE_S[0], le=DURATION_RANGE_S[1])
    reason: str = Field(default="", max_length=REASON_MAX_CHARS)  # evidence only; never executed

    @field_validator("speed_mps")
    @classmethod
    def _speed_is_zero_or_in_range(cls, value):
        if value != 0 and value < SPEED_RANGE_MPS[0]:
            raise ValueError("speed_mps must be 0 or within the speed range")
        return value


class ImageSettings(Model):
    exposure_ev: float = Field(default=0, ge=-2, le=3)
    contrast: float = Field(default=1, ge=0.5, le=2)
    saturation: float = Field(default=1, ge=0, le=2)


class SpeedSettings(Model):
    """Longitudinal feedback coefficients."""

    gain: float = Field(default=0.5, gt=0)
    max_accel_mps2: float = Field(default=1.0, gt=0)
    max_decel_mps2: float = Field(default=2.0, gt=0)
    accel_slew_mps3: float = Field(default=1.0, gt=0)
    target_rate_feedforward: float = Field(default=0, ge=0)
    acceleration_damping_s: float = Field(default=1.0, ge=0)


# The cart's four USB cameras, in priority order: the producer opens them in this order, so if
# the USB bus cannot carry all four, front is the one that keeps streaming. Device paths are udev
# names from cart_ws config/99-cart-cameras.rules; mounts from cart_ws config/static_tfs.yaml.
# Each description is sent to the model with its image on every observe, so keep them exact.
CAMERAS = {
    "front": (
        "Roof, centred, looking straight ahead. Primary view for steering and speed: the path "
        "the cart will drive into. The cart's front edge can show along the bottom of the image."
    ),
    "left": (
        "Left side of the roof, aimed about 50 degrees left of straight ahead (front-left "
        "diagonal). Covers the front-left corner and the left side. The cart's own left roof "
        "pillar and mirror appear at the right edge of this image."
    ),
    "right": (
        "Right side of the roof, aimed about 50 degrees right of straight ahead (front-right "
        "diagonal). Covers the front-right corner and the right side. The cart's own right roof "
        "pillar appears at the left edge of this image."
    ),
    "rear": (
        "Back of the cart, looking backward. Not mirrored: the left half of this image is the "
        "cart's RIGHT side and the right half is the cart's LEFT side. The cart only drives "
        "forward; use this view for what is behind, never as the path ahead."
    ),
}
CAMERA_NAMES = tuple(CAMERAS)
# Which cameras observe returns. More cameras cost more model input and observe latency.
CAMERA_MODES = {
    "all": CAMERA_NAMES,
    "front_and_sides": ("front", "left", "right"),
    "front_only": ("front",),
}
CAMERA_RIG = (
    "Four wide-angle cameras (straight lines bend near the image edges). observe returns the "
    "selected ones in the order front, left, right, rear, each image preceded by a text label "
    "naming the camera and its view: "
    + " ".join(f"{name}: {text}" for name, text in CAMERAS.items())
    + " Judge distances and the path from front first; use left and right for the corners and "
    "for clearance while turning. "
)


class Settings(Model):
    # Steering-wheel degrees at 100%. 180 is what the Corolla's EPS reached at crawl speed
    # (~120-130) plus headroom; recalibrate per docs/steering-calibration.md.
    max_steering_angle_deg: float = Field(default=180, gt=0)
    # Per-session operator ceiling within the hard range; commands above it are rejected.
    speed_limit_mps: float = Field(default=3.0, ge=SPEED_RANGE_MPS[0], le=SPEED_RANGE_MPS[1])
    camera_mode: Literal["all", "front_and_sides", "front_only"] = "all"
    image_adjustments: ImageSettings = Field(default_factory=ImageSettings)
    speed_control: SpeedSettings = Field(default_factory=SpeedSettings)
    objective: str = "Follow the operator-designated course and stop at the destination."
    prompt: str = ""


class SessionLabel(Model):
    """Operator-declared benchmark session; the model never sees it."""

    model: str = Field(min_length=1, max_length=80)
    harness: Literal["codex", "claude", "cursor", "other"]
    notes: str = Field(default="", max_length=REASON_MAX_CHARS)


class SessionEnd(Model):
    outcome: Literal["completed", "collision", "aborted"]
    note: str = Field(default="", max_length=REASON_MAX_CHARS)


class Request(Model):
    """Internal transport envelope. No client identity grants control authority."""

    protocol: int = PROTOCOL
    request_id: str = Field(min_length=1, max_length=128)
    operation: Literal["motion", "stop"]
    boot_id: str
    command_epoch: int
    motion: Motion | None = None
    settings: Settings | None = None


INSTRUCTIONS = (
    "Use only the drivingbench_sandbox MCP. You are driving a golf cart. Observe the cameras, "
    "then use set_motion(direction, steering_percent, speed_mps, duration_s, reason). "
    + CAMERA_RIG
    + f"Speed is {SPEED_RANGE_MPS[0]}–{SPEED_RANGE_MPS[1]} m/s "
    f"(typically {TYPICAL_SPEED_MPS}); speeds above the operator's shared ceiling are "
    f"rejected. Duration is {DURATION_RANGE_S[0]:g}–{DURATION_RANGE_S[1]:g} s; "
    f"aim for about {TYPICAL_DURATION_S:g} s and adjust to the latency you measure "
    f"from the timestamps. {DURATION_GUIDANCE}"
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
    "retrying. timestamp is UTC response time; image_age_s is the oldest image's age. "
    "If a camera is listed in cameras_unavailable, drive more cautiously in that direction. "
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
