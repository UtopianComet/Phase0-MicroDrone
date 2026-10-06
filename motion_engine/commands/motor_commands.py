"""Motor command stubs for directional movement.

Phase-0 scope: no real ESC/PWM output. This module translates a requested
direction (or raw pitch/roll/yaw/throttle deltas) into a stub MotorCommand —
one throttle value per motor on a quad-X frame — so Navigation and the test
harness have something concrete to log and assert against before real motor
control exists (Phase-1).

Motor layout (quad-X, viewed from above, nose pointing toward FORWARD):

        FRONT
    FL         FR
       \\     /
        \\   /
        /   \\
       /     \\
    RL         RR
        REAR
"""

from dataclasses import dataclass
from enum import Enum


class Direction(Enum):
    """Discrete movement directions the Motion Engine can be asked to produce."""

    HOVER = "hover"
    FORWARD = "forward"
    BACKWARD = "backward"
    LEFT = "left"
    RIGHT = "right"
    UP = "up"
    DOWN = "down"
    YAW_LEFT = "yaw_left"
    YAW_RIGHT = "yaw_right"


# Baseline throttle that keeps the stub airframe hovering, on a 0-100 stub scale.
BASE_THROTTLE = 50.0

# How much a single direction command nudges each motor away from BASE_THROTTLE.
# Tune later once Phase-1 has a real frame to fly; for now these are placeholders.
PITCH_DELTA = 10.0
ROLL_DELTA = 10.0
YAW_DELTA = 8.0
THROTTLE_DELTA = 15.0

MIN_THROTTLE = 0.0
MAX_THROTTLE = 100.0


@dataclass
class MotorCommand:
    """Stub motor throttle values for a quad-X frame (0-100 scale each)."""

    front_left: float
    front_right: float
    rear_left: float
    rear_right: float

    def clamp(self) -> "MotorCommand":
        """Return a copy with every motor value clamped to [MIN_THROTTLE, MAX_THROTTLE]."""
        return MotorCommand(
            front_left=min(max(self.front_left, MIN_THROTTLE), MAX_THROTTLE),
            front_right=min(max(self.front_right, MIN_THROTTLE), MAX_THROTTLE),
            rear_left=min(max(self.rear_left, MIN_THROTTLE), MAX_THROTTLE),
            rear_right=min(max(self.rear_right, MIN_THROTTLE), MAX_THROTTLE),
        )

    def as_dict(self) -> dict:
        return {
            "front_left": self.front_left,
            "front_right": self.front_right,
            "rear_left": self.rear_left,
            "rear_right": self.rear_right,
        }


def _hover_command() -> MotorCommand:
    return MotorCommand(BASE_THROTTLE, BASE_THROTTLE, BASE_THROTTLE, BASE_THROTTLE)


def direction_to_motor_command(direction: Direction) -> MotorCommand:
    """Translate a Direction into a stub MotorCommand.

    This is a simplified quad-X mixer used for Phase-0 simulation/logging only:
    - Pitch forward/back raises the rear/front pair to tip the frame.
    - Roll left/right raises the opposite side pair.
    - Yaw left/right speeds up one diagonal pair relative to the other
      (real yaw comes from motor spin direction + reactive torque, which
      Phase-0 does not model).
    - Up/down raises or lowers all four motors together.
    """
    command = _hover_command()

    if direction == Direction.HOVER:
        pass
    elif direction == Direction.FORWARD:
        command.front_left -= PITCH_DELTA
        command.front_right -= PITCH_DELTA
        command.rear_left += PITCH_DELTA
        command.rear_right += PITCH_DELTA
    elif direction == Direction.BACKWARD:
        command.front_left += PITCH_DELTA
        command.front_right += PITCH_DELTA
        command.rear_left -= PITCH_DELTA
        command.rear_right -= PITCH_DELTA
    elif direction == Direction.LEFT:
        command.front_left -= ROLL_DELTA
        command.rear_left -= ROLL_DELTA
        command.front_right += ROLL_DELTA
        command.rear_right += ROLL_DELTA
    elif direction == Direction.RIGHT:
        command.front_left += ROLL_DELTA
        command.rear_left += ROLL_DELTA
        command.front_right -= ROLL_DELTA
        command.rear_right -= ROLL_DELTA
    elif direction == Direction.UP:
        command.front_left += THROTTLE_DELTA
        command.front_right += THROTTLE_DELTA
        command.rear_left += THROTTLE_DELTA
        command.rear_right += THROTTLE_DELTA
    elif direction == Direction.DOWN:
        command.front_left -= THROTTLE_DELTA
        command.front_right -= THROTTLE_DELTA
        command.rear_left -= THROTTLE_DELTA
        command.rear_right -= THROTTLE_DELTA
    elif direction == Direction.YAW_LEFT:
        command.front_left -= YAW_DELTA
        command.rear_right -= YAW_DELTA
        command.front_right += YAW_DELTA
        command.rear_left += YAW_DELTA
    elif direction == Direction.YAW_RIGHT:
        command.front_left += YAW_DELTA
        command.rear_right += YAW_DELTA
        command.front_right -= YAW_DELTA
        command.rear_left -= YAW_DELTA
    else:
        raise ValueError(f"Unknown direction: {direction!r}")

    return command.clamp()
