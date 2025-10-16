#!/usr/bin/env python

# MIT License - Copyright (c) 2025 Stephen

from dataclasses import dataclass, field

from lerobot.cameras import CameraConfig
from lerobot.robots.config import RobotConfig


@RobotConfig.register_subclass("bi_so101_follower")
@dataclass
class BiSO101FollowerConfig(RobotConfig):
    """Configuration for bimanual SO-101 follower arms.

    Requires two USB serial ports - one for each arm.

    Example:
        config = BiSO101FollowerConfig(
            left_arm_port="/dev/ttyUSB0",
            right_arm_port="/dev/ttyUSB1",
        )
    """

    left_arm_port: str
    right_arm_port: str

    # Optional parameters for left arm
    left_arm_disable_torque_on_disconnect: bool = True
    left_arm_max_relative_target: float | dict[str, float] | None = None
    left_arm_use_degrees: bool = False

    # Optional parameters for right arm
    right_arm_disable_torque_on_disconnect: bool = True
    right_arm_max_relative_target: float | dict[str, float] | None = None
    right_arm_use_degrees: bool = False

    # Cameras (shared between both arms)
    cameras: dict[str, CameraConfig] = field(default_factory=dict)
