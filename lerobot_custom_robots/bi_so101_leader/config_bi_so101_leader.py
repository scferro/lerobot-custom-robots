#!/usr/bin/env python

# MIT License - Copyright (c) 2025 Stephen

from dataclasses import dataclass

from lerobot.teleoperators.config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("bi_so101_leader")
@dataclass
class BiSO101LeaderConfig(TeleoperatorConfig):
    """Configuration for bimanual SO-101 leader arms.

    Requires two USB serial ports - one for each leader arm.

    Example:
        config = BiSO101LeaderConfig(
            left_arm_port="/dev/ttyUSB0",
            right_arm_port="/dev/ttyUSB1",
        )
    """

    left_arm_port: str
    right_arm_port: str

    # Optional parameters
    left_arm_use_degrees: bool = False
    right_arm_use_degrees: bool = False
