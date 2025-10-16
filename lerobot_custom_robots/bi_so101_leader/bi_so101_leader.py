#!/usr/bin/env python

# MIT License - Copyright (c) 2025 Stephen

import logging
from functools import cached_property
from typing import Any

from lerobot.teleoperators.teleoperator import Teleoperator
from lerobot.teleoperators.so101_leader import SO101Leader
from lerobot.teleoperators.so101_leader.config_so101_leader import SO101LeaderConfig

from .config_bi_so101_leader import BiSO101LeaderConfig

logger = logging.getLogger(__name__)


class BiSO101Leader(Teleoperator):
    """
    Bimanual SO-101 Leader Arms for teleoperation.

    This class wraps two SO101Leader instances to create a bimanual teleoperator setup.
    All actions are prefixed with "left_" or "right_" to match the BiSO101Follower
    action space.

    Hardware Setup:
        - Connect left leader arm to one USB serial port (e.g., /dev/ttyUSB0)
        - Connect right leader arm to another USB serial port (e.g., /dev/ttyUSB1)
        - Each arm has 6 motors with descriptive names:
          shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper

    Action Keys (12 motors total):
        - left_shoulder_pan.pos, left_shoulder_lift.pos, ..., left_gripper.pos
        - right_shoulder_pan.pos, right_shoulder_lift.pos, ..., right_gripper.pos

    Example usage:
        config = BiSO101LeaderConfig(
            left_arm_port="/dev/ttyUSB0",
            right_arm_port="/dev/ttyUSB1",
        )
        leader = BiSO101Leader(config)
        leader.connect()

        # Get actions with left_/right_ prefixes
        action = leader.get_action()
        # action = {
        #     "left_shoulder_pan.pos": 0.0,
        #     "left_shoulder_lift.pos": 0.0,
        #     ...
        #     "right_shoulder_pan.pos": 0.0,
        #     "right_shoulder_lift.pos": 0.0,
        #     ...
        # }

        leader.disconnect()
    """

    config_class = BiSO101LeaderConfig
    name = "bi_so101_leader"

    def __init__(self, config: BiSO101LeaderConfig):
        super().__init__(config)
        self.config = config

        # Create config for left leader arm with separate calibration ID
        left_arm_config = SO101LeaderConfig(
            id=f"{config.id}_left" if config.id else None,
            calibration_dir=config.calibration_dir,
            port=config.left_arm_port,
            use_degrees=config.left_arm_use_degrees,
        )

        # Create config for right leader arm with separate calibration ID
        right_arm_config = SO101LeaderConfig(
            id=f"{config.id}_right" if config.id else None,
            calibration_dir=config.calibration_dir,
            port=config.right_arm_port,
            use_degrees=config.right_arm_use_degrees,
        )

        # Instantiate both leader arms
        self.left_arm = SO101Leader(left_arm_config)
        self.right_arm = SO101Leader(right_arm_config)

    @cached_property
    def action_features(self) -> dict[str, type]:
        """Action features for all motors (left and right with prefixes)."""
        return {f"left_{motor}.pos": float for motor in self.left_arm.bus.motors} | {
            f"right_{motor}.pos": float for motor in self.right_arm.bus.motors
        }

    @cached_property
    def feedback_features(self) -> dict[str, type]:
        """Feedback features (not implemented)."""
        return {}

    @property
    def is_connected(self) -> bool:
        """Check if both leader arms are connected."""
        return self.left_arm.bus.is_connected and self.right_arm.bus.is_connected

    def connect(self, calibrate: bool = True) -> None:
        """Connect to both leader arms."""
        logger.info("Connecting left leader arm...")
        self.left_arm.connect(calibrate)

        logger.info("Connecting right leader arm...")
        self.right_arm.connect(calibrate)

        logger.info(f"{self} connected successfully.")

    @property
    def is_calibrated(self) -> bool:
        """Check if both leader arms are calibrated."""
        return self.left_arm.is_calibrated and self.right_arm.is_calibrated

    def calibrate(self) -> None:
        """Calibrate both leader arms sequentially."""
        logger.info("Calibrating left leader arm...")
        self.left_arm.calibrate()

        logger.info("Calibrating right leader arm...")
        self.right_arm.calibrate()

    def configure(self) -> None:
        """Configure both leader arms."""
        self.left_arm.configure()
        self.right_arm.configure()

    def setup_motors(self) -> None:
        """Setup motors for both leader arms."""
        logger.info("Setting up left leader arm motors...")
        self.left_arm.setup_motors()

        logger.info("Setting up right leader arm motors...")
        self.right_arm.setup_motors()

    def get_action(self) -> dict[str, float]:
        """
        Get actions from both leader arms with left_/right_ prefixes.

        Returns:
            Dictionary with keys like:
            - left_shoulder_pan.pos, left_shoulder_lift.pos, ..., left_gripper.pos
            - right_shoulder_pan.pos, right_shoulder_lift.pos, ..., right_gripper.pos
        """
        action_dict = {}

        # Add "left_" prefix to left leader arm actions
        left_action = self.left_arm.get_action()
        action_dict.update({f"left_{key}": value for key, value in left_action.items()})

        # Add "right_" prefix to right leader arm actions
        right_action = self.right_arm.get_action()
        action_dict.update({f"right_{key}": value for key, value in right_action.items()})

        return action_dict

    def send_feedback(self, feedback: dict[str, float]) -> None:
        """
        Send feedback to both leader arms (not implemented).

        Args:
            feedback: Dictionary with feedback for both arms (prefixed keys)
        """
        # TODO: Implement force feedback for bimanual setup
        raise NotImplementedError

    def disconnect(self) -> None:
        """Disconnect from both leader arms."""
        self.left_arm.disconnect()
        self.right_arm.disconnect()

        logger.info(f"{self} disconnected.")
