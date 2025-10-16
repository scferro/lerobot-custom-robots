#!/usr/bin/env python

# MIT License - Copyright (c) 2025 Stephen

import logging
import time
from functools import cached_property
from typing import Any

from lerobot.cameras.utils import make_cameras_from_configs
from lerobot.robots.robot import Robot
from lerobot.robots.so101_follower import SO101Follower
from lerobot.robots.so101_follower.config_so101_follower import SO101FollowerConfig

from .config_bi_so101_follower import BiSO101FollowerConfig

logger = logging.getLogger(__name__)


class BiSO101Follower(Robot):
    """
    Bimanual SO-101 Follower Arms designed by TheRobotStudio and Hugging Face.

    This class wraps two SO101Follower instances to create a bimanual robot setup.
    All observations and actions are prefixed with "left_" or "right_" to distinguish
    between the two arms.

    Hardware Setup:
        - Connect left arm to one USB serial port (e.g., /dev/ttyUSB0)
        - Connect right arm to another USB serial port (e.g., /dev/ttyUSB1)
        - Each arm has 6 motors with descriptive names:
          shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper

    Observation Keys (12 motors total):
        - left_shoulder_pan.pos, left_shoulder_lift.pos, ..., left_gripper.pos
        - right_shoulder_pan.pos, right_shoulder_lift.pos, ..., right_gripper.pos

    Example usage:
        config = BiSO101FollowerConfig(
            left_arm_port="/dev/ttyUSB0",
            right_arm_port="/dev/ttyUSB1",
        )
        robot = BiSO101Follower(config)
        robot.connect()

        # Get observations with left_/right_ prefixes
        obs = robot.get_observation()
        # obs = {
        #     "left_shoulder_pan.pos": 0.0,
        #     "left_shoulder_lift.pos": 0.0,
        #     ...
        #     "right_shoulder_pan.pos": 0.0,
        #     "right_shoulder_lift.pos": 0.0,
        #     ...
        # }

        # Send actions with left_/right_ prefixes
        action = {
            "left_shoulder_pan.pos": 0.0,
            "left_shoulder_lift.pos": 0.0,
            "left_elbow_flex.pos": 0.0,
            "left_wrist_flex.pos": 0.0,
            "left_wrist_roll.pos": 0.0,
            "left_gripper.pos": 0.0,
            "right_shoulder_pan.pos": 0.0,
            "right_shoulder_lift.pos": 0.0,
            "right_elbow_flex.pos": 0.0,
            "right_wrist_flex.pos": 0.0,
            "right_wrist_roll.pos": 0.0,
            "right_gripper.pos": 0.0,
        }
        robot.send_action(action)

        robot.disconnect()
    """

    config_class = BiSO101FollowerConfig
    name = "bi_so101_follower"

    def __init__(self, config: BiSO101FollowerConfig):
        super().__init__(config)
        self.config = config

        # Create config for left arm with separate calibration ID
        left_arm_config = SO101FollowerConfig(
            id=f"{config.id}_left" if config.id else None,
            calibration_dir=config.calibration_dir,
            port=config.left_arm_port,
            disable_torque_on_disconnect=config.left_arm_disable_torque_on_disconnect,
            max_relative_target=config.left_arm_max_relative_target,
            use_degrees=config.left_arm_use_degrees,
            cameras={},  # Cameras are handled at the bimanual level
        )

        # Create config for right arm with separate calibration ID
        right_arm_config = SO101FollowerConfig(
            id=f"{config.id}_right" if config.id else None,
            calibration_dir=config.calibration_dir,
            port=config.right_arm_port,
            disable_torque_on_disconnect=config.right_arm_disable_torque_on_disconnect,
            max_relative_target=config.right_arm_max_relative_target,
            use_degrees=config.right_arm_use_degrees,
            cameras={},  # Cameras are handled at the bimanual level
        )

        # Instantiate both arms
        self.left_arm = SO101Follower(left_arm_config)
        self.right_arm = SO101Follower(right_arm_config)

        # Shared cameras (if any)
        self.cameras = make_cameras_from_configs(config.cameras)

    @property
    def _motors_ft(self) -> dict[str, type]:
        """Feature types for all motors (left and right with prefixes)."""
        return {f"left_{motor}.pos": float for motor in self.left_arm.bus.motors} | {
            f"right_{motor}.pos": float for motor in self.right_arm.bus.motors
        }

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        """Feature types for cameras."""
        return {
            cam: (self.config.cameras[cam].height, self.config.cameras[cam].width, 3) for cam in self.cameras
        }

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        """All observation features (motors + cameras)."""
        return {**self._motors_ft, **self._cameras_ft}

    @cached_property
    def action_features(self) -> dict[str, type]:
        """All action features (motors only)."""
        return self._motors_ft

    @property
    def is_connected(self) -> bool:
        """Check if both arms and all cameras are connected."""
        return (
            self.left_arm.bus.is_connected
            and self.right_arm.bus.is_connected
            and all(cam.is_connected for cam in self.cameras.values())
        )

    def connect(self, calibrate: bool = True) -> None:
        """Connect to both arms and cameras."""
        logger.info("Connecting left arm...")
        self.left_arm.connect(calibrate)

        logger.info("Connecting right arm...")
        self.right_arm.connect(calibrate)

        for cam in self.cameras.values():
            cam.connect()

        logger.info(f"{self} connected successfully.")

    @property
    def is_calibrated(self) -> bool:
        """Check if both arms are calibrated."""
        return self.left_arm.is_calibrated and self.right_arm.is_calibrated

    def calibrate(self) -> None:
        """Calibrate both arms sequentially."""
        logger.info("Calibrating left arm...")
        self.left_arm.calibrate()

        logger.info("Calibrating right arm...")
        self.right_arm.calibrate()

    def configure(self) -> None:
        """Configure both arms."""
        self.left_arm.configure()
        self.right_arm.configure()

    def setup_motors(self) -> None:
        """Setup motors for both arms."""
        logger.info("Setting up left arm motors...")
        self.left_arm.setup_motors()

        logger.info("Setting up right arm motors...")
        self.right_arm.setup_motors()

    def get_observation(self) -> dict[str, Any]:
        """
        Get observations from both arms with left_/right_ prefixes.

        Returns:
            Dictionary with keys like:
            - left_shoulder_pan.pos, left_shoulder_lift.pos, ..., left_gripper.pos
            - right_shoulder_pan.pos, right_shoulder_lift.pos, ..., right_gripper.pos
            - Plus any camera keys
        """
        obs_dict = {}

        # Add "left_" prefix to left arm observations
        left_obs = self.left_arm.get_observation()
        obs_dict.update({f"left_{key}": value for key, value in left_obs.items()})

        # Add "right_" prefix to right arm observations
        right_obs = self.right_arm.get_observation()
        obs_dict.update({f"right_{key}": value for key, value in right_obs.items()})

        # Add camera observations (no prefix needed)
        for cam_key, cam in self.cameras.items():
            start = time.perf_counter()
            obs_dict[cam_key] = cam.async_read()
            dt_ms = (time.perf_counter() - start) * 1e3
            logger.debug(f"{self} read {cam_key}: {dt_ms:.1f}ms")

        return obs_dict

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        """
        Send actions to both arms. Actions must have left_/right_ prefixes.

        Args:
            action: Dictionary with keys like:
                - left_shoulder_pan.pos, left_shoulder_lift.pos, ..., left_gripper.pos
                - right_shoulder_pan.pos, right_shoulder_lift.pos, ..., right_gripper.pos

        Returns:
            The action actually sent (potentially clipped), with prefixes restored.
        """
        # Extract left arm actions (remove "left_" prefix)
        left_action = {
            key.removeprefix("left_"): value for key, value in action.items() if key.startswith("left_")
        }

        # Extract right arm actions (remove "right_" prefix)
        right_action = {
            key.removeprefix("right_"): value for key, value in action.items() if key.startswith("right_")
        }

        # Send actions to both arms
        send_action_left = self.left_arm.send_action(left_action)
        send_action_right = self.right_arm.send_action(right_action)

        # Add prefixes back to return values
        prefixed_send_action_left = {f"left_{key}": value for key, value in send_action_left.items()}
        prefixed_send_action_right = {f"right_{key}": value for key, value in send_action_right.items()}

        return {**prefixed_send_action_left, **prefixed_send_action_right}

    def disconnect(self):
        """Disconnect from both arms and cameras."""
        self.left_arm.disconnect()
        self.right_arm.disconnect()

        for cam in self.cameras.values():
            cam.disconnect()

        logger.info(f"{self} disconnected.")
