"""Safely gated color sorting for a calibrated physical SO-101 follower.

The command defaults to a vision-only dry run. Motor commands are sent only when
``--execute`` is present and the hardware configuration has ``configured: true``.
"""

from __future__ import annotations

import argparse
import json
import os
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

from demo.auto_sort import AutoSortController, COLORS, detect_color_regions
from env import HERE, JOINTS, SCENE_FILE, build_model


def _array(value: Any, shape: tuple[int, ...], name: str) -> np.ndarray:
    array = np.asarray(value, dtype=float)
    if array.shape != shape or not np.all(np.isfinite(array)):
        raise ValueError(f"{name} must contain finite numbers with shape {shape}")
    return array


def _range(value: Any, name: str) -> np.ndarray:
    limits = _array(value, (2,), name)
    if limits[0] >= limits[1]:
        raise ValueError(f"{name} minimum must be smaller than its maximum")
    return limits


def fit_homography(image_points: np.ndarray, robot_points: np.ndarray) -> np.ndarray:
    """Fit a projective mapping from image pixels to the robot's table XY plane."""
    image_points = np.asarray(image_points, dtype=float)
    robot_points = np.asarray(robot_points, dtype=float)
    if image_points.ndim != 2 or image_points.shape[1:] != (2,) or len(image_points) < 4:
        raise ValueError("calibration.image_points_px must contain at least four [x, y] points")
    if robot_points.shape != image_points.shape or not np.all(np.isfinite(robot_points)):
        raise ValueError("calibration.robot_points_m must match image_points_px")
    if not np.all(np.isfinite(image_points)):
        raise ValueError("calibration.image_points_px must contain finite numbers")

    rows = []
    for (u, v), (x, y) in zip(image_points, robot_points):
        rows.append([-u, -v, -1, 0, 0, 0, u * x, v * x, x])
        rows.append([0, 0, 0, -u, -v, -1, u * y, v * y, y])
    _, singular_values, vectors = np.linalg.svd(np.asarray(rows, dtype=float))
    if singular_values[-2] < 1e-10:
        raise ValueError("calibration points are degenerate; use four separated table points")
    homography = vectors[-1].reshape(3, 3)
    scale = homography[2, 2]
    if abs(scale) < 1e-12:
        scale = np.linalg.norm(homography)
    return homography / scale


@dataclass(frozen=True)
class CameraCalibration:
    image_width: int
    image_height: int
    homography: np.ndarray
    camera_position: np.ndarray

    @classmethod
    def load(cls, data: Any, label: str) -> "CameraCalibration":
        if not isinstance(data, dict):
            raise ValueError(f"{label} must be a camera calibration object")
        image_size = data.get("image_size_px")
        if not isinstance(image_size, list) or len(image_size) != 2:
            raise ValueError(f"{label}.image_size_px must be [width, height]")
        image_width, image_height = (int(image_size[0]), int(image_size[1]))
        if image_width < 1 or image_height < 1:
            raise ValueError(f"{label}.image_size_px values must be positive")
        calibration = data.get("calibration", {})
        homography = fit_homography(
            np.asarray(calibration.get("image_points_px"), dtype=float),
            np.asarray(calibration.get("robot_points_m"), dtype=float),
        )
        camera_position = _array(
            data.get("camera_position_m"), (3,), f"{label}.camera_position_m"
        )
        return cls(image_width, image_height, homography, camera_position)

    def pixel_to_table(self, pixel, width: int, height: int) -> np.ndarray:
        if (width, height) != (self.image_width, self.image_height):
            raise ValueError(
                f"Camera returned {width}x{height}; calibration requires "
                f"{self.image_width}x{self.image_height}"
            )
        ray_on_table = self.homography @ np.array([pixel[0], pixel[1], 1.0])
        if abs(ray_on_table[2]) < 1e-12:
            raise ValueError("Pixel cannot be mapped by the camera calibration")
        result = ray_on_table[:2] / ray_on_table[2]
        if not np.all(np.isfinite(result)):
            raise ValueError("Camera calibration produced a non-finite position")
        return result

    def pixel_to_robot(self, pixel, plane_z: float, width: int, height: int, table_z: float) -> np.ndarray:
        table_xy = self.pixel_to_table(pixel, width, height)
        scale = (self.camera_position[2] - plane_z) / (
            self.camera_position[2] - table_z
        )
        result = self.camera_position[:2] + scale * (table_xy - self.camera_position[:2])
        if not np.all(np.isfinite(result)):
            raise ValueError("Camera calibration produced a non-finite position")
        return result


@dataclass(frozen=True)
class HardwareSortConfig:
    image_width: int
    image_height: int
    homography: np.ndarray
    camera_position: np.ndarray
    workspace_x: np.ndarray
    workspace_y: np.ndarray
    workspace_z: np.ndarray
    table_z: float
    cube_top_offset: float
    cube_center_offset: float
    box_floor_offset: float
    box_center_offset: float
    approach_height: float
    box_approach_height: float
    clear_x: float
    clear_z: float
    camera_park: np.ndarray
    fps: float
    max_joint_step: float
    max_gripper_step: float
    max_run_seconds: float
    secondary_camera: CameraCalibration | None = None
    camera_agreement_tolerance_m: float = 0.03

    @classmethod
    def load(cls, path: str | Path) -> "HardwareSortConfig":
        config_path = Path(path)
        try:
            data = json.loads(config_path.read_text(encoding="utf-8"))
        except FileNotFoundError as exc:
            raise ValueError(f"Hardware config not found: {config_path}") from exc
        except json.JSONDecodeError as exc:
            raise ValueError(f"Hardware config is not valid JSON: {exc}") from exc
        if data.get("configured") is not True:
            raise ValueError(
                "Hardware config is not approved: measure every value, then set configured to true"
            )

        primary_camera = CameraCalibration.load(data, "primary_camera")
        image_width = primary_camera.image_width
        image_height = primary_camera.image_height
        homography = primary_camera.homography
        camera_position = primary_camera.camera_position
        secondary_camera_data = data.get("secondary_camera")
        secondary_camera = (
            CameraCalibration.load(secondary_camera_data, "secondary_camera")
            if secondary_camera_data is not None
            else None
        )
        try:
            camera_agreement_tolerance_m = float(
                data.get("camera_agreement_tolerance_m", 0.03)
            )
        except (TypeError, ValueError) as exc:
            raise ValueError("camera_agreement_tolerance_m must be a finite number") from exc
        if not np.isfinite(camera_agreement_tolerance_m) or not (
            0 < camera_agreement_tolerance_m <= 0.1
        ):
            raise ValueError("camera_agreement_tolerance_m must be greater than 0 and at most 0.1 m")

        workspace = data.get("workspace_m", {})
        workspace_x = _range(workspace.get("x"), "workspace_m.x")
        workspace_y = _range(workspace.get("y"), "workspace_m.y")
        workspace_z = _range(workspace.get("z"), "workspace_m.z")

        geometry = data.get("geometry_m", {})
        required_geometry = (
            "table_z",
            "cube_top_offset",
            "cube_center_offset",
            "box_floor_offset",
            "box_center_offset",
            "approach_height",
            "box_approach_height",
            "clear_x",
            "clear_z",
        )
        values = {}
        for key in required_geometry:
            try:
                values[key] = float(geometry[key])
            except (KeyError, TypeError, ValueError) as exc:
                raise ValueError(f"geometry_m.{key} must be a finite number") from exc
            if not np.isfinite(values[key]):
                raise ValueError(f"geometry_m.{key} must be a finite number")
        if camera_position[2] <= values["table_z"]:
            raise ValueError("camera_position_m z must be above geometry_m.table_z")
        if secondary_camera is not None and secondary_camera.camera_position[2] <= values["table_z"]:
            raise ValueError("secondary_camera.camera_position_m z must be above geometry_m.table_z")
        for key in (
            "cube_top_offset",
            "cube_center_offset",
            "box_floor_offset",
            "box_center_offset",
            "approach_height",
            "box_approach_height",
        ):
            if values[key] < 0:
                raise ValueError(f"geometry_m.{key} cannot be negative")
        highest_vision_plane = values["table_z"] + max(
            values["cube_top_offset"], values["box_floor_offset"]
        )
        if highest_vision_plane >= camera_position[2]:
            raise ValueError("camera_position_m must be above every detected object plane")
        if secondary_camera is not None and highest_vision_plane >= secondary_camera.camera_position[2]:
            raise ValueError("secondary_camera.camera_position_m must be above every detected object plane")

        camera_park = _array(data.get("camera_park_degrees"), (6,), "camera_park_degrees")
        motion = data.get("motion", {})
        try:
            fps = float(motion["fps"])
            max_joint_step = float(motion["max_joint_step_degrees"])
            max_gripper_step = float(motion["max_gripper_step"])
            max_run_seconds = float(motion["max_run_seconds"])
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError("motion values are incomplete or invalid") from exc
        if not 1 <= fps <= 30:
            raise ValueError("motion.fps must be between 1 and 30")
        if not 0 < max_joint_step <= 5:
            raise ValueError("motion.max_joint_step_degrees must be greater than 0 and at most 5")
        if not 0 < max_gripper_step <= 10:
            raise ValueError("motion.max_gripper_step must be greater than 0 and at most 10")
        if not 10 <= max_run_seconds <= 600:
            raise ValueError("motion.max_run_seconds must be between 10 and 600")

        config = cls(
            image_width=image_width,
            image_height=image_height,
            homography=homography,
            camera_position=camera_position,
            workspace_x=workspace_x,
            workspace_y=workspace_y,
            workspace_z=workspace_z,
            table_z=values["table_z"],
            cube_top_offset=values["cube_top_offset"],
            cube_center_offset=values["cube_center_offset"],
            box_floor_offset=values["box_floor_offset"],
            box_center_offset=values["box_center_offset"],
            approach_height=values["approach_height"],
            box_approach_height=values["box_approach_height"],
            clear_x=values["clear_x"],
            clear_z=values["clear_z"],
            camera_park=camera_park,
            fps=fps,
            max_joint_step=max_joint_step,
            max_gripper_step=max_gripper_step,
            max_run_seconds=max_run_seconds,
            secondary_camera=secondary_camera,
            camera_agreement_tolerance_m=camera_agreement_tolerance_m,
        )
        if not config.contains(np.array([config.clear_x, 0.0, config.clear_z]), check_y=False):
            raise ValueError("geometry_m.clear_x/clear_z are outside the configured workspace")
        return config

    def contains(self, target: np.ndarray, *, check_y: bool = True) -> bool:
        target = np.asarray(target, dtype=float)
        return bool(
            target.shape == (3,)
            and np.all(np.isfinite(target))
            and self.workspace_x[0] <= target[0] <= self.workspace_x[1]
            and (not check_y or self.workspace_y[0] <= target[1] <= self.workspace_y[1])
            and self.workspace_z[0] <= target[2] <= self.workspace_z[1]
        )

    def pixel_to_robot(self, pixel, plane_z: float, width: int, height: int) -> np.ndarray:
        return CameraCalibration(
            self.image_width,
            self.image_height,
            self.homography,
            self.camera_position,
        ).pixel_to_robot(pixel, plane_z, width, height, self.table_z)

    def pixel_to_secondary_robot(
        self, pixel, plane_z: float, width: int, height: int
    ) -> np.ndarray:
        if self.secondary_camera is None:
            raise ValueError("No secondary camera calibration is configured")
        return self.secondary_camera.pixel_to_robot(
            pixel, plane_z, width, height, self.table_z
        )


class HardwareKinematics:
    """The minimum MuJoCo kinematics interface required by AutoSortController."""

    def __init__(self, model):
        self.model = model
        self.data = mujoco.MjData(model)
        self.qadr = np.array([model.joint(name).qposadr[0] for name in JOINTS])
        self.dofs = np.array([model.joint(name).dofadr[0] for name in JOINTS[:5]])
        actuators = np.array([model.actuator(name).id for name in JOINTS])
        self.lo, self.hi = model.actuator_ctrlrange[actuators].T.copy()
        joint_limits = np.array([model.joint(name).range for name in JOINTS])
        self.lo = np.maximum(self.lo, joint_limits[:, 0])
        self.hi = np.minimum(self.hi, joint_limits[:, 1])
        self.site = model.site("gripperframe").id
        self.jac_pos = np.zeros((3, model.nv))
        self.jac_rot = np.zeros_like(self.jac_pos)
        self.last_axes = None

    @staticmethod
    def _bounded_solve(matrix, residual, lower, upper):
        hessian = matrix.T @ matrix + np.eye(matrix.shape[1]) * 0.002**2
        gradient = matrix.T @ residual
        unconstrained = np.linalg.solve(hessian, gradient)
        if np.all(unconstrained >= lower) and np.all(unconstrained <= upper):
            return unconstrained
        step = np.clip(unconstrained, lower, upper)
        for _ in range(80):
            previous = step.copy()
            for joint in range(len(step)):
                correction = (gradient[joint] - hessian[joint] @ step) / hessian[joint, joint]
                step[joint] = np.clip(step[joint] + correction, lower[joint], upper[joint])
            if np.max(np.abs(step - previous)) < 1e-7:
                break
        return step

    def _kinematics(self, q):
        self.data.qpos[self.qadr] = q
        mujoco.mj_kinematics(self.model, self.data)
        mujoco.mj_comPos(self.model, self.data)
        mujoco.mj_jacSite(self.model, self.data, self.jac_pos, self.jac_rot, self.site)

    @property
    def public_limits(self) -> tuple[np.ndarray, np.ndarray]:
        low = np.r_[np.rad2deg(self.lo[:5]), 0.0]
        high = np.r_[np.rad2deg(self.hi[:5]), 100.0]
        return low, high


def _camera_size(config: HardwareSortConfig, camera_slot: str) -> tuple[int, int]:
    if camera_slot == "primary":
        return config.image_width, config.image_height
    if camera_slot == "secondary" and config.secondary_camera is not None:
        return config.secondary_camera.image_width, config.secondary_camera.image_height
    raise ValueError(f"No calibration configured for camera slot {camera_slot!r}")


def open_camera(index: int, config: HardwareSortConfig, camera_slot: str = "primary"):
    import cv2

    width, height = _camera_size(config, camera_slot)
    backend = cv2.CAP_DSHOW if os.name == "nt" else cv2.CAP_ANY
    camera = cv2.VideoCapture(index, backend)
    if not camera.isOpened():
        camera.release()
        raise RuntimeError(f"Could not open camera index {index}")
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    for _ in range(10):
        ok, _ = camera.read()
        if not ok:
            camera.release()
            raise RuntimeError(f"Camera index {index} stopped during warm-up")
    return camera


def read_frame(
    camera, config: HardwareSortConfig, camera_slot: str = "primary"
) -> np.ndarray:
    import cv2

    expected_width, expected_height = _camera_size(config, camera_slot)
    ok, bgr = camera.read()
    if not ok or bgr is None:
        raise RuntimeError("Camera stopped returning frames")
    if (bgr.shape[1], bgr.shape[0]) != (expected_width, expected_height):
        raise RuntimeError(
            f"Camera returned {bgr.shape[1]}x{bgr.shape[0]}; "
            f"expected {expected_width}x{expected_height}"
        )
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)


def report_dry_run(
    image: np.ndarray,
    config: HardwareSortConfig,
    colors: tuple[str, ...] = COLORS,
) -> bool:
    regions = detect_color_regions(image)
    complete = True
    for color in colors:
        items = []
        for kind in ("cube", "box"):
            region = regions[color].get(kind)
            if region is None:
                complete = False
                items.append(f"{kind}=not detected")
                continue
            plane_z = config.table_z + (
                config.cube_top_offset if kind == "cube" else config.box_floor_offset
            )
            xy = config.pixel_to_robot(region[:2], plane_z, image.shape[1], image.shape[0])
            inside = (
                config.workspace_x[0] <= xy[0] <= config.workspace_x[1]
                and config.workspace_y[0] <= xy[1] <= config.workspace_y[1]
            )
            complete &= inside
            items.append(f"{kind}=({xy[0]:.3f}, {xy[1]:.3f})m inside={inside}")
        print(f"{color}: " + ", ".join(items))
    return bool(complete)


def report_camera_agreement(
    primary_image: np.ndarray,
    secondary_image: np.ndarray,
    config: HardwareSortConfig,
    colors: tuple[str, ...] = COLORS,
) -> bool:
    """Check that both cameras locate every requested cube and box consistently."""
    complete = True
    for color in colors:
        agrees, status = camera_detections_agree(
            primary_image, secondary_image, config, color
        )
        print(f"{color}: {status}")
        complete &= agrees
    return bool(complete)


def camera_detections_agree(
    primary_image: np.ndarray,
    secondary_image: np.ndarray,
    config: HardwareSortConfig,
    color: str,
) -> tuple[bool, str]:
    """Require both fixed cameras to see matching objects at consistent robot coordinates."""
    if config.secondary_camera is None:
        raise ValueError("A secondary camera image requires a secondary camera calibration")
    primary = detect_color_regions(primary_image)[color]
    secondary = detect_color_regions(secondary_image)[color]
    for kind, offset in (
        ("cube", config.cube_top_offset),
        ("box", config.box_floor_offset),
    ):
        primary_region = primary.get(kind)
        secondary_region = secondary.get(kind)
        if primary_region is None or secondary_region is None:
            return False, f"{color} {kind} is not visible in both cameras"
        plane_z = config.table_z + offset
        primary_xy = config.pixel_to_robot(
            primary_region[:2], plane_z, primary_image.shape[1], primary_image.shape[0]
        )
        secondary_xy = config.pixel_to_secondary_robot(
            secondary_region[:2],
            plane_z,
            secondary_image.shape[1],
            secondary_image.shape[0],
        )
        error = float(np.linalg.norm(primary_xy - secondary_xy))
        if error > config.camera_agreement_tolerance_m:
            return (
                False,
                f"{color} {kind} camera positions disagree by {error:.3f} m "
                f"(limit {config.camera_agreement_tolerance_m:.3f} m)",
            )
    return True, f"{color} cube and box agree across cameras"


def _joint_state(observation: dict[str, Any]) -> np.ndarray:
    measured = np.array([observation[f"{joint}.pos"] for joint in JOINTS], dtype=float)
    if not np.all(np.isfinite(measured)):
        raise RuntimeError("Motor observation contains a non-finite joint position")
    return measured


def run_hardware_sort(
    camera,
    config: HardwareSortConfig,
    port: str,
    robot_id: str,
    colors: tuple[str, ...] = COLORS,
    secondary_camera=None,
) -> bool:
    from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

    model = build_model(HERE / SCENE_FILE, HERE)
    kinematics = HardwareKinematics(model)
    low, high = kinematics.public_limits
    if np.any(config.camera_park < low) or np.any(config.camera_park > high):
        raise ValueError("camera_park_degrees exceeds the SO-101 model joint limits")

    sorter = AutoSortController(
        model,
        kinematics,
        pixel_mapper=config.pixel_to_robot,
        pose_validator=config.contains,
        table_z=config.table_z,
        cube_top_offset=config.cube_top_offset,
        cube_center_offset=config.cube_center_offset,
        box_floor_offset=config.box_floor_offset,
        box_center_offset=config.box_center_offset,
        approach_height=config.approach_height,
        box_approach_height=config.box_approach_height,
        clear_x=config.clear_x,
        clear_z=config.clear_z,
        camera_park=config.camera_park,
        colors=colors,
        repark_before_find=True,
    )
    relative_limits = {joint: config.max_joint_step for joint in JOINTS[:5]}
    relative_limits["gripper"] = config.max_gripper_step
    robot = SO101Follower(
        SO101FollowerConfig(
            port=port,
            id=robot_id,
            max_relative_target=relative_limits,
            disable_torque_on_disconnect=True,
        )
    )
    if not robot.calibration_fpath.is_file():
        raise RuntimeError(
            f"No saved calibration for robot id {robot_id!r}; run lerobot-calibrate first"
        )
    if (config.secondary_camera is None) != (secondary_camera is None):
        raise ValueError("A second camera device and secondary camera calibration must be supplied together")

    print("Motor execution enabled. Keep the emergency power cut-off within reach.")
    print("The arm starts in 5 seconds; press Ctrl+C now to cancel.")
    for remaining in range(5, 0, -1):
        print(f"Starting in {remaining}...", flush=True)
        time.sleep(1)

    last_status = None
    period = 1.0 / config.fps
    started = time.monotonic()
    try:
        robot.connect(calibrate=False)
        if not robot.is_calibrated:
            raise RuntimeError("The connected SO-101 is not calibrated")
        sorter.start()
        while sorter.running:
            loop_started = time.monotonic()
            if loop_started - started > config.max_run_seconds:
                raise RuntimeError("Automatic sorting exceeded motion.max_run_seconds")
            measured = _joint_state(robot.get_observation())
            if np.any(measured < low - 2) or np.any(measured > high + 2):
                raise RuntimeError("Measured joint position is outside the SO-101 model limits")

            live_image = read_frame(camera, config)
            secondary_image = (
                read_frame(secondary_camera, config, "secondary")
                if secondary_camera is not None
                else None
            )
            if secondary_image is not None and sorter.stage == "find":
                agrees, agreement_status = camera_detections_agree(
                    live_image,
                    secondary_image,
                    config,
                    sorter.colors[sorter.index],
                )
                if not agrees:
                    if agreement_status != last_status:
                        print(f"Waiting for camera agreement: {agreement_status}", flush=True)
                        last_status = agreement_status
                    time.sleep(period)
                    continue
            image = live_image if sorter.stage == "find" else None
            axes, status = sorter.update(image, measured, period)
            if status != last_status:
                print(status, flush=True)
                last_status = status

            if sorter.joint_target is not None:
                desired = np.asarray(sorter.joint_target, dtype=float)
            else:
                desired = measured.copy()
                desired[5] += axes[5] * config.max_gripper_step
            if np.any(desired < low) or np.any(desired > high):
                raise RuntimeError("Controller generated a joint target outside the model limits")

            step_limits = np.r_[
                np.full(5, config.max_joint_step),
                config.max_gripper_step,
            ]
            command = measured + np.clip(desired - measured, -step_limits, step_limits)
            robot.send_action(
                {f"{joint}.pos": float(value) for joint, value in zip(JOINTS, command)}
            )
            time.sleep(max(0.0, period - (time.monotonic() - loop_started)))

        print(sorter.status, flush=True)
        return sorter.stage == "done"
    finally:
        if robot.is_connected:
            robot.disconnect()
        print("Motor torque is disabled on disconnect; support the arm before it drops.")


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Sort colored cubes with a calibrated physical SO-101. "
            "Without --execute this command only checks vision and mapped coordinates."
        )
    )
    parser.add_argument("--config", required=True, help="measured hardware JSON configuration")
    parser.add_argument("--camera-index", type=int, required=True)
    parser.add_argument(
        "--camera-index-2",
        type=int,
        help="second fixed overhead camera; requires secondary_camera calibration in the config",
    )
    parser.add_argument("--port", help="SO-101 follower serial port, required with --execute")
    parser.add_argument("--robot-id", default="so101_follower")
    parser.add_argument(
        "--max-colors",
        type=int,
        choices=range(1, len(COLORS) + 1),
        default=1,
        help="sort this many colors in red/green/blue/yellow order; defaults to one for first tests",
    )
    parser.add_argument(
        "--execute",
        action="store_true",
        help="allow calibrated joint targets to be sent to the physical arm",
    )
    args = parser.parse_args()
    if args.camera_index < 0:
        parser.error("--camera-index cannot be negative")
    if args.camera_index_2 is not None and args.camera_index_2 < 0:
        parser.error("--camera-index-2 cannot be negative")
    if args.camera_index_2 == args.camera_index:
        parser.error("--camera-index and --camera-index-2 must refer to different cameras")
    if args.execute and not args.port:
        parser.error("--port is required with --execute")
    if not args.execute and args.port:
        parser.error("--port is only used with --execute")

    try:
        config = HardwareSortConfig.load(args.config)
    except ValueError as exc:
        parser.error(str(exc))
    if (args.camera_index_2 is None) != (config.secondary_camera is None):
        parser.error(
            "Configure secondary_camera and provide --camera-index-2 together, "
            "or omit both to use one camera"
        )

    camera = None
    secondary_camera = None
    try:
        camera = open_camera(args.camera_index, config)
        if args.camera_index_2 is not None:
            secondary_camera = open_camera(args.camera_index_2, config, "secondary")
        colors = COLORS[: args.max_colors]
        initial = read_frame(camera, config)
        ready = report_dry_run(initial, config, colors)
        if secondary_camera is not None:
            secondary_initial = read_frame(secondary_camera, config, "secondary")
            ready = (
                report_camera_agreement(initial, secondary_initial, config, colors)
                and ready
            )
        if not ready:
            raise RuntimeError(
                "Vision/calibration preflight failed; all cubes and boxes must be detected inside the workspace"
            )
        if not args.execute:
            print("Dry run passed. No motor commands were sent.")
            return
        success = run_hardware_sort(
            camera,
            config,
            args.port,
            args.robot_id,
            colors,
            secondary_camera,
        )
        if not success:
            raise RuntimeError("Automatic sorting stopped before completing all colors")
    finally:
        if camera is not None:
            camera.release()
        if secondary_camera is not None:
            secondary_camera.release()


if __name__ == "__main__":
    main()
