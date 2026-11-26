"""Color-based visual sorting for the SO-101 MuJoCo tabletop."""

import argparse
import base64
import os
import time
from contextlib import nullcontext

os.environ.setdefault("MUJOCO_GL", "glfw")

import mujoco
import mujoco.viewer
import numpy as np

from env import DEFAULTS, SO101Env

VISION_WIDTH = 160
VISION_HEIGHT = 120
# Processing order is fixed; this is a scripted controller, not a trained policy.
COLORS = ("red", "green", "blue", "yellow")
# Connected regions in this pixel-area range are treated as cubes; larger ones as boxes.
PIXEL_AREAS = {"cube": (8, 180), "box": (180, None)}
BOX_FLOOR_Z = 0.004
CUBE_TOP_Z = 0.03
APPROACH_HEIGHT = 0.07
BOX_APPROACH_HEIGHT = 0.05
CLEAR_X = 0.18
CLEAR_Z = 0.074
CAMERA_PARK = np.array([0.0, -30.0, 30.0, 90.0, 0.0, 30.0])


def decode_browser_frame(encoded: str) -> np.ndarray:
    """Decode the fixed-size bottom-up RGBA frame read from the browser's WebGL camera."""
    if not isinstance(encoded, str):
        raise ValueError("Vision frame must be a base64 string")
    expected = VISION_WIDTH * VISION_HEIGHT * 4
    if len(encoded) != 4 * ((expected + 2) // 3):
        raise ValueError("Vision frame has an unexpected encoded size")
    pixels = base64.b64decode(encoded, validate=True)
    if len(pixels) != expected:
        raise ValueError(f"Expected {expected} vision bytes, got {len(pixels)}")
    rgba = np.frombuffer(pixels, dtype=np.uint8).reshape(VISION_HEIGHT, VISION_WIDTH, 4)
    # WebGL readback starts at the bottom row; the controller expects top-down RGB.
    return rgba[::-1, :, :3].copy()


def _components(mask: np.ndarray) -> list[tuple[int, float, float]]:
    """Clean a binary color mask and return each connected region's area and centroid."""
    height, width = mask.shape
    padded = np.pad(mask, 1)
    neighbors = sum(
        padded[dy : dy + height, dx : dx + width]
        for dy in range(3)
        for dx in range(3)
    )
    mask = neighbors >= 4
    seen = np.zeros(mask.shape, dtype=bool)
    components = []
    for y, x in zip(*np.nonzero(mask)):
        if seen[y, x]:
            continue
        seen[y, x] = True
        stack = [(int(y), int(x))]
        area = sum_x = sum_y = 0
        while stack:
            row, col = stack.pop()
            area += 1
            sum_x += col
            sum_y += row
            for next_row, next_col in (
                (row - 1, col),
                (row + 1, col),
                (row, col - 1),
                (row, col + 1),
            ):
                if (
                    0 <= next_row < height
                    and 0 <= next_col < width
                    and mask[next_row, next_col]
                    and not seen[next_row, next_col]
                ):
                    seen[next_row, next_col] = True
                    stack.append((next_row, next_col))
        components.append((area, sum_x / area, sum_y / area))
    return components


def detect_color_regions(image: np.ndarray) -> dict[str, dict[str, tuple[float, float, int]]]:
    """Find small cube-colored regions and large matching box regions in an RGB image."""
    rgb = np.asarray(image)
    if rgb.ndim != 3 or rgb.shape[2] < 3 or not rgb.size:
        raise ValueError("Vision image must be a non-empty RGB or RGBA array")
    # Downsample before color segmentation so camera resolution does not dominate runtime.
    stride_y = max(1, int(np.ceil(rgb.shape[0] / VISION_HEIGHT)))
    stride_x = max(1, int(np.ceil(rgb.shape[1] / VISION_WIDTH)))
    rgb = rgb[::stride_y, ::stride_x, :3].astype(np.float32) / 255.0
    red, green, blue = rgb.transpose(2, 0, 1)
    maximum = np.maximum(np.maximum(red, green), blue)
    minimum = np.minimum(np.minimum(red, green), blue)
    delta = maximum - minimum
    hue = np.zeros_like(maximum)
    nonzero = delta > 1e-6
    red_max = nonzero & (maximum == red)
    green_max = nonzero & (maximum == green)
    blue_max = nonzero & (maximum == blue)
    hue[red_max] = ((green[red_max] - blue[red_max]) / delta[red_max]) % 6
    hue[green_max] = (blue[green_max] - red[green_max]) / delta[green_max] + 2
    hue[blue_max] = (red[blue_max] - green[blue_max]) / delta[blue_max] + 4
    hue *= 60
    saturation = np.divide(delta, maximum, out=np.zeros_like(delta), where=maximum > 1e-6)
    vivid = (saturation > 0.32) & (maximum > 0.12)
    # Hue intervals distinguish the four known object colors under the simulated lighting.
    ranges = {
        "red": (hue < 22) | (hue >= 340),
        "yellow": (hue >= 24) & (hue <= 72),
        "green": (hue >= 76) & (hue <= 166),
        "blue": (hue >= 195) & (hue <= 275),
    }
    found = {color: {} for color in COLORS}
    for color, hue_range in ranges.items():
        components = _components(vivid & hue_range)
        cubes = [item for item in components if PIXEL_AREAS["cube"][0] <= item[0] <= PIXEL_AREAS["cube"][1]]
        boxes = [item for item in components if item[0] > PIXEL_AREAS["box"][0]]
        if cubes:
            # Use the largest region of the expected size to reject small color noise.
            area, x, y = max(cubes, key=lambda item: item[0])
            found[color]["cube"] = ((x + 0.5) * stride_x - 0.5, (y + 0.5) * stride_y - 0.5, area)
        if boxes:
            area, x, y = max(boxes, key=lambda item: item[0])
            found[color]["box"] = ((x + 0.5) * stride_x - 0.5, (y + 0.5) * stride_y - 0.5, area)
    return found


class AutoSortController:
    """Scripted pick-and-place state machine driven by top-camera color regions."""

    def __init__(
        self,
        model,
        controller,
        *,
        pixel_mapper=None,
        pose_validator=None,
        table_z=0.0,
        cube_top_offset=CUBE_TOP_Z,
        cube_center_offset=0.015,
        box_floor_offset=BOX_FLOOR_Z,
        box_center_offset=0.022,
        approach_height=APPROACH_HEIGHT,
        box_approach_height=BOX_APPROACH_HEIGHT,
        clear_x=CLEAR_X,
        clear_z=CLEAR_Z,
        camera_park=CAMERA_PARK,
        colors=COLORS,
        repark_before_find=False,
    ):
        self.model = model
        self.controller = controller
        self.camera = model.camera("top").id
        self.fovy = float(model.cam_fovy[self.camera])
        self.pixel_mapper = pixel_mapper
        self.pose_validator = pose_validator
        self.table_z = float(table_z)
        self.cube_top_z = self.table_z + float(cube_top_offset)
        self.cube_center_z = self.table_z + float(cube_center_offset)
        self.box_floor_z = self.table_z + float(box_floor_offset)
        self.box_center_z = self.table_z + float(box_center_offset)
        self.approach_height = float(approach_height)
        self.box_approach_height = float(box_approach_height)
        self.clear_x = float(clear_x)
        self.clear_z = float(clear_z)
        self.camera_park = np.asarray(camera_park, dtype=float).copy()
        if self.camera_park.shape != (6,):
            raise ValueError("camera_park must contain six joint positions")
        self.colors = tuple(colors)
        if not self.colors or any(color not in COLORS for color in self.colors):
            raise ValueError(f"colors must be a non-empty subset of {COLORS}")
        self.repark_before_find = bool(repark_before_find)
        self.reset()

    def reset(self):
        self.running = False
        self.stage = "idle"
        self.index = 0
        self.elapsed = 0.0
        self.cube_xy = None
        self.box_xy = None
        self.object_offset = None
        self.regions = None
        self.image_shape = None
        self.joint_target = None
        self.gripper_target = 30.0
        self.grasp_orientation = None
        self.status = "Manual control"

    def start(self):
        self.running = True
        self.stage = "park"
        self.index = 0
        self.elapsed = 0.0
        self.cube_xy = None
        self.box_xy = None
        self.object_offset = None
        self.regions = None
        self.image_shape = None
        self.joint_target = self.camera_park.copy()
        self.gripper_target = 30.0
        self.grasp_orientation = None
        self.controller.last_axes = None
        self.status = "Moving to a camera-clear position"

    def stop(self):
        self.running = False
        self.stage = "idle"
        self.controller.last_axes = None
        self.status = "Automatic sorting stopped"

    def _pixel_to_world(self, pixel, plane_z, width, height):
        """Project an image pixel onto a known horizontal plane using camera calibration."""
        if self.pixel_mapper is not None:
            return np.asarray(self.pixel_mapper(pixel, plane_z, width, height), dtype=float)
        x, y = pixel
        camera_pos = self.model.cam_pos[self.camera]
        rotation = self.model.cam_mat0[self.camera].reshape(3, 3)
        tangent = np.tan(np.deg2rad(self.fovy) / 2)
        local_ray = np.array(
            [
                (2 * (x + 0.5) / width - 1) * width / height * tangent,
                (1 - 2 * (y + 0.5) / height) * tangent,
                -1.0,
            ]
        )
        ray = rotation @ local_ray
        distance = (plane_z - camera_pos[2]) / ray[2]
        return (camera_pos + distance * ray)[:2]

    def _tip(self, measured):
        """Update cached kinematics from measured joints and return the gripper-tip position."""
        q = np.deg2rad(np.asarray(measured, dtype=float))
        q[5] = self.controller.lo[5] + measured[5] / 100 * (
            self.controller.hi[5] - self.controller.lo[5]
        )
        self.controller._kinematics(q)
        return self.controller.data.site_xpos[self.controller.site].copy()

    def _grasp_offset(self):
        body = self.model.body("gripper").id
        data = self.controller.data
        body_rotation = data.xmat[body].reshape(3, 3)
        site_rotation = data.site_xmat[self.controller.site].reshape(3, 3)
        body_to_object = np.array([0.007, -0.000218, -0.088])
        object_from_site_world = (
            body_rotation @ body_to_object
            - (data.site_xpos[self.controller.site] - data.xpos[body])
        )
        return site_rotation.T @ object_from_site_world

    def _solve_pose(self, target, measured):
        """Find a bounded joint target that reaches a Cartesian pose near the current posture."""
        target = np.asarray(target, dtype=float)
        if self.pose_validator is not None and not self.pose_validator(target):
            raise ValueError(f"Target pose is outside the configured workspace: {target.tolist()}")
        controller = self.controller
        gripper_q = controller.lo[5] + measured[5] / 100 * (
            controller.hi[5] - controller.lo[5]
        )
        pan = np.clip(
            -1.4 * np.rad2deg(np.arctan2(target[1], target[0])),
            -85,
            85,
        )
        # Try the measured posture first, then alternate IK seeds to avoid local failures.
        seeds = (
            np.asarray(measured[:5], dtype=float),
            [pan, 25, -25, 90, 0],
            [pan, 25, -5, 70, 0],
            [pan, 15, 5, 70, 0],
            [pan, -65, 75, 80, 0],
            [pan, -30, 55, 65, 0],
        )
        best_q = None
        best_cost = np.inf
        for seed in seeds:
            q = np.deg2rad(np.r_[seed, measured[5]])
            q[5] = gripper_q
            q[:5] = np.clip(q[:5], controller.lo[:5], controller.hi[:5])
            controller._kinematics(q)
            basis = controller.jac_rot[:, controller.dofs[3:5]].T.copy()

            def residual():
                rotation = controller.data.site_xmat[controller.site].reshape(3, 3)
                angular = 0.5 * np.cross(
                    rotation.T, self.grasp_orientation.T
                ).sum(axis=0)
                angular = basis.T @ (basis @ angular)
                return np.r_[
                    target - controller.data.site_xpos[controller.site],
                    0.1 * (basis @ angular),
                ]

            for _ in range(80):
                current_error = residual()
                if np.linalg.norm(current_error[:3]) < 0.001:
                    break
                matrix = np.vstack(
                    [
                        controller.jac_pos[:, controller.dofs],
                        0.1 * basis @ controller.jac_rot[:, controller.dofs],
                    ]
                )
                max_step = np.deg2rad(5)
                step = controller._bounded_solve(
                    matrix,
                    current_error,
                    np.maximum(controller.lo[:5] - q[:5], -max_step),
                    np.minimum(controller.hi[:5] - q[:5], max_step),
                )
                if np.max(np.abs(step)) < 1e-8:
                    break
                old_cost = current_error @ current_error
                accepted = False
                for scale in (1, 0.5, 0.25, 0.125):
                    candidate = q.copy()
                    candidate[:5] += scale * step
                    controller._kinematics(candidate)
                    candidate_error = residual()
                    if candidate_error @ candidate_error < old_cost:
                        q = candidate
                        accepted = True
                        break
                if not accepted:
                    break

            final_error = residual()
            position_error = np.linalg.norm(final_error[:3])
            orientation_error = np.linalg.norm(final_error[3:])
            if position_error > 0.003 or orientation_error > 0.01:
                continue
            cost = np.linalg.norm(np.rad2deg(q[:5]) - measured[:5])
            if cost < best_cost:
                best_q = q.copy()
                best_cost = cost
        if best_q is None:
            return None
        return np.r_[np.rad2deg(best_q[:5]), self.gripper_target]

    def _next(self):
        self.index += 1
        self.elapsed = 0
        self.cube_xy = None
        self.box_xy = None
        self.object_offset = None
        if self.index == len(self.colors):
            self.running = False
            self.stage = "done"
            self.status = "All configured colors sorted"
        else:
            self.stage = "park" if self.repark_before_find else "find"
            self.status = (
                "Moving to a camera-clear position"
                if self.repark_before_find
                else f"Looking for the {self.colors[self.index]} cube"
            )

    def update(self, image, measured, dt):
        """Advance one state-machine tick and return manual jog axes plus status."""
        axes = np.zeros(6)
        if not self.running:
            return axes, self.status
        self._tip(measured)
        if image is not None:
            # Refresh detections only when a camera frame is supplied by the caller.
            self.regions = detect_color_regions(image)
            self.image_shape = image.shape[:2]
        if self.regions is None and self.stage != "park":
            return axes, "Waiting for the top-camera image"
        regions = self.regions or {}
        color = self.colors[self.index]
        if self.stage == "find":
            cube = regions[color].get("cube")
            box = regions[color].get("box")
            if cube is None or box is None:
                self.status = f"Looking for the {color} cube and box"
                return axes, self.status
            height, width = self.image_shape
            self.cube_xy = self._pixel_to_world(cube[:2], self.cube_top_z, width, height)
            self.box_xy = self._pixel_to_world(box[:2], self.box_floor_z, width, height)
            self.object_offset = self._grasp_offset()
            self.stage = "align_pick"
            self.elapsed = 0

        # Fail closed if a pick/place sequence cannot make progress.
        self.elapsed += dt
        if self.elapsed > 20:
            self.running = False
            self.status = f"Could not reach the {color} cube; sorting stopped"
            return axes, self.status

        positions = {}
        if (
            self.cube_xy is not None
            and self.object_offset is not None
            and self.grasp_orientation is not None
        ):
            offset = self.grasp_orientation @ self.object_offset
            cube_center = np.array([*self.cube_xy, self.cube_center_z])
            positions.update(
                {
                    "align_pick": np.array([self.clear_x, self.cube_xy[1], self.clear_z]),
                    "above_pick": cube_center + [0, 0, self.approach_height] - offset,
                    "grasp": cube_center - offset,
                    "lift": cube_center + [0, 0, self.approach_height] - offset,
                    "transfer_box": np.array([self.clear_x, self.cube_xy[1], self.clear_z]),
                }
            )
        if (
            self.box_xy is not None
            and self.object_offset is not None
            and self.grasp_orientation is not None
        ):
            offset = self.grasp_orientation @ self.object_offset
            box_center = np.array([*self.box_xy, self.box_center_z])
            positions.update(
                {
                    "align_box": np.array([self.clear_x, self.box_xy[1], self.clear_z]),
                    "above_box": box_center + [0, 0, self.box_approach_height] - offset,
                    "place": box_center - offset,
                    "retreat": box_center + [0, 0, self.box_approach_height] - offset,
                }
            )
        transitions = {
            "align_pick": "above_pick",
            "above_pick": "grasp",
            "grasp": "close",
            "lift": "transfer_box",
            "transfer_box": "align_box",
            "align_box": "above_box",
            "above_box": "place",
            "retreat": "open_clear",
            "place": "open",
        }
        if self.stage in positions:
            if self.joint_target is None:
                self.joint_target = self._solve_pose(positions[self.stage], measured)
                if self.joint_target is None:
                    self.running = False
                    self.status = f"Could not solve a reachable pose for the {color} cube"
                    return axes, self.status
                self.controller.last_axes = None
            if np.max(np.abs(np.asarray(measured[:5]) - self.joint_target[:5])) < 1.5:
                self.joint_target = None
                self.stage = transitions[self.stage]
                self.elapsed = 0
                if self.stage == "close":
                    self.gripper_target = 0
        elif self.stage == "park":
            self.joint_target = self.camera_park.copy()
            if np.max(np.abs(np.asarray(measured[:5]) - self.camera_park[:5])) < 1.5:
                self.joint_target = None
                self.grasp_orientation = self.controller.data.site_xmat[
                    self.controller.site
                ].reshape(3, 3).copy()
                self.stage = "find"
                self.elapsed = 0
                self.status = f"Looking for the {color} cube"
                return axes, self.status
        elif self.stage == "close":
            # Gripper movement is issued through the jog interface until its measured position closes.
            if measured[5] > 20:
                axes[5] = -1
            else:
                self.stage = "lift"
                self.elapsed = 0
        elif self.stage == "open":
            if measured[5] < 40:
                axes[5] = 1
            else:
                self.stage = "retreat"
                self.gripper_target = 100
                self.elapsed = 0
        elif self.stage == "open_clear":
            if measured[5] < 96:
                axes[5] = 1
            else:
                self.stage = "settle"
                self.elapsed = 0
        elif self.stage == "settle":
            if self.elapsed >= 0.7:
                self._next()
        self.status = (
            "All configured colors sorted"
            if self.stage == "done"
            else (
                "Setting the gripper camera pose"
                if self.stage == "park"
                else f"{color.capitalize()}: {self.stage.replace('_', ' ')}"
            )
        )
        return axes, self.status


def run_episode(seed: int, max_steps: int, show_viewer: bool = False) -> bool:
    """Run the same sorter headlessly or synchronized with MuJoCo's native viewer."""
    from demo.app import KeyboardController

    env = SO101Env(
        cameras={"top": "top"},
        observation_height=VISION_HEIGHT,
        observation_width=VISION_WIDTH,
        randomize=DEFAULTS["randomize"],
    )
    try:
        env.reset(seed=seed)
        world = env._ensure_world()
        controller = KeyboardController(world.model, ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"))
        sorter = AutoSortController(world.model, controller)
        sorter.start()
        target = world.joint_state().astype(float)
        viewer_context = (
            mujoco.viewer.launch_passive(world.model, world.data)
            if show_viewer
            else nullcontext(None)
        )
        with viewer_context as viewer:
            if viewer is not None:
                viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
                viewer.cam.fixedcamid = world.model.camera("top").id
                viewer.sync()
            for step in range(max_steps):
                if viewer is not None and not viewer.is_running():
                    return False
                step_started = time.perf_counter()
                measured = world.joint_state()
                # Vision is only needed when the controller is searching for the next color.
                image = world.render("top") if sorter.stage == "find" else None
                axes, status = sorter.update(image, measured, 1 / 30)
                if sorter.joint_target is not None:
                    max_step = np.array([5.0, 5.0, 5.0, 5.0, 5.0, 12.0])
                    target += np.clip(sorter.joint_target - target, -max_step, max_step)
                    controller.last_axes = None
                else:
                    target, _ = controller.jog(measured, target[5], axes, 1 / 30)
                # The sorter outputs joint targets; the environment converts them to actuator units.
                world.data.ctrl[world.actuators] = world.to_ctrl(target)
                mujoco.mj_step(world.model, world.data, nstep=world.substeps)
                reward = world.cubes_in_box() / len(COLORS)
                terminated = reward == 1
                truncated = step + 1 >= max_steps
                if viewer is not None:
                    viewer.sync()
                    time.sleep(max(0, 1 / 30 - (time.perf_counter() - step_started)))
                if step % 150 == 0:
                    print(f"step={step:4d} reward={reward:.2f} {status}", flush=True)
                if terminated or truncated:
                    print(
                        f"finished after {step + 1} steps: "
                        f"{world.cubes_in_box()}/4 cubes in matching boxes",
                        flush=True,
                    )
                    return bool(terminated)
            print(f"stopped at max_steps={max_steps}; reward={reward:.2f}", flush=True)
            return False
    finally:
        env.close()


def main():
    """Parse CLI options and start a seeded sorting episode."""
    parser = argparse.ArgumentParser(description="Sort the MuJoCo tabletop cubes using top-camera color vision.")
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--max-steps", type=int, default=3600)
    parser.add_argument(
        "--viewer",
        action="store_true",
        help="show the automatic sorting episode in the MuJoCo viewer",
    )
    args = parser.parse_args()
    if args.max_steps < 1:
        parser.error("--max-steps must be positive")
    run_episode(args.seed, args.max_steps, show_viewer=args.viewer)


if __name__ == "__main__":
    main()
