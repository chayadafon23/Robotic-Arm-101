"""Check a physical camera and, optionally, read a calibrated SO-101 follower."""

import argparse
import os

import numpy as np

from demo.auto_sort import COLORS, detect_color_regions


JOINTS = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
)


def read_camera(camera_index: int, width: int, height: int) -> np.ndarray:
    import cv2

    backend = cv2.CAP_DSHOW if os.name == "nt" else cv2.CAP_ANY
    camera = cv2.VideoCapture(camera_index, backend)
    if not camera.isOpened():
        camera.release()
        raise RuntimeError(f"Could not open camera index {camera_index}")
    try:
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        frame = None
        for _ in range(10):
            ok, bgr = camera.read()
            if ok:
                frame = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        if frame is None:
            raise RuntimeError(f"Camera index {camera_index} opened but returned no frames")
        return frame
    finally:
        camera.release()


def read_follower(port: str, robot_id: str) -> dict[str, float]:
    from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

    robot = SO101Follower(SO101FollowerConfig(port=port, id=robot_id))
    if not robot.calibration_fpath.is_file():
        raise RuntimeError(
            f"No saved calibration for robot id {robot_id!r}; "
            "run lerobot-calibrate before checking the arm"
        )
    try:
        robot.connect(calibrate=False)
        if not robot.is_calibrated:
            raise RuntimeError("The connected SO-101 is not calibrated; run lerobot-calibrate")
        observation = robot.get_observation()
        return {joint: float(observation[f"{joint}.pos"]) for joint in JOINTS}
    finally:
        if robot.is_connected:
            robot.disconnect()


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Check the real camera and optionally read SO-101 joint positions; never sends motion targets."
    )
    parser.add_argument("--camera-index", type=int, required=True)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--check-arm", action="store_true", help="connect to a calibrated follower and read joints")
    parser.add_argument("--port", help="SO-101 follower serial port, for example COM5")
    parser.add_argument("--robot-id", default="so101_follower")
    args = parser.parse_args()
    if args.width < 1 or args.height < 1:
        parser.error("--width and --height must be positive")
    if args.check_arm != (args.port is not None):
        parser.error("--check-arm and --port must be specified together")

    image = read_camera(args.camera_index, args.width, args.height)
    regions = detect_color_regions(image)
    print(f"Camera {args.camera_index}: frame={image.shape[1]}x{image.shape[0]} RGB")
    for color in COLORS:
        detections = ", ".join(sorted(regions[color])) or "not detected"
        print(f"{color}: {detections}")

    if args.check_arm:
        positions = read_follower(args.port, args.robot_id)
        print("SO-101 joint positions (degrees; gripper 0-100):")
        for joint, position in positions.items():
            print(f"  {joint}: {position:.1f}")


if __name__ == "__main__":
    main()
