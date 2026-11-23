"""Capture pixel correspondences for a fixed overhead hardware camera."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path

import numpy as np


def load_click_config(path: Path):
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise ValueError(f"Config not found: {path}") from exc
    except json.JSONDecodeError as exc:
        raise ValueError(f"Config is not valid JSON: {exc}") from exc
    size = data.get("image_size_px")
    if not isinstance(size, list) or len(size) != 2:
        raise ValueError("image_size_px must be [width, height]")
    width, height = int(size[0]), int(size[1])
    points = np.asarray(data.get("calibration", {}).get("robot_points_m"), dtype=float)
    if points.ndim != 2 or points.shape[1:] != (2,) or len(points) < 4:
        raise ValueError("Fill calibration.robot_points_m with at least four measured [x, y] points first")
    if not np.all(np.isfinite(points)):
        raise ValueError("calibration.robot_points_m still contains null or non-numeric values")
    return data, width, height, points


def capture_points(camera_index: int, width: int, height: int, robot_points: np.ndarray):
    import cv2

    backend = cv2.CAP_DSHOW if os.name == "nt" else cv2.CAP_ANY
    camera = cv2.VideoCapture(camera_index, backend)
    if not camera.isOpened():
        camera.release()
        raise RuntimeError(f"Could not open camera index {camera_index}")
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    frame = None
    try:
        for _ in range(15):
            ok, current = camera.read()
            if ok:
                frame = current
        if frame is None:
            raise RuntimeError("Camera opened but returned no frames")
    finally:
        camera.release()
    if (frame.shape[1], frame.shape[0]) != (width, height):
        raise RuntimeError(
            f"Camera returned {frame.shape[1]}x{frame.shape[0]}; expected {width}x{height}"
        )

    selected: list[list[float]] = []
    window = "SO-101 camera calibration"

    def on_mouse(event, x, y, _flags, _parameter):
        if event == cv2.EVENT_LBUTTONDOWN and len(selected) < len(robot_points):
            selected.append([float(x), float(y)])
            target = robot_points[len(selected) - 1]
            print(
                f"Point {len(selected)}: pixel=({x}, {y}) -> "
                f"robot=({target[0]:.4f}, {target[1]:.4f}) m",
                flush=True,
            )

    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(window, on_mouse)
    try:
        while True:
            display = frame.copy()
            for index, (x, y) in enumerate(selected):
                cv2.circle(display, (int(x), int(y)), 6, (0, 255, 0), -1)
                cv2.putText(
                    display,
                    str(index + 1),
                    (int(x) + 8, int(y) - 8),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2,
                )
            next_number = min(len(selected) + 1, len(robot_points))
            message = (
                f"Click point {next_number}/{len(robot_points)}; "
                "R=reset, S=save, Esc=cancel"
            )
            cv2.putText(display, message, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)
            cv2.imshow(window, display)
            key = cv2.waitKey(30) & 0xFF
            if key == 27:
                raise RuntimeError("Camera calibration cancelled")
            if key in (ord("r"), ord("R")):
                selected.clear()
            if key in (ord("s"), ord("S")):
                if len(selected) != len(robot_points):
                    print("Click every reference point before saving.", flush=True)
                    continue
                return selected
    finally:
        cv2.destroyWindow(window)


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Click fixed table reference points in the same order as "
            "calibration.robot_points_m in a hardware config."
        )
    )
    parser.add_argument("--config", required=True)
    parser.add_argument("--camera-index", type=int, required=True)
    args = parser.parse_args()
    if args.camera_index < 0:
        parser.error("--camera-index cannot be negative")

    path = Path(args.config)
    try:
        data, width, height, robot_points = load_click_config(path)
        image_points = capture_points(args.camera_index, width, height, robot_points)
    except (RuntimeError, ValueError) as exc:
        parser.error(str(exc))
    data["calibration"]["image_points_px"] = image_points
    data["configured"] = False
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"Saved pixel points to {path}")
    print("Measure/fill every remaining value and set configured to true only after checking them.")


if __name__ == "__main__":
    main()
