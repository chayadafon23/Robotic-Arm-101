import json
import tempfile
import unittest
from pathlib import Path

import numpy as np

from demo.hardware_auto_sort import HardwareSortConfig, fit_homography
from demo.hardware_camera_calibrate import load_click_config


def valid_config():
    return {
        "configured": True,
        "image_size_px": [640, 480],
        "calibration": {
            "image_points_px": [[100, 100], [500, 100], [500, 400], [100, 400]],
            "robot_points_m": [[0.1, -0.2], [0.3, -0.2], [0.3, 0.2], [0.1, 0.2]],
        },
        "camera_position_m": [0.2, 0.0, 0.7],
        "workspace_m": {"x": [0.05, 0.35], "y": [-0.25, 0.25], "z": [0.02, 0.25]},
        "geometry_m": {
            "table_z": 0.0,
            "cube_top_offset": 0.03,
            "cube_center_offset": 0.015,
            "box_floor_offset": 0.004,
            "box_center_offset": 0.022,
            "approach_height": 0.07,
            "box_approach_height": 0.05,
            "clear_x": 0.18,
            "clear_z": 0.12,
        },
        "camera_park_degrees": [0, -30, 30, 90, 0, 30],
        "motion": {
            "fps": 10,
            "max_joint_step_degrees": 1,
            "max_gripper_step": 3,
            "max_run_seconds": 300,
        },
    }


class HardwareAutoSortTests(unittest.TestCase):
    def test_homography_maps_calibration_corners(self):
        image = np.array([[0, 0], [100, 0], [100, 100], [0, 100]], dtype=float)
        robot = np.array([[0.1, -0.2], [0.3, -0.2], [0.3, 0.2], [0.1, 0.2]], dtype=float)
        transform = fit_homography(image, robot)
        for pixel, expected in zip(image, robot):
            mapped = transform @ np.r_[pixel, 1.0]
            np.testing.assert_allclose(mapped[:2] / mapped[2], expected, atol=1e-9)

    def test_unapproved_config_is_rejected(self):
        data = valid_config()
        data["configured"] = False
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "hardware.json"
            path.write_text(json.dumps(data), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "not approved"):
                HardwareSortConfig.load(path)

    def test_config_maps_table_and_elevated_points(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "hardware.json"
            path.write_text(json.dumps(valid_config()), encoding="utf-8")
            config = HardwareSortConfig.load(path)

        table_xy = config.pixel_to_robot((100, 100), 0.0, 640, 480)
        raised_xy = config.pixel_to_robot((100, 100), 0.035, 640, 480)
        np.testing.assert_allclose(table_xy, [0.1, -0.2], atol=1e-9)
        self.assertGreater(raised_xy[0], table_xy[0])
        self.assertGreater(raised_xy[1], table_xy[1])
        self.assertTrue(config.contains(np.array([0.18, 0.0, 0.12])))
        self.assertFalse(config.contains(np.array([0.5, 0.0, 0.12])))

    def test_click_calibration_reads_measured_robot_points(self):
        data = valid_config()
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "hardware.json"
            path.write_text(json.dumps(data), encoding="utf-8")
            loaded, width, height, points = load_click_config(path)

        self.assertEqual((width, height), (640, 480))
        self.assertIs(loaded["configured"], True)
        np.testing.assert_allclose(points, data["calibration"]["robot_points_m"])

    def test_click_calibration_rejects_unmeasured_points(self):
        data = valid_config()
        data["calibration"]["robot_points_m"][0] = [None, None]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "hardware.json"
            path.write_text(json.dumps(data), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "null"):
                load_click_config(path)


if __name__ == "__main__":
    unittest.main()
