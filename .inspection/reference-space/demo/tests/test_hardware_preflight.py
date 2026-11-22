import sys
import tempfile
import types
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

import numpy as np

from demo.hardware_preflight import read_camera, read_follower


class HardwarePreflightTests(unittest.TestCase):
    def test_camera_is_released_and_returned_as_rgb(self):
        frame = np.zeros((3, 4, 3), dtype=np.uint8)
        rgb = np.full((3, 4, 3), 17, dtype=np.uint8)
        camera = MagicMock()
        camera.isOpened.return_value = True
        camera.read.return_value = (True, frame)
        cv2 = types.SimpleNamespace(
            CAP_DSHOW=1,
            CAP_ANY=0,
            CAP_PROP_FRAME_WIDTH=3,
            CAP_PROP_FRAME_HEIGHT=4,
            COLOR_BGR2RGB=5,
            VideoCapture=MagicMock(return_value=camera),
            cvtColor=MagicMock(return_value=rgb),
        )

        with patch.dict(sys.modules, {"cv2": cv2}):
            image = read_camera(2, 640, 480)

        self.assertIs(image, rgb)
        cv2.VideoCapture.assert_called_once_with(2, cv2.CAP_DSHOW)
        camera.release.assert_called_once()

    def test_uncalibrated_arm_is_rejected_before_connection(self):
        with tempfile.TemporaryDirectory() as directory:
            follower = MagicMock()
            follower.calibration_fpath = Path(directory) / "missing-calibration.json"
            module = types.ModuleType("lerobot.robots.so_follower")
            module.SO101Follower = MagicMock(return_value=follower)
            module.SO101FollowerConfig = MagicMock()
            modules = {
                "lerobot": types.ModuleType("lerobot"),
                "lerobot.robots": types.ModuleType("lerobot.robots"),
                "lerobot.robots.so_follower": module,
            }
            with patch.dict(sys.modules, modules):
                with self.assertRaisesRegex(RuntimeError, "run lerobot-calibrate"):
                    read_follower("COM5", "test-arm")

        follower.connect.assert_not_called()


if __name__ == "__main__":
    unittest.main()
