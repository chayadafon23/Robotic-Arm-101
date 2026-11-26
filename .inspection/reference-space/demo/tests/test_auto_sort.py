import base64
import unittest

import numpy as np

from demo.app import KeyboardController
from demo.auto_sort import (
    AutoSortController,
    CAMERA_PARK,
    COLORS,
    VISION_HEIGHT,
    VISION_WIDTH,
    decode_browser_frame,
    detect_color_regions,
)
from env import HERE, HOME, JOINTS, SCENE_FILE, build_model


class VisionTests(unittest.TestCase):
    def test_auto_sort_starts_by_moving_to_park_joint_pose(self):
        model = build_model(HERE / SCENE_FILE, HERE)
        controller = KeyboardController(model, JOINTS)
        sorter = AutoSortController(model, controller)

        sorter.start()

        self.assertEqual(sorter.stage, "park")
        np.testing.assert_array_equal(sorter.joint_target, CAMERA_PARK)

    def test_hardware_sorter_returns_to_camera_pose_before_next_detection(self):
        model = build_model(HERE / SCENE_FILE, HERE)
        controller = KeyboardController(model, JOINTS)
        sorter = AutoSortController(
            model, controller, colors=("red", "green"), repark_before_find=True
        )
        sorter.start()

        sorter._next()

        self.assertEqual(sorter.stage, "park")
        self.assertEqual(sorter.status, "Moving to a camera-clear position")

    def test_simulation_sorter_keeps_existing_direct_find_transition(self):
        model = build_model(HERE / SCENE_FILE, HERE)
        controller = KeyboardController(model, JOINTS)
        sorter = AutoSortController(model, controller, colors=("red", "green"))
        sorter.start()

        sorter._next()

        self.assertEqual(sorter.stage, "find")
        self.assertEqual(sorter.status, "Looking for the green cube")

    def test_inverse_kinematics_solves_camera_derived_pick_pose(self):
        model = build_model(HERE / SCENE_FILE, HERE)
        controller = KeyboardController(model, JOINTS)
        sorter = AutoSortController(model, controller)
        measured = CAMERA_PARK.copy()
        sorter._tip(measured)
        sorter.grasp_orientation = controller.data.site_xmat[
            controller.site
        ].reshape(3, 3).copy()

        target = np.array([0.283, -0.091, 0.055])
        joint_target = sorter._solve_pose(target, measured)

        self.assertIsNotNone(joint_target)
        np.testing.assert_allclose(sorter._tip(joint_target), target, atol=0.003)

    def test_detects_matching_small_cubes_and_large_boxes(self):
        image = np.zeros((VISION_HEIGHT, VISION_WIDTH, 3), dtype=np.uint8)
        colors = {
            "red": (230, 35, 35),
            "green": (35, 220, 45),
            "blue": (35, 65, 230),
            "yellow": (230, 205, 30),
        }
        expected = {}
        for index, color in enumerate(COLORS):
            x = 8 + index * 38
            image[10:34, x : x + 24] = colors[color]
            image[48:56, x + 8 : x + 16] = colors[color]
            expected[color] = (x + 11.5, 21.5, x + 11.5, 51.5)

        regions = detect_color_regions(image)
        for color, (box_x, box_y, cube_x, cube_y) in expected.items():
            with self.subTest(color=color):
                self.assertIn("box", regions[color])
                self.assertIn("cube", regions[color])
                self.assertAlmostEqual(regions[color]["box"][0], box_x, delta=1)
                self.assertAlmostEqual(regions[color]["box"][1], box_y, delta=1)
                self.assertAlmostEqual(regions[color]["cube"][0], cube_x, delta=1)
                self.assertAlmostEqual(regions[color]["cube"][1], cube_y, delta=1)

    def test_browser_frame_decoder_restores_top_down_rgb(self):
        image = np.zeros((VISION_HEIGHT, VISION_WIDTH, 4), dtype=np.uint8)
        image[0, :, :3] = [12, 34, 56]
        image[-1, :, :3] = [78, 90, 123]
        encoded = base64.b64encode(image[::-1].tobytes()).decode("ascii")

        decoded = decode_browser_frame(encoded)

        np.testing.assert_array_equal(decoded[0, 0], [12, 34, 56])
        np.testing.assert_array_equal(decoded[-1, 0], [78, 90, 123])

    def test_top_camera_center_ray_maps_to_scene_center(self):
        model = build_model(HERE / SCENE_FILE, HERE)
        controller = KeyboardController(model, JOINTS)
        sorter = AutoSortController(model, controller)
        sorter._tip(np.asarray(HOME, dtype=float))

        point = sorter._pixel_to_world(
            (VISION_WIDTH / 2 - 0.5, VISION_HEIGHT / 2 - 0.5),
            0.03,
            VISION_WIDTH,
            VISION_HEIGHT,
        )

        np.testing.assert_allclose(point, [0.2, 0.0], atol=1e-6)


if __name__ == "__main__":
    unittest.main()
