"""Regression checks for keyboard IK, without rendering or a GPU."""

import unittest
from unittest.mock import patch

import mujoco
import numpy as np

from demo.app import DemoSession, KeyboardController
from env import HERE, HOME, JOINTS, SCENE_FILE, build_model


class ControllerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = build_model(HERE / SCENE_FILE, HERE)

    def setUp(self):
        self.controller = KeyboardController(self.model, JOINTS)

    def pose(self, joints):
        data = mujoco.MjData(self.model)
        q = np.deg2rad(joints)
        q[5] = self.controller.lo[5] + joints[5] / 100 * (self.controller.hi[5] - self.controller.lo[5])
        data.qpos[self.controller.qadr] = q
        mujoco.mj_forward(self.model, data)
        return (
            data.site_xpos[self.controller.site].copy(),
            data.site_xmat[self.controller.site].reshape(3, 3).copy(),
        )

    def tip(self, joints):
        return self.pose(joints)[0]

    def angular_change(self, before, after):
        _, first = self.pose(before)
        _, second = self.pose(after)
        return 0.5 * np.cross(first.T, second.T).sum(axis=0)

    def orientation_axes(self, joints):
        self.controller._kinematics(np.deg2rad(joints))
        return self.controller.jac_rot[:, self.controller.dofs[3:5]].T.copy()

    def test_cartesian_directions_from_home(self):
        for axis in range(3):
            for direction in (-1, 1):
                with self.subTest(axis=axis, direction=direction):
                    axes = np.zeros(6)
                    axes[axis] = direction
                    target, _ = self.controller.jog(HOME, 30, axes, 0.1)
                    delta = self.tip(target) - self.tip(HOME)
                    self.assertGreater(delta[axis] * direction, 0.001)

    def test_vertical_jog_keeps_orientation_and_sideways_jog_compensates_roll(self):
        before = np.array(HOME, dtype=float)
        up, _ = self.controller.jog(before, 30, [0, 0, 1, 0, 0, 0], 0.1)
        # The three parallel pitch axes must counter-rotate while the tip rises.
        self.assertLess(abs(np.deg2rad(up[1:4].sum() - before[1:4].sum())), 0.01)
        for direction in (-1, 1):
            sideways, _ = self.controller.jog(before, 30, [0, direction, 0, 0, 0, 0], 0.1)
            self.assertGreater(abs(sideways[4] - before[4]), 0.2)
            self.assertGreater((self.tip(sideways)[1] - self.tip(before)[1]) * direction, 0.001)
        roll, _ = self.controller.jog(before, 30, [0, 0, 0, 0, 1, 0], 0.1)
        self.assertGreater(roll[4], before[4] + 5)

    def test_motor_tracking_does_not_accumulate_cross_axis_drift(self):
        actuators = np.array([self.model.actuator(name).id for name in JOINTS])
        for axis in range(5):
            with self.subTest(axis=axis):
                controller = KeyboardController(self.model, JOINTS)
                data = mujoco.MjData(self.model)
                q = np.deg2rad(HOME)
                q[5] = controller.lo[5] + 0.3 * (controller.hi[5] - controller.lo[5])
                data.qpos[controller.qadr] = q
                data.ctrl[actuators] = q
                mujoco.mj_step(self.model, data, nstep=600)
                before = data.site_xpos[controller.site].copy()
                axes = np.zeros(6)
                axes[axis] = 0.3
                for _ in range(12):
                    measured = np.rad2deg(data.qpos[controller.qadr])
                    measured[5] = (
                        100
                        * (data.qpos[controller.qadr[5]] - controller.lo[5])
                        / (controller.hi[5] - controller.lo[5])
                    )
                    target, _ = controller.jog(measured, 30, axes, 1 / 30)
                    ctrl = np.deg2rad(target)
                    ctrl[5] = q[5]
                    data.ctrl[actuators] = ctrl
                    mujoco.mj_step(self.model, data, nstep=8)
                requested = axes[:3] * 0.065 * 12 / 30
                np.testing.assert_allclose(data.site_xpos[controller.site] - before, requested, atol=0.001)

    def test_success_resets_to_the_same_seed_after_two_seconds(self):
        session = DemoSession()
        try:
            world = session.world
            for body, box in zip(world.cube_bodies, world.box_bodies):
                adr = world._free_qadr(body)
                rotation = session.data.xmat[box].reshape(3, 3)
                session.data.qpos[adr : adr + 3] = session.data.xpos[box] + rotation @ [0, 0, 0.02]
                session.data.qpos[adr + 3 : adr + 7] = session.data.xquat[box]
            mujoco.mj_forward(session.model, session.data)
            with patch("demo.app.time.monotonic", return_value=100):
                frame = session.update({})
            self.assertEqual(frame["cubes"], 4)
            self.assertTrue(frame["success"])
            with patch("demo.app.time.monotonic", return_value=101.9):
                self.assertTrue(session.update({})["success"])
            with patch("demo.app.time.monotonic", return_value=102.1):
                frame = session.update({})
            self.assertEqual(frame["seed"], 7)
            self.assertFalse(frame["success"])
            self.assertEqual(frame["cubes"], 0)
            self.assertIsNone(session.reset_at)
        finally:
            session.close()

    def test_cubes_only_count_in_their_matching_boxes(self):
        session = DemoSession()
        try:
            world = session.world
            wrong_boxes = world.box_bodies[1:] + world.box_bodies[:1]
            for body, box in zip(world.cube_bodies, wrong_boxes):
                adr = world._free_qadr(body)
                rotation = session.data.xmat[box].reshape(3, 3)
                session.data.qpos[adr : adr + 3] = session.data.xpos[box] + rotation @ [0, 0, 0.02]
                session.data.qpos[adr + 3 : adr + 7] = session.data.xquat[box]
            mujoco.mj_forward(session.model, session.data)
            self.assertEqual(world.cubes_in_box(), 0)
        finally:
            session.close()

    def test_demo_never_creates_a_server_renderer(self):
        session = DemoSession()
        try:
            frame = session.update({})
            self.assertNotIn("images", frame)
            self.assertEqual(len(frame["bodies"]), session.model.nbody)
            self.assertEqual(set(frame["cameras"]), {"front", "wrist", "top"})
            self.assertFalse(hasattr(session.world, "renderer"))
        finally:
            session.close()

    def test_tilt_and_roll_rotate_the_tool_without_translating_the_tip(self):
        for pan, roll in [(0, 0), (45, 40), (-45, -30)]:
            pose = np.array(HOME, dtype=float)
            pose[[0, 4]] = [pan, roll]
            basis = self.orientation_axes(pose)
            for axis, speed in [(3, 0.8), (4, np.deg2rad(65))]:
                for direction in [-1, 1]:
                    with self.subTest(pan=pan, roll=roll, axis=axis, direction=direction):
                        axes = np.zeros(6)
                        axes[axis] = direction
                        target, reason = self.controller.jog(pose, 30, axes, 0.1)
                        self.assertEqual(reason, "")
                        self.assertLess(np.linalg.norm(self.tip(target) - self.tip(pose)), 0.0001)
                        change = basis @ self.angular_change(pose, target)
                        self.assertAlmostEqual(change[axis - 3], direction * speed * 0.1, delta=0.001)
                        self.assertLess(abs(change[4 - axis]), 0.001)

    def test_simultaneous_xyz_tilt_and_roll(self):
        axes = np.array([0.3, -0.2, 0.2, 0.2, -0.2, 0])
        basis = self.orientation_axes(HOME)
        target, reason = self.controller.jog(HOME, 30, axes, 0.1)
        self.assertEqual(reason, "")
        np.testing.assert_allclose(self.tip(target) - self.tip(HOME), axes[:3] * 0.065 * 0.1, atol=0.0001)
        np.testing.assert_allclose(
            basis @ self.angular_change(HOME, target),
            axes[3:5] * [0.8, np.deg2rad(65)] * 0.1,
            atol=0.001,
        )

    def test_wrist_has_no_windup_after_holding_a_limit(self):
        measured = np.array(HOME, dtype=float)
        measured[3] = np.rad2deg(self.controller.hi[3])
        # A collision can keep measured joints fixed despite repeated orientation requests.
        for _ in range(200):
            self.controller.jog(measured, 30, [0, 0, 0, 1, 0, 0], 0.1)
        target, _ = self.controller.jog(measured, 30, [0, 0, 0, -1, 0, 0], 0.1)
        fresh, _ = KeyboardController(self.model, JOINTS).jog(measured, 30, [0, 0, 0, -1, 0, 0], 0.1)
        np.testing.assert_allclose(target, fresh)
        self.assertLess(
            (self.orientation_axes(measured) @ self.angular_change(measured, target))[0],
            -0.005,
        )

    def test_screenshot_limit_pose_can_immediately_back_out(self):
        pose = np.array([89, -100, np.rad2deg(self.controller.hi[2]), 95, -7, 32])
        pose[1] = np.rad2deg(self.controller.lo[1])
        pose[3] = np.rad2deg(self.controller.hi[3])
        for axis in range(3):
            results = []
            for direction in (-1, 1):
                axes = np.zeros(6)
                axes[axis] = direction
                target, _ = self.controller.jog(pose, 32, axes, 0.1)
                delta = self.tip(target) - self.tip(pose)
                results.append(delta[axis] * direction)
                self.assertTrue(np.isfinite(target).all())
            self.assertGreater(max(results), 0.0005, (axis, results))
        target, _ = self.controller.jog(pose, 32, [0, 0, 0, -1, 0, 0], 0.1)
        self.assertLess((self.orientation_axes(pose) @ self.angular_change(pose, target))[0], -0.005)

    def test_collision_stalled_target_does_not_affect_next_jog(self):
        controller = self.controller
        # A previous command may be unreachable physically. Repeated attempts cannot accumulate state.
        for _ in range(200):
            controller.jog(HOME, 30, [0, 0, -1, 0, 0, 0], 0.1)
        target, _ = controller.jog(HOME, 30, [0, 0, 1, 0, 0, 0], 0.1)
        fresh, _ = KeyboardController(self.model, JOINTS).jog(HOME, 30, [0, 0, 1, 0, 0, 0], 0.1)
        np.testing.assert_allclose(target, fresh)
        self.assertGreater(self.tip(target)[2], self.tip(HOME)[2])

    def test_bounds_during_long_mixed_control_sequence(self):
        rng = np.random.default_rng(7)
        measured = np.array(HOME, dtype=float)
        for _ in range(500):
            target, _ = self.controller.jog(measured, measured[5], rng.integers(-1, 2, 6), 0.1)
            self.assertTrue(np.isfinite(target).all())
            self.assertTrue(np.all(target[:5] >= np.rad2deg(self.controller.lo[:5]) - 1e-8))
            self.assertTrue(np.all(target[:5] <= np.rad2deg(self.controller.hi[:5]) + 1e-8))
            self.assertGreaterEqual(target[5], 0)
            self.assertLessEqual(target[5], 100)
            measured = target


if __name__ == "__main__":
    unittest.main()
