import unittest

import mujoco
import numpy as np

from demo.app import KeyboardController
from env import HERE, JOINTS, SCENE_FILE, build_model


def run(offset=0, speculative=False):
    m = build_model(HERE / SCENE_FILE, HERE)
    if speculative:
        m.opt.enableflags |= mujoco.mjtEnableBit.mjENBL_SPECULATIVE
    substeps = round(1 / (30 * m.opt.timestep))
    m.opt.timestep = 1 / (30 * substeps)
    d = mujoco.MjData(m)
    c = KeyboardController(m, JOINTS)
    actuators = np.array([m.actuator(n).id for n in JOINTS])
    q = np.deg2rad([0, -22, 54, 58, 0, 0])
    q[5] = c.hi[5]
    d.qpos[c.qadr] = q
    d.ctrl[actuators] = q
    mujoco.mj_forward(m, d)
    grip = m.body("gripper").id
    rotation = d.xmat[grip].reshape(3, 3)
    for i, name in enumerate(["red_cube", "green_cube", "blue_cube", "yellow_cube"]):
        body = m.body(name).id
        adr = m.jnt_qposadr[m.body_jntadr[body]]
        pos = (
            d.xpos[grip] + rotation @ np.array([0.007 + offset, -0.000218, -0.088])
            if i == 0
            else np.array([-0.1 + i * 0.07, 0.25, 0.015])
        )
        d.qpos[adr : adr + 3] = pos
        d.qpos[adr + 2] = 0.015
        d.qpos[adr + 3 : adr + 7] = d.xquat[grip] if i == 0 else [1, 0, 0, 0]
    mujoco.mj_forward(m, d)
    mujoco.mj_step(m, d, nstep=100)
    d.ctrl[actuators[5]] = c.lo[5]
    mujoco.mj_step(m, d, nstep=250)
    close_height = float(d.xpos[m.body("red_cube").id, 2])
    tip0 = d.site_xpos[c.site].copy()
    for _ in range(45):
        measured = np.rad2deg(d.qpos[c.qadr])
        measured[5] = (d.qpos[c.qadr[5]] - c.lo[5]) / (c.hi[5] - c.lo[5]) * 100
        target, _ = c.jog(measured, 0, [0, 0, 1, 0, 0, 0], 1 / 30)
        ctrl = np.deg2rad(target)
        ctrl[5] = c.lo[5]
        d.ctrl[actuators] = ctrl
        mujoco.mj_step(m, d, nstep=substeps)
    mujoco.mj_step(m, d, nstep=200)
    held = float(d.xpos[m.body("red_cube").id, 2])
    tip_lift = float(d.site_xpos[c.site, 2] - tip0[2])
    d.ctrl[actuators[5]] = c.hi[5]
    mujoco.mj_step(m, d, nstep=350)
    released = float(d.xpos[m.body("red_cube").id, 2])
    return {
        "offset_mm": offset * 1000,
        "close_z": close_height,
        "held_z": held,
        "released_z": released,
        "tip_lift": tip_lift,
        "finite": bool(np.isfinite(d.qpos).all()),
    }


class GraspTests(unittest.TestCase):
    def test_close_lift_hold_release(self):
        for offset in [-0.002, 0, 0.002]:
            with self.subTest(offset=offset):
                trial = run(offset)
                self.assertTrue(trial["finite"])
                self.assertGreater(trial["held_z"], 0.08)
                self.assertGreater(trial["tip_lift"], 0.07)
                self.assertLess(trial["released_z"], 0.025)


if __name__ == "__main__":
    unittest.main()
