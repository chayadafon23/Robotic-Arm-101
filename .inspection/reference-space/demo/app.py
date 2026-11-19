"""CPU-only keyboard demo for the bundled SO-101 EnvHub, with one world per visitor."""

import asyncio
import logging
import os
import time
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from functools import lru_cache
from pathlib import Path

# The demo never renders on the server. EnvHub camera observations remain runtime-neutral.
os.environ.setdefault("MUJOCO_GL", "disable")
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")

import mujoco
import numpy as np
import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.gzip import GZipMiddleware
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles

from env import (
    BOX_BODIES,
    CUBE_BODIES,
    DEFAULTS,
    HERE,
    HOME,
    JOINTS,
    SCENE_FILE,
    _World,
    build_model,
)

logger = logging.getLogger(__name__)

ROOT = Path(__file__).absolute().parent
MAX_SESSIONS = int(os.environ.get("DEMO_MAX_SESSIONS", "10"))
active_sessions = 0
app = FastAPI(title="SO-101 tabletop", docs_url=None, redoc_url=None)
app.mount("/demo", StaticFiles(directory=ROOT / "web"), name="demo")
app.add_middleware(GZipMiddleware, minimum_size=1000)


@lru_cache(maxsize=1)
def scene_description():
    """Export display geometry once; live frames contain body/camera poses only."""
    model = build_model(HERE / SCENE_FILE, HERE)
    types = {
        int(getattr(mujoco.mjtGeom, "mjGEOM_" + name.upper())): name
        for name in ("plane", "sphere", "capsule", "ellipsoid", "cylinder", "box", "mesh")
    }
    geoms, meshes = [], {}
    for geom in range(model.ngeom):
        if model.geom_group[geom] > 2:
            continue
        kind = types.get(int(model.geom_type[geom]))
        if kind is None:
            continue
        material = int(model.geom_matid[geom])
        rgba = model.mat_rgba[material] if material >= 0 else model.geom_rgba[geom]
        if rgba[3] == 0:
            continue
        item = {
            "body": int(model.geom_bodyid[geom]),
            "type": kind,
            "pos": model.geom_pos[geom].tolist(),
            "quat": model.geom_quat[geom].tolist(),
            "size": model.geom_size[geom].tolist(),
            "rgba": rgba.tolist(),
            "wood": material >= 0 and "_wood" in model.material(material).name,
        }
        if kind == "mesh":
            mesh = int(model.geom_dataid[geom])
            item["mesh"] = str(mesh)
            if str(mesh) not in meshes:
                start, count = model.mesh_vertadr[mesh], model.mesh_vertnum[mesh]
                face, nface = model.mesh_faceadr[mesh], model.mesh_facenum[mesh]
                normal, nnormal = model.mesh_normaladr[mesh], model.mesh_normalnum[mesh]
                meshes[str(mesh)] = {
                    "vertices": model.mesh_vert[start : start + count]
                    .astype(float)
                    .round(7)
                    .ravel()
                    .tolist(),
                    "faces": model.mesh_face[face : face + nface].ravel().tolist(),
                    "normals": model.mesh_normal[normal : normal + nnormal]
                    .astype(float)
                    .round(7)
                    .ravel()
                    .tolist(),
                }
        geoms.append(item)
    return {"bodies": model.nbody, "geoms": geoms, "meshes": meshes}


@app.get("/scene")
def scene():
    return scene_description()


class KeyboardController:
    """Control fingertip XYZ and tool tilt/roll with one bounded five-axis IK task."""

    def __init__(self, model, joint_names):
        """Cache the arm's kinematic indices and joint/actuator bounds."""
        self.model = model
        self.data = mujoco.MjData(model)
        self.qadr = np.array([model.joint(name).qposadr[0] for name in joint_names])
        self.dofs = np.array([model.joint(name).dofadr[0] for name in joint_names[:5]])
        actuators = np.array([model.actuator(name).id for name in joint_names])
        self.lo, self.hi = model.actuator_ctrlrange[actuators].T.copy()
        joints = np.array([model.joint(name).range for name in joint_names])
        self.lo = np.maximum(self.lo, joints[:, 0])
        self.hi = np.minimum(self.hi, joints[:, 1])
        self.site = model.site("gripperframe").id
        self.jac_pos = np.zeros((3, model.nv))
        self.jac_rot = np.zeros_like(self.jac_pos)
        self.last_axes = None

    @staticmethod
    def _bounded_solve(matrix, residual, lower, upper):
        """Solve a small damped box-constrained least squares problem by coordinate descent."""
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

    @staticmethod
    def _rotate(orientation, rotation):
        angle = np.linalg.norm(rotation)
        if not angle:
            return orientation.copy()
        x, y, z = rotation / angle
        skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
        return (np.eye(3) + np.sin(angle) * skew + (1 - np.cos(angle)) * skew @ skew) @ orientation

    def jog(self, measured, gripper_target, axes, dt):
        """Jog world XYZ plus tool tilt/roll, keeping the other task coordinates fixed.

        Tilt and roll are rotations about the current wrist flex/roll axes, not direct
        joint commands. The third rotation component is free: five joints cannot hold a full 6D pose.
        Pose tracking corrects motor lag; bounded targets and rebasing on release/direction
        changes prevent blocked motion from queuing up future movement.
        """
        q = np.deg2rad(np.asarray(measured, dtype=float))
        q[5] = self.lo[5] + measured[5] / 100 * (self.hi[5] - self.lo[5])
        q = np.clip(q, self.lo, self.hi)
        self._kinematics(q)
        start = q.copy()
        actual_position = self.data.site_xpos[self.site].copy()
        actual_orientation = self.data.site_xmat[self.site].reshape(3, 3).copy()
        axes = np.asarray(axes, dtype=float)
        if self.last_axes is None or not np.array_equal(axes[:5], self.last_axes):
            self.position_target = actual_position
            self.orientation_target = actual_orientation
            # These two orthogonal world axes define the achievable orientation task.
            self.orientation_basis = self.jac_rot[:, self.dofs[3:5]].T.copy()
        self.last_axes = axes[:5].copy()
        basis = self.orientation_basis
        # Bound tracking error even against a collision or limit.
        position = actual_position + np.clip(self.position_target - actual_position, -0.008, 0.008)
        position += axes[:3] * 0.065 * dt
        angular_error = 0.5 * np.cross(actual_orientation.T, self.orientation_target.T).sum(axis=0)
        angular_error = basis.T @ (basis @ angular_error)
        angular_error *= min(1, 0.15 / max(np.linalg.norm(angular_error), 1e-12))
        orientation = self._rotate(actual_orientation, angular_error)
        rotation = basis.T @ (axes[3:5] * [0.8, np.deg2rad(65)] * dt)
        orientation = self._rotate(orientation, rotation)
        self.position_target, self.orientation_target = position, orientation
        # Limit the whole solve, including compensation, to the same per-joint speed.
        lower = np.maximum(self.lo[:5], start[:5] - 1.2 * dt)
        upper = np.minimum(self.hi[:5], start[:5] + 1.2 * dt)

        def error():
            actual = self.data.site_xmat[self.site].reshape(3, 3)
            angular = 0.5 * np.cross(actual.T, orientation.T).sum(axis=0)
            return np.concatenate([position - self.data.site_xpos[self.site], 0.1 * (basis @ angular)])

        residual = error()
        for _ in range(8):
            if np.linalg.norm(residual[:3]) < 2e-5 and np.linalg.norm(residual[3:]) < 2e-5:
                break
            matrix = np.vstack([self.jac_pos[:, self.dofs], 0.1 * basis @ self.jac_rot[:, self.dofs]])
            step = self._bounded_solve(matrix, residual, lower - q[:5], upper - q[:5])
            if np.max(np.abs(step)) < 1e-7:
                break
            # Backtrack if a nonlinear step makes the requested task worse near a singularity.
            accepted = False
            for scale in (1, 0.5, 0.25):
                candidate = q.copy()
                candidate[:5] += scale * step
                self._kinematics(candidate)
                candidate_error = error()
                if candidate_error @ candidate_error < residual @ residual:
                    q, residual = candidate, candidate_error
                    accepted = True
                    break
            if not accepted:
                self._kinematics(q)
                break
        limited = ""
        if np.linalg.norm(residual[:3]) > 0.0005 or np.linalg.norm(residual[3:]) > 0.001:
            at_limit = np.any((q[:5] - self.lo[:5] < 1e-5) | (self.hi[:5] - q[:5] < 1e-5))
            limited = "joint" if at_limit else "reach"
        target = np.rad2deg(q)
        target[5] = np.clip(gripper_target + axes[5] * 100 * dt, 0, 100)
        return target, limited


class DemoWorld(_World):
    """Renderer-free demo world reusing the scene's reset and joint-unit conventions."""

    def __init__(self):
        self.model = build_model(HERE / SCENE_FILE, HERE)
        model = self.model
        self.substeps = max(1, round(1 / (30 * model.opt.timestep)))
        model.opt.timestep = 1 / (30 * self.substeps)
        self.data = mujoco.MjData(model)
        self.qadr = np.array([model.joint(name).qposadr[0] for name in JOINTS])
        self.actuators = np.array([model.actuator(name).id for name in JOINTS])
        self.lo, self.hi = model.actuator_ctrlrange[self.actuators].T.copy()
        self.cube_bodies = [model.body(name).id for name in CUBE_BODIES]
        self.box_bodies = [model.body(name).id for name in BOX_BODIES]
        randomize = DEFAULTS["randomize"]
        self.joint_noise = float(randomize.get("joints", 0))
        self.bodies = [self._body_noise(name, cfg) for name, cfg in randomize["bodies"].items()]
        self.cameras = {
            model.camera(name).id: np.array(cfg["pos"]) for name, cfg in randomize["cameras"].items()
        }
        self.body_pos0, self.body_quat0, self.cam_pos0 = (
            model.body_pos.copy(),
            model.body_quat.copy(),
            model.cam_pos.copy(),
        )
        randomized = {noise.body for noise in self.bodies}
        self.fixed_free = [
            body
            for body in range(1, model.nbody)
            if self._free_qadr(body) is not None and body not in randomized
        ]


class DemoSession:
    """Own all MuJoCo/GL objects on one thread, including their cleanup."""

    def __init__(self):
        """Create one private world for a visitor."""
        self.world = DemoWorld()
        self.cameras = DEFAULTS["cameras"]
        self.seed = 7
        self.reset(self.seed)

    def reset(self, seed):
        """Reset the world and controller to a reproducible scene."""
        self.seed = int(seed) % 1_000_000
        self.world.reset(np.random.default_rng(self.seed), np.array(HOME, dtype=float))
        self.model, self.data = self.world.model, self.world.data
        self.target = self.world.joint_state().astype(float)
        self.controller = KeyboardController(self.model, JOINTS)
        self.site = self.model.site("gripperframe").id
        self.limits = np.rad2deg(np.stack([self.controller.lo, self.controller.hi]))
        self.limits[:, 5] = [0, 100]
        self.motion_limit = ""
        self.steps = 0
        self.ended = False
        self.reset_at = None
        self.last_time = time.monotonic()
        self.last_action = self.target.copy()

    def move_tip(self, axes, dt):
        """Track the bounded Cartesian target from the measured joints."""
        measured = self.world.joint_state().astype(float)
        self.target, self.motion_limit = self.controller.jog(measured, self.target[5], axes, dt)
        self.last_action = measured

    def update(self, message):
        """Apply one command, advance physics and return the two camera views."""
        command = message.get("command", "step")
        auto_reset = self.reset_at is not None and time.monotonic() >= self.reset_at
        if auto_reset:
            self.reset(self.seed)
        if command == "reset":
            self.reset(message.get("seed", self.seed))
        elif command == "home":
            self.target = np.array(HOME, dtype=float)
        elif command == "joints":
            target = np.asarray(message.get("target"), dtype=float)
            if target.shape != (6,) or not np.isfinite(target).all():
                raise ValueError("Expected six finite joint targets")
            self.target = np.clip(target, *self.limits)
        elif command == "gripper":
            self.target[5] = 90 if self.target[5] < 50 else 10
        elif command != "step":
            raise ValueError("Unknown command")
        if command in ("home", "joints"):
            self.controller.last_axes = None
        dt = min(max(time.monotonic() - self.last_time, 1 / 30), 0.1)
        self.last_time = time.monotonic()
        axes = np.asarray(message.get("axes", [0] * 6), dtype=float)
        if axes.shape != (6,) or not np.isfinite(axes).all():
            raise ValueError("Expected six finite control axes")
        if auto_reset:
            axes = np.zeros(6)
        self.motion_limit = ""
        before_tip = self.data.site_xpos[self.site].copy()
        if not self.ended:
            # Update IK at each 30 Hz control tick, even when CPU rendering produces fewer frames.
            ticks = max(1, min(3, round(dt * 30)))
            if not np.any(axes):
                self.controller.last_axes = None
            for _ in range(ticks):
                if np.any(axes):
                    self.move_tip(np.clip(axes, -1, 1), dt / ticks * (0.3 if message.get("precision") else 1))
                delta = np.clip(self.target - self.last_action, [-5] * 5 + [-12], [5] * 5 + [12])
                self.last_action += delta
                self.data.ctrl[self.world.actuators] = self.world.to_ctrl(self.last_action)
                mujoco.mj_step(self.model, self.data, nstep=self.world.substeps)
            self.steps += ticks
            displacement = axes[:3] * 0.065 * dt * (0.3 if message.get("precision") else 1)
            actual_motion = self.data.site_xpos[self.site] - before_tip
            if np.any(displacement) and actual_motion @ displacement < 0.1 * (displacement @ displacement):
                self.motion_limit = self.motion_limit or "contact"
        count = self.world.cubes_in_box()
        if count == 4 and self.reset_at is None:
            self.reset_at = time.monotonic() + 2
        self.ended = count == 4 or self.steps >= 30 * 600
        bodies = np.column_stack([self.data.xpos, self.data.xquat]).round(6).tolist()
        cameras = {}
        for key, name in self.cameras.items():
            camera = self.model.camera(name).id
            quat = np.empty(4)
            mujoco.mju_mat2Quat(quat, self.data.cam_xmat[camera])
            cameras[key] = {
                "pos": self.data.cam_xpos[camera].round(6).tolist(),
                "quat": quat.round(6).tolist(),
                "fovy": float(self.model.cam_fovy[camera]),
            }
        return {
            "bodies": bodies,
            "cameras": cameras,
            "joints": self.world.joint_state().round(1).tolist(),
            "target": self.target.round(1).tolist(),
            "limits": self.limits.round(1).tolist(),
            "tip": self.data.site_xpos[self.site].round(3).tolist(),
            "cubes": count,
            "seed": self.seed,
            "sim_time": round(self.steps / 30, 1),
            "success": count == 4,
            "ended": self.ended,
            "motion_limit": self.motion_limit,
        }

    def close(self):
        """Release the renderer on its owning thread."""
        # No renderer or native viewer is allocated by the demo.


@app.get("/")
def index():
    """Serve the playground page."""
    return FileResponse(ROOT / "web" / "index.html")


@app.get("/health")
def health():
    """Report readiness and the number of allocated sessions."""
    return {
        "status": "ok",
        "active_sessions": active_sessions,
        "max_sessions": MAX_SESSIONS,
    }


@app.websocket("/ws")
async def session(websocket: WebSocket):
    """Run an isolated world for one browser and release it on disconnect."""
    global active_sessions
    await websocket.accept()
    if active_sessions >= MAX_SESSIONS:
        await websocket.send_json({"error": "The demo is full. Please try again in a moment."})
        await websocket.close(code=1013)
        return
    active_sessions += 1
    executor = ThreadPoolExecutor(max_workers=1)
    loop = asyncio.get_running_loop()
    world = None
    receiver = None
    controls = {"axes": [0] * 6, "precision": False, "visible": True}
    commands = deque(maxlen=16)
    last_input = time.monotonic()

    async def receive_controls():
        nonlocal last_input
        while True:
            message = await asyncio.wait_for(websocket.receive_json(), timeout=60)
            if not isinstance(message, dict):
                raise TypeError("Expected an object")
            controls.update({key: message[key] for key in controls if key in message})
            last_input = time.monotonic()
            if message.get("command", "step") != "step":
                commands.append(message)

    try:
        world = await loop.run_in_executor(executor, DemoSession)
        await websocket.send_json(await loop.run_in_executor(executor, world.update, {}))
        receiver = asyncio.create_task(receive_controls())
        while not receiver.done():
            if not controls["visible"]:
                await asyncio.sleep(0.1)
                continue
            message = {**(commands.popleft() if commands else {}), **controls}
            # Stop movement promptly if key updates stop arriving.
            if time.monotonic() - last_input > 0.5:
                message["axes"] = [0] * 6
            started = time.perf_counter()
            frame = await loop.run_in_executor(executor, world.update, message)
            frame["server_ms"] = round((time.perf_counter() - started) * 1000, 1)
            await websocket.send_json(frame)
            await asyncio.sleep(max(0, 1 / 30 - (time.perf_counter() - started)))
        await receiver
    except (TimeoutError, WebSocketDisconnect):
        pass
    except (TypeError, ValueError) as exc:
        await websocket.send_json({"error": str(exc)})
        await websocket.close(code=1008)
    except Exception:
        logger.exception("Demo session failed")
        try:
            await websocket.send_json({"error": "The simulator could not start. Try reconnecting."})
            await websocket.close(code=1011)
        except RuntimeError:
            pass
    finally:
        if receiver is not None:
            receiver.cancel()
            await asyncio.gather(receiver, return_exceptions=True)
        try:
            if world is not None:
                await loop.run_in_executor(executor, world.close)
        finally:
            executor.shutdown(wait=False)
            active_sessions -= 1


if __name__ == "__main__":
    uvicorn.run(
        app,
        host=os.environ.get("HOST", "0.0.0.0"),
        port=int(os.environ.get("PORT", "7860")),
        ws_max_size=16384,
    )
