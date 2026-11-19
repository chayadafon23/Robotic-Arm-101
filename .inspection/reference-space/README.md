---
title: SO-101 table top MuJoCo
emoji: 🤖
colorFrom: green
colorTo: blue
sdk: docker
app_port: 7860
suggested_hardware: cpu-basic
license: apache-2.0
library_name: lerobot
tags: [lerobot, envhub, robotics, simulation, mujoco, so101]
---

# SO-101 tabletop

A LeRobot [EnvHub](https://huggingface.co/docs/lerobot/envhub) environment: an SO-101 arm in the turquoise of the LeRobot kits, four painted wooden 3 cm cubes and four color-matched open boxes on a table in a simple room, simulated with MuJoCo 4.0.1. Each reset places the cubes and front camera with seeded offsets.

The environment source is [env.py](https://huggingface.co/spaces/lerobot/so101-tabletop-mujoco/blob/main/env.py).

![Front and wrist cameras](media/cameras.png)

|              |                                                                                                                                                                                                            |
| ------------ | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Observation  | `pixels`: `front` (scene camera) and `wrist` (gripper camera), 480×640 RGB; `agent_pos`: 6 joint positions                                                                                                 |
| Action       | 6 absolute joint position targets: `shoulder_pan`, `shoulder_lift`, `elbow_flex`, `wrist_flex`, `wrist_roll` in degrees, `gripper` 0-100 (the units of LeRobot's `so101_follower` with `use_degrees=True`) |
| Control rate | 30 fps                                                                                                                                                                                                     |
| Task         | "Put each cube in the box with the matching color."                                                                                                                                                        |
| Reward       | the fraction of cubes in the correct boxes at each step (0, 0.25, ... 1); `info["cubes_in_correct_boxes"]` gives the count                                                                                |
| Episode end  | success (terminated) when all four cubes are correctly sorted; truncated after 3600 steps (2 minutes), set with `episode_length`                                                                           |
| Reset        | seeded cube and box poses, front camera jitter, joint noise                                                                                                                                                |

## With the LeRobot simulator server

From a LeRobot source checkout with Space EnvHub support, start the server. Docker mode requires Docker and the NVIDIA Container Toolkit. The image installs `requirements.txt`, and `env.py` declares the control rate and joint names in `LEROBOT_SIM`. Serving runs this repository's code, so the server asks for your consent with `--trust_remote_code=true`: read `env.py` and `requirements.txt` first. These commands use the Space-aware EnvHub loader. Replace `<commit>` with a revision from this Space:

```bash
uv pip install -e ".[core_scripts,evaluation,remote]"
lerobot-env-server --envhub=lerobot/so101-tabletop-mujoco@<commit> --trust_remote_code=true --docker=true
```

Then drive it as a robot, here with a policy:

```bash
lerobot-rollout --robot.type=sim \
  --robot.env.type=envhub --robot.env.hub_path=lerobot/so101-tabletop-mujoco@<commit> --robot.env.task=so101_tabletop \
  --robot.env.server.endpoint=tcp/127.0.0.1:7448 \
  --policy.path=<your SO-101 checkpoint> --fps=30
```

Without `--robot.env.server.endpoint`, the same command runs the environment in-process (with `--robot.env.trust_remote_code=true` and the requirements installed next to LeRobot).

Observations reach policies as `observation.images.front`, `observation.images.wrist` and `observation.state`; use `--rename_map` if your checkpoint names its cameras differently.

## Evaluate a policy

With the simulator server running, evaluate a compatible SO-101 checkpoint:

```bash
lerobot-eval --policy.path=<checkpoint> \
  --env.type=envhub --env.hub_path=lerobot/so101-tabletop-mujoco@<commit> \
  --env.task=so101_tabletop \
  --env.server.endpoint=tcp/127.0.0.1:7448 \
  --eval.n_episodes=10 --eval.batch_size=1 \
  --eval.use_async_envs=false
```

Install your policy's extra as needed, and use `--rename_map` if its camera names differ.

## In-process

```python
from lerobot.envs import HubEnvConfig, make_env

envs = make_env(HubEnvConfig(hub_path="lerobot/so101-tabletop-mujoco@<commit>", trust_remote_code=True), n_envs=1)
```

`make_env_kwargs` (`--env.make_env_kwargs` on the command line) override the defaults of `SO101Env` in `env.py` (episode length, image size, cameras, initial joints, reset randomization). Headless machines need `MUJOCO_GL=egl`.

## Files

| File                 | Content                                                                                           |
| -------------------- | ------------------------------------------------------------------------------------------------- |
| `env.py`             | the EnvHub entry point (`make_env`), the Gymnasium env and `LEROBOT_SIM`                          |
| `so101_tabletop.xml` | the scene: room, table, cubes, box, lights, front camera and the arm's mount site                 |
| `robotstudio_so101/` | MuJoCo Menagerie's SO-101 model, attached at the mount site (`env.py` recolors its printed parts) |
| `textures/wood.png`  | a seamless wood grain, tinted per cube                                                            |

## Licenses

- This environment is [Apache-2.0](https://www.apache.org/licenses/LICENSE-2.0), including `textures/wood.png` (generated procedurally for it).
- `robotstudio_so101/` is MuJoCo Menagerie's SO-101 model (commit `0059d4335f8156206f63a35662313385f7ad6d74`), derived from TheRobotStudio's SO-ARM100, under its bundled [Apache-2.0 license](https://huggingface.co/spaces/lerobot/so101-tabletop-mujoco/blob/main/robotstudio_so101/LICENSE).
