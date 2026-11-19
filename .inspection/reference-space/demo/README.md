# SO-101 interactive demo

A separate browser demo of the tabletop task, using MuJoCo 4.0.1 for server-side physics and Three.js/WebGL for front and wrist views. Each visitor gets an independent world.

The reusable LeRobot environment is documented in the [root README](../README.md). The demo's HTTP/WebSocket interface is separate from the LeRobot simulator-server protocol. Its root Dockerfile launches this demo; LeRobot builds its own simulator image from `env.py` and the root dependencies.

## Run

From the repository root:

```bash
docker build -t so101-tabletop-demo .
docker run --rm -p 7860:7860 so101-tabletop-demo
```

Or with Python 3.12+:

```bash
uv venv
uv pip install -r demo/requirements.txt
.venv/bin/python -m demo.app
```

Open http://localhost:7860. No server GPU or software renderer is required; the browser needs WebGL.

## Controls

| Control             | Action                             |
| ------------------- | ---------------------------------- |
| W / S               | Move forward / back                |
| A / D               | Move left / right                  |
| Q / E               | Move up / down                     |
| Up / Down arrows    | Tilt the gripper upward / downward |
| Left / Right arrows | Roll the wrist                     |
| Space               | Open / close the gripper           |
| Shift or Slow mode  | Precision movement                 |
| Joint sliders       | Set joint targets directly         |

Hold a key or button to move. The controller solves XYZ, tilt and roll together, with bounded pose tracking and joint limits. The third rotation component is unconstrained. After all four cubes are in their color-matched boxes, the demo shows the result for two seconds and automatically resets to the same seed. Reset replays the same seed; New scene changes it. End session releases the world. There are ten concurrent sessions by default (`DEMO_MAX_SESSIONS`).

## Layout

- `app.py`: headless physics session, IK, scene export and pose stream.
- `web/`: interface, Three.js renderer and browser assets.
- `requirements.txt`: demo dependencies, independent of the root environment requirements.
- `tests/`: controller, motor tracking and pose-stream checks.

The browser uses the original full-resolution robot meshes, with crease-preserving normals prepared in the browser. Physics and controls run at 30 Hz; the browser draws and interpolates poses independently. Hidden tabs pause the session. Browser views are visualizations, not the RGB camera observations used for policy evaluation.

## Checks

From the repository root:

```bash
python -m unittest discover -s demo/tests -v
```

## Assets and licenses

The robot and scene retain the licenses documented in the root README. Three.js is MIT-licensed (`web/vendor/Three-LICENSE.txt`). Source Sans 3 uses the SIL Open Font License (`web/SourceSans3-OFL.txt`). The logo comes from the [LeRobot branding Space](https://huggingface.co/spaces/lerobot/LeRobot-branding), revision `b86bad97891f8ecd6a2bd30f6441dfea1a72ad86`.
