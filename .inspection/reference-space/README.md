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

## โครงสร้างไฟล์ใน `reference-space`

ไฟล์ที่ควรเริ่มอ่านคือ [`demo/AUTO_SORT_GUIDE_TH.md`](demo/AUTO_SORT_GUIDE_TH.md) ซึ่งรวมวิธีติดตั้ง คำสั่งรัน simulation การเตรียมแขนจริง การปรับเทียบกล้อง และข้อจำกัดด้านความปลอดภัย

> โฟลเดอร์ `.venv-hardware`, `myenv`, `__pycache__` และ `demo/__pycache__` เป็นไฟล์ที่โปรแกรมสร้างขึ้น จึงอธิบายเป็นกลุ่ม ไม่ได้ไล่รายการ dependency ภายในทุกไฟล์ และไม่ควรแก้ไฟล์ข้างในด้วยมือ

### ไฟล์และโฟลเดอร์ระดับบนสุด

| ไฟล์/โฟลเดอร์ | หน้าที่ |
| --- | --- |
| `README.md` | ไฟล์ที่กำลังอ่าน: ภาพรวม environment, วิธีใช้กับ EnvHub และแผนผังไฟล์ |
| `env.py` | สร้างโมเดล MuJoCo, Gymnasium environment, observation/action, กล้อง, reward และจุดเชื่อมกับ LeRobot EnvHub |
| `so101_tabletop.xml` | ฉาก MuJoCo: ห้อง โต๊ะ ลูกบาศก์ กล่อง ไฟ กล้อง และตำแหน่งติดตั้งแขน |
| `requirements.txt` | dependency ขั้นพื้นฐานของ simulation เช่น MuJoCo, Gymnasium และ NumPy |
| `requirements-hardware.txt` | dependency สำหรับแขนจริง: ติดตั้งรายการ simulation และ `lerobot[feetech]` |
| `Dockerfile` | สร้าง container สำหรับรัน browser demo/Space |
| `.dockerignore` | ระบุไฟล์ที่ไม่ต้องส่งเข้า Docker build context |
| `.gitignore` | ระบุ environment, cache และไฟล์ชั่วคราวที่ Git ไม่ต้องติดตาม |
| `.gitattributes` | ตั้งค่าการจัดการชนิดไฟล์ของ Git/Hugging Face repository |
| `MUJOCO_LOG.TXT` | log คำเตือนที่ MuJoCo สร้างระหว่างรัน ไม่ใช่ source code |
| `media/cameras.png` | ภาพตัวอย่างมุมกล้อง front และ wrist ที่แสดงใน README |
| `textures/wood.png` | texture ลายไม้สำหรับฉาก MuJoCo |
| `myenv/` | Python virtual environment สำหรับ simulation และ demo |
| `.venv-hardware/` | Python 3.12 virtual environment สำหรับ LeRobot, Feetech, กล้อง และแขนจริง |
| `__pycache__/` | bytecode cache ที่ Python สร้างอัตโนมัติ |

### โปรแกรมใน `demo/`

| ไฟล์/โฟลเดอร์ | หน้าที่ | สั่งแขนจริงหรือไม่ |
| --- | --- | --- |
| `demo/app.py` | FastAPI/WebSocket server และ controller สำหรับ browser demo | ไม่ |
| `demo/auto_sort.py` | ตรวจสี คำนวณ IK และทำลำดับหยิบ–วางใน MuJoCo | ไม่ |
| `demo/hardware_preflight.py` | เปิดกล้องจริง ตรวจสี และเลือกอ่านค่าข้อต่อ SO-101 | ไม่ส่งเป้าหมายการเคลื่อนที่ |
| `demo/hardware_camera_calibrate.py` | เปิดภาพกล้องให้คลิกจุดอ้างอิง แล้วบันทึก pixel coordinates ลงไฟล์ตั้งค่า | ไม่ |
| `demo/hardware_auto_sort.py` | ตรวจ configuration, ทำ dry run และเมื่อระบุ `--execute` จะคำนวณ IK/ส่งเป้าหมายไปยัง SO-101 จริง | ใช่ เฉพาะเมื่อมี `--execute` |
| `demo/hardware_auto_sort.example.json` | แม่แบบค่ากล้อง จุดอ้างอิง workspace geometry ท่าพัก และขีดจำกัดการเคลื่อนที่ ต้องคัดลอกและกรอกค่าที่วัดจริง | ไม่ |
| `demo/AUTO_SORT_GUIDE_TH.md` | คู่มือภาษาไทยฉบับหลัก ตั้งแต่ติดตั้งจนถึงคำสั่งทดลองแขนจริงทีละขั้น | ไม่ใช่โปรแกรม |
| `demo/README.md` | ภาพรวมและวิธีใช้ demo ภาษาอังกฤษ | ไม่ใช่โปรแกรม |
| `demo/requirements.txt` | dependency ของ simulation และ web server เช่น FastAPI/Uvicorn | ไม่ใช่โปรแกรม |
| `demo/tests/` | ชุดทดสอบอัตโนมัติ ไม่เชื่อมต่อหรือขยับแขนจริง | ไม่ |
| `demo/web/` | HTML, JavaScript, CSS, รูป และไลบรารีที่ใช้แสดง browser demo | ไม่ |

คำสั่ง `demo.hardware_auto_sort` จะไม่ส่งคำสั่งมอเตอร์ในโหมดปกติ ต้องมีไฟล์ calibration ที่ผ่านการตรวจและใส่ `--execute` อย่างชัดเจน รายละเอียดอยู่ในคู่มือภาษาไทย

### ชุดทดสอบใน `demo/tests/`

| ไฟล์ | สิ่งที่ทดสอบ |
| --- | --- |
| `test_auto_sort.py` | การตรวจสี การแปลงพิกัด ท่าเริ่มต้น และ IK ของ Auto-sort |
| `test_controller.py` | ขอบเขต controller, การ jog, การติดตามมอเตอร์จำลอง และ reward |
| `test_grasp.py` | ลำดับปิด gripper ยก ถือ และปล่อยใน simulation |
| `test_hardware_preflight.py` | การอ่านกล้องและการปฏิเสธแขนที่ยังไม่มี calibration โดยใช้ mock ไม่ต่อฮาร์ดแวร์จริง |
| `test_hardware_auto_sort.py` | homography, configuration, workspace และข้อมูลจุดปรับเทียบ โดยไม่ส่งคำสั่งมอเตอร์ |

### หน้าเว็บใน `demo/web/`

| ไฟล์/โฟลเดอร์ | หน้าที่ |
| --- | --- |
| `index.html` | โครงหน้า browser demo |
| `app.js` | เชื่อม WebSocket รับสถานะ และควบคุมปุ่ม/ข้อมูลบนหน้าเว็บ |
| `viewer.js` | สร้างฉากและแสดงหุ่นยนต์ด้วย Three.js |
| `style.css` | รูปแบบและ responsive layout ของหน้าเว็บ |
| `poster.jpg`, `wrist-poster.jpg` | ภาพเริ่มต้นของกล้องก่อนมี live frame |
| `wood.png` | texture ลายไม้สำหรับ renderer ฝั่ง browser |
| `lerobot-logo.svg` | โลโก้ LeRobot บนหน้าเว็บ |
| `source-sans-3.woff2` | ฟอนต์ Source Sans 3 ที่ใช้ในหน้าเว็บ |
| `SourceSans3-OFL.txt` | ใบอนุญาตของฟอนต์ |
| `vendor/three.module.js` | Three.js ที่เก็บไว้ในโปรเจกต์เพื่อ render 3D |
| `vendor/BufferGeometryUtils.js` | utility สำหรับรวม/จัดการ geometry ของ Three.js |
| `vendor/Three-LICENSE.txt` | ใบอนุญาตของไฟล์ Three.js |

### โมเดลใน `robotstudio_so101/`

| ไฟล์/โฟลเดอร์ | หน้าที่ |
| --- | --- |
| `robotstudio_so101/so101.xml` | โมเดลข้อต่อ actuator collision และ mesh ของ SO-101 สำหรับ MuJoCo |
| `robotstudio_so101/assets/*.stl` | mesh 3D ของฐาน แขน ข้อมือ gripper มอเตอร์ และ camera mount แต่ละชิ้น ชื่อไฟล์บอกชิ้นส่วนที่ใช้ |
| `robotstudio_so101/README.md` | ที่มาและคำอธิบายโมเดล SO-101 |
| `robotstudio_so101/CHANGELOG.md` | ประวัติการเปลี่ยนแปลงของโมเดล |
| `robotstudio_so101/LICENSE` | ใบอนุญาต Apache-2.0 ของโมเดล |

ไฟล์ `.stl` เป็น geometry สำหรับการแสดงผล/ชนใน simulation ไม่ใช่โปรแกรมควบคุมมอเตอร์จริง

## Licenses

- This environment is [Apache-2.0](https://www.apache.org/licenses/LICENSE-2.0), including `textures/wood.png` (generated procedurally for it).
- `robotstudio_so101/` is MuJoCo Menagerie's SO-101 model (commit `0059d4335f8156206f63a35662313385f7ad6d74`), derived from TheRobotStudio's SO-ARM100, under its bundled [Apache-2.0 license](https://huggingface.co/spaces/lerobot/so101-tabletop-mujoco/blob/main/robotstudio_so101/LICENSE).
