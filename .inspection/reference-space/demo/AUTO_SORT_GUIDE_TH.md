# คู่มือติดตั้งและใช้งาน Auto-sort SO-101

## สถานะของโปรเจกต์ก่อนเริ่ม

- Auto-sort ที่คีบและแยกสีอัตโนมัติทำงานแล้วใน **MuJoCo simulation**
- `demo.hardware_preflight` ใช้ตรวจกล้องจริง ตรวจสี และอ่านข้อต่อของแขนจริง แต่ **ไม่ส่งคำสั่งให้มอเตอร์เคลื่อนที่**
- `demo.hardware_auto_sort` เป็นคำสั่ง Auto-sort สำหรับแขนจริง แต่จะส่งคำสั่งมอเตอร์เฉพาะเมื่อมีไฟล์ calibration ที่กรอกค่าจริงครบและระบุ `--execute` โหมดนี้ยังต้องทดสอบกับอุปกรณ์จริงทีละขั้นก่อนใช้งานครบสี่สี

อย่าใช้คำสั่ง simulation หรือ joint target จาก simulation ไปสั่งมอเตอร์จริงโดยตรง

## ติดตั้งจากเครื่องใหม่บน Windows

คำสั่งทั้งหมดในคู่มือนี้ใช้ PowerShell และต้องเริ่มจากโฟลเดอร์หลักของ environment:

```powershell
cd C:\Users\chaya\SO_ARM101\.inspection\reference-space
```

### 1. สิ่งที่ต้องเตรียม

- Python สำหรับ simulation และ Python 3.12 สำหรับ LeRobot/แขนจริง
- แขน SO-101 follower, แหล่งจ่ายไฟ และสาย USB-to-serial ที่ Windows มองเห็นเป็น COM port
- กล้อง USB ที่ Windows เปิดใช้งานได้
- พื้นที่ว่างรอบแขน และสวิตช์หรือวิธีตัดไฟมอเตอร์ที่เข้าถึงได้ทันที

ตรวจ Python 3.12 ด้วย:

```powershell
py -3.12 --version
```

ถ้าคำสั่งนี้หา Python ไม่พบ ให้ติดตั้ง Python 3.12 และ Python Launcher ก่อนทำขั้นตอนฮาร์ดแวร์

### 2. ติดตั้ง environment สำหรับ MuJoCo simulation

ถ้ายังไม่มีโฟลเดอร์ `myenv` ให้สร้างและติดตั้ง dependency ทั้ง simulation และ browser demo ด้วย:

```powershell
python -m venv myenv
.\myenv\Scripts\python.exe -m pip install --upgrade pip
.\myenv\Scripts\python.exe -m pip install -r demo\requirements.txt
```

ตรวจการติดตั้งและรันชุดทดสอบ:

```powershell
.\myenv\Scripts\python.exe -c "import mujoco; print(mujoco.__version__)"
.\myenv\Scripts\python.exe -m unittest discover -s demo\tests -v
```

ถ้ามี `myenv` อยู่แล้ว ไม่ต้องสร้างใหม่ สามารถรันคำสั่ง `pip install -r demo\requirements.txt` ซ้ำเพื่อเติม dependency ที่ขาดได้

### 3. รัน Auto-sort ใน simulation

เปิดหน้าต่าง MuJoCo และให้แขนจำลองคีบแยกสีอัตโนมัติ:

```powershell
.\myenv\Scripts\python.exe -m demo.auto_sort --viewer --seed 7
```

รันแบบไม่มีหน้าต่าง:

```powershell
.\myenv\Scripts\python.exe -m demo.auto_sort --seed 7 --max-steps 3600
```

เปิด browser demo:

```powershell
.\myenv\Scripts\python.exe -m demo.app
```

จากนั้นเปิด `http://localhost:7860` แล้วกด **Auto sort by color** คำสั่งทั้งสามแบบนี้ควบคุมเฉพาะ simulation ไม่ได้สั่งแขนจริง

### 4. ติดตั้ง environment สำหรับกล้องและแขนจริง

ใช้ Python 3.12 และ environment แยกชื่อ `.venv-hardware`:

```powershell
py -3.12 -m venv .venv-hardware
.\.venv-hardware\Scripts\python.exe -m pip install --upgrade pip
.\.venv-hardware\Scripts\python.exe -m pip install -r requirements-hardware.txt
```

ตรวจว่า LeRobot, OpenCV และ MuJoCo import ได้:

```powershell
.\.venv-hardware\Scripts\python.exe -c "import lerobot, cv2, mujoco; print('hardware environment ready')"
```

ถ้ามี `.venv-hardware` อยู่แล้ว ไม่ต้องสร้างใหม่ ให้รันเฉพาะคำสั่งติดตั้ง `requirements-hardware.txt` ซ้ำเมื่อมี dependency ขาด

### 5. สรุปว่าคำสั่งใดทำอะไร

| งาน | คำสั่งหลัก | ทำให้แขนจริงเคลื่อนที่หรือไม่ |
| --- | --- | --- |
| Auto-sort ใน MuJoCo | `python -m demo.auto_sort` ผ่าน `myenv` | ไม่ |
| ตรวจกล้องและสี | `python -m demo.hardware_preflight --camera-index ...` ผ่าน `.venv-hardware` | ไม่ |
| อ่านกล้องและข้อต่อแขน | เพิ่ม `--check-arm --port ...` | ไม่ส่งเป้าหมายการเคลื่อนที่ |
| ตรวจพิกัด Auto-sort แขนจริง | `python -m demo.hardware_auto_sort` โดยไม่ใส่ `--execute` | ไม่ |
| Auto-sort ด้วยแขนจริง | เพิ่ม `--port COMx --execute` | ใช่ ต้องผ่าน calibration และ dry run ก่อน |

## ระบบทำงานอย่างไร

โค้ดหลักอยู่ใน [`auto_sort.py`](auto_sort.py) และใช้สภาพแวดล้อมกับโมเดลใน [`env.py`](../env.py):

1. **เริ่ม simulation และหลบมุมกล้อง** — รีเซ็ตฉาก แล้วสั่งแขนไปยังท่าที่กล้องด้านบนมองเห็นวัตถุได้
2. **ตรวจสีจากภาพ** — อ่านภาพ RGB จากกล้อง `top` ลดขนาดภาพเพื่อประมวลผล แล้วแยกบริเวณสีด้วยช่วง hue/saturation และการรวมพิกเซลที่ติดกัน บริเวณเล็กใช้แทนลูกบาศก์ ส่วนบริเวณใหญ่ใช้แทนกล่อง
3. **แปลงภาพเป็นตำแหน่งในฉาก** — ใช้ตำแหน่งและค่ามุมมองของกล้องในโมเดล MuJoCo แปลงจุดกึ่งกลางของวัตถุในภาพเป็นพิกัดบนระนาบโต๊ะ
4. **คำนวณท่าหยิบและวาง** — ใช้ inverse kinematics (IK) แบบมีขอบเขตข้อต่อ เพื่อหามุมข้อต่อสำหรับเข้าเหนือวัตถุ ลด gripper ลง หนีบ ยก เคลื่อนไปเหนือกล่อง ลดลง และปล่อย
5. **ทำซ้ำทีละสี** — ทำขั้นตอนเดิมตามลำดับสี red, green, blue และ yellow พร้อมตรวจสถานะจากฟิสิกส์ของ MuJoCo ว่าวัตถุอยู่ในกล่องที่ตรงสีหรือไม่
6. **ขยับ simulation และ viewer** — ส่ง joint targets เข้า actuator ของโมเดล จำลองฟิสิกส์ แล้วซิงก์หน้าต่าง viewer ในแต่ละรอบ

การตรวจจับทำงานกับสีและรูปทรงของฉากสาธิตที่กำหนดไว้ จึงไม่ใช่ระบบรู้จำวัตถุทั่วไป และไม่ได้เรียนรู้การหยิบจากข้อมูล
**ตอนนี้โปรเจกต์มี environment จำลองที่เตรียมรูปแบบ action/state ให้เข้ากับ LeRobot ไว้ใน env.py และมีคำแนะนำการใช้งานใน README แต่ ตัวแพ็กเกจ LeRobot ยังไม่ได้ติดตั้งใน myenv**

## มี policy หรือไม่?

มี **กติกาควบคุมที่เขียนไว้ล่วงหน้า**: ตรวจวัตถุ → คำนวณ IK → ทำลำดับหยิบและวาง นี่เรียกได้ว่าเป็น scripted policy หรือ controller แบบกำหนดกฎ แต่ **ไม่มี learned policy** เช่น PPO, SAC, ACT หรือโมเดล neural network ไม่มีการฝึกโมเดลและไม่มี checkpoint ที่สคริปต์นี้โหลดมาใช้

แม้สภาพแวดล้อมใน [`env.py`](../env.py) จะนิยาม observation และ action สำหรับการใช้กับ LeRobot ได้ แต่คำสั่ง `demo.auto_sort` ไม่ได้เรียก policy ภายนอก มันควบคุม MuJoCo โดยตรง

## ถ้าจะทำให้มีpolicyแบบACT
ถ้าจะเพิ่ม learned policy แนะนำเริ่มด้วย ACT แบบ imitation learning: ให้โมเดลเรียนจากตัวอย่างการหยิบวาง แทนการให้ RL ลองผิดลองถูกเอง

ขั้นตอนหลัก ๆ คือ:
เตรียมข้อมูลสาธิต — เก็บภาพจากกล้องและสถานะข้อต่อ พร้อมคำสั่งข้อต่อที่ถูกต้องในแต่ละจังหวะ ข้อมูลควรครอบคลุมตำแหน่งวัตถุหลายแบบและมีทั้ง 4 สี
สร้างชุดข้อมูล LeRobot — environment ปัจจุบันมีภาพ front/wrist, สถานะข้อต่อ และ action เป็นเป้าหมายข้อต่อ ดูโครงสร้างได้ใน README ของ environment และ env.py
ฝึก ACT — ให้โมเดลเรียนจับคู่ภาพกับสถานะข้อต่อ เพื่อทำนายคำสั่งหยิบและวาง แล้วบันทึก checkpoint
ทดสอบใน MuJoCo — ใช้ seed/ตำแหน่งที่ไม่ได้อยู่ในชุดฝึก วัดว่าเรียงครบได้กี่ครั้ง ก่อนลองกับแขนจริง


## ขอบเขตและการต่อกับแขนจริง

`demo.auto_sort` ควบคุมเฉพาะโมเดลใน MuJoCo โดยใช้ภาพจาก renderer และส่งคำสั่งไปยัง MuJoCo actuators ส่วน `demo.hardware_auto_sort` เป็น hardware backend แยกสำหรับกล้องและ SO-101 จริง ก่อนใส่ `--execute` ต้องตั้งค่าและ calibrate มอเตอร์ ปรับเทียบกล้องกับ workspace จริง ผ่าน dry run และเตรียมมาตรการหยุดฉุกเฉิน โหมดฮาร์ดแวร์ยังต้องตรวจสอบกับอุปกรณ์จริงทีละขั้น ไม่ถือว่าปลอดภัยจากผลทดสอบ simulation เพียงอย่างเดียว

### เตรียมอุปกรณ์ก่อนเพิ่มโหมดฮาร์ดแวร์

LeRobot ติดตั้งไว้ใน `.venv-hardware` แยกจาก `myenv` แล้ว แต่ยังไม่ได้กำหนดพอร์ตหรือกล้องจริง และแขนอาจยังไม่ calibrate จึงยังไม่ควรรันตัวคัดแยกกับแขนจริง หากต้องสร้าง environment ใหม่ ให้ทำตามหัวข้อ **ติดตั้ง environment สำหรับกล้องและแขนจริง** ด้านบน ไฟล์ `requirements-hardware.txt` ติดตั้ง LeRobot 0.7.0 พร้อม Feetech และ dependency ของ MuJoCo โดยไม่เปลี่ยน `myenv`

ทำตามลำดับต่อไปนี้เพื่อเตรียมแขน:

1. ประกอบและจ่ายไฟให้แขน พร้อมจัดพื้นที่ให้แขนขยับได้โดยไม่ชนคนหรือสิ่งของ
2. ติดตั้ง LeRobot พร้อม Feetech SDK ตามคู่มือ แนะนำให้ใช้ environment แยกจาก `myenv` ซึ่งใช้รัน MuJoCo
3. หา COM port ของแขน follower ด้วย:

   ```powershell
   .\.venv-hardware\Scripts\lerobot-find-port.exe
   ```

   ใช้คำสั่งนี้ดูหมายเลขกล้องที่เชื่อมต่อ:

   ```powershell
   .\.venv-hardware\Scripts\lerobot-find-cameras.exe
   ```

4. ถ้าเป็นแขนที่เพิ่งประกอบและมอเตอร์ยังไม่ได้ตั้ง ID/baudrate ให้ตั้งมอเตอร์ก่อน โดยแทน `COMx` ด้วยพอร์ตจริง:

   ```powershell
   .\.venv-hardware\Scripts\lerobot-setup-motors.exe --robot.type=so101_follower --robot.port=COMx
   ```

   ทำตามข้อความบนหน้าจอและขั้นตอนการต่อมอเตอร์ของชุด SO-101 คำสั่งนี้ใช้ตั้งค่ามอเตอร์ จึงไม่ควรรันซ้ำกับแขนที่ตั้ง ID ครบแล้วโดยไม่ทราบสถานะเดิม จากนั้น calibrate แขน:

   ```powershell
   .\.venv-hardware\Scripts\lerobot-calibrate.exe --robot.type=so101_follower --robot.port=COMx --robot.id=so101_follower
   ```

   ทำตามคำแนะนำบนหน้าจอ โดยเว้นพื้นที่ให้แขนขยับตลอดช่วงข้อต่อ ห้ามเริ่ม auto-sort ระหว่าง calibrate

5. ระบุกล้องที่จะใช้และตำแหน่งติดตั้ง แล้วจึงปรับเทียบกล้องกับโต๊ะและพิกัดแขน

### นำหมายเลขกล้องและพอร์ตแขนไปใส่ตรงไหน

ไม่ต้องนำหมายเลขไปเขียนทับใน `auto_sort.py` ให้ส่งค่าเข้าโปรแกรมผ่าน command line โดยรันคำสั่งจากโฟลเดอร์ `.inspection\reference-space`:

```powershell
cd C:\Users\chaya\SO_ARM101\.inspection\reference-space
```

- **หมายเลขกล้อง** เป็นตัวเลข เช่น `0`, `1` หรือ `2` ใส่ต่อท้าย `--camera-index`
- **พอร์ตแขน follower** เป็นชื่อ COM port เช่น `COM5` ใส่ต่อท้าย `--port`
- ถ้าใช้หมายเลขกล้องหรือพอร์ตอื่น ให้แทนค่าตัวอย่างด้วยค่าที่ `lerobot-find-cameras.exe` และ `lerobot-find-port.exe` ตรวจพบ

ถ้าแขนยังไม่เคย calibrate ให้ทำก่อน โดยตัวอย่างนี้ใช้แขนที่พอร์ต `COM5`:

```powershell
.\.venv-hardware\Scripts\lerobot-calibrate.exe --robot.type=so101_follower --robot.port=COM5 --robot.id=so101_follower
```

ระหว่าง calibrate ต้องเคลียร์พื้นที่รอบแขนและทำตามคำแนะนำบนหน้าจอให้ครบ โปรแกรมจะบันทึก calibration โดยอ้างอิงจาก `robot.id` ดังนั้นควรใช้ `so101_follower` เหมือนกันในคำสั่งต่อไป

หลังติดตั้งกล้อง สามารถเช็กภาพจริงและการตรวจสีอย่างเดียวได้โดยไม่เชื่อมแขน ตัวอย่างนี้ใช้กล้องหมายเลข `0`:

```powershell
.\.venv-hardware\Scripts\python.exe -m demo.hardware_preflight --camera-index 0
```

ถ้าคำสั่งทำงาน จะรายงานขนาดภาพและผลตรวจสี `red`, `green`, `blue` และ `yellow` ถ้าขึ้น `not detected` ให้ตรวจแสง มุมกล้อง สีของวัตถุ และตรวจว่าหมายเลขกล้องถูกต้อง

หลัง calibrate แล้ว จึงค่อยเช็กกล้องและอ่านสถานะข้อต่อพร้อมกัน ตัวอย่างนี้ใช้กล้อง `0` และแขน `COM5`:

```powershell
.\.venv-hardware\Scripts\python.exe -m demo.hardware_preflight --camera-index 0 --check-arm --port COM5
```

คำสั่ง preflight ทั้งสองแบบ **ไม่สั่งให้แขนเคลื่อนที่หรือคีบวัตถุ** แบบที่มี `--check-arm` จะเชื่อมต่อเพื่ออ่านค่าข้อต่อเท่านั้น แต่ LeRobot จะ configure มอเตอร์ขณะเชื่อมต่อและอาจปิด torque เมื่อตัดการเชื่อมต่อ จึงต้องพยุงแขนและเคลียร์พื้นที่ไว้เสมอ

### ปัญหาที่พบบ่อยระหว่างติดตั้งและตรวจอุปกรณ์

- `py -3.12` ใช้ไม่ได้ — ติดตั้ง Python 3.12 พร้อม Python Launcher แล้วเปิด PowerShell ใหม่
- `No module named ...` — ตรวจว่าอยู่ใน `.inspection\reference-space` และใช้ `python.exe` จาก environment ที่ตรงกับงาน จากนั้นติดตั้ง requirements ของ environment นั้นซ้ำ
- เปิดกล้องไม่ได้ — ปิดโปรแกรมอื่นที่กำลังใช้กล้อง แล้วตรวจ `--camera-index` ด้วย `lerobot-find-cameras.exe`
- เปิด COM port ไม่ได้ — ตรวจสายและไฟเลี้ยง ปิดโปรแกรมอื่นที่ใช้พอร์ต แล้วรัน `lerobot-find-port.exe` ใหม่ หมายเลข COM อาจเปลี่ยนเมื่อเสียบ USB คนละช่อง
- แจ้งว่าไม่มี calibration — รัน `lerobot-calibrate.exe` โดยใช้ `--robot.id=so101_follower` และพอร์ตเดียวกับ preflight
- ตรวจสีไม่พบ — ตรวจแสง เงา มุมกล้อง และให้สีของวัตถุอยู่ในชุดแดง เขียว น้ำเงิน หรือเหลืองที่ตัวตรวจจับรองรับ

### ทำอย่างไรให้แขนคีบวัตถุอัตโนมัติ

การรู้หมายเลขกล้องและ COM port ยังไม่เพียงพอ ต้องสร้างไฟล์ calibration ที่บอกความสัมพันธ์ระหว่างภาพกับฐานแขน รวมทั้งกำหนดพื้นที่ปลอดภัยก่อน โปรแกรม `demo.hardware_auto_sort` จะไม่ยอมทำงานถ้าไฟล์ยังมี `configured: false` หรือมีค่าที่ไม่ครบ

#### 1. สร้างไฟล์ตั้งค่าจากแม่แบบ

```powershell
Copy-Item demo\hardware_auto_sort.example.json demo\hardware_auto_sort.json
```

เปิด `demo\hardware_auto_sort.json` แล้วกรอกค่าที่วัดจากชุดจริง:

- `image_size_px` — ความละเอียดกล้องที่ใช้ตลอดการปรับเทียบและรัน
- `calibration.robot_points_m` — พิกัด `[X, Y]` ของจุดอ้างอิงบนโต๊ะอย่างน้อย 4 จุด วัดเป็นเมตรจากฐานแขน จุดต้องกระจายรอบพื้นที่และไม่เรียงอยู่บนเส้นเดียวกัน
- `camera_position_m` — ตำแหน่ง `[X, Y, Z]` ของจุดศูนย์กลางกล้องเทียบกับฐานแขน หน่วยเมตร
- `workspace_m` — ช่วง `[ต่ำสุด, สูงสุด]` ของ X/Y/Z ที่อนุญาตให้จุดอ้างอิงปลายแขนเคลื่อนที่ ต้องตั้งให้แคบกว่าพื้นที่จริงและไม่รวมโต๊ะ คน หรือสิ่งกีดขวาง
- `geometry_m` — ระดับโต๊ะ ความสูงวัตถุ จุดยก และ waypoint ที่ปลอดภัย หน่วยเมตร
- `camera_park_degrees` — ท่าข้อต่อที่แขนไม่บังกล้อง ต้องตรวจทีละข้อต่อด้วยความเร็วต่ำก่อนใช้
- `motion` — ความถี่ควบคุม ระยะขยับสูงสุดต่อคำสั่ง และเวลารันสูงสุด ค่าเริ่มต้นในแม่แบบตั้งไว้สำหรับการเริ่มทดสอบแบบช้า

แกนพิกัดต้องใช้กรอบเดียวกับฐานแขนตลอดทั้งไฟล์: X ตามแนวเข้า–ออกจากฐาน, Y ด้านข้าง และ Z ชี้ขึ้น ห้ามใช้พิกัดจากขอบภาพหรือมุมโต๊ะแทนพิกัดจากฐานแขน

#### 2. คลิกจุดอ้างอิงในภาพ

กรอก `calibration.robot_points_m` ให้ครบก่อน แล้วรัน:

```powershell
.\.venv-hardware\Scripts\python.exe -m demo.hardware_camera_calibrate --config demo\hardware_auto_sort.json --camera-index 0
```

คลิกจุดบนภาพตามลำดับเดียวกับ `robot_points_m` กด `R` เพื่อล้างแล้วเริ่มใหม่, `S` เพื่อบันทึก หรือ `Esc` เพื่อยกเลิก โปรแกรมจะเขียน `image_points_px` ให้ แต่จะคง `configured: false` ไว้

ตรวจค่าทุกช่องอีกครั้ง จากนั้นจึงเปลี่ยน `configured` เป็น `true` การย้ายกล้อง โต๊ะ หรือฐานแขนภายหลังทำให้ calibration เดิมใช้ไม่ได้ ต้องปรับเทียบใหม่

#### 3. Dry run โดยไม่สั่งมอเตอร์

ตรวจหนึ่งสีแรกก่อน คำสั่งนี้เปิดกล้อง ตรวจสี แปลงพิกัด และตรวจ workspace แต่ไม่เชื่อมต่อหรือสั่งมอเตอร์:

```powershell
.\.venv-hardware\Scripts\python.exe -m demo.hardware_auto_sort --config demo\hardware_auto_sort.json --camera-index 0
```

เมื่อต้องการตรวจว่ากล้องเห็นลูกบาศก์และกล่องครบทั้งสี่สี:

```powershell
.\.venv-hardware\Scripts\python.exe -m demo.hardware_auto_sort --config demo\hardware_auto_sort.json --camera-index 0 --max-colors 4
```

ต้องเห็นพิกัดของ cube/box ทุกสีเป็น `inside=True` และค่าพิกัดที่พิมพ์ต้องตรงกับตำแหน่งที่วัดด้วยไม้บรรทัดก่อนทำขั้นต่อไป การเห็นเพียง `inside=True` ยังไม่ยืนยันว่า calibration ถูกต้อง ถ้าขึ้น `not detected`, `inside=False`, พิกัดคลาดเคลื่อน หรือโปรแกรมปฏิเสธไฟล์ ห้ามเติม `--execute`

#### 4. ทดลองแขนจริงหนึ่งสี

เคลียร์พื้นที่ จับสวิตช์ตัดไฟให้พร้อม พยุงแขนในกรณี torque ถูกปิด และแทน `0`/`COM5` ด้วยหมายเลขจริง จากนั้นรัน:

```powershell
.\.venv-hardware\Scripts\python.exe -m demo.hardware_auto_sort --config demo\hardware_auto_sort.json --camera-index 0 --port COM5 --max-colors 1 --execute
```

เมื่อใส่ `--execute` โปรแกรมจะตรวจภาพก่อน แล้วนับถอยหลัง 5 วินาทีก่อนส่งเป้าหมายข้อต่อ กด `Ctrl+C` หรือตัดไฟทันทีเมื่อทิศทางผิด ระยะผิด กล้องหลุด หรือมีโอกาสชน เมื่อโปรแกรมจบหรือล้มเหลวจะตัด torque ตอน disconnect จึงต้องเตรียมพยุงแขน

#### 5. รันครบสี่สี

ใช้คำสั่งนี้เฉพาะเมื่อการทดสอบหนึ่งสีหยิบและวางตรงตำแหน่งซ้ำได้อย่างปลอดภัยแล้ว:

```powershell
.\.venv-hardware\Scripts\python.exe -m demo.hardware_auto_sort --config demo\hardware_auto_sort.json --camera-index 0 --port COM5 --max-colors 4 --execute
```

คำสั่งนี้คำนวณ IK ด้วยโมเดล SO-101 แล้วจำกัดเป้าหมายด้วย joint limits, workspace, ระยะขยับต่อคำสั่ง และเวลารันสูงสุด อย่างไรก็ตามปุ่มหยุดฉุกเฉินทางกายภาพและการเฝ้าดูแขนตลอดเวลายังจำเป็น

ก่อนเปิด `--execute` ต้องตรวจตามลำดับนี้:

1. **Calibrate มอเตอร์** — รัน `lerobot-calibrate` และตรวจว่าคำสั่ง preflight อ่านข้อต่อทั้ง 6 ค่าได้
2. **ยึดกล้องให้อยู่กับที่** — กล้องควรมองเห็นพื้นที่วางลูกบาศก์และกล่องทั้งหมด โดยแขนไม่บังภาพในท่าเริ่มต้น
3. **ปรับเทียบกล้องกับโต๊ะและฐานแขน** — หาความสัมพันธ์ระหว่างพิกเซลในภาพกับพิกัดจริงบนโต๊ะ รวมถึงตำแหน่งและทิศของกล้องเทียบกับฐานแขน การย้ายกล้องหลังปรับเทียบจะทำให้ตำแหน่งหยิบคลาดเคลื่อนและต้องปรับเทียบใหม่
4. **ตรวจ hardware backend** — รัน dry run ให้การตรวจสีและพิกัดผ่าน แล้วทดสอบการส่งเป้าหมายข้อต่อและ gripper ผ่าน LeRobot กับหนึ่งสี
5. **เพิ่มข้อจำกัดความปลอดภัย** — จำกัดพื้นที่ทำงาน ความเร็ว ระยะยก และแรงหนีบ พร้อมมีวิธีหยุดฉุกเฉินและหยุดเมื่อกล้องหรือมอเตอร์ผิดพลาด
6. **ทดสอบทีละขั้น** — เริ่มจากตรวจสีโดยไม่ขยับแขน จากนั้นทดสอบท่าเหนือวัตถุโดยยังไม่ลดแขน แล้วจึงทดลองหยิบหนึ่งชิ้นด้วยความเร็วต่ำ ก่อนเปิดการทำซ้ำหลายสี

เมื่อปรับเทียบพิกัดจริงครบแล้ว ลำดับการทำงานคือ: รันโปรแกรมบนคอม → ตรวจภาพและ workspace → นับถอยหลัง → อ่านภาพจากกล้อง → หาลูกบาศก์และกล่องสีเดียวกัน → แปลงตำแหน่งภาพเป็นพิกัดแขน → คำนวณ IK → เคลื่อนแขนไปหยิบ → หนีบ ยก และนำไปวาง → ทำซ้ำตามจำนวนสีที่กำหนด

**ห้ามนำ joint target จาก simulation ส่งเข้ามอเตอร์จริงโดยตรง** เพราะตำแหน่งฐานแขน กล้อง โต๊ะ ระยะข้อต่อ และสิ่งกีดขวางของจริงอาจไม่ตรงกับโมเดล


ขั้นต่อไปคือ calibrate มอเตอร์ รัน preflight สร้าง `hardware_auto_sort.json` จากค่าที่วัดจริง แล้วผ่าน dry run ก่อนทดลอง `--max-colors 1 --execute`

เมื่อส่วนนั้นพร้อม การทำงานจะเป็นแบบนี้: คุณสั่งรันโปรแกรมบนคอม → แขนอ่านภาพจากกล้อง → หา cube และกล่องสีเดียวกัน → คำนวณตำแหน่งหยิบวาง → ส่งเป้าหมายข้อต่อและ gripper ไปที่ SO-101 แล้วทำซ้ำทีละสี โดยเริ่มทดสอบช้า ๆ ทีละการเคลื่อนไหวก่อน

***ชุดสีที่ตั้งไว้ตอนนี้คือ แดง เขียว น้ำเงิน (blue) และเหลือง แต่ไม่ได้ “เรียนรู้” สีเอง***

ตัวตรวจจับใน auto_sort.py ใช้เกณฑ์สีที่กำหนดไว้ล่วงหน้าเพื่อแยกลูกบาศก์กับกล่อง แล้วจับคู่ตามสี และทำตามลำดับ แดง → เขียว → น้ำเงิน → เหลือง ไม่ใช่โมเดล AI ที่เรียนจากตัวอย่าง 


## ผลทดสอบ

การรันด้วย seed 7 ใน MuJoCo เรียงครบ 4/4 ชิ้นใน 2163 simulation steps และชุดทดสอบของ demo ผ่าน 25 tests ผลนี้ยืนยันการทำงานใน simulation และส่วนตรวจสอบแบบไม่สั่งมอเตอร์เท่านั้น ยังไม่ใช่การยืนยันว่า Auto-sort ทำงานถูกต้องกับแขนจริง ต้องทดสอบกับอุปกรณ์จริงตามลำดับในคู่มือ
