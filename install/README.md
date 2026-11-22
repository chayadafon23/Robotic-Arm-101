***Config Project***
open terminal and follow on the steps below.

1. set environment
<<<<<<< HEAD
<<<<<<< HEAD
go to path:
cd Robotic-Arm-101\.inspection\reference-space
run this command to set env:
python -m venv myenv
install MuJoCo:
.\myenv\Scripts\python.exe -m pip install -r requirements.txt
Check whether the installation was successful:
=======
=======
>>>>>>> bd371e9c8fbdb515088b84dbd0d03dd121a665e5

go to path:

cd Robotic-Arm-101\.inspection\reference-space

run this command to set env:

python -m venv myenv

install MuJoCo:

.\myenv\Scripts\python.exe -m pip install -r requirements.txt

Check whether the installation was successful:

<<<<<<< HEAD
>>>>>>> simulation-model-second-version
=======
>>>>>>> bd371e9c8fbdb515088b84dbd0d03dd121a665e5
.\myenv\Scripts\python.exe -c "import mujoco; print(mujoco.__version__)"

2. run robotic on MuJoCo:

first choice:
To automatically sort the colored cubes in MuJoCo's interactive viewer, run this from `.inspection\reference-space`:


run this command to run on MuJoCo:

.\myenv\Scripts\python.exe -m demo.auto_sort --viewer --seed 7

This opens the native MuJoCo window with a top-down view and runs the controller while the simulation is visible. Keep the terminal open to see sorting progress. **To run without opening a window**, omit `--viewer`:
.\myenv\Scripts\python.exe -m demo.auto_sort --seed 7 --max-steps 3600

second choice:
The script prints progress and sorts the four cubes into their matching boxes when successful. The old `mujoco.viewer.launch(...)` command only displays the scene; it does not run the sorting controller. To see the interactive browser demo, start `python -m demo.app`, open `http://localhost:7860`, and click **Auto sort by color**.


--optional: run on Docker--
build:

docker build -t so101-tabletop .

run:

docker run --rm -p 7860:7860 so101-tabletop

open on browser and paste this:
http://localhost:7860
