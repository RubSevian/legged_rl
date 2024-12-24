# legged_rl

A [ROS Control](https://wiki.ros.org/ros_control) based reinforcement learning framework for legged robot's traing and deployment. 

# Installation

## Mujoco

```bash
sudo apt install libglfw3-dev libxinerama-dev libxcursor-dev libxi-dev
```
```bash
# it is not necessary to download the repo in the catkin workspace, you can download, build and install it anywhere you like.
git clone https://github.com/google-deepmind/mujoco.git
mkdir build && cd build
cmake ..
make -j4
sudo make install # install to /usr/local/bin
```
Test:
```bash
simulate
```
If the mujoco simulator pops up, the installation is successful.

## Libtorch

Download libtorch from [pytorch](https://pytorch.org/) (**CPU** version is efficient enough for rl network). Extract it to a `legged_rl/libs`. The final path should be `legged_rl/libs/libtorch`.

<!-- Possible issues:

- Libtorch cannot find CUDA: 
  - Make sure you have CUDA installed. If not, you may refer to: https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html#ubuntu
  - Make sure you have set the environment variable `CUDA_HOME` to the path of your CUDA installation. You can add the following line to your `.bashrc` file:
    ```bash
    export CUDA_HOME=/usr/local/cuda
    export CUDACXX=/usr/local/cuda/bin/nvcc
    ``` -->

## Build this repo

Clone this repo:
```bash
# in ros workspace's src folder
git clone https://github.com/zitongbai/legged_rl.git --recurse-submodules
```

Build:
```bash
# in ros workspace root folder
catkin config --install
catkin build -DPYTHON_EXECUTABLE=/usr/bin/python3 -DCMAKE_EXPORT_COMPILE_COMMANDS=1
```

## Legged gym

1. Create a conda env with python 3.6, 3.7 or 3.8 (3.8 recommended) to train and play RL policy.
2. Install [pytorch](https://pytorch.org/) with cuda 12.1 (11.x might work as well but not tested)
3. Install Isaac Gym (not necessary in this workspace)
   - Download and install Isaac Gym Preview 4 (Preview 2 will not work!) from https://developer.nvidia.com/isaac-gym
   - `cd isaacgym/python && pip install -e .`
   - Try running an example `cd examples && python 1080_balls_of_solitude.py`
   - For troubleshooting check docs `isaacgym/docs/index.html`)
4. Install rsl_rl (PPO implementation)
   - We have a forked version in this repo.
   - `cd rsl_rl && git checkout v1.0.2 && pip install -e .` 
5. Install legged_gym
    - We have a forked version in this repo.
   - `cd legged_gym && pip install -e .`

You can refer to https://github.com/leggedrobotics/legged_gym for detailed information. 

# Usage

## Train and export
Train go2 with HIMLoco algorithm: 
```bash
cd legged_gym
python legged_gym/scripts/train.py --task=go2_him --headless --max_iterations=1000
```

After training, play once to export the jit file:
```bash
# in legged_rl\legged_gym
python legged_gym/scripts/play_him.py --task=go2_him
```

Export rl config: 
```bash
# in ros workspace root folder
python src/legged_rl/legged_gym/legged_gym/scripts/export_config.py --target_dir=src/legged_rl/legged_robot_example/unitree_go2_description/config --task=go2_him
```

## Sim2Sim in mujoco

Run mujoco:

```bash
roslaunch unitree_go2_description mujoco.launch
```

in another terminal, run controller:

```bash
roslaunch unitree_go2_description load_controllers.launch
```

Then you can use a joystick to publish commands.

# TODO

- [ ] Detailed doc
- [ ] Test unitree_sdk2_hw on real robot
- [ ] Sim to real deployment and test
- [ ] More RL algorithms

# Known Issues:

- [Robot penetrate the terrain](https://github.com/unitreerobotics/unitree_mujoco/issues/34#issue-2754752899)

# Acknowledge and Reference

- [legged_control](https://github.com/qiayuanl/legged_control)
- [legged_gym](https://github.com/leggedrobotics/legged_gym)
- [rsl_rl](https://github.com/leggedrobotics/rsl_rl)
- [HIMLoco](https://github.com/OpenRobotLab/HIMLoco)
- [rl_sar](https://github.com/fan-ziqi/rl_sar)

