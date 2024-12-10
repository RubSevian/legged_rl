# legged_rl
Reinforcement learning framework for legged robot. 


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
catkin config --install
catkin build -DPYTHON_EXECUTABLE=/usr/bin/python3 -DCMAKE_EXPORT_COMPILE_COMMANDS=1
```