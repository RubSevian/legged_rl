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
