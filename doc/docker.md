## 0. Prerequisite

- [Docker](https://docs.docker.com/)

  - For a lightweight installation, see [Docker Engine](https://docs.docker.com/engine/install/ubuntu/).  
  - For a GUI-based setup, see [Docker Desktop](https://docs.docker.com/desktop/setup/install/linux/).  


- [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)

  After installation, make sure you complete the *Configuring Docker* section. Otherwise, the container may fail to access the host GPU.  

## 1. Build Docker Image
```bash
git clone https://github.com/CUHKSiriusLeggedRobotTeam/Sirius_RL_Gym.git
cd Sirius_RL_Gym
docker build -t sirius_rl_gym .
```

## 2. Quick start

### 2.1 Start container
```bash
# Allow container to access the host display
xhost +

# Make sure a joystick is connected
docker run -it \
  --network=host \
  --gpus=all \
  --ipc=host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  --mount type=bind,src=/dev/input,dst=/dev/input \
  -v /run/udev:/run/udev:ro \
  --device-cgroup-rule='c 13:* rmw' \
  --device=/dev/input/js0 \
  --security-opt seccomp=unconfined \
  --security-opt apparmor=unconfined \
  sirius_rl_gym:latest /bin/bash
```

All following steps should be executed inside the container.
You can use tmux to manage multiple terminals.

### 2.2 RL training

In this part, activate the conda environment before running scripts:
```bash
conda activate sirius
```

```bash
# train
cd legged_gym
python legged_gym/scripts/train.py --task=sirius --headless
```

```bash
# export JIT model
python legged_gym/scripts/export_JIT_model.py --task=sirius --model_path=<path/to/model>
# eg: python legged_gym/scripts/export_JIT_model.py --task=sirius --model_path=./logs/sirius_diff_release/Jun12_18-48-39_/model_300.pt
# The model will be saved as [policy.jit] in the same directory as the .pt model.
```

### 2.3 Sim2Sim

In this part, deactivate the conda environment before running ROS 2 packages:
```bash
conda deactivate
```

```bash
# In current terminal
cd deploy/sim2sim/scripts
bash ./launch_simulator.sh

# Open another terminal
cd deploy/sim2sim/scripts
bash ./launch_ros2topic.sh
```

```bash
# Run Secondary Development RL controller demo
cd deploy/ros2_RL_controller
source install/setup.bash
ros2 run RL_controller joystick
```