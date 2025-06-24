## Installation

### System Requirements

- **Operating System**: Recommended Ubuntu 18.04 or later  
- **GPU**: Nvidia GPU  
- **Driver Version**: Recommended version 525 or later  


### Download Isacc Gym Preview 4
```bash
wget https://developer.nvidia.com/isaac-gym-preview-4
tar -zxvf ./isaac-gym-preview-4
```
### Create Conda Environment

```bash
conda env create -f conda_env.yml
conda activate sirius
```

### Install PyTorch
```bash
pip install torch==1.13.0+cu116 torchvision==0.14.0+cu116 torchaudio==0.13.0 --extra-index-url https://download.pytorch.org/whl/cu116
```

### Install Isaac Gym
```bash
cd isaacgym/python && pip install -e . && cd ../..
```
- Try running an example `cd examples && python 1080_balls_of_solitude.py`

### Install rsl_rl & legged_gym
``` bash
cd rsl_rl && pip install -e . && cd .. 
cd legged_gym &&  pip install -e . && cd .. 
```
