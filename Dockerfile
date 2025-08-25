FROM ros:humble-ros-base-jammy
WORKDIR /sirius
COPY . .
ENV DEBIAN_FRONTEND=noninteractive

# Change ros2 packge source
RUN rm /etc/apt/sources.list.d/ros2.sources && \
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros2-archive-keyring.gpg] http://mirrors.ustc.edu.cn/ros2/ubuntu $(lsb_release -sc) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install miscellaneous
RUN apt update
RUN apt install -y tmux wget vim ros-humble-joy

# Install iceoryx
RUN apt install -y gcc g++ cmake libacl1-dev libncurses5-dev pkg-config liblcm-dev libglfw3 libusb-1.0-0
RUN cd deploy/iceoryx && \
	cmake -Bbuild -Hiceoryx_meta && \
    cmake --build build && \
	cmake --build build --target install

# Install training dependencies
RUN apt-get install -y software-properties-common && \
	add-apt-repository -y ppa:deadsnakes/ppa && \
	apt-get update && \
	apt-get install -y libpython3.8
SHELL ["/bin/bash", "-lc"]
RUN mkdir -p /etc/pip && printf "[global]\nindex-url = https://pypi.tuna.tsinghua.edu.cn/simple\n" > /etc/pip.conf

RUN wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh
RUN bash ./Miniconda3-latest-Linux-x86_64.sh -b -p $HOME/miniconda3
ENV PATH=/root/miniconda3/condabin:$PATH
RUN conda init
RUN source $HOME/miniconda3/etc/profile.d/conda.sh && \
    conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/main && \
    conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/r && \
    conda create -n sirius -y python=3.8 && \
    conda activate sirius && \
    pip install torch==1.13.0+cu116 torchvision==0.14.0+cu116 torchaudio==0.13.0 --extra-index-url https://download.pytorch.org/whl/cu116 && \
    wget https://developer.nvidia.com/isaac-gym-preview-4 && \ 
    tar -zxvf ./isaac-gym-preview-4 && \
    cd isaacgym/python && pip install -e . && \
    cd ../../rsl_rl && pip install -e . && \
    cd ../legged_gym &&  pip install -e . && \
    pip uninstall -y numpy && pip install numpy==1.23.5 tensorboard && \
    pip uninstall -y protobuf && pip install protobuf==3.20.3
RUN apt-get install -y --no-install-recommends python3-pip && \
    pip install empy==3.3.2 rospkg lark && \
    pip install torch==1.13.0+cu116 torchvision==0.14.0+cu116 torchaudio==0.13.0 --extra-index-url https://download.pytorch.org/whl/cu116

# Build ros2 package
RUN source /opt/ros/humble/setup.bash && \
    cd deploy/sim2sim && \
    colcon build
RUN source /opt/ros/humble/setup.bash && \
    cd deploy/ros2_RL_controller && \
    colcon build
