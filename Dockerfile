ARG AUTOWARE_VERSION=latest-melodic-cuda

FROM autoware/autoware:$AUTOWARE_VERSION

USER autoware
ENV USERNAME autoware

WORKDIR /home/autoware

# Update autoware/simulation package version to latest.
COPY --chown=autoware update_sim_version.patch /home/$USERNAME/Autoware
RUN patch ./Autoware/autoware.ai.repos /home/$USERNAME/Autoware/update_sim_version.patch

ENV http_proxy=http://192.168.121.170:7890 
ENV https_proxy=http://192.168.121.170:7890
RUN cd /home/$USERNAME/Autoware \
    && vcs import src < autoware.ai.repos \
    && git --git-dir=./src/autoware/simulation/.git --work-tree=./src/autoware/simulation pull 

# Change code in autoware/simulation package.
COPY --chown=autoware update_sim_code.patch /home/$USERNAME/Autoware/src/autoware/simulation
RUN cd /home/$USERNAME/Autoware/src/autoware/simulation \
    && git apply update_sim_code.patch

# Compile with colcon build.
ENV http_proxy=http://192.168.121.170:7890 
ENV https_proxy=http://192.168.121.170:7890
RUN cd ./Autoware \
    && source /opt/ros/melodic/setup.bash \
    && AUTOWARE_COMPILE_WITH_CUDA=1 colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# CARLA PythonAPI
RUN mkdir ./PythonAPI
# ADD --chown=autoware https://carla-releases.s3.eu-west-3.amazonaws.com/Backup/carla-0.9.11-py2.7-linux-x86_64.egg ./PythonAPI
ADD --chown=autoware carla-api/carla-0.9.13-py2.7-linux-x86_64.egg ./PythonAPI
RUN echo "export PYTHON2_EGG=$(ls /home/autoware/PythonAPI | grep py2.)" >> .bashrc \
    && echo "export PYTHONPATH=\$PYTHONPATH:~/PythonAPI/\$PYTHON2_EGG" >> .bashrc

RUN sudo sed -i '/developer\.download\.nvidia\.com\/compute\/cuda\/repos/d' /etc/apt/sources.list
RUN wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/cuda-keyring_1.0-1_all.deb \
    && sudo dpkg -i cuda-keyring_1.0-1_all.deb
RUN sudo apt-key del 7fa2af80 \
    && sudo rm /etc/apt/sources.list.d/cuda.list \
    && sudo rm /etc/apt/sources.list.d/nvidia-ml.list

# CARLA ROS Bridge
# There is some kind of mismatch between the ROS debian packages installed in the Autoware image and
# the latest ros-melodic-ackermann-msgs and ros-melodic-derived-objects-msgs packages. As a
# workaround we use a snapshot of the ROS apt repository to install an older version of the required
# packages.
USER root
RUN echo "nameserver 8.8.8.8" > /etc/resolv.conf && \
    echo "nameserver 8.8.4.4" >> /etc/resolv.conf
ENV http_proxy=http://192.168.121.170:7890 
ENV https_proxy=http://192.168.121.170:7890
RUN rm -f /etc/apt/sources.list.d/ros1-latest.list
# RUN apt-key adv \
#     --keyserver-options http-proxy=http://192.168.121.169:7890/ \
#     --keyserver hkps://keyserver.ubuntu.com:443 \
#     --recv-keys 4B63CF8FDE49746E98FA01DDAD19BAB3CBF125EA
RUN wget -O- "http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xAD19BAB3CBF125EA" > /tmp/ros-key.gpg
RUN sudo apt-key add /tmp/ros-key.gpg && \
    rm /tmp/ros-key.gpg
ENV http_proxy=http://192.168.121.170:7890 
ENV https_proxy=http://192.168.121.170:7890
RUN sh -c 'echo "deb http://snapshots.ros.org/melodic/2020-08-07/ubuntu $(lsb_release -sc) main" >> /etc/apt/sources.list.d/ros-snapshots.list' \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
        python-pip \
        python-wheel \
        ros-melodic-ackermann-msgs \
        ros-melodic-derived-object-msgs
RUN rm -rf /var/lib/apt/lists/*

ENV http_proxy=http://192.168.121.170:7890 
ENV https_proxy=http://192.168.121.170:7890
RUN pip install simple-pid pygame networkx==2.2

USER autoware

ENV http_proxy=http://192.168.121.170:7890 
ENV https_proxy=http://192.168.121.170:7890
RUN git clone --recurse-submodules https://github.com/cfs4819/ros-bridge.git \
    && ros-bridge/install_dependencies.sh

# Update code in carla-ros-bridge package and fix the tf tree issue.
# The fix has been introduced in latest version (since 0.9.12):
# https://github.com/carla-simulator/ros-bridge/pull/570/commits/9f903cf43c4ef3dd0b909721e044c62a8796f841
# COPY --chown=autoware update_ros_bridge.patch /home/$USERNAME/ros-bridge
# RUN cd /home/$USERNAME/ros-bridge \
#     && git apply update_ros_bridge.patch

# CARLA Autoware agent
COPY --chown=autoware . ./carla-autoware

RUN mkdir -p carla_ws/src
RUN cd carla_ws/src \
    && ln -s ../../ros-bridge \
    && ln -s ../../carla-autoware/carla-autoware-agent \
    && cd .. \
    && source /opt/ros/melodic/setup.bash \
    && catkin_make

RUN echo "export CARLA_AUTOWARE_CONTENTS=~/autoware-contents" >> .bashrc \
    && echo "source ~/carla_ws/devel/setup.bash" >> .bashrc \
    && echo "source ~/Autoware/install/setup.bash" >> .bashrc

USER root

# (Optional) Install vscode
# ENV http_proxy=http://192.168.121.170:7890 
# ENV https_proxy=http://192.168.121.170:7890
# RUN apt-get update \
#     && apt-get install -y software-properties-common apt-transport-https wget \
#     && wget -q https://packages.microsoft.com/keys/microsoft.asc -O- | apt-key add - \
#     && add-apt-repository "deb [arch=amd64] https://packages.microsoft.com/repos/vscode stable main" \
#     && apt-get -y install code

CMD ["/bin/bash"]

COPY --chown=autoware ./entrypoint.sh /tmp
USER autoware
ENTRYPOINT ["/tmp/entrypoint.sh"]

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics