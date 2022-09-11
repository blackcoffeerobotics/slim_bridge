FROM ros:foxy

ARG DEBIAN_FRONTEND=noninteractive

# Preliminaries
RUN apt-get update \
    && apt-get upgrade -y

# ROS2 Related Installations
RUN apt-get install -q -y --no-install-recommends \
    ros-foxy-rmw-cyclonedds-cpp \
    python3-argcomplete curl nano

# ROS1 Related Installations
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' > /dev/null && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add - && \
    apt-get update && apt-get install -q -y --no-install-recommends \
    ros-noetic-ros-base ros-noetic-tf2-msgs

USER root

SHELL ["/bin/bash", "-c"]

ENV ROS1_INSTALL_PATH=/opt/ros/noetic
ENV ROS2_INSTALL_PATH=/opt/ros/foxy


# Building ROS2 Dependencies
RUN echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /root/.bashrc && \
    mkdir -p /root/bridge_ws/src/slim_bridge

# Slim Bridge Installation
COPY . /root/bridge_ws/src/slim_bridge/

# Copy over scripts for convenience
COPY scripts/* /

RUN source ${ROS1_INSTALL_PATH}/setup.bash && \
    source ${ROS2_INSTALL_PATH}/setup.bash && \
    cd /root/bridge_ws && \
    colcon build --symlink-install --packages-select slim_bridge

CMD ["./cmd_script.sh"]