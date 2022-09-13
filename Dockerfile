FROM ros:foxy

# This is required else apt-get update throws Hash mismatch error
RUN apt-get clean && rm -rf /var/lib/apt/lists/* && apt-get update -yqq

# Install dependencies
RUN export DEBIAN_FRONTEND=noninteractive && \
    apt-get install --no-install-recommends -yqq \
    apt-utils \
    curl \
    ros-foxy-rmw-cyclonedds-cpp

# ROS1 Related Installations
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' > /dev/null && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | (OUT=$(sudo apt-key add - 2>&1) || echo $OUT) && \
    apt-get update && \
    export DEBIAN_FRONTEND=noninteractive && \
    apt-get install --no-install-recommends -yqq \
    ros-noetic-ros-base \
    ros-noetic-tf2-msgs

# Using shell to use bash commands like source
SHELL ["/bin/bash", "-c"]

# Export DDS environment variables
RUN echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /root/.bashrc && \
    mkdir -p /root/bridge_ws/src/slim_bridge

# Slim Bridge Installation
COPY . /root/bridge_ws/src/slim_bridge/

# Copy over scripts for convenience
COPY scripts/* /

# Install Slim Bridge
RUN source /opt/ros/noetic/setup.bash && \
    source /opt/ros/foxy/setup.bash && \
    cd /root/bridge_ws && \
    colcon build --symlink-install --packages-select slim_bridge

CMD ["./cmd_script.sh"]