# Bridge communication between ROS 1 and ROS 2 (But Slimmer!)

This package provides a 2-way communication interface between ROS2 and ROS1 transport systems. Our motivation for developing this arose from working with multi-robot systems where each robot ran a ROS1 navigation system provided by a third-party vendor, but the need to facilitate inter-robot and server <-> robot communications.  

Primarily we addressed the following issues per our requirements -

	1. Ability to setup varying QoS settings of each topic in ROS1<->ROS2 communications.
	2. Allow for introduction of new robots at runtime in the communication pipeline.
	3. Faster build time to facilitate compilation on edge SBCs (such as RPi/Jetson Nano)


There are two main modes of using this package, one is directly using source and the other one using Dockers.

1. Source Instructions

### Build

```
	source /opt/ros/noetic/setup.bash
	source /opt/ros/foxy/setup.bash
	colcon build
```

### Configure

Specify the topic names, QoS settings, direction of message movement and message types in the `config/initial_topics.yaml` . If you're not using symlink install for ROS2, you'll have to build the workspace again. More details are provided in the "Parameter Configuration" section below.

### Run

```
	roscore (if it isn't already running)
	cd bridge_ws # workspace where slim bridge is built
	ros2 launch slim_bridge bridge.launch.py
```

2. Docker Instructions

### Build

	docker build -t slim_bridge:latest .


### Configure
	Change the topic configuration in `scripts/initial_params.yaml`. At runtime, these parameters are mounted to the sourced workspace inside the docker

### Run
	In the scripts/ folder, simply run `start_docker.sh`. Done!


## Parameter configuration

The options presented here can be interpreted in two different ways, depending
on if the bridge is from ros2 to ros1 (or) ros1 to ros2.

```
ros2 to ros1 (ros2 subscriber, ros1 publisher)

topic_name: str # just the topic name
latch: bool # ros1 publisher option

reliability: str # "best_effort" or "reliable"
durability: str # "transient_local" or "volatile"
history: str # "keep_last" or "keep_all"
depth: int # Only considered if history is "keep_last"

queue_size: int # ros1 publisher option
type: str # The delimiter is '/', the string before '/' is the package name, and the one after is the message type.
```

```
ros1 to ros2 (ros1 subscriber, ros2 publisher)

topic_name: str # just the topic name
latch: bool # invalid/ignored

reliability: str # "best_effort" or "reliable"
durability: str # "transient_local" or "volatile"

queue_size: int # ros1 subscriber option
type: str # The delimiter is '/', the string before '/' is the package name, and the one after is the message type.
```

For initial topics: check file `initial_topics.yaml`


### Advanced Parameter configuration

The package allows dynamic addition of topics to be bridged at runtime. Calls to add topics can be made both via ROS1 and ROS2 using `std_msgs/String` type topic calls. Contents of the message are interpreted in `yaml` format, same as `initial_topics.yaml` . Sample scripts are provided in the `tests/` directory.