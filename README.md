# Bridge communication between ROS 1 and ROS 2

This package provides a 2-way communication interface between ROS2 and ROS1 transport systems. Our motivation for developing this arose from working with multi-robot systems where each robot ran a ROS1 navigation system provided by a third-party vendor, but the need to facilitate inter-robot and server <-> robot communications. We took inspiration from the [ROS1 bridge](https://github.com/ros2/ros1_bridge) package but made several changes to serve our use case.

Primarily we addressed the following limitations found in existing ROS1Bridge implementation to suit our requirements -

1. Ability to setup varying QoS settings of each topic in ROS1<->ROS2 communications.
2. Allow for introduction of new robots at runtime in the communication pipeline.
3. Faster build time to facilitate compilation on edge SBCs (such as RPi/Jetson Nano)

There are two main modes of using this package, one is directly using source and the other one using Dockers.

## Using Docker

Easiest way to setup and use this package is using Dockers. A demo video presenting the entire setup and bridging process is provided [here](res/images/demo.gif). Instructions are provided below - 
#### Build
	docker build -t slim_bridge:latest .
#### Configure
Change the topic configuration in `scripts/initial_params.yaml`. At runtime, these parameters are mounted to the sourced workspace inside the docker
#### Run
In the `scripts/` folder, simply run
	 `./start_docker.sh`

## Using Source
#### Build
	source /opt/ros/noetic/setup.bash
	source /opt/ros/foxy/setup.bash
	rosdep install --from-paths src --ignore-src -r -y
	colcon build
#### Configure
Specify the topic names, QoS settings, direction of message movement and message types in the `config/initial_topics.yaml` . If you're not using symlink install for ROS2, you'll have to build the workspace again. More details are provided in the "Parameter Configuration" section below.
#### Run
	roscore
	ros2 launch slim_bridge bridge.launch.py

## Parameter configuration
The options presented here can be interpreted in two different ways, depending
on if the bridge is from ROS2 to ROS1 (or) ROS1 to ROS2.

`ROS2 to ROS1` (ROS2 subscriber, ROS1 publisher)
```
topic_name: str # just the topic name
latch: bool # ROS1 publisher option

reliability: str # "best_effort" or "reliable"
durability: str # "transient_local" or "volatile"
history: str # "keep_last" or "keep_all"
depth: int # Only considered if history is "keep_last"

queue_size: int # ROS1 publisher option
type: str # package_name/message_type (such as "std_msgs/String")
```

`ROS1 to ROS2` (ROS1 subscriber, ROS2 publisher)
```
topic_name: str # just the topic name
latch: bool # invalid/ignored

reliability: str # "best_effort" or "reliable"
durability: str # "transient_local" or "volatile"

queue_size: int # ROS1 subscriber option
type: str # package_name/message_type (such as "std_msgs/String")
```

For initial topics: check file `initial_topics.yaml`


### Advanced Parameter configuration
The package allows dynamic addition of topics to be bridged at runtime. Calls to add topics can be made both via ROS1 and ROS2 using `std_msgs/String` type topic calls. Contents of the message are interpreted in `yaml` format, same as `initial_topics.yaml` . The calls to introduced new topics to the bridge use the following topics:
1. `/ros1_to_2` - To add transport from ROS1 to ROS2
2. `/ros2_to_1` - To add transport from ROS2 to ROS1

Sample scripts are provided in the `examples/` directory.


#### NOTE
Both "from source" and Docker based run instructions support namespacing of `slim_bridge` node and topics. If `ROBOT_NAME` environment variable is defined, the topics defined in the parameter file (and added via advanced configuration) take the namespace as per the environent variable. To avoid topic namespacing, use absolute topic names (prepend with a `/`)