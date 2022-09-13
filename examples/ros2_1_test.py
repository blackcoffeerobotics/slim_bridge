#!/usr/bin/env python3

"""
* Copyright 2020, 2021, 2022 Black Coffee Robotics LLP (https://www.blackcoffeerobotics.com).

* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*/
"""
import yaml
import os

import rclpy
from std_msgs.msg import String


def main(args=None):
    """ Summary: Test script to enable new topics on the slim_bridge via ROS2 API."""
    rclpy.init(args=args)

    node = rclpy.create_node('ros2_1_test')
    qos = rclpy.qos.QoSProfile(durability=rclpy.qos.QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL, depth=1)
    publisher = node.create_publisher(String, '/ros2_to_1', qos_profile=qos)

    msg = String()

    dictionary = {
        'topic_name': '/test_string_2_1',
        'latch' : True,
        'reliability': 'reliable',
        'durability': 'transient_local',
        'history': 'keep_last',
        'depth': 1,
        'queue_size': 1,
        'type': 'std_msgs/String'
    }

    msg.data = str(yaml.dump(dictionary, Dumper=yaml.dumper.SafeDumper))
    publisher.publish(msg)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
