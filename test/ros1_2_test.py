#!/usr/bin/env python3
import yaml
import os

import rospy
from std_msgs.msg import String


def main():
    rospy.init_node('ros1_2_test')
    robot_name = os.environ['ROBOT_NAME']
    publisher = rospy.Publisher(robot_name+'/ros1_to_2', String, queue_size=1, latch=True)
    msg = String()
    dictionary = {
    'topic_name': '/test_string_1_2',
    'latch': True,
    'reliability': 'reliable',
    'durability': 'transient_local',
    'history': 'keep_last',
    'depth': 1,
    'queue_size': 1,
    'type': 'std_msgs/String'
    }

    msg.data = str(yaml.dump(dictionary, Dumper=yaml.dumper.SafeDumper))
    publisher.publish(msg)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass